/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/spi/spi.h>
#include "cam_lens_driver_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_lens_driver_soc.h"
#include "cam_lens_driver_core.h"
#include "cam_trace.h"

static int cam_lens_driver_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_lens_driver_ctrl_t *l_ctrl =
		v4l2_get_subdevdata(sd);

	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "l_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(l_ctrl->lens_driver_mutex));
	cam_lens_driver_shutdown(l_ctrl);
	mutex_unlock(&(l_ctrl->lens_driver_mutex));

	return 0;
}

static int cam_lens_driver_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open(CAM_LENS_DRIVER);

	if (crm_active) {
		CAM_DBG(CAM_LENS_DRIVER, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}
	return cam_lens_driver_subdev_close_internal(sd, fh);
}

static long cam_lens_driver_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_lens_driver_ctrl_t *l_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_lens_driver_cmd(l_ctrl, arg);
		if (rc)
			CAM_ERR(CAM_LENS_DRIVER, "Failed for driver_cmd: %d", rc);

		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
			return 0;
		}
		rc = cam_lens_driver_subdev_close_internal(sd, NULL);
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid ioctl cmd: %u", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}
	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_lens_driver_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		cmd = VIDIOC_CAM_CONTROL;
		rc = cam_lens_driver_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Failed in lens_driver subdev handling rc: %d",
				rc);
			return rc;
		}
	break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid compat ioctl: %d", cmd);
		rc = -ENOIOCTLCMD;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static struct v4l2_subdev_core_ops cam_lens_driver_subdev_core_ops = {
	.ioctl = cam_lens_driver_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_lens_driver_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_lens_driver_subdev_ops = {
	.core = &cam_lens_driver_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_lens_driver_internal_ops = {
	.close = cam_lens_driver_subdev_close,
};

static int cam_lens_driver_init_subdev(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int rc = 0;

	l_ctrl->v4l2_dev_str.internal_ops =
		&cam_lens_driver_internal_ops;
	l_ctrl->v4l2_dev_str.ops =
		&cam_lens_driver_subdev_ops;
	strlcpy(l_ctrl->device_name, CAM_LENS_DRIVER_NAME,
		sizeof(l_ctrl->device_name));
	l_ctrl->v4l2_dev_str.name =
		l_ctrl->device_name;
	l_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	l_ctrl->v4l2_dev_str.ent_function =
		CAM_LDM_DEVICE_TYPE;
	l_ctrl->v4l2_dev_str.token = l_ctrl;

	rc = cam_register_subdev(&(l_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_LENS_DRIVER, "Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static int cam_lens_driver_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                             rc = 0;
	int32_t                             i = 0;
	struct cam_lens_driver_ctrl_t      *l_ctrl = NULL;
	struct cam_lens_driver_soc_private *soc_private = NULL;
	struct cam_sensor_spi_client       *spi_client;
	struct spi_device                  *spi = to_spi_device(dev);

	if (spi == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "spi device is NULL");
		return -ENODEV;
	}

	CAM_DBG(CAM_LENS_DRIVER, "irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]",
		spi->irq, spi->chip_select, (spi->mode & SPI_CPHA) ? 1 : 0,
		(spi->mode & SPI_CPOL) ? 1 : 0,
		(spi->mode & SPI_CS_HIGH) ? 1 : 0);
	CAM_ERR(CAM_LENS_DRIVER, "max_speed[%u]", spi->max_speed_hz);

	spi->bits_per_word = 8;
	spi_setup(spi);

	CAM_DBG(CAM_LENS_DRIVER, "irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]",
		spi->irq, spi->chip_select, (spi->mode & SPI_CPHA) ? 1 : 0,
		(spi->mode & SPI_CPOL) ? 1 : 0,
		(spi->mode & SPI_CS_HIGH) ? 1 : 0);
	CAM_DBG(CAM_LENS_DRIVER, "max_speed[%u]", spi->max_speed_hz);

	/* Create lens_driver control structure */
	l_ctrl = kzalloc(sizeof(*l_ctrl), GFP_KERNEL);
	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "kzalloc failed");
		return -ENOMEM;
	}

	spi_client = kzalloc(sizeof(*spi_client), GFP_KERNEL);
	if (!spi_client) {
		kfree(l_ctrl);
		return -ENOMEM;
	}
	/*fill in platform device*/
	l_ctrl->soc_info.dev = &spi->dev;
	l_ctrl->soc_info.dev_name = spi->modalias;
	l_ctrl->ldm_device_type = MSM_CAMERA_SPI_DEVICE;
	l_ctrl->io_master_info.master_type = SPI_MASTER;
	l_ctrl->io_master_info.spi_client = spi_client;
	l_ctrl->isPowerUpSeqApply = 0;
	spi_client->spi_master = spi;
	spi_client->retry_delay = 1;
	spi_client->retries = 0;
	spi_client->spi_communication_ver = SPI_COMM_VERSION_1;

	soc_private = kzalloc(sizeof(struct cam_lens_driver_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_ctrl;
	}

	l_ctrl->soc_info.soc_private = soc_private;
	memset(&l_ctrl->fw_info, 0, sizeof(struct cam_cmd_ldm_fw_info));
	INIT_LIST_HEAD(&(l_ctrl->i2c_init_data.list_head));
	INIT_LIST_HEAD(&(l_ctrl->motor_control_data.list_head));
	INIT_LIST_HEAD(&(l_ctrl->ldm_calib_data.list_head));
	INIT_LIST_HEAD(&(l_ctrl->streamon_settings.list_head));
	INIT_LIST_HEAD(&(l_ctrl->streamoff_settings.list_head));
	for (i = 0; i < MAX_LDM_FW_COUNT; i++) {
		INIT_LIST_HEAD(&(l_ctrl->fw_init_data[i].list_head));
		INIT_LIST_HEAD(&(l_ctrl->fw_finalize_data[i].list_head));
	}

	rc = cam_lens_driver_parse_dt(l_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Parsing lens_driver dt failed rc %d", rc);
		goto free_soc;
	}

	rc = cam_lens_driver_init_subdev(l_ctrl);
	if (rc)
		goto free_soc;

	l_ctrl->bridge_intf.device_hdl = -1;
	l_ctrl->bridge_intf.link_hdl = -1;
	l_ctrl->bridge_intf.ops.get_dev_info = NULL;
	l_ctrl->bridge_intf.ops.link_setup = NULL;
	l_ctrl->bridge_intf.ops.apply_req = NULL;
	l_ctrl->bridge_intf.ops.flush_req = NULL;
	l_ctrl->last_flush_req = 0;

	spi_set_drvdata(spi, l_ctrl);
	l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
	v4l2_set_subdevdata(&l_ctrl->v4l2_dev_str.sd, l_ctrl);

	return rc;

free_soc:
	kfree(soc_private);
free_ctrl:
	kfree(l_ctrl);

	return rc;
}

static void cam_lens_driver_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int                            i;
	struct cam_lens_driver_ctrl_t *l_ctrl;
	struct spi_device             *spi = to_spi_device(dev);
	struct cam_hw_soc_info        *soc_info;

	l_ctrl = spi_get_drvdata(spi);
	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "Lens Driver device is NULL");
		return;
	}

	soc_info = &l_ctrl->soc_info;

	for (i = 0; i < soc_info->num_clk; i++) {
		if (!soc_info->clk[i]) {
			CAM_DBG(CAM_LENS_DRIVER, "%s handle is NULL skip put",
				soc_info->clk_name[i]);
			continue;
		}
		devm_clk_put(soc_info->dev, soc_info->clk[i]);
	}

	mutex_lock(&(l_ctrl->lens_driver_mutex));
	cam_lens_driver_shutdown(l_ctrl);
	mutex_unlock(&(l_ctrl->lens_driver_mutex));
	cam_unregister_subdev(&(l_ctrl->v4l2_dev_str));
	kfree(l_ctrl->soc_info.soc_private);
	l_ctrl->soc_info.soc_private = NULL;
	v4l2_set_subdevdata(&l_ctrl->v4l2_dev_str.sd, NULL);
	spi_set_drvdata(spi, NULL);
	kfree(l_ctrl);
}
const static struct component_ops cam_lens_driver_component_ops = {
	.bind = cam_lens_driver_component_bind,
	.unbind = cam_lens_driver_component_unbind,
};

static int32_t cam_lens_driver_spi_driver_probe(struct spi_device *spi)
{
	int32_t rc = 0;

	rc = component_add(&spi->dev, &cam_lens_driver_component_ops);
	if (rc)
		CAM_ERR(CAM_LENS_DRIVER, "failed to add component rc: %d", rc);

	return rc;
}

static int32_t cam_lens_driver_spi_driver_remove(struct spi_device *spi)
{
	component_del(&spi->dev, &cam_lens_driver_component_ops);
	return 0;
}

static int cam_lens_driver_i2c_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int                                 rc = 0;
	int32_t                             i = 0;
	struct i2c_client                   *client = NULL;
	struct cam_lens_driver_ctrl_t       *l_ctrl = NULL;
	struct cam_lens_driver_soc_private  *soc_private = NULL;

	client = container_of(dev, struct i2c_client, dev);
	if (client == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args client: %pK",
			client);
		return -EINVAL;
	}

	l_ctrl = kzalloc(sizeof(*l_ctrl), GFP_KERNEL);
	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "kzalloc failed");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, l_ctrl);

	l_ctrl->soc_info.dev = &client->dev;
	l_ctrl->soc_info.dev_name = client->name;
	l_ctrl->ldm_device_type = MSM_CAMERA_I2C_DEVICE;
	l_ctrl->io_master_info.master_type = I2C_MASTER;
	l_ctrl->io_master_info.client = client;
	l_ctrl->isPowerUpSeqApply = 0;
	soc_private = kzalloc(sizeof(struct cam_lens_driver_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto lctrl_free;
	}

	l_ctrl->soc_info.soc_private = soc_private;
	memset(&l_ctrl->fw_info, 0, sizeof(struct cam_cmd_ldm_fw_info));
	INIT_LIST_HEAD(&(l_ctrl->i2c_init_data.list_head));
	INIT_LIST_HEAD(&(l_ctrl->motor_control_data.list_head));
	INIT_LIST_HEAD(&(l_ctrl->ldm_calib_data.list_head));
	INIT_LIST_HEAD(&(l_ctrl->streamon_settings.list_head));
	INIT_LIST_HEAD(&(l_ctrl->streamoff_settings.list_head));
	for (i = 0; i < MAX_LDM_FW_COUNT; i++) {
		INIT_LIST_HEAD(&(l_ctrl->fw_init_data[i].list_head));
		INIT_LIST_HEAD(&(l_ctrl->fw_finalize_data[i].list_head));
	}

	rc = cam_lens_driver_parse_dt(l_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Parsing lens_driver dt failed rc %d", rc);
		goto free_soc;
	}

	rc = cam_lens_driver_init_subdev(l_ctrl);
	if (rc)
		goto free_soc;

	l_ctrl->bridge_intf.device_hdl = -1;
	l_ctrl->bridge_intf.link_hdl = -1;
	l_ctrl->bridge_intf.ops.get_dev_info = NULL;
	l_ctrl->bridge_intf.ops.link_setup = NULL;
	l_ctrl->bridge_intf.ops.apply_req = NULL;
	l_ctrl->bridge_intf.ops.flush_req = NULL;
	l_ctrl->last_flush_req = 0;

	l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
	v4l2_set_subdevdata(&l_ctrl->v4l2_dev_str.sd, l_ctrl);

	return rc;

free_soc:
	kfree(soc_private);
lctrl_free:
	kfree(l_ctrl);
probe_failure:
	return rc;
}

static void cam_lens_driver_i2c_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int                             i;
	struct i2c_client              *client = NULL;
	struct cam_lens_driver_ctrl_t  *l_ctrl = NULL;
	struct cam_hw_soc_info         *soc_info;

	client = container_of(dev, struct i2c_client, dev);
	if (!client) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Failed to get i2c client");
		return;
	}

	l_ctrl = i2c_get_clientdata(client);
	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "lens_driver device is NULL");
		return;
	}

	CAM_INFO(CAM_LENS_DRIVER, "i2c driver remove invoked");
	soc_info = &l_ctrl->soc_info;

	for (i = 0; i < soc_info->num_clk; i++) {
		if (!soc_info->clk[i]) {
			CAM_DBG(CAM_LENS_DRIVER, "%s handle is NULL skip put",
				soc_info->clk_name[i]);
			continue;
		}
		devm_clk_put(soc_info->dev, soc_info->clk[i]);
	}

	mutex_lock(&(l_ctrl->lens_driver_mutex));
	cam_lens_driver_shutdown(l_ctrl);
	mutex_unlock(&(l_ctrl->lens_driver_mutex));
	cam_unregister_subdev(&(l_ctrl->v4l2_dev_str));

	kfree(l_ctrl->soc_info.soc_private);
	v4l2_set_subdevdata(&l_ctrl->v4l2_dev_str.sd, NULL);
	kfree(l_ctrl);
}

const static struct component_ops cam_lens_driver_i2c_component_ops = {
	.bind = cam_lens_driver_i2c_component_bind,
	.unbind = cam_lens_driver_i2c_component_unbind,
};

static int cam_lens_driver_i2c_driver_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_LENS_DRIVER, "%s :: i2c_check_functionality failed",
			client->name);
		return -EFAULT;
	}

	CAM_DBG(CAM_LENS_DRIVER, "Adding sensor lens_driver component");
	rc = component_add(&client->dev, &cam_lens_driver_i2c_component_ops);
	if (rc)
		CAM_ERR(CAM_LENS_DRIVER, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_lens_driver_i2c_driver_remove(struct i2c_client *client)
{
	component_del(&client->dev, &cam_lens_driver_i2c_component_ops);

	return 0;
}

////////////////////////////////////
static const struct of_device_id cam_lens_driver_spi_dt_match[] = {
	{.compatible = "qcom,cam-spi-lens-driver"},
	{}
};
MODULE_DEVICE_TABLE(of, cam_lens_driver_spi_dt_match);

struct spi_driver cam_lens_driver_spi_driver = {
	.driver = {
		.name = LENS_DRIVER_SPI,
		.owner = THIS_MODULE,
		.of_match_table = cam_lens_driver_spi_dt_match,
	},
	.probe = cam_lens_driver_spi_driver_probe,
	.remove = cam_lens_driver_spi_driver_remove,
};

static const struct of_device_id cam_lens_driver_i2c_dt_match[] = {
	{.compatible = "qcom,cam-i2c-lens-driver"},
	{}
};
MODULE_DEVICE_TABLE(of, cam_lens_driver_i2c_dt_match);

static const struct i2c_device_id cam_lens_driver_i2c_id[] = {
	{ LENS_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

struct i2c_driver cam_lens_driver_i2c_driver = {
	.id_table = cam_lens_driver_i2c_id,
	.probe  = cam_lens_driver_i2c_driver_probe,
	.remove = cam_lens_driver_i2c_driver_remove,
	.driver = {
		.name = LENS_DRIVER_I2C,
		.owner = THIS_MODULE,
		.of_match_table = cam_lens_driver_i2c_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_lens_driver_init(void)
{
	int32_t rc = 0;

	rc = i2c_add_driver(&cam_lens_driver_i2c_driver);
	if (rc) {
		CAM_ERR(CAM_LENS_DRIVER, "i2c_add_driver failed rc = %d", rc);
		return rc;
	}

	rc = spi_register_driver(&cam_lens_driver_spi_driver);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "spi_register_driver failed rc = %d", rc);
		return rc;
	}

	return rc;
}

void cam_lens_driver_exit(void)
{
	i2c_del_driver(&cam_lens_driver_i2c_driver);
	spi_unregister_driver(&cam_lens_driver_spi_driver);
}

MODULE_DESCRIPTION("cam_lens_driver");
MODULE_LICENSE("GPL v2");
