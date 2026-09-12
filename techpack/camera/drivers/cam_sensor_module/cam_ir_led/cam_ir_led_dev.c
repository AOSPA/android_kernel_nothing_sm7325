/* Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/pwm.h>
#include "cam_sensor_cmn_header.h"
#include "cam_ir_led_dev.h"
#include "cam_ir_led_soc.h"
#include "cam_ir_led_core.h"
#include "cam_packet_util.h"
#include "camera_main.h"

static int32_t cam_pmic_ir_cut_off(struct cam_ir_led_ctrl *ictrl);

static struct cam_ir_led_table cam_pmic_ir_led_table;
static struct cam_ir_led_table cam_gpio_ir_led_table;
static struct cam_ir_led_table cam_i2c_ir_led_table;

static struct cam_ir_led_table *ir_led_table[] = {
	&cam_pmic_ir_led_table,
	&cam_gpio_ir_led_table,
	&cam_i2c_ir_led_table,
};

static int32_t cam_pmic_ir_led_init(
	struct cam_ir_led_ctrl *ictrl)
{
	return ictrl->func_tbl->camera_ir_led_off(ictrl);
}

static int32_t cam_pmic_ir_led_release(
	struct cam_ir_led_ctrl *ictrl)
{
	int32_t rc = 0;

	CAM_DBG(CAM_IR_LED, "Enter");
	rc = ictrl->func_tbl->camera_ir_led_off(ictrl);
	if (rc < 0) {
		CAM_ERR(CAM_IR_LED, "camera_ir_led_off failed (%d)", rc);
		return rc;
	}
	return rc;
}

static int32_t cam_pmic_ir_led_off(struct cam_ir_led_ctrl *ictrl)
{
	int32_t rc = 0;

	CAM_DBG(CAM_IR_LED, "Enter");
	if (ictrl->pwm_dev) {
		pwm_disable(ictrl->pwm_dev);
	} else {
		CAM_ERR(CAM_IR_LED, "pwm device is null");
		return -EINVAL;
	}

	rc = gpio_direction_input(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio);
	if (rc)
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);

	CAM_INFO(CAM_IR_LED, "CAM_IR_CUT_PACKET_OPCODE_OFF_Output_GPIO_1:%d",
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio);
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio,
		0);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	CAM_INFO(CAM_IR_LED, "CAM_IR_CUT_PACKET_OPCODE_OFF_Output_GPIO_2:%d",
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio);
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
		1);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	msleep(CAM_IR_MSLEEP_VALUE);
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
		0);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	return rc;
}

static int32_t cam_pmic_ir_led_on(
	struct cam_ir_led_ctrl *ictrl,
	struct cam_ir_led_set_on_off *ir_led_data)
{
	int rc;

	if (ictrl->pwm_dev) {
		rc = pwm_config(ictrl->pwm_dev,
			ir_led_data->pwm_duty_on_ns,
			ir_led_data->pwm_period_ns);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "PWM config failed (%d)", rc);
			return rc;
		}

		rc = pwm_enable(ictrl->pwm_dev);
		CAM_DBG(CAM_IR_LED, "enabled=%d, period=%llu, duty_cycle=%llu",
			ictrl->pwm_dev->state.enabled,
			ictrl->pwm_dev->state.period,
			ictrl->pwm_dev->state.duty_cycle);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "PWM enable failed(%d)", rc);
			return rc;
		}
		rc = gpio_direction_output(
			ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
			1);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
			return rc;
		}
		msleep(CAM_IR_MSLEEP_VALUE);
		rc = gpio_direction_output(
			ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio,
			1);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
			return rc;
		}
	} else {
		CAM_ERR(CAM_IR_LED, "pwm device is null");
	}

	return 0;
}

static int32_t cam_pmic_ir_cut_off(struct cam_ir_led_ctrl *ictrl)
{
	int32_t rc = 0;
	CAM_DBG(CAM_IR_LED, "Enter cam_pmic_ir_cut_off");

	rc = gpio_direction_input(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio);
	if (rc)
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);

	CAM_INFO(CAM_IR_LED, "CAM_IR_CUT_PACKET_OPCODE_OFF_Output_GPIO_0:%d   1",
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio);
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio,
		1);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
		0);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	msleep(CAM_IR_MSLEEP_VALUE);
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio,
		0);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	CAM_INFO(CAM_IR_LED, "sleep CAM_IR_CUT_PACKET_OPCODE_OFF_Output_GPIO_1:%d    0",
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio);
	rc = gpio_direction_output(
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
		0);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
		return rc;
	}
	return rc;
}

static int32_t cam_pmic_ir_cut_on(
	struct cam_ir_led_ctrl *ictrl,
	struct cam_ir_led_set_on_off *ir_led_data)
{
    int32_t rc = 0;
    CAM_DBG(CAM_IR_LED, "Enter cam_pmic_ir_cut_on, ictrl->ir_led_state:%d", ictrl->ir_led_state);

    if (ictrl->ir_led_state == CAM_IR_LED_STATE_ON) {
        if (ictrl->pwm_dev)
            pwm_disable(ictrl->pwm_dev);
    }

    rc = gpio_direction_output(
            ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio,
            0);
    if (rc) {
        CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
        return rc;
    }

    rc = gpio_direction_output(
            ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
            1);
    if (rc) {
        CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
        return rc;
    }
    msleep(CAM_IR_MSLEEP_VALUE);
    rc = gpio_direction_output(
            ictrl->soc_info.gpio_data->cam_gpio_common_tbl[0].gpio,
            0);
    if (rc) {
        CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
        return rc;
    }
	CAM_DBG(CAM_IR_LED, "sleep CAM_IR_CUT_PACKET_OPCODE_ON_Output_GPIO_1:%d   1",
		ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio);
    rc = gpio_direction_output(
            ictrl->soc_info.gpio_data->cam_gpio_common_tbl[1].gpio,
            0);
    if (rc) {
        CAM_ERR(CAM_IR_LED, "gpio operation failed(%d)", rc);
        return rc;
    }
    return rc;
}

static int32_t cam_ir_led_handle_init(
	struct cam_ir_led_ctrl *ictrl)
{
	uint32_t i = 0;
	int32_t rc = -EFAULT;
	enum cam_ir_led_driver_type ir_led_driver_type =
					ictrl->ir_led_driver_type;

	CAM_DBG(CAM_IR_LED, "IRLED HW type=%d", ir_led_driver_type);
	for (i = 0; i < ARRAY_SIZE(ir_led_table); i++) {
		if (ir_led_driver_type == ir_led_table[i]->ir_led_driver_type) {
			ictrl->func_tbl = &ir_led_table[i]->func_tbl;
			rc = 0;
			break;
		}
	}

	if (rc < 0) {
		CAM_ERR(CAM_IR_LED, "failed invalid ir_led_driver_type %d",
				ir_led_driver_type);
		return -EINVAL;
	}

	if (ictrl->func_tbl->camera_ir_led_init != NULL) {
		rc = ictrl->func_tbl->camera_ir_led_init(ictrl);
		if (rc < 0)
			CAM_ERR(CAM_IR_LED, "ir_led init failed (%d)", rc);
	}

	return rc;
}

static int32_t cam_irled_slaveInfo_pkt_parser(struct cam_ir_led_ctrl *ictrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_i2c_info *i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;

	if (len < sizeof(struct cam_cmd_i2c_info)) {
		CAM_ERR(CAM_IR_LED, "Not enough buffer");
		return -EINVAL;
	}

	if (ictrl->io_master_info.master_type == CCI_MASTER) {
		ictrl->io_master_info.cci_client->cci_i2c_master =
			ictrl->cci_i2c_master;
		ictrl->io_master_info.cci_client->i2c_freq_mode =
			i2c_info->i2c_freq_mode;
		ictrl->io_master_info.cci_client->sid =
			i2c_info->slave_addr >> 1;
		CAM_DBG(CAM_IR_LED, "Slave addr: 0x%x Freq Mode: %d",
			i2c_info->slave_addr, i2c_info->i2c_freq_mode);
	} else if (ictrl->io_master_info.master_type == I2C_MASTER) {
		ictrl->io_master_info.client->addr = i2c_info->slave_addr;
		CAM_DBG(CAM_IR_LED, "Slave addr: 0x%x", i2c_info->slave_addr);
	} else {
		CAM_ERR(CAM_IR_LED, "Invalid Master type: %d",
			ictrl->io_master_info.master_type);
		 rc = -EINVAL;
	}

	return rc;
}

int cam_i2c_ir_cut_ops(struct cam_ir_led_ctrl *ictrl,
	uint64_t req_id)
{
	int rc = 0;
	struct cam_hw_soc_info *soc_info = &ictrl->soc_info;

	struct cam_sensor_power_ctrl_t *ircut_info =
			&(ictrl->per_frame[req_id % MAX_PER_FRAME_ARRAY].ircut_info);
	if (!ircut_info || !soc_info) {
		CAM_ERR(CAM_IR_LED, "IRCUT Info is NULL");
		return -EINVAL;
	}
	ircut_info->dev = soc_info->dev;

	rc = cam_config_ircut(ircut_info, soc_info, &ictrl->is_ircut_gpio_requested);

	return rc;
}

int cam_i2c_ir_led_power_ops(struct cam_ir_led_ctrl *ictrl,
	bool regulator_enable)
{
	int rc = 0;
	struct cam_hw_soc_info *soc_info = &ictrl->soc_info;
	struct cam_sensor_power_ctrl_t *power_info =
		&ictrl->power_info;

	if (!power_info || !soc_info) {
		CAM_ERR(CAM_IR_LED, "Power Info is NULL");
		return -EINVAL;
	}

	// Some irled device doesn't need to power settings.
	if ((power_info->power_setting_size == 0) &&
		(power_info->power_down_setting_size == 0)) {
		CAM_WARN(CAM_IR_LED, "Power Setting is NULL");
		return rc;
	}

	power_info->dev = soc_info->dev;

	if (power_info->power_setting_size == 0) {
		rc = 0;
	} else if (soc_info->num_rgltr > 0){
		/* Parse and fill vreg params for power up settings */
		rc = msm_camera_fill_vreg_params(soc_info,
			power_info->power_setting,
			power_info->power_setting_size);
		if (rc) {
			CAM_ERR(CAM_IR_LED,
				"failed to fill vreg params for power up rc:%d", rc);
			return rc;
		}
	}

	if (power_info->power_down_setting_size == 0) {
		rc = 0;
	} else if (soc_info->num_rgltr > 0){
		/* Parse and fill vreg params for power down settings*/
		rc = msm_camera_fill_vreg_params(
			soc_info,
			power_info->power_down_setting,
			power_info->power_down_setting_size);
		if (rc) {
			CAM_ERR(CAM_IR_LED,
				"failed to fill vreg params power down rc:%d", rc);
			return rc;
		}
	}

	if (regulator_enable && !ictrl->is_regulator_enabled) {
		rc = cam_sensor_core_power_up(power_info, soc_info);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "failed in irled power up rc %d", rc);
			return rc;
		}

		rc = camera_io_init(&(ictrl->io_master_info));
		if (rc) {
			CAM_ERR(CAM_IR_LED, "cci_init failed");
			cam_sensor_util_power_down(power_info, soc_info);
			goto free_pwr_settings;
		}
		ictrl->is_regulator_enabled = true;
	} else if (!regulator_enable && ictrl->is_regulator_enabled) {
		rc = cam_sensor_util_power_down(power_info, soc_info);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "failed in irled power down rc %d", rc);
			return rc;
		}

		camera_io_release(&(ictrl->io_master_info));
		ictrl->is_regulator_enabled = false;
		goto free_pwr_settings;
	}

	return rc;

free_pwr_settings:
	kfree(power_info->power_down_setting);
	kfree(power_info->power_setting);
	power_info->power_down_setting = NULL;
	power_info->power_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	return rc;
}

static int cam_ir_led_i2c_flush_nrt(struct cam_ir_led_ctrl *ictrl)
{
	int rc = 0;

	if (ictrl->i2c_data.init_settings.is_settings_valid == true) {
		rc = delete_request(&ictrl->i2c_data.init_settings);
		if (rc) {
			CAM_WARN(CAM_IR_LED,
				"Failed to delete Init i2c_setting: %d",
				rc);
			return rc;
		}
	}
	if (ictrl->i2c_data.config_settings.is_settings_valid == true) {
		rc = delete_request(&ictrl->i2c_data.config_settings);
		if (rc) {
			CAM_WARN(CAM_IR_LED,
				"Failed to delete NRT i2c_setting: %d",
				rc);
			return rc;
		}
	}
	if (ictrl->i2c_data.streamoff_settings.is_settings_valid == true) {
		rc = delete_request(&ictrl->i2c_data.streamoff_settings);
		if (rc) {
			CAM_WARN(CAM_IR_LED,
				"Failed to delete NRT i2c_setting: %d",
				rc);
			return rc;
		}
	}

	return rc;
}

static int cam_ir_led_i2c_delete_req(struct cam_ir_led_ctrl *ictrl,
	uint64_t req_id)
{
	int i = 0, rc = 0;
	uint64_t top = 0, del_req_id = 0;

	if (req_id == 0) {
		cam_ir_led_i2c_flush_nrt(ictrl);
	} else {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((req_id >=
				ictrl->i2c_data.per_frame[i].request_id) &&
				(top <
				ictrl->i2c_data.per_frame[i].request_id) &&
				(ictrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				del_req_id = top;
				top = ictrl->i2c_data.per_frame[i].request_id;
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			return rc;

		CAM_DBG(CAM_IR_LED, "top: %llu, del_req_id:%llu",
			top, del_req_id);

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((del_req_id >
				 ictrl->i2c_data.per_frame[i].request_id) && (
				 ictrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				ictrl->i2c_data.per_frame[i].request_id = 0;
				rc = delete_request(
					&(ictrl->i2c_data.per_frame[i]));
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"Delete request Fail:%lld rc:%d",
						del_req_id, rc);
			}
		}
	}

	return 0;
}

static void cam_ir_cut_delete_req(struct cam_ir_led_ctrl *ictrl,
	uint64_t req_id)
{
	struct cam_ir_cut_frame_setting *ir_cut_data = NULL;
	int frame_offset = 0;

	uint64_t top = 0, del_req_id = 0;
	int i = 0;

	if (req_id != 0) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			ir_cut_data = &ictrl->per_frame[i];
			if (req_id >= ir_cut_data->request_id &&
				ir_cut_data->is_settings_valid
				== 1) {
				if (top < ir_cut_data->request_id) {
					del_req_id = top;
					top = ir_cut_data->request_id;
				} else if (top >
					ir_cut_data->request_id &&
					del_req_id <
					ir_cut_data->request_id) {
					del_req_id =
						ir_cut_data->request_id;
				}
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			return;

		CAM_DBG(CAM_IR_LED, "top: %llu, del_req_id:%llu",
			top, del_req_id);
	}
	/* delete the request */
	frame_offset = del_req_id % MAX_PER_FRAME_ARRAY;
	//frame_offset = req_id % MAX_PER_FRAME_ARRAY;
	ir_cut_data = &ictrl->per_frame[frame_offset];
	ir_cut_data->request_id = 0;
	ir_cut_data->is_settings_valid = false;
}

int cam_i2c_ir_led_apply_setting(struct cam_ir_led_ctrl *ictrl,
	uint64_t req_id)
{
	struct i2c_settings_list *i2c_list;
	struct i2c_settings_array *i2c_set = NULL;
	int frame_offset = 0, rc = 0;

	CAM_DBG(CAM_IR_LED, "req_id=%llu", req_id);

	if (req_id == 0) {
		/* NonRealTime Init settings*/
		if (ictrl->i2c_data.init_settings.is_settings_valid == true) {
			list_for_each_entry(i2c_list,
				&(ictrl->i2c_data.init_settings.list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(ictrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed to apply init settings: %d",
					rc);
					return rc;
				}
			}
		}

		if (ictrl->i2c_data.config_settings.is_settings_valid == true) {
			list_for_each_entry(i2c_list,
				&(ictrl->i2c_data.config_settings.list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(ictrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed to apply config settings: %d",
					rc);
					return rc;
				}
			}
		}

		if (ictrl->i2c_data.streamoff_settings.is_settings_valid == true) {
			list_for_each_entry(i2c_list,
				&(ictrl->i2c_data.streamoff_settings.list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(ictrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed to apply stream off settings: %d",
					rc);
					return rc;
				}
			}
		}

	} else {
		/* RealTime */
		frame_offset = req_id % MAX_PER_FRAME_ARRAY;
		i2c_set = &ictrl->i2c_data.per_frame[frame_offset];
		if ((i2c_set->is_settings_valid == true) &&
			(i2c_set->request_id == req_id)) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(ictrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed to apply config settings: %d",
					rc);
					return rc;
				}
			}
		}

		if ((ictrl->per_frame[frame_offset].ircut_info.power_setting_size > 0) ||
			(ictrl->per_frame[frame_offset].ircut_info.power_down_setting_size > 0)) {
			if ((ictrl->per_frame[frame_offset].is_settings_valid) &&
				(ictrl->per_frame[frame_offset].request_id == req_id)) {
				if (ictrl->func_tbl->ircut_ops != NULL) {
					rc = ictrl->func_tbl->ircut_ops(ictrl, req_id);
					if (rc) {
						CAM_ERR(CAM_IR_LED,
							"Update IRCUT Failed rc = %d", rc);
					}
				}
			}
		}
	}

	cam_ir_led_i2c_delete_req(ictrl, req_id);
	cam_ir_cut_delete_req(ictrl, req_id);
	return rc;
}

int cam_i2c_ir_led_flush_request(struct cam_ir_led_ctrl *ictrl,
	enum cam_ir_led_flush_type type, uint64_t req_id)
{
	int rc = 0;
	int i = 0;
	uint32_t cancel_req_id_found = 0;
	struct i2c_settings_array *i2c_set = NULL;

	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "Device data is NULL");
		return -EINVAL;
	}
	if ((type == IR_FLUSH_REQ) && (req_id == 0)) {
		/* This setting will be called only when NonRealTime
		 * settings needs to clean.
		 */
		cam_ir_led_i2c_flush_nrt(ictrl);
	} else {
		/* All other usecase will be handle here */
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(ictrl->i2c_data.per_frame[i]);

			if ((type == IR_FLUSH_REQ) &&
				(i2c_set->request_id != req_id))
				continue;

			if (i2c_set->is_settings_valid == 1) {
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_IR_LED,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);

				if (type == IR_FLUSH_REQ) {
					cancel_req_id_found = 1;
					break;
				}
			}
		}
	}

	if ((type == IR_FLUSH_REQ) && (req_id != 0) &&
			(!cancel_req_id_found))
		CAM_DBG(CAM_IR_LED,
			"Flush request id:%lld not found in the pending list",
			req_id);

	return rc;
}

static int32_t cam_ir_led_config(struct cam_ir_led_ctrl *ictrl,
	void *arg)
{
	int rc = 0, i = 0;
	uint32_t  *cmd_buf =  NULL;
	uint32_t total_cmd_buf_in_bytes = 0;
	uint32_t processed_cmd_buf_in_bytes = 0;
	uint16_t cmd_length_in_bytes = 0;
	uintptr_t generic_ptr;
	uint32_t  *offset = NULL;
	uint32_t frm_offset = 0;
	size_t len_of_buffer;
	size_t remain_len;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_config_dev_cmd config;
	struct cam_req_mgr_add_request add_req;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct cam_ir_led_set_on_off *cam_ir_led_info = NULL;
	struct common_header  *cmn_hdr = NULL;
	struct cam_irled_init *irled_init = NULL;
	struct i2c_data_settings *i2c_data = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;

	if (!ictrl || !arg) {
		CAM_ERR(CAM_IR_LED, "enter cam_ir_led_config");
		return -EINVAL;
	}
	/* getting CSL Packet */
	ioctl_ctrl = (struct cam_control *)arg;

	if (copy_from_user((&config), u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config))) {
		CAM_ERR(CAM_IR_LED, "Copy cmd handle from user failed");
		rc = -EFAULT;
		return rc;
	}

	rc = cam_mem_get_cpu_buf(config.packet_handle,
		(uintptr_t *)&generic_ptr, &len_of_buffer);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "Failed in getting the buffer : %d", rc);
		return rc;
	}
	remain_len = len_of_buffer;

	if (config.offset > len_of_buffer) {
		CAM_ERR(CAM_IR_LED,
			"offset is out of bounds: offset: %lld len: %zu",
			config.offset, len_of_buffer);
		return -EINVAL;
	}

	remain_len -= (size_t)config.offset;
	/* Add offset to the ir_led csl header */
	csl_packet = (struct cam_packet *)(uintptr_t)(generic_ptr +
			config.offset);
	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_IR_LED, "Invalid packet params");
		cam_mem_put_cpu_buf(config.packet_handle);
		return -EINVAL;
	}

	offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
		csl_packet->cmd_buf_offset);
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_IR_LED_PACKET_OPCODE_INIT &&
		csl_packet->header.request_id <= ictrl->last_flush_req
		&& ictrl->last_flush_req != 0) {
		CAM_DBG(CAM_IR_LED,
			"reject request %lld, last request to flush %lld",
			csl_packet->header.request_id, ictrl->last_flush_req);
		cam_mem_put_cpu_buf(config.packet_handle);
		return -EINVAL;
	}

	if (csl_packet->header.request_id > ictrl->last_flush_req)
		ictrl->last_flush_req = 0;

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_IR_LED_PACKET_OPCODE_INIT:
		rc = cam_ir_led_handle_init(ictrl);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed to init device: rc=%d", rc);
			return rc;
		}

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc)
				return rc;

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			processed_cmd_buf_in_bytes = 0;
			if (!total_cmd_buf_in_bytes)
				continue;
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buffer);
			if (rc < 0) {
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "Failed to get cpu buf");
				return rc;
			}

			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_IR_LED, "invalid cmd buf");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return -EINVAL;
			}

			if ((len_of_buffer < sizeof(struct common_header)) ||
				(cmd_desc[i].offset >
				(len_of_buffer -
				sizeof(struct common_header)))) {
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "invalid cmd buf length");
				return -EINVAL;
			}

			remain_len = len_of_buffer - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmn_hdr = (struct common_header *)cmd_buf;
			/* Loop through cmd formats in one cmd buffer */
			CAM_DBG(CAM_IR_LED,
				"command Type: %d,Processed: %d,Total: %d",
				cmn_hdr->cmd_type, processed_cmd_buf_in_bytes,
				total_cmd_buf_in_bytes);

			switch (cmn_hdr->cmd_type) {
			case CAMERA_SENSOR_IRLED_CMD_TYPE_INIT_INFO:
				if (len_of_buffer <
					sizeof(struct cam_irled_init)) {
					CAM_ERR(CAM_IR_LED, "Not enough buffer");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return -EINVAL;
				}

				irled_init = (struct cam_irled_init *)cmd_buf;
				ictrl->irled_type = irled_init->irled_type;
				ictrl->is_regulator_enabled = false;
				cmd_length_in_bytes =
					sizeof(struct cam_irled_init);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_irled_slaveInfo_pkt_parser(
					ictrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_IR_LED,
					"Failed parsing slave info: rc: %d",
					rc);
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				cmd_length_in_bytes =
					sizeof(struct cam_cmd_i2c_info);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_IR_LED,
					"Received power settings");
				cmd_length_in_bytes =
					total_cmd_buf_in_bytes;
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					&ictrl->power_info, remain_len);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed update power settings");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			default:
				CAM_DBG(CAM_IR_LED,
					"Received initSettings");
				i2c_data = &(ictrl->i2c_data);
				i2c_reg_settings =
					&ictrl->i2c_data.init_settings;

				i2c_reg_settings->request_id = 0;
				i2c_reg_settings->is_settings_valid = true;
				rc = cam_sensor_i2c_command_parser(
					&ictrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_IR_LED,
					"pkt parsing failed: %d", rc);
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				cmd_length_in_bytes =
					cmd_desc[i].length;
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);

				break;
			}
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);

		}

		if (ictrl->func_tbl->power_ops != NULL) {
			rc = ictrl->func_tbl->power_ops(ictrl, true);
			if (rc) {
				CAM_WARN(CAM_IR_LED,
					"Enable Regulator Failed rc = %d", rc);
			}
		}

		if (ictrl->func_tbl->apply_setting != NULL) {
			rc = ictrl->func_tbl->apply_setting(ictrl, 0);
			if (rc) {
				CAM_ERR(CAM_IR_LED, "cannot apply settings rc = %d", rc);
				cam_mem_put_cpu_buf(config.packet_handle);
				return rc;
			}
		}

		ictrl->ir_led_state = CAM_IR_LED_STATE_CONFIG;
		break;
	case CAM_IR_LED_PACKET_OPCODE_SET_OPS:
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc)
				return rc;

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			processed_cmd_buf_in_bytes = 0;
			if (!total_cmd_buf_in_bytes)
				continue;
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buffer);
			if (rc < 0) {
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "Failed to get cpu buf");
				return rc;
			}

			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_IR_LED, "invalid cmd buf");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return -EINVAL;
			}

			if ((len_of_buffer < sizeof(struct common_header)) ||
				(cmd_desc[i].offset >
				(len_of_buffer -
				sizeof(struct common_header)))) {
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "invalid cmd buf length");
				return -EINVAL;
			}
			remain_len = len_of_buffer - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmn_hdr = (struct common_header *)cmd_buf;

			/* Loop through cmd formats in one cmd buffer */
			CAM_DBG(CAM_IR_LED,
				"command Type: %d,Processed: %d,Total: %d",
				cmn_hdr->cmd_type, processed_cmd_buf_in_bytes,
				total_cmd_buf_in_bytes);

			switch (cmn_hdr->cmd_type) {
			case CAMERA_SENSOR_IRCUT_CMD_TYPE_ON:
			case CAMERA_SENSOR_IRCUT_CMD_TYPE_OFF:
				CAM_DBG(CAM_IR_LED,
					"Received ircut settings");
				frm_offset = csl_packet->header.request_id %
				    MAX_PER_FRAME_ARRAY;
				cmd_length_in_bytes =
					total_cmd_buf_in_bytes;

				/* Reuse power implementions to control gpio for ircut */
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					&(ictrl->per_frame[frm_offset].ircut_info), remain_len);
				ictrl->per_frame[frm_offset].request_id = csl_packet->header.request_id;
				ictrl->per_frame[frm_offset].is_settings_valid = true;

				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed update power settings");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;

			default:
				/* add support for handling i2c_data*/
				frm_offset = csl_packet->header.request_id %
					MAX_PER_FRAME_ARRAY;
				i2c_reg_settings =
					&ictrl->i2c_data.per_frame[frm_offset];
				if (i2c_reg_settings->is_settings_valid == true) {
					CAM_DBG(CAM_IR_LED, "settings already valid");
					i2c_reg_settings->request_id = 0;
					i2c_reg_settings->is_settings_valid = false;
				}

				i2c_reg_settings->is_settings_valid = true;
				i2c_reg_settings->request_id =
					csl_packet->header.request_id;
				rc = cam_sensor_i2c_command_parser(
					&ictrl->io_master_info,
					i2c_reg_settings, cmd_desc, 1, NULL);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
						"Failed in parsing i2c RT packets");
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			}
		}

		if ((ictrl->func_tbl->apply_setting != NULL) && ((i2c_reg_settings != NULL) || (ictrl->per_frame[1].is_settings_valid == true))) {
			rc = ictrl->func_tbl->apply_setting(ictrl, 1);
			if (rc) {
				CAM_ERR(CAM_IR_LED, "cannot apply settings rc = %d", rc);
				cam_mem_put_cpu_buf(config.packet_handle);
				return rc;
			}
		}

		break;
	case CAM_IR_LED_PACKET_OPCODE_NON_REALTIME_SET_OPS:
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc)
				return rc;

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			processed_cmd_buf_in_bytes = 0;
			if (!total_cmd_buf_in_bytes)
				continue;
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buffer);
			if (rc < 0) {
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "Failed to get cpu buf");
				return rc;
			}

			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_IR_LED, "invalid cmd buf");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return -EINVAL;
			}

			if ((len_of_buffer < sizeof(struct common_header)) ||
				(cmd_desc[i].offset >
				(len_of_buffer -
				sizeof(struct common_header)))) {
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "invalid cmd buf length");
				return -EINVAL;
			}
			remain_len = len_of_buffer - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmn_hdr = (struct common_header *)cmd_buf;

			/* Loop through cmd formats in one cmd buffer */
			CAM_DBG(CAM_IR_LED,
				"command Type: %d,Processed: %d,Total: %d",
				cmn_hdr->cmd_type, processed_cmd_buf_in_bytes,
				total_cmd_buf_in_bytes);

			switch (cmn_hdr->cmd_type) {
			case CAMERA_SENSOR_IRCUT_CMD_TYPE_ON:
			case CAMERA_SENSOR_IRCUT_CMD_TYPE_OFF:
				CAM_DBG(CAM_IR_LED,
					"Received ircut settings");
				cmd_length_in_bytes =
					total_cmd_buf_in_bytes;
				/* Reuse power implementions to control gpio for ircut */
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					&ictrl->per_frame[0].ircut_info, remain_len);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed update power settings");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;

			default:
				/* add support for handling i2c_data*/
				i2c_reg_settings = &ictrl->i2c_data.config_settings;
				if (i2c_reg_settings->is_settings_valid == true) {
					i2c_reg_settings->request_id = 0;
					i2c_reg_settings->is_settings_valid = false;

					rc = delete_request(i2c_reg_settings);
					if (rc) {
						CAM_ERR(CAM_IR_LED,
						"Failed in Deleting the err: %d", rc);
						cam_mem_put_cpu_buf(config.packet_handle);
						return rc;
					}
				}

				i2c_reg_settings->is_settings_valid = true;
				i2c_reg_settings->request_id =
					csl_packet->header.request_id;
				rc = cam_sensor_i2c_command_parser(
					&ictrl->io_master_info,
					i2c_reg_settings, cmd_desc, 1, NULL);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed in parsing i2c NRT packets");
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			}
		}

		if ((ictrl->per_frame[0].ircut_info.power_setting_size != 0) ||
			(ictrl->per_frame[0].ircut_info.power_down_setting_size != 0)) {
			if (ictrl->func_tbl->ircut_ops != NULL) {
				rc = ictrl->func_tbl->ircut_ops(ictrl, 0);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
						"Update IRCUT Failed rc = %d", rc);
				}
			}
		}

		if (ictrl->func_tbl->apply_setting != NULL) {
			rc = ictrl->func_tbl->apply_setting(ictrl, 0);
			if (rc) {
				CAM_ERR(CAM_IR_LED, "cannot apply settings rc = %d", rc);
				cam_mem_put_cpu_buf(config.packet_handle);
				return rc;
			}
		}
		break;
	case CAM_IR_CUT_PACKET_OPCODE_ON:
		if (ictrl->func_tbl->camera_ir_cut_on != NULL) {
			rc = ictrl->func_tbl->camera_ir_cut_on(
					ictrl, cam_ir_led_info);
			if (rc < 0) {
				CAM_ERR(CAM_IR_LED,
					"Fail to turn ircut ON rc=%d", rc);
				return rc;
			}
		}
		break;
	case CAM_IR_CUT_PACKET_OPCODE_OFF:
		if (ictrl->func_tbl->camera_ir_cut_off != NULL) {
			rc = ictrl->func_tbl->camera_ir_cut_off(ictrl);
			if (rc < 0) {
				CAM_ERR(CAM_IR_LED,
					"Fail to turn ircut OFF rc=%d", rc);
				return rc;
			}
		}
		break;
	case CAM_IR_LED_PACKET_OPCODE_STREAM_OFF:
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc)
				return rc;

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			processed_cmd_buf_in_bytes = 0;
			if (!total_cmd_buf_in_bytes)
				continue;
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buffer);
			if (rc < 0) {
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "Failed to get cpu buf");
				return rc;
			}

			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_IR_LED, "invalid cmd buf");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return -EINVAL;
			}

			if ((len_of_buffer < sizeof(struct common_header)) ||
				(cmd_desc[i].offset >
				(len_of_buffer -
				sizeof(struct common_header)))) {
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				CAM_ERR(CAM_IR_LED, "invalid cmd buf length");
				return -EINVAL;
			}
			remain_len = len_of_buffer - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmn_hdr = (struct common_header *)cmd_buf;

			/* Loop through cmd formats in one cmd buffer */
			CAM_DBG(CAM_IR_LED,
				"command Type: %d,Processed: %d,Total: %d",
				cmn_hdr->cmd_type, processed_cmd_buf_in_bytes,
				total_cmd_buf_in_bytes);

			switch (cmn_hdr->cmd_type) {
			case CAMERA_SENSOR_IRCUT_CMD_TYPE_ON:
			case CAMERA_SENSOR_IRCUT_CMD_TYPE_OFF:
				CAM_DBG(CAM_IR_LED,
					"Received ircut settings");
				cmd_length_in_bytes =
					total_cmd_buf_in_bytes;
				/* Reuse power implementions to control gpio for ircut */
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					&(ictrl->per_frame[0].ircut_info), remain_len);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed update power settings");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;

			default:
				/* add support for handling i2c_data*/
				i2c_reg_settings = &ictrl->i2c_data.streamoff_settings;
				if (i2c_reg_settings->is_settings_valid == true) {
					i2c_reg_settings->request_id = 0;
					i2c_reg_settings->is_settings_valid = false;

					rc = delete_request(i2c_reg_settings);
					if (rc) {
						CAM_ERR(CAM_IR_LED,
						"Failed in Deleting the err: %d", rc);
						cam_mem_put_cpu_buf(config.packet_handle);
						return rc;
					}
				}

				i2c_reg_settings->is_settings_valid = true;
				i2c_reg_settings->request_id =
					csl_packet->header.request_id;
				rc = cam_sensor_i2c_command_parser(
					&ictrl->io_master_info,
					i2c_reg_settings, cmd_desc, 1, NULL);
				if (rc) {
					CAM_ERR(CAM_IR_LED,
					"Failed in parsing i2c NRT packets");
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			}
		}
	break;
	case CAM_PKT_NOP_OPCODE:
		frm_offset = csl_packet->header.request_id %
			MAX_PER_FRAME_ARRAY;
		if ((ictrl->ir_led_state == CAM_IR_LED_STATE_INIT) ||
			(ictrl->ir_led_state == CAM_IR_LED_STATE_ACQUIRE)) {
			CAM_WARN(CAM_IR_LED,
				"Rxed NOP packets without linking");
			ictrl->i2c_data.per_frame[frm_offset].is_settings_valid
				= false;
			cam_mem_put_cpu_buf(config.packet_handle);
			return 0;
		}
		i2c_reg_settings =
			&ictrl->i2c_data.per_frame[frm_offset];
		i2c_reg_settings->is_settings_valid = true;
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		CAM_DBG(CAM_IR_LED, "NOP Packet is Received: req_id: %u",
			csl_packet->header.request_id);
		goto update_req_mgr;
	default:
		CAM_ERR(CAM_IR_LED, "Invalid Opcode : %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		return -EINVAL;
	}
update_req_mgr:
	if (((csl_packet->header.op_code  & 0xFFFFF) ==
		CAM_PKT_NOP_OPCODE) ||
		((csl_packet->header.op_code & 0xFFFFF) ==
		CAM_IR_LED_PACKET_OPCODE_SET_OPS)) {
		memset(&add_req, 0, sizeof(add_req));
		add_req.link_hdl = ictrl->bridge_intf.link_hdl;
		add_req.req_id = csl_packet->header.request_id;
		add_req.dev_hdl = ictrl->bridge_intf.device_hdl;

		if ((csl_packet->header.op_code & 0xFFFFF) ==
			CAM_IR_LED_PACKET_OPCODE_SET_OPS) {
			add_req.trigger_eof = true;
			add_req.skip_at_sof = 1;
		}

		if (ictrl->bridge_intf.crm_cb &&
			ictrl->bridge_intf.crm_cb->add_req) {
			rc = ictrl->bridge_intf.crm_cb->add_req(&add_req);
			if  (rc) {
				CAM_ERR(CAM_IR_LED,
					"Failed in adding request: %llu to request manager",
					csl_packet->header.request_id);
				cam_mem_put_cpu_buf(config.packet_handle);
				return rc;
			}
			CAM_DBG(CAM_IR_LED,
				"add req %lld to req_mgr, trigger_eof %d",
				add_req.req_id, add_req.trigger_eof);
		}
	}
	cam_mem_put_cpu_buf(config.packet_handle);

	return rc;
}

static int32_t cam_ir_led_driver_cmd(struct cam_ir_led_ctrl *ictrl,
		void *arg, struct cam_ir_led_private_soc *soc_private)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_sensor_acquire_dev ir_led_acq_dev;
	struct cam_create_dev_hdl dev_hdl;
	struct cam_ir_led_query_cap_info ir_led_cap = {0};

	if (!ictrl || !arg) {
		CAM_ERR(CAM_IR_LED, "ictrl/arg is NULL with arg:%pK ictrl%pK",
			ictrl, arg);
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_IR_LED, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	mutex_lock(&(ictrl->ir_led_mutex));
	CAM_DBG(CAM_IR_LED, "cmd->op_code %d", cmd->op_code);
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV:
		if (ictrl->ir_led_state != CAM_IR_LED_STATE_INIT) {
			CAM_ERR(CAM_IR_LED,
				"Cannot apply Acquire dev: Prev state: %d",
				ictrl->ir_led_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		rc = copy_from_user(&ir_led_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(ir_led_acq_dev));
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed Copy from User rc=%d", rc);
			goto release_mutex;
		}

		dev_hdl.session_hdl = ir_led_acq_dev.session_handle;
		dev_hdl.v4l2_sub_dev_flag = 0;
		dev_hdl.media_entity_flag = 0;
		dev_hdl.dev_id = CAM_IR_LED;
		dev_hdl.ops = &ictrl->bridge_intf.ops;
		dev_hdl.priv = ictrl;

		ir_led_acq_dev.device_handle =
			cam_create_device_hdl(&dev_hdl);
		ictrl->bridge_intf.device_hdl =
			ir_led_acq_dev.device_handle;
		ictrl->bridge_intf.session_hdl =
			ir_led_acq_dev.session_handle;
		ictrl->device_hdl =
			ir_led_acq_dev.device_handle;

		rc = copy_to_user(u64_to_user_ptr(cmd->handle), &ir_led_acq_dev,
			sizeof(struct cam_sensor_acquire_dev));
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed Copy to User rc=%d", rc);
			rc = -EFAULT;
			goto release_mutex;
		}

		ictrl->ir_led_state = CAM_IR_LED_STATE_ACQUIRE;
		break;
	case CAM_RELEASE_DEV:
		if ((ictrl->ir_led_state == CAM_IR_LED_STATE_INIT) ||
			(ictrl->ir_led_state == CAM_IR_LED_STATE_START)) {
			CAM_WARN(CAM_IR_LED,
				" Cannot apply Release dev: Prev state:%d",
				ictrl->ir_led_state);
		}

		if (ictrl->device_hdl == -1 &&
			ictrl->ir_led_state == CAM_IR_LED_STATE_ACQUIRE) {
			CAM_ERR(CAM_IR_LED,
				" Invalid Handle: device hdl: %d",
				ictrl->device_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		if ((ictrl->ir_led_state == CAM_IR_LED_STATE_CONFIG) ||
			(ictrl->ir_led_state == CAM_IR_LED_STATE_START)) {
			if (ictrl->func_tbl->flush_req != NULL)
				ictrl->func_tbl->flush_req(ictrl, IR_FLUSH_ALL, 0);
		}
		rc = cam_ir_led_release_dev(ictrl);
		if (rc)
			CAM_ERR(CAM_IR_LED,
				" Failed in destroying the device Handle rc= %d",
				rc);

		if (ictrl->func_tbl->power_ops != NULL) {
			rc = ictrl->func_tbl->power_ops(ictrl, false);
			if (rc)
				CAM_WARN(CAM_IR_LED, "Power Down Failed");
		}

		ictrl->ir_led_state = CAM_IR_LED_STATE_INIT;
		break;
	case CAM_QUERY_CAP:
		ir_led_cap.slot_info = ictrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle), &ir_led_cap,
			sizeof(struct cam_ir_led_query_cap_info))) {
			CAM_ERR(CAM_IR_LED, " Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	case CAM_START_DEV:
		if ((ictrl->ir_led_state == CAM_IR_LED_STATE_INIT) ||
			(ictrl->ir_led_state == CAM_IR_LED_STATE_START)) {
			CAM_ERR(CAM_IR_LED,
				"Cannot apply Start Dev: Prev state: %d",
				ictrl->ir_led_state);
			rc = -EINVAL;
			goto release_mutex;
		}
		ictrl->ir_led_state = CAM_IR_LED_STATE_START;
		break;
	case CAM_STOP_DEV:
		if (ictrl->ir_led_state != CAM_IR_LED_STATE_START) {
			CAM_ERR(CAM_IR_LED,
				"Cannot apply Stop Dev: Prev state: %d",
				ictrl->ir_led_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		rc = cam_ir_led_stop_dev(ictrl);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed STOP_DEV: rc=%d", rc);
			goto release_mutex;
		}
		if (ictrl->func_tbl->flush_req != NULL)
			ictrl->func_tbl->flush_req(ictrl, IR_FLUSH_ALL, 0);

		ictrl->last_flush_req = 0;
		ictrl->ir_led_state = CAM_IR_LED_STATE_ACQUIRE;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ir_led_config(ictrl, arg);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed CONFIG_DEV: rc=%d", rc);
			goto release_mutex;
		}
		break;
	case CAM_FLUSH_REQ:
		rc = cam_ir_led_stop_dev(ictrl);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed FLUSH_REQ: rc=%d", rc);
			goto release_mutex;
		}
		ictrl->ir_led_state = CAM_IR_LED_STATE_ACQUIRE;
		break;
	default:
		CAM_ERR(CAM_IR_LED, "Invalid Opcode:%d", cmd->op_code);
		rc = -EINVAL;
	}

release_mutex:
	mutex_unlock(&(ictrl->ir_led_mutex));
	return rc;
}

static const struct of_device_id cam_ir_led_dt_match[] = {
	{.compatible = "qcom,camera-ir-led", .data = NULL},
	{}
};

static long cam_ir_led_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_ir_led_ctrl *ictrl = NULL;
	struct cam_ir_led_private_soc *soc_private = NULL;

	CAM_DBG(CAM_IR_LED, "Enter");

	ictrl = v4l2_get_subdevdata(sd);
	soc_private = ictrl->soc_info.soc_private;

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ir_led_driver_cmd(ictrl, arg,
			soc_private);
		break;
	default:
		CAM_ERR(CAM_IR_LED, " Invalid ioctl cmd type %u", cmd);
		rc = -EINVAL;
		break;
	}

	CAM_DBG(CAM_IR_LED, "Exit");
	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_ir_led_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_IR_LED,
			" Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ir_led_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc)
			CAM_ERR(CAM_IR_LED, "cam_ir_led_ioctl failed");
		break;
	default:
		CAM_ERR(CAM_IR_LED, " Invalid compat ioctl cmd_type:%d",
			cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_IR_LED,
				" Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}
#endif

static int cam_ir_led_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_ir_led_ctrl *ictrl =
		v4l2_get_subdevdata(sd);

	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, " Ir_led ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&ictrl->ir_led_mutex);
	cam_ir_led_shutdown(ictrl);
	mutex_unlock(&ictrl->ir_led_mutex);

	return 0;
}

static struct v4l2_subdev_core_ops cam_ir_led_subdev_core_ops = {
	.ioctl = cam_ir_led_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_ir_led_subdev_do_ioctl
#endif
};

static struct v4l2_subdev_ops cam_ir_led_subdev_ops = {
	.core = &cam_ir_led_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_ir_led_internal_ops = {
	.close = cam_ir_led_subdev_close,
};

static int cam_ir_led_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = 0, i = 0;
	struct cam_ir_led_ctrl *ictrl = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	CAM_DBG(CAM_IR_LED, "Enter");
	if (!pdev->dev.of_node) {
		CAM_ERR(CAM_IR_LED, "of_node NULL");
		return -EINVAL;
	}

	ictrl = kzalloc(sizeof(struct cam_ir_led_ctrl), GFP_KERNEL);
	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "kzalloc failed!!");
		return -ENOMEM;
	}

	ictrl->pdev = pdev;
	ictrl->soc_info.pdev = pdev;
	ictrl->soc_info.dev = &pdev->dev;
	ictrl->soc_info.dev_name = pdev->name;

	rc = cam_ir_led_get_dt_data(ictrl, &ictrl->soc_info);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "cam_ir_led_get_dt_data failed rc=%d", rc);
		if (ictrl->soc_info.soc_private != NULL) {
			kfree(ictrl->soc_info.soc_private);
			ictrl->soc_info.soc_private = NULL;
		}
		kfree(ictrl);
		ictrl = NULL;
		return -EINVAL;
	}

	ictrl->v4l2_dev_str.internal_ops =
		&cam_ir_led_internal_ops;
	ictrl->v4l2_dev_str.ops = &cam_ir_led_subdev_ops;
	ictrl->v4l2_dev_str.name = CAMX_IR_LED_DEV_NAME;
	ictrl->v4l2_dev_str.sd_flags =
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	ictrl->v4l2_dev_str.ent_function = CAM_IRLED_DEVICE_TYPE;
	ictrl->v4l2_dev_str.token = ictrl;

	rc = cam_register_subdev(&(ictrl->v4l2_dev_str));
	if (rc) {
		CAM_ERR(CAM_IR_LED, "Fail to create subdev with %d", rc);
		kfree(ictrl);
		return rc;
	}

	INIT_LIST_HEAD(&(ictrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(ictrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(ictrl->i2c_data.streamoff_settings.list_head));

	ictrl->i2c_data.per_frame =
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (ictrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		return rc;
	}
	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(ictrl->i2c_data.per_frame[i].list_head));

	ictrl->device_hdl = -1;
	ictrl->bridge_intf.link_hdl = -1;

	ictrl->bridge_intf.ops.get_dev_info =
		cam_ir_led_publish_dev_info;
	ictrl->bridge_intf.ops.link_setup =
		cam_ir_led_establish_link;
	ictrl->bridge_intf.ops.apply_req =
		cam_ir_led_apply_request;
	ictrl->bridge_intf.ops.flush_req =
		cam_ir_led_flush_request;
	ictrl->last_flush_req = 0;

	platform_set_drvdata(pdev, ictrl);
	v4l2_set_subdevdata(&ictrl->v4l2_dev_str.sd, ictrl);
	mutex_init(&(ictrl->ir_led_mutex));
	ictrl->ir_led_state = CAM_IR_LED_STATE_INIT;
	ictrl->is_ircut_gpio_requested = false;
	CAM_DBG(CAM_IR_LED, "%s component bound successfully", pdev->name);
	return rc;
}

static void cam_ir_led_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_ir_led_ctrl *ictrl;
	struct platform_device *pdev = to_platform_device(dev);

	ictrl = platform_get_drvdata(pdev);
	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, " Ir_led device is NULL");
		return;
	}
	kfree(ictrl->i2c_data.per_frame);
	ictrl->i2c_data.per_frame = NULL;

	kfree(ictrl);
}

const static struct component_ops cam_ir_led_component_ops = {
	.bind = cam_ir_led_component_bind,
	.unbind = cam_ir_led_component_unbind,
};

static int cam_ir_led_platform_probe(struct platform_device *pdev)
{
	int rc = 0;

	rc = component_add(&pdev->dev, &cam_ir_led_component_ops);
	if (rc)
		CAM_ERR(CAM_IR_LED, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_ir_led_platform_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_ir_led_component_ops);
	return 0;
}

static struct cam_ir_led_table cam_pmic_ir_led_table = {
	.ir_led_driver_type = IR_LED_DRIVER_PMIC,
	.func_tbl = {
		.camera_ir_led_init = &cam_pmic_ir_led_init,
		.camera_ir_led_release = &cam_pmic_ir_led_release,
		.camera_ir_led_off = &cam_pmic_ir_led_off,
		.camera_ir_led_on = &cam_pmic_ir_led_on,
		.camera_ir_cut_off = &cam_pmic_ir_cut_off,
		.camera_ir_cut_on = &cam_pmic_ir_cut_on,
	},
};

static struct cam_ir_led_table cam_gpio_ir_led_table = {
	.ir_led_driver_type = IR_LED_DRIVER_GPIO,
	.func_tbl = {
		.camera_ir_led_init = NULL,
		.camera_ir_led_release = NULL,
		.camera_ir_led_off = NULL,
		.camera_ir_led_on = NULL,
		.camera_ir_cut_off = NULL,
		.camera_ir_cut_on = NULL,
	},
};

static struct cam_ir_led_table cam_i2c_ir_led_table = {
	.ir_led_driver_type = IR_LED_DRIVER_I2C,
	.func_tbl = {
		.power_ops = &cam_i2c_ir_led_power_ops,
		.apply_setting = &cam_i2c_ir_led_apply_setting,
		.flush_req = &cam_i2c_ir_led_flush_request,
		.ircut_ops = &cam_i2c_ir_cut_ops,
	},
};

MODULE_DEVICE_TABLE(of, cam_ir_led_dt_match);

struct platform_driver cam_ir_led_platform_driver = {
	.probe = cam_ir_led_platform_probe,
	.remove = cam_ir_led_platform_remove,
	.driver = {
		.name = "CAM-IR-LED-DRIVER",
		.owner = THIS_MODULE,
		.of_match_table = cam_ir_led_dt_match,
		.suppress_bind_attrs = true,
	},
};

int32_t cam_ir_led_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&cam_ir_led_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_IR_LED, "platform probe failed rc: %d", rc);
		return rc;
	}

	return rc;
}

void cam_ir_led_exit_module(void)
{
	platform_driver_unregister(&cam_ir_led_platform_driver);
}

MODULE_DESCRIPTION("CAM IR_LED");
MODULE_LICENSE("GPL v2");
