/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_lens_driver_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

static int32_t cam_lens_driver_power_down(
	struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&l_ctrl->soc_info;
	struct cam_lens_driver_soc_private *soc_private;

	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "failed: l_ctrl %pK", l_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &l_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_LENS_DRIVER, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_LENS_DRIVER, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&l_ctrl->io_master_info);
	l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_ACQUIRE;

	return rc;
}

static int cam_lens_driver_power_up(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int                                     rc = 0;
	struct cam_hw_soc_info                 *soc_info = &l_ctrl->soc_info;
	struct cam_lens_driver_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t         *power_info;

	soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_ERR(CAM_LENS_DRIVER,
			"power_setting and power_down_setting are NULL");
		return -EINVAL;
	}

	if (l_ctrl->isAlwaysPowerOn == 0) {
		/* Parse and fill vreg params for power up settings */
		rc = msm_camera_fill_vreg_params(
			soc_info,
			power_info->power_setting,
			power_info->power_setting_size);
		if (rc) {
			CAM_ERR(CAM_LENS_DRIVER,
				"failed to fill vreg params for power up rc:%d", rc);
			return rc;
		}

		/* Parse and fill vreg params for power down settings*/
		rc = msm_camera_fill_vreg_params(
			soc_info,
			power_info->power_down_setting,
			power_info->power_down_setting_size);
		if (rc) {
			CAM_ERR(CAM_LENS_DRIVER,
			"failed to fill vreg params for power down rc:%d", rc);
			return rc;
		}
	}
	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_LENS_DRIVER, "failed in ldm power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&l_ctrl->io_master_info);
	if (rc) {
		CAM_ERR(CAM_LENS_DRIVER, "cci_init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_LENS_DRIVER, "Power Down failed");

	return rc;
}

static struct i2c_settings_list*
	cam_ldm_get_i2c_ptr(struct i2c_settings_array *i2c_reg_settings,
		uint32_t size)
{
	struct i2c_settings_list *tmp;

	tmp = kzalloc(sizeof(struct i2c_settings_list), GFP_KERNEL);

	if (tmp != NULL)
		list_add_tail(&(tmp->list),
			&(i2c_reg_settings->list_head));
	else
		return NULL;

	tmp->i2c_settings.reg_setting = (struct cam_sensor_i2c_reg_array *)
		vzalloc(size * sizeof(struct cam_sensor_i2c_reg_array));
	if (tmp->i2c_settings.reg_setting == NULL) {
		list_del(&(tmp->list));
		kfree(tmp);
		return NULL;
	}
	tmp->i2c_settings.size = size;

	return tmp;
}

static int32_t cam_ldm_get_io_buffer(
	struct cam_buf_io_cfg *io_cfg,
	struct cam_sensor_i2c_reg_setting *i2c_settings)
{
	uintptr_t buf_addr = 0x0;
	size_t buf_size = 0;
	int32_t rc = 0;

	if (io_cfg == NULL || i2c_settings == NULL) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Invalid args, io buf or i2c settings is NULL");
		return -EINVAL;
	}

	if (io_cfg->direction == CAM_BUF_OUTPUT) {
		rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
			&buf_addr, &buf_size);
		if ((rc < 0) || (!buf_addr)) {
			CAM_ERR(CAM_LENS_DRIVER,
				"invalid buffer, rc: %d, buf_addr: %pK",
				rc, buf_addr);
			return -EINVAL;
		}
		CAM_DBG(CAM_LENS_DRIVER,
			"buf_addr: %pK, buf_size: %zu, offsetsize: %d",
			(void *)buf_addr, buf_size, io_cfg->offsets[0]);
		if (io_cfg->offsets[0] >= buf_size) {
			CAM_ERR(CAM_LENS_DRIVER,
				"invalid size:io_cfg->offsets[0]: %d, buf_size: %d",
				io_cfg->offsets[0], buf_size);
			cam_mem_put_cpu_buf(io_cfg->mem_handle[0]);
			return -EINVAL;
		}
		i2c_settings->read_buff =
			 (uint8_t *)buf_addr + io_cfg->offsets[0];
		i2c_settings->read_buff_len =
			buf_size - io_cfg->offsets[0];

		CAM_DBG(CAM_LENS_DRIVER, "read_buff_len: %d", i2c_settings->read_buff_len);
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid direction: %d",
			io_cfg->direction);
		rc = -EINVAL;
	}
	cam_mem_put_cpu_buf(io_cfg->mem_handle[0]);
	return rc;
}

int32_t cam_ldm_handle_continuous_read(
	struct cam_cmd_i2c_continuous_rd *cmd_i2c_continuous_rd,
	struct i2c_settings_array *i2c_reg_settings,
	uint16_t *cmd_length_in_bytes, int32_t *offset,
	struct list_head **list,
	struct cam_buf_io_cfg *io_cfg)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0, cnt = 0, payload_count;

	payload_count = cmd_i2c_continuous_rd->header.count;
	i2c_list = cam_ldm_get_i2c_ptr(i2c_reg_settings, payload_count);
	if ((i2c_list == NULL) ||
		(i2c_list->i2c_settings.reg_setting == NULL)) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Failed in allocating i2c_list: %pK",
			i2c_list);
		return -ENOMEM;
	}

	rc = cam_ldm_get_io_buffer(io_cfg, &(i2c_list->i2c_settings));
	if (rc) {
		CAM_ERR(CAM_LENS_DRIVER, "Failed to get read buffer: %d", rc);
	} else {
		*cmd_length_in_bytes = (sizeof(struct i2c_rdwr_header) +
			sizeof(cmd_i2c_continuous_rd->reg_addr) +
			sizeof(cmd_i2c_continuous_rd->num_bytes) +
			sizeof(struct cam_cmd_read) * (payload_count));
		i2c_list->op_code = CAM_SENSOR_I2C_READ_SEQ;
		i2c_list->i2c_settings.addr_type =
			cmd_i2c_continuous_rd->header.addr_type;
		i2c_list->i2c_settings.data_type =
			cmd_i2c_continuous_rd->header.data_type;
		i2c_list->i2c_settings.size =
			cmd_i2c_continuous_rd->header.count;
		i2c_list->i2c_settings.read_bytes =
			cmd_i2c_continuous_rd->num_bytes;

		for (cnt = 0; cnt < (cmd_i2c_continuous_rd->header.count);
			cnt++) {
			i2c_list->i2c_settings.reg_setting[cnt].reg_addr =
				cmd_i2c_continuous_rd->reg_addr;
			i2c_list->i2c_settings.reg_setting[cnt].reg_data =
				cmd_i2c_continuous_rd->data_read[cnt].reg_data;
		}

		*offset = cnt;
		*list = &(i2c_list->list);
	}

	return rc;
}

static int cam_ldm_command_parser(
	struct camera_io_master *io_master,
	struct i2c_settings_array *i2c_reg_settings,
	struct cam_cmd_buf_desc *cmd_desc,
	int32_t num_cmd_buffers,
	struct cam_buf_io_cfg *io_cfg)
{
	int16_t                   rc = 0, i = 0;
	size_t                    len_of_buff = 0;
	uintptr_t                 generic_ptr;
	size_t                    remain_len = 0;
	size_t                    tot_size = 0;

	for (i = 0; i < num_cmd_buffers; i++) {
		uint32_t                  *cmd_buf = NULL;
		struct common_header      *cmm_hdr;
		uint16_t                  generic_op_code;
		uint32_t                  byte_cnt = 0;
		uint32_t                  j = 0;
		struct list_head          *list = NULL;
		uint32_t                  payload_count = 0;

		/*
		 * It is not expected the same settings to
		 * be spread across multiple cmd buffers
		 */
		CAM_DBG(CAM_LENS_DRIVER, "Total cmd Buf in Bytes: %d",
			cmd_desc[i].length);

		if (!cmd_desc[i].length)
			continue;

		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER,
				"cmd hdl failed:%d, Err: %d, Buffer_len: %zd",
				cmd_desc[i].mem_handle, rc, len_of_buff);
			return rc;
		}

		remain_len = len_of_buff;
		if ((len_of_buff < sizeof(struct common_header)) ||
			(cmd_desc[i].offset >
			(len_of_buff - sizeof(struct common_header)))) {
			CAM_ERR(CAM_LENS_DRIVER, "buffer provided too small");
			rc = -EINVAL;
			goto end;
		}
		cmd_buf = (uint32_t *)generic_ptr;
		cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);

		remain_len -= cmd_desc[i].offset;
		CAM_DBG(CAM_LENS_DRIVER, "remain_len %d cmd_desc_length %d",
			remain_len, cmd_desc[i].length);
		if (remain_len < cmd_desc[i].length) {
			CAM_ERR(CAM_LENS_DRIVER, "buffer provided too small");
			rc = -EINVAL;
			goto end;
		}

		while (byte_cnt < cmd_desc[i].length) {
			if ((remain_len - byte_cnt) <
				sizeof(struct common_header)) {
				CAM_ERR(CAM_LENS_DRIVER, "Not enough buffer");
				rc = -EINVAL;
				goto end;
			}
			cmm_hdr = (struct common_header *)cmd_buf;
			generic_op_code = cmm_hdr->fifth_byte;
			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR: {
				uint32_t cmd_length_in_bytes = 0;
				struct cam_cmd_i2c_random_wr
					*cam_cmd_i2c_random_wr =
					(struct cam_cmd_i2c_random_wr *)cmd_buf;
				payload_count = cam_cmd_i2c_random_wr->header.count;

				if ((remain_len - byte_cnt) <
					sizeof(struct cam_cmd_i2c_random_wr)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided");
					rc = -EINVAL;
					goto end;
				}
				tot_size = sizeof(struct i2c_rdwr_header) +
					(sizeof(struct i2c_random_wr_payload) *
					payload_count);

				if (tot_size > (remain_len - byte_cnt)) {
					CAM_ERR(CAM_SENSOR,
						"Not enough buffer provided");
					rc = -EINVAL;
					goto end;
				}

				rc = cam_sensor_handle_random_write(
					cam_cmd_i2c_random_wr,
					i2c_reg_settings,
					&cmd_length_in_bytes, &j, &list, payload_count);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed in random write %d", rc);
					rc = -EINVAL;
					goto end;
				}

				cmd_buf += cmd_length_in_bytes /
					sizeof(uint32_t);
				byte_cnt += cmd_length_in_bytes;
				break;
			}
			case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_WR: {
				uint32_t cmd_length_in_bytes   = 0;
				struct cam_cmd_i2c_continuous_wr
				*cam_cmd_i2c_continuous_wr =
				(struct cam_cmd_i2c_continuous_wr *)
				cmd_buf;
				payload_count = cam_cmd_i2c_continuous_wr->header.count;

				if ((remain_len - byte_cnt) <
				sizeof(struct cam_cmd_i2c_continuous_wr)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided");
					rc = -EINVAL;
					goto end;
				}

				tot_size = sizeof(struct i2c_rdwr_header) +
				sizeof(cam_cmd_i2c_continuous_wr->reg_addr) +
				(sizeof(struct cam_cmd_read) *
				payload_count);

				if (tot_size > (remain_len - byte_cnt)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided");
					rc = -EINVAL;
					goto end;
				}

				rc = cam_sensor_handle_continuous_write(
					cam_cmd_i2c_continuous_wr,
					i2c_reg_settings,
					&cmd_length_in_bytes, &j, &list, payload_count);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed in continuous write %d", rc);
					goto end;
				}

				cmd_buf += cmd_length_in_bytes;
				byte_cnt += cmd_length_in_bytes;
				break;
			}
			case CAMERA_SENSOR_CMD_TYPE_WAIT: {
				if ((remain_len - byte_cnt) <
				sizeof(struct cam_cmd_unconditional_wait)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer space");
					rc = -EINVAL;
					goto end;
				}
				if (generic_op_code ==
					CAMERA_SENSOR_WAIT_OP_HW_UCND ||
					generic_op_code ==
						CAMERA_SENSOR_WAIT_OP_SW_UCND) {
					rc = cam_sensor_handle_delay(
						&cmd_buf, generic_op_code,
						i2c_reg_settings, j, &byte_cnt,
						list);
					if (rc < 0) {
						CAM_ERR(CAM_LENS_DRIVER,
							"delay hdl failed: %d",
							rc);
						goto end;
					}

				} else if (generic_op_code ==
					CAMERA_SENSOR_WAIT_OP_COND) {
					rc = cam_sensor_handle_poll(
						&cmd_buf, i2c_reg_settings,
						&byte_cnt, &j, &list);
					if (rc < 0) {
						CAM_ERR(CAM_LENS_DRIVER,
							"Random read fail: %d",
							rc);
						goto end;
					}
				} else {
					CAM_ERR(CAM_LENS_DRIVER,
						"Wrong Wait Command: %d",
						generic_op_code);
					rc = -EINVAL;
					goto end;
				}
				break;
			}
			case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_RD: {
				uint16_t cmd_length_in_bytes   = 0;
				struct cam_cmd_i2c_random_rd *i2c_random_rd =
				(struct cam_cmd_i2c_random_rd *)cmd_buf;
				payload_count = i2c_random_rd->header.count;

				if (remain_len - byte_cnt <
					sizeof(struct cam_cmd_i2c_random_rd)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer space");
					rc = -EINVAL;
					goto end;
				}

				tot_size = sizeof(struct i2c_rdwr_header) +
					(sizeof(struct cam_cmd_read) *
					payload_count);

				if (tot_size > (remain_len - byte_cnt)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided %d, %d, %d",
						tot_size, remain_len, byte_cnt);
					rc = -EINVAL;
					goto end;
				}

				rc = cam_sensor_handle_random_read(
					i2c_random_rd,
					i2c_reg_settings,
					&cmd_length_in_bytes, &j, &list,
					io_cfg, payload_count);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed in random read %d", rc);
					goto end;
				}

				cmd_buf += cmd_length_in_bytes /
					sizeof(uint32_t);
				byte_cnt += cmd_length_in_bytes;
				break;
			}
			case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD: {
				uint16_t cmd_length_in_bytes   = 0;
				struct cam_cmd_i2c_continuous_rd
				*i2c_continuous_rd =
				(struct cam_cmd_i2c_continuous_rd *)cmd_buf;
				if (remain_len - byte_cnt <
				    sizeof(struct cam_cmd_i2c_continuous_rd)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer space");
					rc = -EINVAL;
					goto end;
				}

				tot_size =
				sizeof(struct cam_cmd_i2c_continuous_rd);

				if (tot_size > (remain_len - byte_cnt)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided %d, %d, %d",
						tot_size, remain_len, byte_cnt);
					rc = -EINVAL;
					goto end;
				}

				rc = cam_ldm_handle_continuous_read(
					i2c_continuous_rd,
					i2c_reg_settings,
					&cmd_length_in_bytes, &j, &list,
					io_cfg);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed in continuous read %d", rc);
					goto end;
				}

				cmd_buf += cmd_length_in_bytes /
					sizeof(uint32_t);
				byte_cnt += cmd_length_in_bytes;
				break;
			}
			default:
				CAM_ERR(CAM_LENS_DRIVER, "Invalid Command Type:%d",
					 cmm_hdr->cmd_type);
				rc = -EINVAL;
				goto end;
			}
		}
		i2c_reg_settings->is_settings_valid = 1;
		cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
	}
	return rc;

end:
	cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
	return rc;
}

static int cam_ldm_slaveInfo_pkt_parser(struct cam_lens_driver_ctrl_t *l_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_lens_driver_info *ldm_info;

	if (!l_ctrl || !cmd_buf ||
		len < sizeof(struct cam_cmd_lens_driver_info)) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	ldm_info = (struct cam_cmd_lens_driver_info *)cmd_buf;
	if (l_ctrl->io_master_info.master_type == CCI_MASTER) {
		l_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ldm_info->i2c_freq_mode;
		l_ctrl->io_master_info.cci_client->sid =
			ldm_info->slave_addr >> 1;
		l_ctrl->ldm_fw_flag = ldm_info->ldm_fw_flag;
		l_ctrl->isAlwaysPowerOn = ldm_info->is_always_power_on;
		l_ctrl->is_ldm_calib = ldm_info->is_ldm_calib;
		l_ctrl->io_master_info.cci_client->retries = 3;
		l_ctrl->io_master_info.cci_client->id_map = 0;
		CAM_DBG(CAM_LENS_DRIVER, "Slave addr: 0x%x Freq Mode: %d",
			ldm_info->slave_addr, ldm_info->i2c_freq_mode);
	} else if (l_ctrl->io_master_info.master_type == I2C_MASTER) {
		l_ctrl->io_master_info.client->addr = ldm_info->slave_addr;
		l_ctrl->ldm_fw_flag = ldm_info->ldm_fw_flag;
		l_ctrl->is_ldm_calib = ldm_info->is_ldm_calib;
		l_ctrl->isAlwaysPowerOn = ldm_info->is_always_power_on;
		CAM_DBG(CAM_LENS_DRIVER, "Slave addr: 0x%x", ldm_info->slave_addr);
	} else if (l_ctrl->io_master_info.master_type == SPI_MASTER) {
		l_ctrl->ldm_fw_flag = ldm_info->ldm_fw_flag;
		l_ctrl->is_ldm_calib = ldm_info->is_ldm_calib;
		l_ctrl->isAlwaysPowerOn = ldm_info->is_always_power_on;
		l_ctrl->io_master_info.spi_client->is_single_byte_txfr = ldm_info->is_single_byte_txfr;
		l_ctrl->io_master_info.spi_client->spi_mode = ldm_info->spi_mode;
		l_ctrl->io_master_info.spi_client->frequency = ldm_info->spi_freq;
		CAM_DBG(CAM_LENS_DRIVER, "spi mode: %d spi freq: %d",
			ldm_info->spi_mode, ldm_info->spi_freq);
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Master type : %d",
			l_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ldm_parse_fw_setting(uint8_t *cmd_buf, uint32_t size,
	struct i2c_settings_array *reg_settings)
{
	int32_t                 rc = 0;
	uint32_t                byte_cnt = 0;
	struct common_header   *cmm_hdr;
	uint16_t                op_code;
	uint32_t                j = 0;
	struct list_head       *list = NULL;
	uint32_t               payload_count = 0;

	while (byte_cnt < size) {
		if ((size - byte_cnt) < sizeof(struct common_header)) {
			CAM_ERR(CAM_LENS_DRIVER, "Not enough buffer");
			rc = -EINVAL;
			goto end;
		}
		cmm_hdr = (struct common_header *)cmd_buf;
		op_code = cmm_hdr->fifth_byte;
		CAM_DBG(CAM_LENS_DRIVER, "Command Type:%d, Op code:%d",
				cmm_hdr->cmd_type, op_code);
		CAM_DBG(CAM_LENS_DRIVER, "byte_cnt :%d",
				byte_cnt);

		switch (cmm_hdr->cmd_type) {
		case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR: {
			uint32_t cmd_length_in_bytes = 0;
			struct cam_cmd_i2c_random_wr
			*cam_cmd_i2c_random_wr =
			(struct cam_cmd_i2c_random_wr *)cmd_buf;
			payload_count = cam_cmd_i2c_random_wr->header.count;

			if ((size - byte_cnt) < sizeof(struct cam_cmd_i2c_random_wr)) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Not enough buffer provided,size %d,byte_cnt %d",
					size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}

			rc = cam_sensor_handle_random_write(
				cam_cmd_i2c_random_wr,
				reg_settings,
				&cmd_length_in_bytes, &j, &list, payload_count);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
				"Failed in random write %d", rc);
				goto end;
			}

			cmd_buf += cmd_length_in_bytes /
				sizeof(uint32_t);
			byte_cnt += cmd_length_in_bytes;

			break;
		}
		case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_WR: {
			uint32_t cmd_length_in_bytes = 0;
			struct cam_cmd_i2c_continuous_wr
			*cam_cmd_i2c_continuous_wr =
			(struct cam_cmd_i2c_continuous_wr *)cmd_buf;
			payload_count = cam_cmd_i2c_continuous_wr->header.count;

			if ((size - byte_cnt) <
				sizeof(struct cam_cmd_i2c_continuous_wr)) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Not enough buffer provided,size %d,byte_cnt %d",
					size, byte_cnt);
				rc = -EINVAL;
				goto end;
			}

			rc = cam_sensor_handle_continuous_write(
				cam_cmd_i2c_continuous_wr,
				reg_settings,
				&cmd_length_in_bytes, &j, &list, payload_count);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
				"Failed in continuous write %d", rc);
				goto end;
			}

			cmd_buf += cmd_length_in_bytes;
			byte_cnt += cmd_length_in_bytes;
			break;
		}
		case CAMERA_SENSOR_CMD_TYPE_WAIT: {
			if (op_code == CAMERA_SENSOR_WAIT_OP_HW_UCND ||
				op_code == CAMERA_SENSOR_WAIT_OP_SW_UCND) {
				if ((size - byte_cnt) <
					sizeof(struct cam_cmd_unconditional_wait)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided,size %d,byte_cnt %d",
						size, byte_cnt);
					rc = -EINVAL;
					goto end;
				}

				rc = cam_sensor_handle_delay(
					(uint32_t **)(&cmd_buf), op_code,
					reg_settings, j, &byte_cnt,
					list);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
						"delay hdl failed: %d",
						rc);
					goto end;
				}
			} else if (op_code == CAMERA_SENSOR_WAIT_OP_COND) {
				if ((size - byte_cnt) <
					sizeof(struct cam_cmd_conditional_wait)) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Not enough buffer provided,size %d,byte_cnt %d",
						size, byte_cnt);
					rc = -EINVAL;
					goto end;
				}
				rc = cam_sensor_handle_poll(
					(uint32_t **)(&cmd_buf), reg_settings,
					&byte_cnt, &j, &list);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
						"parsing POLL fail: %d",
						rc);
					goto end;
				}
			} else {
				CAM_ERR(CAM_LENS_DRIVER,
					"Wrong Wait Command: %d",
					op_code);
				rc = -EINVAL;
				goto end;
			}
			break;
		}
		default:
			CAM_ERR(CAM_LENS_DRIVER, "Invalid Command Type:%d",
				 cmm_hdr->cmd_type);
			rc = -EINVAL;
			goto end;
		}
	}

end:
	return rc;
}

static int cam_ldm_fw_info_pkt_parser(struct cam_lens_driver_ctrl_t *l_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t                         rc = 0;
	struct cam_cmd_ldm_fw_info     *ldm_fw_info;
	uint8_t                        *pSettingData = NULL;
	uint32_t                        size = 0;
	struct i2c_settings_array      *reg_settings = NULL;
	uint8_t                         count = 0;
	uint32_t                        idx;

	if (!l_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ldm_fw_info)) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args,l_ctrl %p,cmd_buf %p,len %d",
			l_ctrl, cmd_buf, len);
		return -EINVAL;
	}
	CAM_DBG(CAM_LENS_DRIVER, "Enter ldm fw info pkt parser");
	ldm_fw_info = (struct cam_cmd_ldm_fw_info *)cmd_buf;

	if (ldm_fw_info->fw_count <= MAX_LDM_FW_COUNT) {
		memcpy(&l_ctrl->fw_info, ldm_fw_info, sizeof(struct cam_cmd_ldm_fw_info));
		pSettingData = (uint8_t *)cmd_buf + sizeof(struct cam_cmd_ldm_fw_info);

		for (count = 0; count < ldm_fw_info->fw_count*2; count++) {
			idx = count / 2;
			/* init settings */
			if ((count & 0x1) == 0) {
				size = ldm_fw_info->fw_param[idx].fw_init_size;
				reg_settings = &l_ctrl->fw_init_data[idx];
				CAM_DBG(CAM_LENS_DRIVER, "init size %d", size);
			/* finalize settings */
			} else if ((count & 0x1) == 1) {
				size = ldm_fw_info->fw_param[idx].fw_finalize_size;
				reg_settings = &l_ctrl->fw_finalize_data[idx];
				CAM_DBG(CAM_LENS_DRIVER, "finalize size %d", size);
			} else {
				size = 0;
				CAM_DBG(CAM_LENS_DRIVER, "Unsupported case");
				return -EINVAL;
			}

			if (size != 0) {
				reg_settings->is_settings_valid = 1;
				rc = cam_ldm_parse_fw_setting(pSettingData, size, reg_settings);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
						"LDM fw pkt parsing failed: %d", rc);
					return rc;
				}
			}
			pSettingData += size;
		}
	} else {
		CAM_ERR(CAM_LENS_DRIVER, "Exceed max fw count");
	}

	return rc;
}

static int cam_ldm_apply_settings(struct cam_lens_driver_ctrl_t *l_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (l_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(l_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(
				&(l_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				CAM_SENSOR_I2C_WRITE_SEQ);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Failed to seq write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
			rc = camera_io_dev_write_continuous(
				&(l_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				CAM_SENSOR_I2C_WRITE_BURST);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Failed to burst write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(l_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
						"i2c poll apply setting Fail");
					return rc;
				} else if (rc ==  I2C_COMPARE_MISMATCH) {
					CAM_ERR(CAM_LENS_DRIVER, "i2c poll mismatch");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int write_ldm_fw(uint8_t *fw_data,
	struct cam_cmd_ldm_fw_param *fw_param,
	struct camera_io_master io_master_info,
	uint8_t i2c_operation)
{
	int32_t     rc = 0;
	uint8_t     *ptr = fw_data;
	int32_t     cnt = 0, wcnt = 0;
	void        *vaddr = NULL;
	uint8_t     data_type = fw_param->fw_data_type;
	uint16_t    len_per_write = fw_param->fw_len_per_write /
								fw_param->fw_data_type;
	struct cam_sensor_i2c_reg_setting setting;

	CAM_DBG(CAM_LENS_DRIVER, "write ldm fw: len_per_write:%d", len_per_write);
	vaddr = vmalloc((sizeof(struct cam_sensor_i2c_reg_array) * len_per_write));
	if (!vaddr) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Failed in allocating i2c_array: size: %u",
			(sizeof(struct cam_sensor_i2c_reg_array) * len_per_write));
		return -ENOMEM;
	}

	CAM_DBG(CAM_LENS_DRIVER, "fw_addr_type:%d", fw_param->fw_addr_type);
	CAM_DBG(CAM_LENS_DRIVER, "fw_data_type:%d", fw_param->fw_data_type);
	CAM_DBG(CAM_LENS_DRIVER, "isOnlyWritefwData:%d",
		fw_param->isOnlyWritefwData);

	setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (vaddr);
	setting.addr_type   = fw_param->fw_addr_type;
	setting.data_type   = fw_param->fw_data_type;
	setting.size        = len_per_write;
	setting.delay       = fw_param->fw_delayUs;
	setting.write_only_data = fw_param->isOnlyWritefwData;

	for (wcnt = 0; wcnt < (fw_param->fw_size/data_type);
		wcnt += len_per_write) {
		for (cnt = 0; cnt < len_per_write; cnt++, ptr += data_type) {
			setting.reg_setting[cnt].reg_addr =
				fw_param->fw_reg_addr + wcnt + cnt;
			switch (data_type) {
			case CAMERA_SENSOR_I2C_TYPE_BYTE:
				setting.reg_setting[cnt].reg_data = *((uint8_t *)ptr);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				setting.reg_setting[cnt].reg_data = *((uint16_t *)ptr);
				break;
			default:
				CAM_ERR(CAM_LENS_DRIVER,
					"Unsupported data type");
				rc = -EINVAL;
				goto End;
			}
			setting.reg_setting[cnt].delay = fw_param->fw_delayUs;
			setting.reg_setting[cnt].data_mask = 0;
		}

		if (i2c_operation == CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(io_master_info),
				&setting);
		} else if (i2c_operation == CAM_SENSOR_I2C_WRITE_BURST ||
			i2c_operation == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(&io_master_info,
				&setting, i2c_operation);
		}

		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Failed in Applying i2c wrt settings");
			break;
		}
	}

End:
	vfree(vaddr);
	vaddr = NULL;

	return rc;
}

int32_t cam_ldm_i2c_read_data(
	struct i2c_settings_array *i2c_settings,
	struct camera_io_master *io_master_info)
{
	int32_t                   rc = 0;
	struct i2c_settings_list  *i2c_list;
	uint8_t                   *read_buff = NULL;
	uint32_t                  buff_length = 0;
	uint32_t                  read_length = 0;

	list_for_each_entry(i2c_list,
		&(i2c_settings->list_head), list) {
		read_buff = i2c_list->i2c_settings.read_buff;
		buff_length = i2c_list->i2c_settings.read_buff_len;
		if ((read_buff == NULL) || (buff_length == 0)) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Invalid input buffer, buffer: %pK, length: %d",
				read_buff, buff_length);
			return -EINVAL;
		}

		if (i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ) {

			read_length = i2c_list->i2c_settings.read_bytes;
			if (read_length > buff_length) {
				CAM_ERR(CAM_LENS_DRIVER,
				"Invalid buffer size, readLen: %d, bufLen: %d",
				read_length, buff_length);
				return -EINVAL;
			}

			rc = camera_io_dev_read_seq_v1(
				io_master_info,
				&i2c_list->i2c_settings);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"failed: seq read I2C settings: %d",
					rc);
				return rc;
			}
		} else {
			CAM_ERR(CAM_LENS_DRIVER,
				"Invalid opcode: %d",
				i2c_list->op_code);
		}
	}

	return rc;
}

static int cam_ldm_fw_download(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t                             rc = 0;
	struct cam_cmd_ldm_fw_param        *fw_param = NULL;
	uint32_t                            fw_size;
	uint16_t                            len_per_write = 0;
	uint8_t                            *ptr = NULL;
	const struct firmware              *fw = NULL;
	struct device                      *dev = l_ctrl->soc_info.dev;
	uint8_t                             count = 0;
	uint8_t                             cont_wr_flag = 0;

	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	for (count = 0; count < l_ctrl->fw_info.fw_count; count++) {
		fw_param      = &l_ctrl->fw_info.fw_param[count];
		fw_size       = fw_param->fw_size;
		len_per_write = fw_param->fw_len_per_write / fw_param->fw_data_type;

		CAM_DBG(CAM_LENS_DRIVER,
			"count: %d, fw_size: %d, data_type: %d, len_per_write: %d",
			count, fw_size, fw_param->fw_data_type, len_per_write);

		/* Load FW */
		fw = NULL;
		rc = request_firmware(&fw, fw_param->fw_name, dev);
		if (rc) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed to locate %s",
				fw_param->fw_name);
			return rc;
		}

		if (0 == rc && NULL != fw &&
			(fw_size <= fw->size - fw_param->fw_start_pos)) {

			/* fw init */
			CAM_DBG(CAM_LENS_DRIVER, "fw init");
			if (l_ctrl->fw_init_data[count].is_settings_valid == 1) {
				rc = cam_ldm_apply_settings(l_ctrl,
					&l_ctrl->fw_init_data[count]);
				if ((rc == -EAGAIN) &&
					(l_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_LENS_DRIVER,
					"CCI HW is resetting: Reapplying FW init settings");
					usleep_range(1000, 1010);
					rc = cam_ldm_apply_settings(l_ctrl,
						&l_ctrl->fw_init_data[count]);
				}
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Cannot apply FW init settings %d",
						rc);
					goto release_firmware;
				} else {
					CAM_DBG(CAM_LENS_DRIVER, "LDM FW init settings success");
				}
			}

			/* send fw */
			CAM_DBG(CAM_LENS_DRIVER, "send fw, operation %d",
				fw_param->fw_operation);

			if (fw->data == NULL) {
				CAM_ERR(CAM_LENS_DRIVER,
					"fw: %s data is NULL", fw_param->fw_name);
				goto release_firmware;
			}

			ptr = (uint8_t *)(fw->data + fw_param->fw_start_pos);
			if (fw_param->fw_operation == CAMERA_SENSOR_I2C_OP_RNDM_WR)
				cont_wr_flag = CAM_SENSOR_I2C_WRITE_RANDOM;
			else if (fw_param->fw_operation ==
				CAMERA_SENSOR_I2C_OP_CONT_WR_BRST)
				cont_wr_flag = CAM_SENSOR_I2C_WRITE_BURST;
			else if (fw_param->fw_operation ==
				CAMERA_SENSOR_I2C_OP_CONT_WR_SEQN)
				cont_wr_flag = CAM_SENSOR_I2C_WRITE_SEQ;

			write_ldm_fw(ptr, fw_param, l_ctrl->io_master_info, cont_wr_flag);

			/* fw finalize */
			CAM_DBG(CAM_LENS_DRIVER, "fw finalize");
			if (l_ctrl->fw_finalize_data[count].is_settings_valid == 1) {
				rc = cam_ldm_apply_settings(l_ctrl,
					&l_ctrl->fw_finalize_data[count]);
				if ((rc == -EAGAIN) &&
					(l_ctrl->io_master_info.master_type == CCI_MASTER)) {
					CAM_WARN(CAM_LENS_DRIVER,
					"CCI HW is resetting: Reapplying FW finalize settings");
					usleep_range(1000, 1010);
					rc = cam_ldm_apply_settings(l_ctrl,
						&l_ctrl->fw_finalize_data[count]);
				}
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Cannot apply FW finalize settings %d",
						rc);
					goto release_firmware;
				} else {
					CAM_DBG(CAM_LENS_DRIVER, "FW finalize settings success");
				}
			}
		}

		if (fw != NULL) {
			release_firmware(fw);
			fw = NULL;
		}
	}

release_firmware:
	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}

	return rc;
}

int32_t cam_lens_driver_config(struct cam_lens_driver_ctrl_t *l_ctrl,
	void *arg)
{
	int32_t    rc = 0;
	int32_t    i = 0;
	size_t     len_of_buff = 0;
	size_t     remain_len = 0;
	uint32_t   *offset = NULL;
	uint32_t   *cmd_buf = NULL;
	uint32_t   total_cmd_buf_in_bytes = 0;
	uintptr_t  generic_ptr;
	uintptr_t  generic_pkt_ptr;
	struct cam_config_dev_cmd config;
	struct cam_control    *ioctl_ctrl = NULL;
	struct cam_packet     *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct common_header     *cmm_hdr = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	struct cam_lens_driver_soc_private     *soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (!l_ctrl || !arg) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&config, u64_to_user_ptr(ioctl_ctrl->handle),
			sizeof(config)))
		return -EFAULT;

	rc = cam_mem_get_cpu_buf(config.packet_handle,
		&generic_pkt_ptr, &len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "Error in converting command Handle %d",
			rc);
		return rc;
	}

	remain_len = len_of_buff;

	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_LENS_DRIVER,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		cam_mem_put_cpu_buf(config.packet_handle);
		return -EINVAL;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)
			(generic_pkt_ptr + (uint32_t)config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
			remain_len)) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid packet params");
		cam_mem_put_cpu_buf(config.packet_handle);
		return -EINVAL;
	}

	CAM_DBG(CAM_LENS_DRIVER, "Pkt opcode: %d", csl_packet->header.op_code);

	if (csl_packet->header.request_id > l_ctrl->last_flush_req)
		l_ctrl->last_flush_req = 0;

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_LENS_DRIVER_PACKET_OPCODE_INIT:{
		CAM_DBG(CAM_LENS_DRIVER,
			"CAM_LENS_DRIVER_PACKET_OPCODE_INIT,num_cmd_buf %d",
			csl_packet->num_cmd_buf);

		memset(&l_ctrl->fw_info, 0, sizeof(struct cam_cmd_ldm_fw_info));

		offset = (uint32_t *)&csl_packet->payload;
		offset += csl_packet->cmd_buf_offset / sizeof(uint32_t);
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
			if (rc)
				return rc;

			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_LENS_DRIVER, "invalid cmd buf");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return -EINVAL;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Invalid length for sensor cmd");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				cam_mem_put_cpu_buf(config.packet_handle);
				return -EINVAL;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			CAM_DBG(CAM_LENS_DRIVER,
					"cmm_hdr->cmd_type: %d", cmm_hdr->cmd_type);
			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ldm_slaveInfo_pkt_parser(
					l_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed in parsing slave info");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_LENS_DRIVER,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed: parse power settings");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			case CAMERA_SENSOR_LDM_CMD_TYPE_FW_INFO:
				CAM_DBG(CAM_LENS_DRIVER,
					"Received fwInfo buffer,total_cmd_buf_in_bytes: %d",
					total_cmd_buf_in_bytes);
				rc = cam_ldm_fw_info_pkt_parser(
					l_ctrl, cmd_buf, total_cmd_buf_in_bytes);
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER,
					"Failed: parse fw info settings");
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
				break;
			default:
			if (l_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_LENS_DRIVER,
				"Received init/config settings");
				i2c_reg_settings =
					&(l_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&l_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
					"init parsing failed: %d", rc);
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
			} else if ((l_ctrl->is_ldm_calib != 0) &&
				(l_ctrl->ldm_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_LENS_DRIVER,
					"Received calib settings");
				i2c_reg_settings = &(l_ctrl->ldm_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&l_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_LENS_DRIVER,
						"Calib parsing failed: %d", rc);
					cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
					cam_mem_put_cpu_buf(config.packet_handle);
					return rc;
				}
			}
			break;
			}
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
		}

		if ((l_ctrl->cam_lens_drv_state != CAM_LENS_DRIVER_CONFIG) &&
				(l_ctrl->isAlwaysPowerOn == 0)) {
			rc = cam_lens_driver_power_up(l_ctrl);
			if (rc) {
				CAM_ERR(CAM_LENS_DRIVER, "lens driver power up failed");
				cam_mem_put_cpu_buf(config.packet_handle);
				return rc;
			} else {
				l_ctrl->isPowerUpSeqApply = 1;
				CAM_DBG(CAM_LENS_DRIVER, "ldm power up success");
			}
		}

		CAM_DBG(CAM_LENS_DRIVER, "ldm_fw_flag: %d", l_ctrl->ldm_fw_flag);
		if (l_ctrl->ldm_fw_flag) {
			CAM_DBG(CAM_LENS_DRIVER, "fw_count: %d", l_ctrl->fw_info.fw_count);
			if (l_ctrl->fw_info.fw_count != 0) {
				rc = cam_lens_driver_power_up(l_ctrl);
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER, "lens driver power up failed");
					goto end;
				}
				l_ctrl->isPowerUpSeqApply = 1;
				CAM_DBG(CAM_LENS_DRIVER, "power up success");
				rc = cam_ldm_fw_download(l_ctrl);
				if (rc) {
					CAM_ERR(CAM_LENS_DRIVER, "Failed LDM FW Download");
					goto pwr_dwn;
				}
			}
		}
		if (l_ctrl->i2c_init_data.is_settings_valid == 1) {
			rc = cam_ldm_apply_settings(l_ctrl, &l_ctrl->i2c_init_data);
			if ((rc == -EAGAIN) &&
				(l_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_LENS_DRIVER,
					"CCI HW is restting: Reapplying INIT settings");
				usleep_range(1000, 1010);
				rc = cam_ldm_apply_settings(l_ctrl,
					&l_ctrl->i2c_init_data);
			}

			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Cannot apply Init settings: rc = %d",
					rc);
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_LENS_DRIVER, "apply Init settings success");
			}
		}

		if (l_ctrl->is_ldm_calib &&
			l_ctrl->ldm_calib_data.is_settings_valid == 1) {
			rc = cam_ldm_apply_settings(l_ctrl,
				&l_ctrl->ldm_calib_data);
			if ((rc == -EAGAIN) &&
				(l_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_LENS_DRIVER,
					"CCI HW is restting: Reapplying calib settings");
				usleep_range(1000, 1010);
				rc = cam_ldm_apply_settings(l_ctrl,
					&l_ctrl->ldm_calib_data);
			}
			if (rc) {
				CAM_ERR(CAM_LENS_DRIVER, "Cannot apply calib data");
				goto pwr_dwn;
			} else {
				CAM_DBG(CAM_LENS_DRIVER, "apply calib settings success");
			}
		}

		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_CONFIG;

		for (i = 0; i < MAX_LDM_FW_COUNT; i++) {
			if (l_ctrl->fw_init_data[i].is_settings_valid == 1) {
				rc = delete_request(&l_ctrl->fw_init_data[i]);
				if (rc < 0) {
					CAM_WARN(CAM_LENS_DRIVER,
						"Fail deleting fw_init_data: rc: %d", rc);
					rc = 0;
				}
			}
			if (l_ctrl->fw_finalize_data[i].is_settings_valid == 1) {
				rc = delete_request(&l_ctrl->fw_finalize_data[i]);
				if (rc < 0) {
					CAM_WARN(CAM_LENS_DRIVER,
						"Fail deleting fw_finalize_data: rc: %d", rc);
					rc = 0;
				}
			}
		}

		rc = delete_request(&l_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_LENS_DRIVER,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}

		rc = delete_request(&l_ctrl->ldm_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_LENS_DRIVER,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}

		break;
	}
	case CAM_LENS_DRIVER_PACKET_OPCODE_READ: {
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		CAM_DBG(CAM_LENS_DRIVER, "CAM_LDM_PACKET_OPCODE_READ");

		if (l_ctrl->cam_lens_drv_state < CAM_LENS_DRIVER_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
				"Not in right state to read lens driver: %d",
				l_ctrl->cam_lens_drv_state);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		CAM_DBG(CAM_LENS_DRIVER, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_LENS_DRIVER, "No I/O configs to process");
			rc = -EINVAL;
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		/* validate read data io config */
		if (io_cfg == NULL) {
			CAM_ERR(CAM_LENS_DRIVER, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		if (l_ctrl->io_master_info.master_type != SPI_MASTER) {
			rc = cam_sensor_i2c_command_parser(&l_ctrl->io_master_info,
				&i2c_read_settings,
				cmd_desc, 1, &io_cfg[0]);
		} else {
			rc = cam_ldm_command_parser(&l_ctrl->io_master_info,
				&i2c_read_settings,
				cmd_desc, 1, &io_cfg[0]);
		}
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "LDM read pkt parsing failed: %d", rc);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		if (l_ctrl->io_master_info.master_type != SPI_MASTER) {
			rc = cam_sensor_i2c_read_data(
				&i2c_read_settings,
				&l_ctrl->io_master_info);
		} else {
			rc = cam_ldm_i2c_read_data(
				&i2c_read_settings,
				&l_ctrl->io_master_info);
		}
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Failed in deleting the read settings");
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		break;
	}
	case CAM_LENS_DRIVER_PACKET_OPCODE_MOTOR_OPERATION:
		CAM_DBG(CAM_LENS_DRIVER,
			"CAM_LENS_DRIVER_PACKET_OPCODE_MOTOR_OPERATION");
		if (l_ctrl->cam_lens_drv_state < CAM_LENS_DRIVER_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
				"Not in right state to control LDM: %d",
				l_ctrl->cam_lens_drv_state);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(l_ctrl->motor_control_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&l_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "LDM pkt parsing failed: %d", rc);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		rc = cam_ldm_apply_settings(l_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Cannot apply mode settings");
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Fail deleting Mode data: rc: %d", rc);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}

		break;
	case CAM_LENS_DRIVER_PACKET_OPCODE_STREAMON:
		CAM_DBG(CAM_LENS_DRIVER,
			"Lens driver streamOn packet received");
		if (l_ctrl->cam_lens_drv_state < CAM_LENS_DRIVER_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
				"Not in right state to control LDM: %d",
				l_ctrl->cam_lens_drv_state);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(l_ctrl->streamon_settings);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&l_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "LDM pkt parsing failed: %d", rc);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		break;
	case CAM_LENS_DRIVER_PACKET_OPCODE_STREAMOFF:
		CAM_DBG(CAM_LENS_DRIVER,
			"Lens driver streamOff packet received");
		if (l_ctrl->cam_lens_drv_state < CAM_LENS_DRIVER_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
				"Not in right state to control LDM: %d",
				l_ctrl->cam_lens_drv_state);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(l_ctrl->streamoff_settings);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&l_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "LDM pkt parsing failed: %d", rc);
			cam_mem_put_cpu_buf(config.packet_handle);
			return rc;
		}
		break;
	case CAM_PKT_NOP_OPCODE:
		CAM_DBG(CAM_LENS_DRIVER, "CAM_PKT_NOP_OPCODE");
		break;

	default:
		CAM_ERR(CAM_LENS_DRIVER, "Wrong Opcode: %d",
			csl_packet->header.op_code & 0xFFFFFF);

		rc = -EINVAL;
		cam_mem_put_cpu_buf(config.packet_handle);
		return rc;
	}

	if (!rc) {
		cam_mem_put_cpu_buf(config.packet_handle);
		return rc;
	}
pwr_dwn:
	if ((l_ctrl->isAlwaysPowerOn == 0) ||
		(l_ctrl->isPowerUpSeqApply == 1)){
		cam_lens_driver_power_down(l_ctrl);
		l_ctrl->isPowerUpSeqApply = 0;
	}
end:
	cam_mem_put_cpu_buf(config.packet_handle);
	return rc;
}

void cam_lens_driver_shutdown(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int rc = 0, i = 0;
	struct cam_lens_driver_soc_private *soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	CAM_DBG(CAM_LENS_DRIVER, "calling lens driver shutdown ");

	if (l_ctrl->cam_lens_drv_state == CAM_LENS_DRIVER_INIT)
		return;

	if ((l_ctrl->cam_lens_drv_state >= CAM_LENS_DRIVER_CONFIG) &&
			((l_ctrl->isAlwaysPowerOn == 0) ||
			(l_ctrl->isPowerUpSeqApply == 1))) {
		rc = cam_lens_driver_power_down(l_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "Lens Driver Power down failed");

		l_ctrl->isPowerUpSeqApply = 0;
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_ACQUIRE;
	}

	if (l_ctrl->cam_lens_drv_state >= CAM_LENS_DRIVER_ACQUIRE) {
		rc = cam_destroy_device_hdl(l_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "destroying dhdl failed");
		l_ctrl->bridge_intf.device_hdl = -1;
		l_ctrl->bridge_intf.link_hdl = -1;
		l_ctrl->bridge_intf.session_hdl = -1;
	}

	if (power_info->power_setting != NULL)
		kfree(power_info->power_setting);
	if (power_info->power_down_setting != NULL)
		kfree(power_info->power_down_setting);

	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	if (l_ctrl->motor_control_data.is_settings_valid == 1)
		delete_request(&l_ctrl->motor_control_data);

	if (l_ctrl->ldm_calib_data.is_settings_valid == 1)
		delete_request(&l_ctrl->ldm_calib_data);

	if (l_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&l_ctrl->i2c_init_data);

	if (l_ctrl->streamon_settings.is_settings_valid == 1)
		delete_request(&l_ctrl->streamon_settings);

	if (l_ctrl->streamoff_settings.is_settings_valid == 1)
		delete_request(&l_ctrl->streamoff_settings);

	for (i = 0; i < MAX_LDM_FW_COUNT; i++) {
		if (l_ctrl->fw_init_data[i].is_settings_valid == 1) {
			rc = delete_request(&l_ctrl->fw_init_data[i]);
			if (rc < 0) {
				CAM_WARN(CAM_LENS_DRIVER,
					"Fail deleting fw_init_data: rc: %d", rc);
				rc = 0;
			}
	}
		if (l_ctrl->fw_finalize_data[i].is_settings_valid == 1) {
			rc = delete_request(&l_ctrl->fw_finalize_data[i]);
			if (rc < 0) {
				CAM_WARN(CAM_LENS_DRIVER,
					"Fail deleting fw_finalize_data: rc: %d", rc);
				rc = 0;
			}
		}
	}

	l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
}

int32_t cam_lens_driver_cmd(struct cam_lens_driver_ctrl_t *l_ctrl,
	void *arg)
{
	int rc = 0, i = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_lens_driver_soc_private *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!l_ctrl || !cmd) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	CAM_DBG(CAM_LENS_DRIVER, "Opcode to Lens Driver: %d", cmd->op_code);

	soc_private =
		(struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(l_ctrl->lens_driver_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP: {
		struct cam_ldm_query_cap lens_driver_cap = {0};

		lens_driver_cap.slot_info = l_ctrl->soc_info.index;
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&lens_driver_cap,
			sizeof(struct cam_ldm_query_cap))) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_LENS_DRIVER, "Query_Cap ID: %d",
			lens_driver_cap.slot_info);
	}
		break;
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev lens_driver_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if (l_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_LENS_DRIVER, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&lens_driver_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(lens_driver_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed Copying from user\n");
			goto release_mutex;
		}

		bridge_params.session_hdl = lens_driver_acq_dev.session_handle;
		bridge_params.ops = &l_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = l_ctrl;
		bridge_params.dev_id = CAM_LENS_DRIVER;
		lens_driver_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		l_ctrl->bridge_intf.device_hdl = lens_driver_acq_dev.device_handle;
		l_ctrl->bridge_intf.session_hdl =
			lens_driver_acq_dev.session_handle;

		CAM_DBG(CAM_LENS_DRIVER, "Device Handle: %d",
			lens_driver_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&lens_driver_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_ACQUIRE;
	}
		break;
	case CAM_START_DEV: {
		if (l_ctrl->cam_lens_drv_state != CAM_LENS_DRIVER_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
			"Not in right state to start : %d",
			l_ctrl->cam_lens_drv_state);
			goto release_mutex;
		}
		if (l_ctrl->streamon_settings.is_settings_valid == 1) {
			rc = cam_ldm_apply_settings(l_ctrl, &l_ctrl->streamon_settings);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Cannot apply streamon settings: rc = %d",
					rc);
			} else {
				CAM_DBG(CAM_LENS_DRIVER, "apply stream on settings success");
			}
		}
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_START;
		l_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_CONFIG_DEV: {
		rc = cam_lens_driver_config(l_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_LENS_DRIVER, "Failed in lens_driver Parsing");
			goto release_mutex;
		}
	}
		break;
	case CAM_RELEASE_DEV: {
		if (l_ctrl->cam_lens_drv_state == CAM_LENS_DRIVER_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
				"Cant release lens_driver: in start state");
			goto release_mutex;
		}

		if ((l_ctrl->cam_lens_drv_state == CAM_LENS_DRIVER_CONFIG) &&
			((l_ctrl->isAlwaysPowerOn == 0) ||
			(l_ctrl->isPowerUpSeqApply == 1))) {
			rc = cam_lens_driver_power_down(l_ctrl);
			l_ctrl->isPowerUpSeqApply = 0;
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Lens Driver Power down failed");
				goto release_mutex;
			}
		}

		if (l_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_LENS_DRIVER, "link hdl: %d device hdl: %d",
				l_ctrl->bridge_intf.device_hdl,
				l_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (l_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_LENS_DRIVER,
				"Device [%d] still active on link 0x%x",
				l_ctrl->cam_lens_drv_state,
				l_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_destroy_device_hdl(l_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_LENS_DRIVER, "destroying the device hdl");

		l_ctrl->bridge_intf.device_hdl = -1;
		l_ctrl->bridge_intf.link_hdl = -1;
		l_ctrl->bridge_intf.session_hdl = -1;
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_INIT;
		l_ctrl->last_flush_req = 0;

		if (power_info->power_setting != NULL)
			kfree(power_info->power_setting);
		if (power_info->power_down_setting != NULL)
			kfree(power_info->power_down_setting);

		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (l_ctrl->motor_control_data.is_settings_valid == 1)
			delete_request(&l_ctrl->motor_control_data);

		if (l_ctrl->ldm_calib_data.is_settings_valid == 1)
			delete_request(&l_ctrl->ldm_calib_data);

		if (l_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&l_ctrl->i2c_init_data);

		if (l_ctrl->streamon_settings.is_settings_valid == 1)
			delete_request(&l_ctrl->streamon_settings);

		if (l_ctrl->streamoff_settings.is_settings_valid == 1)
			delete_request(&l_ctrl->streamoff_settings);

		for (i = 0; i < MAX_LDM_FW_COUNT; i++) {
			if (l_ctrl->fw_init_data[i].is_settings_valid == 1) {
				rc = delete_request(&l_ctrl->fw_init_data[i]);
				if (rc < 0) {
					CAM_WARN(CAM_LENS_DRIVER,
						"Fail deleting fw_init_data: rc: %d", rc);
					rc = 0;
				}
			}
			if (l_ctrl->fw_finalize_data[i].is_settings_valid == 1) {
				rc = delete_request(&l_ctrl->fw_finalize_data[i]);
				if (rc < 0) {
					CAM_WARN(CAM_LENS_DRIVER,
						"Fail deleting fw_finalize_data: rc: %d", rc);
					rc = 0;
				}
			}
		}
	}
		break;
	case CAM_STOP_DEV: {
		if (l_ctrl->cam_lens_drv_state != CAM_LENS_DRIVER_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_LENS_DRIVER,
			"Not in right state to stop : %d",
			l_ctrl->cam_lens_drv_state);
			goto release_mutex;
		}

		if (l_ctrl->streamoff_settings.is_settings_valid == 1) {
			rc = cam_ldm_apply_settings(l_ctrl, &l_ctrl->streamoff_settings);
			if (rc < 0) {
				CAM_ERR(CAM_LENS_DRIVER,
					"Cannot apply streamoff settings: rc = %d",
					rc);
			} else {
				CAM_DBG(CAM_LENS_DRIVER, "apply stream off settings success");
			}
		}

		l_ctrl->last_flush_req = 0;
		l_ctrl->cam_lens_drv_state = CAM_LENS_DRIVER_CONFIG;
		memset(&l_ctrl->fw_info, 0, sizeof(struct cam_cmd_ldm_fw_info));
	}
		break;
	case CAM_FLUSH_REQ:
		CAM_DBG(CAM_LENS_DRIVER, "Flush recveived");
		break;
	default:
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Opcode %d", cmd->op_code);
		rc = -EINVAL;
	}

release_mutex:
	mutex_unlock(&(l_ctrl->lens_driver_mutex));

	return rc;
}
