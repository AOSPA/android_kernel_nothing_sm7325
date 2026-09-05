/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include <cam_req_mgr_util.h>
#include "cam_lens_driver_soc.h"
#include "cam_soc_util.h"

int32_t cam_lens_driver_parse_dt(struct cam_lens_driver_ctrl_t *l_ctrl)
{
	int32_t                             i, rc = 0;
	struct cam_hw_soc_info             *soc_info = NULL;
	struct cam_lens_driver_soc_private *soc_private = NULL;
	struct device_node                 *of_node = NULL;
	struct cam_sensor_power_ctrl_t     *power_info = NULL;

	if (!l_ctrl) {
		CAM_ERR(CAM_LENS_DRIVER, "Invalid Args");
		return -EINVAL;
	}

	soc_info = &l_ctrl->soc_info;
	soc_private = (struct cam_lens_driver_soc_private *)l_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	of_node = soc_info->dev->of_node;
	if (!of_node) {
		CAM_ERR(CAM_LENS_DRIVER, "of_node is NULL, device type %d",
			l_ctrl->ldm_device_type);
		return -EINVAL;
	}
	/* Initialize mutex */
	mutex_init(&(l_ctrl->lens_driver_mutex));
	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_LENS_DRIVER, "parsing common soc dt(rc %d)", rc);
		return rc;
	}

	rc = cam_sensor_util_regulator_powerup(soc_info);
	if (rc < 0)
		return rc;

	if (!soc_info->gpio_data) {
		CAM_INFO(CAM_LENS_DRIVER, "No GPIO found");
		return 0;
	}

	if (!soc_info->gpio_data->cam_gpio_common_tbl_size) {
		CAM_INFO(CAM_LENS_DRIVER, "No GPIO found");
		return -EINVAL;
	}

	rc = cam_sensor_util_init_gpio_pin_tbl(soc_info,
		&power_info->gpio_num_info);
	if ((rc < 0) || (!power_info->gpio_num_info)) {
		CAM_ERR(CAM_LENS_DRIVER, "No/Error LDM GPIOs");
		return -EINVAL;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		soc_info->clk[i] = devm_clk_get(soc_info->dev,
			soc_info->clk_name[i]);
		if (!soc_info->clk[i]) {
			CAM_ERR(CAM_LENS_DRIVER, "get failed for %s",
				soc_info->clk_name[i]);
			rc = -ENOENT;
			return rc;
		}
	}

	return rc;
}
