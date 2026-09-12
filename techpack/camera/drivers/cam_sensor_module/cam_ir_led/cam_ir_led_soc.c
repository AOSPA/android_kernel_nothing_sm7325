/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include "cam_ir_led_soc.h"
#include "cam_res_mgr_api.h"
#include "linux/i2c.h"

static int ir_cut_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct cam_soc_gpio_data *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		CAM_ERR(CAM_IR_LED, "gpio-req-tbl-num 0");
		return 0;
	}

	val_array = kcalloc(count, sizeof(uint32_t), GFP_KERNEL);
	if (!val_array)
		return -ENOMEM;

	gconf->cam_gpio_req_tbl = kcalloc(count, sizeof(struct gpio),
		GFP_KERNEL);
	if (!gconf->cam_gpio_req_tbl) {
		rc = -ENOMEM;
		goto free_val_array;
	}
	gconf->cam_gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-num",
		val_array, count);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "failed in reading gpio-req-tbl-num, rc = %d",
			rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			CAM_ERR(CAM_IR_LED, "gpio req tbl index %d invalid",
				val_array[i]);
			goto free_gpio_req_tbl;
		}
		gconf->cam_gpio_req_tbl[i].gpio = gpio_array[val_array[i]];

	}

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
		val_array, count);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "Failed in gpio-req-tbl-flags, rc %d", rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		gconf->cam_gpio_req_tbl[i].flags = val_array[i];
		CAM_DBG(CAM_IR_LED, "cam_gpio_req_tbl[%d].flags = %ld", i,
			gconf->cam_gpio_req_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"gpio-req-tbl-label", i,
			&gconf->cam_gpio_req_tbl[i].label);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "Failed rc %d", rc);
			goto free_gpio_req_tbl;
		}
		CAM_DBG(CAM_IR_LED, "cam_gpio_req_tbl[%d].label = %s", i,
			gconf->cam_gpio_req_tbl[i].label);
	}

	kfree(val_array);

	return rc;

free_gpio_req_tbl:
	kfree(gconf->cam_gpio_req_tbl);
free_val_array:
	kfree(val_array);
	gconf->cam_gpio_req_tbl_size = 0;

	return rc;
}

int cam_ir_cut_get_gpio_info(struct cam_hw_soc_info *soc_info)
{
	int32_t rc = 0, i = 0;
	uint16_t *gpio_array = NULL;
	int16_t gpio_array_size = 0;
	struct cam_soc_gpio_data *gconf = NULL;
	struct device_node *of_node = NULL;

	if (!soc_info || !soc_info->dev)
		return -EINVAL;

	of_node = soc_info->dev->of_node;
	CAM_DBG(CAM_IR_LED, "debug_log cam_ir_cut_get_gpio_info of_node->name:%s", of_node->name);
	/* Validate input parameters */
	if (!of_node) {
		CAM_ERR(CAM_IR_LED, "Invalid param of_node");
		return -EINVAL;
	}

	gpio_array_size = of_gpio_count(of_node);
	if (gpio_array_size <= 0)
		return 0;

	CAM_DBG(CAM_IR_LED, " gpio count %d", gpio_array_size);

	gpio_array = kcalloc(gpio_array_size, sizeof(uint16_t), GFP_KERNEL);
	if (!gpio_array)
		goto free_gpio_conf;
	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_gpio(of_node, i);
		CAM_DBG(CAM_IR_LED, "gpio_array[%d] = %d", i, gpio_array[i]);
	}

	gconf = kzalloc(sizeof(*gconf), GFP_KERNEL);
	if (!gconf)
		return -ENOMEM;

	rc = ir_cut_get_dt_gpio_req_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "failed in msm_camera_get_dt_gpio_req_tbl");
		goto free_gpio_array;
	}

	gconf->cam_gpio_common_tbl = kcalloc(gpio_array_size,
				sizeof(struct gpio), GFP_KERNEL);
	if (!gconf->cam_gpio_common_tbl) {
		rc = -ENOMEM;
		goto free_gpio_array;
	}

	for (i = 0; i < gpio_array_size; i++)
		gconf->cam_gpio_common_tbl[i].gpio = gpio_array[i];

	gconf->cam_gpio_common_tbl_size = gpio_array_size;
	soc_info->gpio_data = gconf;
	kfree(gpio_array);

	return rc;

free_gpio_array:
	kfree(gpio_array);
free_gpio_conf:
	kfree(gconf);
	soc_info->gpio_data = NULL;

	return rc;
}

int cam_ir_led_get_dt_data(struct cam_ir_led_ctrl *ictrl,
	struct cam_hw_soc_info *soc_info)
{
	int32_t rc = 0;

	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "NULL ir_led control structure");
		return -EINVAL;
	}

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_IR_LED, "get_dt_properties failed rc %d", rc);
		return rc;
	}

	soc_info->soc_private =
		kzalloc(sizeof(struct cam_ir_led_private_soc), GFP_KERNEL);
	if (!soc_info->soc_private) {
		CAM_ERR(CAM_IR_LED, "soc_info->soc_private is NULL");
		rc = -ENOMEM;
		goto release_soc_res;
	}

	rc = cam_ir_cut_get_gpio_info(&ictrl->soc_info);
	if (rc) {
		CAM_ERR(CAM_IR_LED, "Fail to cam_ir_cut_get_gpio_info rc:%d", rc);
	}

	if (of_property_read_bool(soc_info->dev->of_node, "pwms")) {
		ictrl->pwm_dev = of_pwm_get(soc_info->dev,
			ictrl->pdev->dev.of_node, NULL);
		if (ictrl->pwm_dev == NULL)
			CAM_ERR(CAM_IR_LED, "Cannot get PWM device");
		ictrl->ir_led_driver_type = IR_LED_DRIVER_PMIC;
	} else if (of_property_read_bool(soc_info->dev->of_node, "i2c")) {
		struct device_node *PCA9632 = of_parse_phandle(ictrl->pdev->dev.of_node, "i2c", 0);
		if (PCA9632 == NULL)
			CAM_ERR(CAM_IR_LED, "Cannot get PCA9632 device");
		else {
			ictrl->io_master_info.client =
				of_find_i2c_device_by_node(PCA9632);

			if (ictrl->io_master_info.client != NULL) {
				ictrl->io_master_info.master_type = I2C_MASTER;
				ictrl->ir_led_driver_type = IR_LED_DRIVER_I2C;
			} else {
				CAM_ERR(CAM_IR_LED, "Cannot get PCA9632 I2C device");
			}

			of_node_put(PCA9632);
		}
	} else {
		ictrl->ir_led_driver_type = IR_LED_DRIVER_GPIO;
	}

	return rc;

release_soc_res:
	cam_soc_util_release_platform_resource(soc_info);
	return rc;
}
