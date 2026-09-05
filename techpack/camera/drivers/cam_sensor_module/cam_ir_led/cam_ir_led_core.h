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

#ifndef _CAM_IR_LED_CORE_H_
#define _CAM_IR_LED_CORE_H_
#include "cam_ir_led_dev.h"

void cam_ir_led_shutdown(struct cam_ir_led_ctrl *ictrl);
int cam_ir_led_stop_dev(struct cam_ir_led_ctrl *ictrl);
int cam_ir_led_release_dev(struct cam_ir_led_ctrl *ictrl);

int32_t cam_ir_led_publish_dev_info(struct cam_req_mgr_device_info *info);
int32_t cam_ir_led_establish_link(struct cam_req_mgr_core_dev_link_setup *link);

int cam_ir_led_apply_request(struct cam_req_mgr_apply_request *apply);
int cam_ir_led_flush_request(struct cam_req_mgr_flush_request *flush);
#endif /*_CAM_IR_LED_CORE_H_*/
