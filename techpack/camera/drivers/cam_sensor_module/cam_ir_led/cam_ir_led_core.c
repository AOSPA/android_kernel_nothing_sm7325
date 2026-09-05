/* Copyright (c) 2019,2020 The Linux Foundation. All rights reserved.
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

#include "cam_ir_led_core.h"

static int cam_ir_led_off(struct cam_ir_led_ctrl *ictrl)
{
	int rc = 0;

	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "IR LED control Null");
		return -EINVAL;
	}
	CAM_DBG(CAM_IR_LED, "IR LED OFF Triggered");

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

	if ((ictrl->i2c_data.streamoff_settings.is_settings_valid) &&
		(ictrl->i2c_data.streamoff_settings.request_id == 0)) {
		if (ictrl->func_tbl->apply_setting != NULL) {
			rc = ictrl->func_tbl->apply_setting(ictrl, 0);
			if (rc) {
				CAM_ERR(CAM_IR_LED, "cannot apply settings rc = %d", rc);
				return rc;
			}
		}
	}
	return rc;
}

int cam_ir_led_stop_dev(struct cam_ir_led_ctrl *ictrl)
{
	int rc = 0;

	cam_ir_led_off(ictrl);

	if (ictrl->func_tbl->camera_ir_led_off != NULL)
		rc = ictrl->func_tbl->camera_ir_led_off(ictrl);

	if (ictrl->func_tbl->camera_ir_cut_off != NULL)
		rc = ictrl->func_tbl->camera_ir_cut_off(ictrl);

	return rc;
}

int cam_ir_led_release_dev(struct cam_ir_led_ctrl *ictrl)
{
	int rc = 0;

	if (ictrl->device_hdl != -1) {
		rc = cam_destroy_device_hdl(ictrl->device_hdl);
		if (rc)
			CAM_ERR(CAM_IR_LED,
				"Failed in destroying device handle rc = %d",
				rc);
		ictrl->device_hdl = -1;
		ictrl->bridge_intf.link_hdl = -1;
		ictrl->bridge_intf.session_hdl = -1;
		ictrl->last_flush_req = 0;
	}

	return rc;
}

void cam_ir_led_shutdown(struct cam_ir_led_ctrl *ictrl)
{
	int rc;

	if (ictrl->ir_led_state == CAM_IR_LED_STATE_INIT)
		return;

	if (ictrl->ir_led_state == CAM_IR_LED_STATE_ON) {
		rc = cam_ir_led_stop_dev(ictrl);
		if (rc)
			CAM_ERR(CAM_IR_LED, "Stop Failed rc: %d", rc);
	}

	rc = cam_ir_led_release_dev(ictrl);
	if (rc)
		CAM_ERR(CAM_IR_LED, "Release failed rc: %d", rc);
	else
		ictrl->ir_led_state = CAM_IR_LED_STATE_INIT;
}

int32_t cam_ir_led_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	if (!info) {
		CAM_ERR(CAM_IR_LED, "Invalid Args");
		return -EINVAL;
	}

	info->dev_id = CAM_REQ_MGR_DEVICE_IRLED;
	strlcpy(info->name, CAM_IR_LED_NAME, sizeof(info->name));
	info->p_delay = 1;
	info->trigger = CAM_TRIGGER_POINT_SOF;

	return 0;
}

int32_t cam_ir_led_establish_link(
	struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_ir_led_ctrl *ictrl = NULL;

	if (!link) {
		CAM_ERR(CAM_IR_LED, "Invalid Args");
		return -EINVAL;
	}

	ictrl = (struct cam_ir_led_ctrl *)
		cam_get_device_priv(link->dev_hdl);
	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(ictrl->ir_led_mutex));
	if (link->link_enable) {
		ictrl->bridge_intf.link_hdl = link->link_hdl;
		ictrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		ictrl->bridge_intf.link_hdl = -1;
		ictrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&(ictrl->ir_led_mutex));

	return 0;
}

int cam_ir_led_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_ir_led_ctrl *ictrl = NULL;

	if (!apply)
		return -EINVAL;

	ictrl = (struct cam_ir_led_ctrl *) cam_get_device_priv(apply->dev_hdl);
	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(ictrl->ir_led_mutex));
	rc = ictrl->func_tbl->apply_setting(ictrl, apply->request_id);
	if (rc)
		CAM_ERR(CAM_IR_LED, "apply_setting failed with rc=%d",
			rc);
	mutex_unlock(&(ictrl->ir_led_mutex));

	return rc;
}

int cam_ir_led_flush_request(struct cam_req_mgr_flush_request *flush)
{
	int rc = 0;
	struct cam_ir_led_ctrl *ictrl = NULL;

	ictrl = (struct cam_ir_led_ctrl *) cam_get_device_priv(flush->dev_hdl);
	if (!ictrl) {
		CAM_ERR(CAM_IR_LED, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&ictrl->ir_led_mutex);
	if (ictrl->ir_led_state == CAM_IR_LED_STATE_INIT)
		goto end;

	if (flush->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		ictrl->last_flush_req = flush->req_id;
		CAM_DBG(CAM_IR_LED, "last reqest to flush is %lld",
			flush->req_id);
		rc = ictrl->func_tbl->flush_req(ictrl, IR_FLUSH_ALL, 0);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "FLUSH_TYPE_ALL failed rc: %d", rc);
			goto end;
		}
	} else if (flush->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
		rc = ictrl->func_tbl->flush_req(ictrl,
				IR_FLUSH_REQ, flush->req_id);
		if (rc) {
			CAM_ERR(CAM_IR_LED, "IR_FLUSH_REQ failed rc: %d", rc);
			goto end;
		}
	}
end:
	mutex_unlock(&ictrl->ir_led_mutex);
	return rc;
}
