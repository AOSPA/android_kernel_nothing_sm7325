/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_LENS_DRIVER_DEV_H_
#define _CAM_LENS_DRIVER_DEV_H_

#include <cam_sensor_io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/ion.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include <cam_sensor_spi.h>
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"
#include "cam_context.h"
#include "cam_req_mgr_workq.h"

#define CAMX_LENS_DRIVER_DEV_NAME "cam-lens-driver-dev"
#define LENS_DRIVER_I2C "cam-i2c-lens-driver"
#define LENS_DRIVER_SPI "cam-spi-lens-driver"

enum cam_lens_driver_state {
	CAM_LENS_DRIVER_INIT,
	CAM_LENS_DRIVER_ACQUIRE,
	CAM_LENS_DRIVER_CONFIG,
	CAM_LENS_DRIVER_START,
};

/**
 * struct cam_ldm_i2c_info_t - I2C info
 * @slave_addr      :   slave address
 * @i2c_freq_mode   :   i2c frequency mode
 *
 */
struct cam_ldm_i2c_info_t {
	uint16_t slave_addr;
	uint8_t i2c_freq_mode;
};

/**
 * struct cam_lens_driver_soc_private
 * @cam_ldm_i2c_info_t: i2c slave info
 * @cam_sensor_power_ctrl_t: ldm power info
 */
struct cam_lens_driver_soc_private {
	struct cam_ldm_i2c_info_t i2c_info;
	struct cam_sensor_power_ctrl_t power_info;
};

/**
 * struct intf_params
 * @device_hdl: Device Handle
 * @session_hdl: Session Handle
 * @ops: KMD operations
 * @crm_cb: Callback API pointers
 */
struct intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

/**
 * struct cam_lens_driver_ctrl_t
 * @device_name: Device name
 * @spi_client: SPI device handle
 * @soc_info: Soc Info of camera hardware driver module
 * @lens_driver_mutex: Lens Driver mutex
 * @spi_sync_mutex: SPI bus sync mutex
 * @cam_lens_driver_state: Lens driver state
 * @motor_drv_method: Motor driving method
 * @v4l2_dev_str: V4L2 device structure
 * @lens_driver_intf: Lens driver interface APIs
 * @stm_motor_info: Stepper motor info
 * @lens_capability: Lens driver Capability
 * @last_flush_req: Last request to flush
 * @cmd_work: Workqueue for motor driving operation
 * @cmd_work_data: Command data for motor operation
 */
struct cam_lens_driver_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	uint8_t ldm_fw_flag;
	uint8_t is_ldm_calib;
	uint8_t isAlwaysPowerOn;
	uint8_t isPowerUpSeqApply;
	uint32_t last_flush_req;
	enum cam_lens_driver_state cam_lens_drv_state;
	enum msm_camera_device_type_t ldm_device_type;
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct camera_io_master io_master_info;
	struct mutex lens_driver_mutex;
	struct cam_subdev v4l2_dev_str;
	struct intf_params bridge_intf;
	struct cam_cmd_ldm_fw_info fw_info;
	struct i2c_settings_array fw_init_data[MAX_LDM_FW_COUNT];
	struct i2c_settings_array fw_finalize_data[MAX_LDM_FW_COUNT];
	struct i2c_settings_array i2c_init_data;
	struct i2c_settings_array ldm_calib_data;
	struct i2c_settings_array motor_control_data;
	struct i2c_settings_array streamon_settings;
	struct i2c_settings_array streamoff_settings;
};

/**
 * @brief : API to register Lens Driver hw to platform framework.
 * @return struct platform_device pointer on success, or ERR_PTR() on error.
 */
int cam_lens_driver_init(void);

/**
 * @brief : API to remove Lens Driver Hw from platform framework.
 */
void cam_lens_driver_exit(void);

#endif /* _CAM_LENS_DRIVER_DEV_H_ */
