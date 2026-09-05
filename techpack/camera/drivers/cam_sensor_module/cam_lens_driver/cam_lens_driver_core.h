/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_LENS_DRIVER_CORE_H_
#define _CAM_LENS_DRIVER_CORE_H_

#include "cam_lens_driver_dev.h"

/**
 * @a_ctrl: Lens Driver ctrl structure
 * @arg:    Camera control command argument
 *
 * This API handles the camera control argument reached to lens_driver
 */
int32_t cam_lens_driver_cmd(struct cam_lens_driver_ctrl_t *a_ctrl, void *arg);

/**
 * @a_ctrl: Lens Driver ctrl structure
 *
 * This API handles the shutdown ioctl/close
 */
void cam_lens_driver_shutdown(struct cam_lens_driver_ctrl_t *a_ctrl);

#endif /* _CAM_LENS_DRIVER_CORE_H_ */
