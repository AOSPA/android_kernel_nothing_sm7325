/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_LENS_DRIVER_SOC_H_
#define _CAM_LENS_DRIVER_SOC_H_

#include "cam_lens_driver_dev.h"

/**
 * @l_ctrl: Lens Driver ctrl structure
 *
 * This API parses lens_driver device tree
 */
int cam_lens_driver_parse_dt(struct cam_lens_driver_ctrl_t *l_ctrl);

#endif /* _CAM_LENS_DRIVER_SOC_H_ */
