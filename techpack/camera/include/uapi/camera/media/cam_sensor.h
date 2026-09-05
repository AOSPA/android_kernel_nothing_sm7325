/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023,2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UAPI_CAM_SENSOR_H__
#define __UAPI_CAM_SENSOR_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <camera/media/cam_defs.h>

#define CAM_SENSOR_PROBE_CMD   (CAM_COMMON_OPCODE_MAX + 1)
#define CAM_FLASH_MAX_LED_TRIGGERS 2
#define MAX_OIS_NAME_SIZE 32
#define CAM_CSIPHY_SECURE_MODE_ENABLED 1
#define CAM_IR_LED_SUPPORTED
#define MAX_LDM_FW_COUNT 3
#define MAX_LDM_NAME_SIZE 32
#define MAX_LDM_SUPPORTED 3

/**
 * struct cam_sensor_query_cap - capabilities info for sensor
 *
 * @slot_info        :  Indicates about the slotId or cell Index
 * @secure_camera    :  Camera is in secure/Non-secure mode
 * @pos_pitch        :  Sensor position pitch
 * @pos_roll         :  Sensor position roll
 * @pos_yaw          :  Sensor position yaw
 * @actuator_slot_id :  Actuator slot id which connected to sensor
 * @eeprom_slot_id   :  EEPROM slot id which connected to sensor
 * @ois_slot_id      :  OIS slot id which connected to sensor
 * @flash_slot_id    :  Flash slot id which connected to sensor
 * @csiphy_slot_id   :  CSIphy slot id which connected to sensor
 * @irled_slot_id    :  IRLED slot id which connected to sensor
 * @ldm_slot_id      :  Lens driver slot id which connected to sensor
 *
 */
struct  cam_sensor_query_cap {
	__u32        slot_info;
	__u32        secure_camera;
	__u32        pos_pitch;
	__u32        pos_roll;
	__u32        pos_yaw;
	__u32        actuator_slot_id;
	__u32        eeprom_slot_id;
	__u32        ois_slot_id;
	__u32        flash_slot_id;
	__u32        csiphy_slot_id;
	__u32        ir_led_slot_id;
	__u32        ldm_slot_id[MAX_LDM_SUPPORTED];
} __attribute__((packed));

/**
 * struct cam_csiphy_query_cap - capabilities info for csiphy
 *
 * @slot_info        :  Indicates about the slotId or cell Index
 * @version          :  CSIphy version
 * @clk lane         :  Of the 5 lanes, informs lane configured
 *                      as clock lane
 * @reserved
 */
struct cam_csiphy_query_cap {
	__u32            slot_info;
	__u32            version;
	__u32            clk_lane;
	__u32            reserved;
} __attribute__((packed));

/**
 * struct cam_actuator_query_cap - capabilities info for actuator
 *
 * @slot_info        :  Indicates about the slotId or cell Index
 * @reserved
 */
struct cam_actuator_query_cap {
	__u32            slot_info;
	__u32            reserved;
} __attribute__((packed));

/**
 * struct cam_eeprom_query_cap_t - capabilities info for eeprom
 *
 * @slot_info                  :  Indicates about the slotId or cell Index
 * @eeprom_kernel_probe        :  Indicates about the kernel or userspace probe
 */
struct cam_eeprom_query_cap_t {
	__u32            slot_info;
	__u16            eeprom_kernel_probe;
	__u16            is_multimodule_mode;
} __attribute__((packed));

/**
 * struct cam_ois_query_cap_t - capabilities info for ois
 *
 * @slot_info                  :  Indicates about the slotId or cell Index
 */
struct cam_ois_query_cap_t {
	__u32            slot_info;
	__u16            reserved;
} __attribute__((packed));

/**
 * struct cam_cmd_i2c_info - Contains slave I2C related info
 *
 * @slave_addr      :    Slave address
 * @i2c_freq_mode   :    4 bits are used for I2c freq mode
 * @cmd_type        :    Explains type of command
 */
struct cam_cmd_i2c_info {
	__u32    slave_addr;
	__u8     i2c_freq_mode;
	__u8     cmd_type;
	__u16    reserved;
} __attribute__((packed));

/**
 * struct cam_ois_opcode - Contains OIS opcode
 *
 * @prog            :    OIS FW prog register address
 * @coeff           :    OIS FW coeff register address
 * @pheripheral     :    OIS pheripheral
 * @memory          :    OIS memory
 */
struct cam_ois_opcode {
	__u32 prog;
	__u32 coeff;
	__u32 pheripheral;
	__u32 memory;
} __attribute__((packed));

/**
 * struct cam_cmd_ois_info - Contains OIS slave info
 *
 * @slave_addr            :    OIS i2c slave address
 * @i2c_freq_mode         :    i2c frequency mode
 * @cmd_type              :    Explains type of command
 * @ois_fw_flag           :    indicates if fw is present or not
 * @is_ois_calib          :    indicates the calibration data is available
 * @ois_name              :    OIS name
 * @opcode                :    opcode
 */
struct cam_cmd_ois_info {
	__u32                 slave_addr;
	__u8                  i2c_freq_mode;
	__u8                  cmd_type;
	__u8                  ois_fw_flag;
	__u8                  is_ois_calib;
	char                  ois_name[MAX_OIS_NAME_SIZE];
	struct cam_ois_opcode opcode;
} __attribute__((packed));

/**
 * struct cam_cmd_probe - Contains sensor slave info
 *
 * @data_type       :   Slave register data type
 * @addr_type       :   Slave register address type
 * @op_code         :   Don't Care
 * @cmd_type        :   Explains type of command
 * @reg_addr        :   Slave register address
 * @expected_data   :   Data expected at slave register address
 * @data_mask       :   Data mask if only few bits are valid
 * @camera_id       :   Indicates the slot to which camera
 *                      needs to be probed
 * @reserved
 */
struct cam_cmd_probe {
	__u8     data_type;
	__u8     addr_type;
	__u8     op_code;
	__u8     cmd_type;
	__u32    reg_addr;
	__u32    expected_data;
	__u32    data_mask;
	__u16    camera_id;
	__u16    reserved;
} __attribute__((packed));

/**
 * struct cam_power_settings - Contains sensor power setting info
 *
 * @power_seq_type  :   Type of power sequence
 * @reserved
 * @config_val_low  :   Lower 32 bit value configuration value
 * @config_val_high :   Higher 32 bit value configuration value
 *
 */
struct cam_power_settings {
	__u16    power_seq_type;
	__u16    reserved;
	__u32    config_val_low;
	__u32    config_val_high;
} __attribute__((packed));

/**
 * struct cam_cmd_power - Explains about the power settings
 *
 * @count           :    Number of power settings follows
 * @reserved
 * @cmd_type        :    Explains type of command
 * @power_settings  :    Contains power setting info
 */
struct cam_cmd_power {
	__u32                       count;
	__u8                        reserved;
	__u8                        cmd_type;
	__u16                       more_reserved;
	struct cam_power_settings   power_settings[1];
} __attribute__((packed));

/**
 * struct i2c_rdwr_header - header of READ/WRITE I2C command
 *
 * @ count           :   Number of registers / data / reg-data pairs
 * @ op_code         :   Operation code
 * @ cmd_type        :   Command buffer type
 * @ data_type       :   I2C data type
 * @ addr_type       :   I2C address type
 * @ frequency       :   Frequency setting for reg-data pairs
 */
struct i2c_rdwr_header {
	__u32    count;
	__u8     op_code;
	__u8     cmd_type;
	__u8     data_type;
	__u8     addr_type;
	__u32    frequency;
} __attribute__((packed));

/**
 * struct i2c_random_wr_payload - payload for I2C random write
 *
 * @ reg_addr        :   Register address
 * @ reg_data        :   Register data
 *
 */
struct i2c_random_wr_payload {
	__u32     reg_addr;
	__u32     reg_data;
} __attribute__((packed));

/**
 * struct cam_cmd_i2c_random_wr - I2C random write command
 * @ header            :   header of READ/WRITE I2C command
 * @ random_wr_payload :   payload for I2C random write
 */
struct cam_cmd_i2c_random_wr {
	struct i2c_rdwr_header       header;
	struct i2c_random_wr_payload random_wr_payload[1];
} __attribute__((packed));

/**
 * struct cam_cmd_read - I2C read command
 * @ reg_data        :   Register data
 * @ reserved
 */
struct cam_cmd_read {
	__u32                reg_data;
	__u32                reserved;
} __attribute__((packed));

/**
 * struct cam_cmd_i2c_continuous_wr - I2C continuous write command
 * @ header          :   header of READ/WRITE I2C command
 * @ reg_addr        :   Register address
 * @ data_read       :   I2C read command
 */
struct cam_cmd_i2c_continuous_wr {
	struct i2c_rdwr_header header;
	__u32                  reg_addr;
	struct cam_cmd_read    data_read[1];
} __attribute__((packed));

/**
 * struct cam_cmd_i2c_random_rd - I2C random read command
 * @ header          :   header of READ/WRITE I2C command
 * @ data_read       :   I2C read command
 */
struct cam_cmd_i2c_random_rd {
	struct i2c_rdwr_header header;
	struct cam_cmd_read    data_read[1];
} __attribute__((packed));

/**
 * struct cam_cmd_i2c_continuous_rd - I2C continuous continuous read command
 * @ header          :   header of READ/WRITE I2C command
 * @ reg_addr        :   Register address
 * @ num_bytes       :   Number of read bytes
 * @ data_read       :   I2C read command
 */
struct cam_cmd_i2c_continuous_rd {
	struct i2c_rdwr_header header;
	__u32                  reg_addr;
	__u32                  num_bytes;
	struct cam_cmd_read    data_read[1];
} __attribute__((packed));

/**
 * struct cam_cmd_conditional_wait - Conditional wait command
 * @data_type       :   Data type
 * @addr_type       :   Address type
 * @op_code         :   Opcode
 * @cmd_type        :   Explains type of command
 * @timeout         :   Timeout for retries
 * @reserved
 * @reg_addr        :   Register Address
 * @reg_data        :   Register data
 * @data_mask       :   Data mask if only few bits are valid
 * @camera_id       :   Indicates the slot to which camera
 *                      needs to be probed
 *
 */
struct cam_cmd_conditional_wait {
	__u8     data_type;
	__u8     addr_type;
	__u16    reserved;
	__u8     op_code;
	__u8     cmd_type;
	__u16    timeout;
	__u32    reg_addr;
	__u32    reg_data;
	__u32    data_mask;
} __attribute__((packed));

/**
 * struct cam_cmd_unconditional_wait - Un-conditional wait command
 * @delay           :   Delay
 * @op_code         :   Opcode
 * @cmd_type        :   Explains type of command
 */
struct cam_cmd_unconditional_wait {
	__s16    delay;
	__s16    reserved;
	__u8     op_code;
	__u8     cmd_type;
	__u16    reserved1;
} __attribute__((packed));

/**
 * cam_csiphy_info       : Provides cmdbuffer structre
 * @lane_assign          : Lane sensor will be using
 * @mipi_flags           : MIPI phy flags
 * @lane_cnt             : Total number of lanes
 * @secure_mode          : Secure mode flag to enable / disable
 * @settle_time          : Settling time in ms
 * @data_rate            : Data rate
 *
 */
struct cam_csiphy_info {
	__u16    reserved;
	__u16    lane_assign;
	__u16    mipi_flags;
	__u8     lane_cnt;
	__u8     secure_mode;
	__u64    settle_time;
	__u64    data_rate;
} __attribute__((packed));

/**
 * cam_csiphy_acquire_dev_info : Information needed for
 *                               csiphy at the time of acquire
 * @combo_mode                 : Indicates the device mode of operation
 * @cphy_dphy_combo_mode       : Info regarding cphy_dphy_combo mode
 * @csiphy_3phase              : Details whether 3Phase / 2Phase operation
 * @mux_mode                   : Indication for Mux mode operation
 *
 */
struct cam_csiphy_acquire_dev_info {
	__u32    combo_mode;
	__u16    cphy_dphy_combo_mode;
	__u8     csiphy_3phase;
	__u8     mux_mode;
} __attribute__((packed));

/**
 * cam_sensor_acquire_dev : Updates sensor acuire cmd
 * @device_handle  :    Updates device handle
 * @session_handle :    Session handle for acquiring device
 * @handle_type    :    Resource handle type
 * @reserved
 * @info_handle    :    Handle to additional info
 *                      needed for sensor sub modules
 *
 */
struct cam_sensor_acquire_dev {
	__u32    session_handle;
	__u32    device_handle;
	__u32    handle_type;
	__u32    reserved;
	__u64    info_handle;
} __attribute__((packed));

/**
 * cam_sensor_streamon_dev : StreamOn command for the sensor
 * @session_handle :    Session handle for acquiring device
 * @device_handle  :    Updates device handle
 * @handle_type    :    Resource handle type
 * @reserved
 * @info_handle    :    Information Needed at the time of streamOn
 *
 */
struct cam_sensor_streamon_dev {
	__u32    session_handle;
	__u32    device_handle;
	__u32    handle_type;
	__u32    reserved;
	__u64    info_handle;
} __attribute__((packed));

/**
 * struct cam_irled_init : Init command for the irled
 * @irled_type  :    irled hw type
 * @reserved
 * @cmd_type    :    command buffer type
 */

struct cam_irled_init {
	__u32    irled_type;
	__u8     reserved;
	__u8     cmd_type;
	__u16    reserved1;
} __attribute__((packed));

/**
 * struct cam_flash_init : Init command for the flash
 * @flash_type  :    flash hw type
 * @reserved
 * @cmd_type    :    command buffer type
 */
struct cam_flash_init {
	__u32    flash_type;
	__u8     reserved;
	__u8     cmd_type;
	__u16    reserved1;
} __attribute__((packed));

/**
 * struct cam_flash_set_rer : RedEyeReduction command buffer
 *
 * @count             :   Number of flash leds
 * @opcode            :   Command buffer opcode
 *			CAM_FLASH_FIRE_RER
 * @cmd_type          :   command buffer operation type
 * @num_iteration     :   Number of led turn on/off sequence
 * @reserved
 * @led_on_delay_ms   :   flash led turn on time in ms
 * @led_off_delay_ms  :   flash led turn off time in ms
 * @led_current_ma    :   flash led current in ma
 *
 */
struct cam_flash_set_rer {
	__u32    count;
	__u8     opcode;
	__u8     cmd_type;
	__u16    num_iteration;
	__u32    led_on_delay_ms;
	__u32    led_off_delay_ms;
	__u32    led_current_ma[CAM_FLASH_MAX_LED_TRIGGERS];
} __attribute__((packed));

/**
 * struct cam_flash_set_on_off : led turn on/off command buffer
 *
 * @count                  : Number of Flash leds
 * @opcode                 : Command buffer opcodes
 *			     CAM_FLASH_FIRE_LOW
 *			     CAM_FLASH_FIRE_HIGH
 *			     CAM_FLASH_OFF
 * @cmd_type               : Command buffer operation type
 * @led_current_ma         : Flash led current in ma
 * @time_on_duration_ms    : Flash time on duration in ns
 *
 */
struct cam_flash_set_on_off {
	__u32    count;
	__u8     opcode;
	__u8     cmd_type;
	__u16    reserved;
	__u32    led_current_ma[CAM_FLASH_MAX_LED_TRIGGERS];
	__u64    time_on_duration_ns;
} __attribute__((packed));

/**
 * struct cam_flash_query_curr : query current command buffer
 *
 * @reserved
 * @opcode            :   command buffer opcode
 * @cmd_type          :   command buffer operation type
 * @query_current_ma  :   battery current in ma
 *
 */
struct cam_flash_query_curr {
	__u16    reserved;
	__u8     opcode;
	__u8     cmd_type;
	__u32    query_current_ma;
} __attribute__ ((packed));

/**
 * struct cam_flash_query_cap  :  capabilities info for flash
 *
 * @slot_info           :  Indicates about the slotId or cell Index
 * @max_current_flash   :  max supported current for flash
 * @max_duration_flash  :  max flash turn on duration
 * @max_current_torch   :  max supported current for torch
 * @flash_type          :  Indicates about the flash type -I2C,GPIO,PMIC
 *
 */
struct cam_flash_query_cap_info {
	__u32    slot_info;
	__u32    max_current_flash[CAM_FLASH_MAX_LED_TRIGGERS];
	__u32    max_duration_flash[CAM_FLASH_MAX_LED_TRIGGERS];
	__u32    max_current_torch[CAM_FLASH_MAX_LED_TRIGGERS];
	__u32    flash_type;
} __attribute__ ((packed));
/**
 * struct cam_ir_led_query_cap  :  capabilities info for ir_led
 *
 * @slot_info           :  Indicates about the slotId or cell Index
 *
 */

struct cam_ir_led_query_cap_info {
       uint32_t    slot_info;
} __attribute__ ((packed));

/**
 * struct cam_ir_ledset_on_off : led turn on/off command buffer
 *
 * @opcode             :   command buffer opcodes
 * @cmd_type           :   command buffer operation type
 * @ir_led_intensity   :   ir led intensity level
 * @pwm_duty_on_ns     :   PWM duty cycle in ns for IRLED intensity
 * @pwm_period_ns      :   PWM period in ns
 * @brightness         :   IRLED brightness step for I2C control
 *
 */

 struct cam_ir_led_set_on_off {
       uint8_t     opcode;
       uint8_t     cmd_type;
       uint32_t    ir_led_intensity;
       uint32_t    pwm_duty_on_ns;
       uint32_t    pwm_period_ns;
       uint8_t     brightness;
} __attribute__((packed));

/**
 * struct cam_ldm_query_cap  :  capabilities info for lens driver
 *
 * @slot_info           :  Indicates about the slotId or cell Index
 *
 */
struct cam_ldm_query_cap {
	__u32    slot_info;
	__u32    reserved;
} __attribute__ ((packed));

/**
 * struct cam_cmd_lens_driver_info - Contains lens driver slave info
 *
 * @slave_addr              :    Lens driver i2c slave address
 * @i2c_freq_mode           :    i2c frequency mode
 * @cmd_type                :    Explains type of command
 * @ldm_fw_flag             :    indicates if fw is present or not
 * @is_ldm_calib            :    indicates the calibration data is available
 * @spi_mode                :    SPI mode of communication
 * @is_always_power_on      :    opcode\
 * @is_single_byte_txfr     : True if single byte transcation required for SPI
 * @spi_freq                :    SPI frequency
 */
struct cam_cmd_lens_driver_info {
	__u32                 slave_addr;
	__u8                  i2c_freq_mode;
	__u8                  cmd_type;
	__u8                  ldm_fw_flag;
	__u8                  is_ldm_calib;
	__u8                  spi_mode;
	__u8                  is_always_power_on;
	__u8                  is_single_byte_txfr;
	__u8                  reserved;
	__u32                 spi_freq;
} __attribute__((packed));

/**
 * struct cam_cmd_ldm_fw_param - Contains LDM firmware param
 *
 * NOTE: if this struct is updated,
 * please also update version in struct cam_cmd_ldm_fw_info
 *
 * @fw_name           :       firmware file name
 * @fw_start_pos      :       data start position in file
 * @fw_size           :       firmware size
 * @fw_len_per_write  :       data length per write in bytes
 * @fw_addr_type      :       addr type
 * @fw_data_type      :       data type
 * @fw_operation      :       type of operation
 * @isOnlyWritefwData :       Only write data during fw write command
 * @fw_delayUs        :       delay in cci write
 * @fw_reg_addr       :       start register addr to write
 * @fw_init_size      :       size of fw download init settings
 * @fw_finalize_size  :       size of fw download finalize settings
 */
struct cam_cmd_ldm_fw_param {
	char        fw_name[MAX_LDM_NAME_SIZE];
	__u32       fw_start_pos;
	__u32       fw_size;
	__u32       fw_len_per_write;
	__u8        fw_addr_type;
	__u8        fw_data_type;
	__u8        fw_operation;
	__u8        isOnlyWritefwData;
	__u32       fw_delayUs;
	__u32       fw_reg_addr;
	__u32       fw_init_size;
	__u32       fw_finalize_size;
} __attribute__((packed));

/**
 * struct cam_cmd_ldm_fw_info - Contains LDM firmware info
 *
 * @version         :       version info
 *                          NOTE: if struct cam_cmd_ldm_fw_param is updated,
 *                          version here needs to be updated too.
 * @reserved        :       reserved
 * @cmd_type        :       Explains type of command
 * @fw_count        :       firmware count
 * @endianness      :       endianness combo:
 *                          bit[3:0] firmware data's endianness
 *                          bit[7:4] endian type of input parameter to ois driver, say QTime
 * @fw_param        :       includes firmware parameters
 */
struct cam_cmd_ldm_fw_info {
	__u32                           version;
	__u8                            reserved;
	__u8                            cmd_type;
	__u8                            fw_count;
	struct cam_cmd_ldm_fw_param     fw_param[MAX_LDM_FW_COUNT];
} __attribute__((packed));

#endif
