// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/dma-contiguous.h>
#include "cam_sensor_spi.h"
#include "cam_debug_util.h"

static int cam_spi_txfr(struct spi_device *spi, char *txbuf,
	char *rxbuf, int num_byte)
{
	struct spi_transfer txfr;
	struct spi_message msg;

	memset(&txfr, 0, sizeof(txfr));
	txfr.tx_buf = txbuf;
	txfr.rx_buf = rxbuf;
	txfr.len = num_byte;
	spi_message_init(&msg);
	spi_message_add_tail(&txfr, &msg);

	return spi_sync(spi, &msg);
}

static int cam_spi_txfr_read(struct spi_device *spi, char *txbuf,
	char *rxbuf, int txlen, int rxlen)
{
	struct spi_transfer tx;
	struct spi_transfer rx;
	struct spi_message m;

	memset(&tx, 0, sizeof(tx));
	memset(&rx, 0, sizeof(rx));
	tx.tx_buf = txbuf;
	rx.rx_buf = rxbuf;
	tx.len = txlen;
	rx.len = rxlen;
	spi_message_init(&m);
	spi_message_add_tail(&tx, &m);
	spi_message_add_tail(&rx, &m);
	return spi_sync(spi, &m);
}

/**
 * cam_set_addr() - helper function to set transfer address
 * @addr:	device address
 * @addr_len:	the addr field length of an instruction
 * @type:	type (i.e. byte-length) of @addr
 * @str:	shifted address output, must be zeroed when passed in
 *
 * This helper function sets @str based on the addr field length of an
 * instruction and the data length.
 */
static void cam_set_addr(uint32_t addr, uint8_t addr_len,
	enum camera_sensor_i2c_type addr_type,
	char *str)
{
	if (!addr_len)
		return;

	if (addr_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
		str[0] = addr;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
		str[0] = addr >> 8;
		str[1] = addr;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_3B) {
		str[0] = addr >> 16;
		str[1] = addr >> 8;
		str[2] = addr;
	} else {
		str[0] = addr >> 24;
		str[1] = addr >> 16;
		str[2] = addr >> 8;
		str[3] = addr;
	}
}

/**
 * cam_spi_tx_helper() - wrapper for SPI transaction
 * @client:     io client
 * @inst:       inst of this transaction
 * @addr:       device addr following the inst
 * @data:       output byte array (could be NULL)
 * @num_byte:   size of @data
 * @tx, rx:     optional transfer buffer.  It must be at least header
 *              + @num_byte long.
 *
 * This is the core function for SPI transaction, except for writes.  It first
 * checks address type, then allocates required memory for tx/rx buffers.
 * It sends out <opcode><addr>, and optionally receives @num_byte of response,
 * if @data is not NULL.  This function does not check for wait conditions,
 * and will return immediately once bus transaction finishes.
 *
 * This function will allocate buffers of header + @num_byte long.  For
 * large transfers, the allocation could fail.  External buffer @tx, @rx
 * should be passed in to bypass allocation.  The size of buffer should be
 * at least header + num_byte long.  Since buffer is managed externally,
 * @data will be ignored, and read results will be in @rx.
 * @tx, @rx also can be used for repeated transfers to improve performance.
 */
static int32_t cam_spi_tx_helper(struct camera_io_master *client,
	struct cam_camera_spi_inst *inst, uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	uint32_t num_byte, char *tx, char *rx)
{
	int32_t rc = -EINVAL;
	struct spi_device *spi = client->spi_client->spi_master;
	struct device *dev = NULL;
	char *ctx = NULL, *crx = NULL;
	uint32_t len, hlen;
	uint8_t retries = client->spi_client->retries;
	uint32_t txr = 0, rxr = 0;
	void  *vaddr_tx = NULL;
	void  *vaddr_rx = NULL;

	hlen = cam_camera_spi_get_hlen(inst);
	len = hlen + num_byte;
	dev = &(spi->dev);

	if (!dev) {
		CAM_ERR(CAM_SENSOR, "Invalid arguments");
		return -EINVAL;
	}

	if (tx) {
		ctx = tx;
	} else {
		txr = len;
		vaddr_tx = vmalloc(txr);
		if (!vaddr_tx) {
			CAM_ERR(CAM_SENSOR,
				 "Fail to allocate Memory: len: %u", txr);
			return -ENOMEM;
		}

		ctx = (char *)vaddr_tx;
	}

	if (num_byte) {
		if (rx) {
			crx = rx;
		} else {
			rxr = len;
			vaddr_rx = vmalloc(rxr);
			if (!vaddr_rx) {
				if (!tx)
					vfree(vaddr_tx);
				CAM_ERR(CAM_SENSOR,
					"Fail to allocate memory: len: %u",
					rxr);
				return -ENOMEM;
			}
			crx = (char *)vaddr_rx;
		}
	} else {
		crx = NULL;
	}

	ctx[0] = inst->opcode;
	cam_set_addr(addr, inst->addr_len, addr_type, ctx + 1);
	while ((rc = cam_spi_txfr(spi, ctx, crx, len)) && retries) {
		retries--;
		msleep(client->spi_client->retry_delay);
	}
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "failed: spi txfr rc %d", rc);
		goto out;
	}
	if (data && num_byte && !rx)
		memcpy(data, crx + hlen, num_byte);

out:
	if (!tx) {
		vfree(vaddr_tx);
		vaddr_tx = NULL;
	}
	if (!rx) {
		vfree(vaddr_rx);
		vaddr_rx = NULL;
	}
	return rc;
}

static int32_t cam_spi_tx_read(struct camera_io_master *client,
	struct cam_camera_spi_inst *inst, uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	uint32_t num_byte, char *tx, char *rx)
{
	int32_t rc = -EINVAL;
	struct spi_device *spi = client->spi_client->spi_master;
	char *ctx = NULL, *crx = NULL;
	uint32_t hlen;
	uint8_t retries = client->spi_client->retries;

	hlen = cam_camera_spi_get_hlen(inst);
	if (tx) {
		ctx = tx;
	} else {
		ctx = kzalloc(hlen, GFP_KERNEL | GFP_DMA);
		if (!ctx)
			return -ENOMEM;
	}
	if (num_byte) {
		if (rx) {
			crx = rx;
		} else {
			crx = kzalloc(num_byte, GFP_KERNEL | GFP_DMA);
			if (!crx) {
				if (!tx)
					kfree(ctx);
				return -ENOMEM;
			}
		}
	} else {
		crx = NULL;
	}

	ctx[0] = inst->opcode;
	cam_set_addr(addr, inst->addr_len, addr_type, ctx + 1);

	CAM_DBG(CAM_EEPROM, "tx(%u): %02x %02x %02x %02x", hlen, ctx[0],
		ctx[1], ctx[2],	ctx[3]);
	while ((rc = cam_spi_txfr_read(spi, ctx, crx, hlen, num_byte))
			&& retries) {
		retries--;
		msleep(client->spi_client->retry_delay);
	}
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed %d", rc);
		goto out;
	}
	if (data && num_byte && !rx)
		memcpy(data, crx, num_byte);
out:
	if (!tx)
		kfree(ctx);
	if (!rx)
		kfree(crx);
	return rc;
}

int cam_spi_read(struct camera_io_master *client,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	int rc = -EINVAL;
	uint8_t temp[CAMERA_SENSOR_I2C_TYPE_MAX];

	if (addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID
		|| addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX
		|| data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID
		|| data_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
		CAM_ERR(CAM_SENSOR, "Failed with addr/data_type verification");
		return rc;
	}

	rc = cam_spi_tx_read(client,
		&client->spi_client->cmd_tbl.read, addr, &temp[0],
		addr_type, data_type, NULL, NULL);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed %d", rc);
		return rc;
	}

	if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE)
		*data = temp[0];
	else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD)
		*data = (temp[0] << BITS_PER_BYTE) | temp[1];
	else if (data_type == CAMERA_SENSOR_I2C_TYPE_3B)
		*data = (temp[0] << 16 | temp[1] << 8 | temp[2]);
	else
		*data = (temp[0] << 24 | temp[1] << 16 | temp[2] << 8 |
			temp[3]);

	CAM_DBG(CAM_SENSOR, "addr 0x%x, data %u", addr, *data);
	return rc;
}

int32_t cam_spi_read_seq(struct camera_io_master *client,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type, int32_t num_bytes)
{
	if ((addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)) {
		CAM_ERR(CAM_SENSOR, "Failed with addr_type verification");
		return -EINVAL;
	}

	if (num_bytes == 0) {
		CAM_ERR(CAM_SENSOR, "num_byte: 0x%x", num_bytes);
		return -EINVAL;
	}

	CAM_DBG(CAM_SENSOR, "Read Seq addr: 0x%x NB:%d",
		addr, num_bytes);
	return cam_spi_tx_helper(client,
		&client->spi_client->cmd_tbl.read_seq, addr, data,
		addr_type, num_bytes, NULL, NULL);
}

int cam_spi_read_seq_v1(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_setting *read_setting)
{
	int32_t i = 0, j = 0;
	uint8_t retries = io_master_info->spi_client->retries;
	int32_t rc = -EINVAL;
	uint32_t len = 0;
	uint32_t hlen = 0;
	char *ctx = NULL, *crx = NULL;
	uint8_t *read_buff = NULL;
	uint32_t read_length = 0;
	struct cam_sensor_i2c_reg_array *reg_setting;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	struct spi_device *spi = io_master_info->spi_client->spi_master;

	if (!io_master_info || !read_setting)
		return rc;

	addr_type = read_setting->addr_type;
	data_type = read_setting->data_type;

	if ((addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (data_type >= CAMERA_SENSOR_I2C_TYPE_MAX))
		return rc;

	read_length = read_setting->read_bytes;
	if (read_length == 0) {
		CAM_ERR(CAM_SENSOR, "Invalid read buffer length");
		return rc;
	}

	hlen = addr_type + (read_setting->size * data_type);
	len = hlen + read_length;
	ctx = kzalloc(len, GFP_KERNEL);
	if (!ctx) {
		CAM_ERR(CAM_SENSOR, "ctx is NULL");
		return -ENOMEM;
	}

	crx = kzalloc(len, GFP_KERNEL);
	if (!crx) {
		CAM_ERR(CAM_SENSOR, "crx is NULL");
		if (ctx != NULL) {
			kfree(ctx);
			ctx = NULL;
		}
		return -ENOMEM;
	}
	reg_setting = read_setting->reg_setting;
	read_buff = read_setting->read_buff;

	CAM_DBG(CAM_SENSOR, "reg addr = 0x%x data type: %d",
			reg_setting->reg_addr, data_type);
	if (addr_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
		ctx[0] = reg_setting->reg_addr;
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i, ctx[i]);
		i = 1;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
		ctx[0] = reg_setting->reg_addr >> 8;
		ctx[1] = reg_setting->reg_addr;
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i, ctx[i]);
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i+1, ctx[i+1]);
		i = 2;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_3B) {
		ctx[0] = reg_setting->reg_addr >> 16;
		ctx[1] = reg_setting->reg_addr >> 8;
		ctx[2] = reg_setting->reg_addr;
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i, ctx[i]);
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i+1, ctx[i+1]);
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i+2, ctx[i+2]);
		i = 3;
	} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
		ctx[0] = reg_setting->reg_addr >> 24;
		ctx[1] = reg_setting->reg_addr >> 16;
		ctx[2] = reg_setting->reg_addr >> 8;
		ctx[3] = reg_setting->reg_addr;
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i, ctx[i]);
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i+1, ctx[i+1]);
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i+2, ctx[i+2]);
		CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", i+3, ctx[i+3]);
		i = 4;
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid I2C addr type");
		rc = -EINVAL;
		goto free_res;
	}

	for (j = 0; j < read_setting->size; j++) {
		if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
			ctx[i] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i, ctx[i]);
			i += 1;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
			ctx[i] = reg_setting->reg_data >> 8;
			ctx[i+1] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i, ctx[i]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i+1, ctx[i+1]);
			i += 2;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_3B) {
			ctx[i] = reg_setting->reg_data >> 16;
			ctx[i + 1] = reg_setting->reg_data >> 8;
			ctx[i + 2] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i, ctx[i]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i+1, ctx[i+1]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i+2, ctx[i+2]);
			i += 3;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
			ctx[i] = reg_setting->reg_data >> 24;
			ctx[i + 1] = reg_setting->reg_data >> 16;
			ctx[i + 2] = reg_setting->reg_data >> 8;
			ctx[i + 3] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i, ctx[i]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i+1, ctx[i+1]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i+2, ctx[i+2]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", i+3, ctx[i+3]);
			i += 4;
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid Data Type");
			rc = -EINVAL;
			goto free_res;
		}
		reg_setting++;
	}

	if (io_master_info->spi_client->is_single_byte_txfr == 1) {
		CAM_DBG(CAM_SENSOR, "single_byte_txfr enable");
		for (i = 0; i < len; i++) {
			while ((rc = cam_spi_txfr(spi, &ctx[i], &crx[i], 1)) && retries) {
				msleep(io_master_info->spi_client->retry_delay);
				retries--;
			}
		}
	} else {
		while ((rc = cam_spi_txfr(spi, ctx, crx, len)) && retries) {
			msleep(io_master_info->spi_client->retry_delay);
			retries--;
		}
	}

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed rc: %d", rc);
		goto free_res;
	}

	if (read_buff && read_length)
		memcpy(read_buff, crx + hlen, read_length);

free_res:
	if (!ctx)
		kfree(ctx);

	if (!crx)
		kfree(crx);

	return rc;
}

int cam_spi_query_id(struct camera_io_master *client,
	uint32_t addr, enum camera_sensor_i2c_type addr_type,
	uint8_t *data, uint32_t num_byte)
{
	CAM_DBG(CAM_SENSOR, "SPI Queryid : 0x%x, addr: 0x%x",
		client->spi_client->cmd_tbl.query_id, addr);
	return cam_spi_tx_helper(client,
		&client->spi_client->cmd_tbl.query_id,
		addr, data, addr_type, num_byte, NULL, NULL);
}

static int32_t cam_spi_read_status_reg(
	struct camera_io_master *client, uint8_t *status,
	enum camera_sensor_i2c_type addr_type)
{
	struct cam_camera_spi_inst *rs =
		&client->spi_client->cmd_tbl.read_status;

	if (rs->addr_len != 0) {
		CAM_ERR(CAM_SENSOR, "not implemented yet");
		return -ENXIO;
	}
	return cam_spi_tx_helper(client, rs, 0, status,
		addr_type, 1, NULL, NULL);
}

static int32_t cam_spi_device_busy(struct camera_io_master *client,
	uint8_t *busy, enum camera_sensor_i2c_type addr_type)
{
	int rc;
	uint8_t st = 0;

	rc = cam_spi_read_status_reg(client, &st, addr_type);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed to read status reg");
		return rc;
	}
	*busy = st & client->spi_client->busy_mask;
	return 0;
}

static int32_t cam_spi_wait(struct camera_io_master *client,
	struct cam_camera_spi_inst *inst,
	enum camera_sensor_i2c_type addr_type)
{
	uint8_t busy;
	int i, rc;

	CAM_DBG(CAM_SENSOR, "op 0x%x wait start", inst->opcode);
	for (i = 0; i < inst->delay_count; i++) {
		rc = cam_spi_device_busy(client, &busy, addr_type);
		if (rc < 0)
			return rc;
		if (!busy)
			break;
		msleep(inst->delay_intv);
		CAM_DBG(CAM_SENSOR, "op 0x%x wait", inst->opcode);
	}
	if (i > inst->delay_count) {
		CAM_ERR(CAM_SENSOR, "op %x timed out", inst->opcode);
		return -ETIMEDOUT;
	}
	CAM_DBG(CAM_SENSOR, "op %x finished", inst->opcode);
	return 0;
}

static int32_t cam_spi_write_enable(struct camera_io_master *client,
	enum camera_sensor_i2c_type addr_type)
{
	struct cam_camera_spi_inst *we =
		&client->spi_client->cmd_tbl.write_enable;
	int rc;

	if (we->opcode == 0)
		return 0;
	if (we->addr_len != 0) {
		CAM_ERR(CAM_SENSOR, "not implemented yet");
		return -EINVAL;
	}
	rc = cam_spi_tx_helper(client, we, 0, NULL, addr_type,
		0, NULL, NULL);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "write enable failed");
	return rc;
}

/**
 * cam_spi_page_program() - core function to perform write
 * @client: need for obtaining SPI device
 * @addr: address to program on device
 * @data: data to write
 * @len: size of data
 * @tx: tx buffer, size >= header + len
 *
 * This function performs SPI write, and has no boundary check.  Writing range
 * should not cross page boundary, or data will be corrupted.  Transaction is
 * guaranteed to be finished when it returns.  This function should never be
 * used outside cam_spi_write_seq().
 */
static int32_t cam_spi_page_program(struct camera_io_master *client,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	uint16_t len, uint8_t *tx)
{
	int rc;
	struct cam_camera_spi_inst *pg =
		&client->spi_client->cmd_tbl.page_program;
	struct spi_device *spi = client->spi_client->spi_master;
	uint8_t retries = client->spi_client->retries;
	uint8_t header_len = sizeof(pg->opcode) + pg->addr_len + pg->dummy_len;

	CAM_DBG(CAM_SENSOR, "addr 0x%x, size 0x%x", addr, len);
	rc = cam_spi_write_enable(client, addr_type);
	if (rc < 0)
		return rc;
	memset(tx, 0, header_len);
	tx[0] = pg->opcode;
	cam_set_addr(addr, pg->addr_len, addr_type, tx + 1);
	memcpy(tx + header_len, data, len);
	CAM_DBG(CAM_SENSOR, "tx(%u): %02x %02x %02x %02x",
		len, tx[0], tx[1], tx[2], tx[3]);
	while ((rc = spi_write(spi, tx, len + header_len)) && retries) {
		rc = cam_spi_wait(client, pg, addr_type);
		msleep(client->spi_client->retry_delay);
		retries--;
	}
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed %d", rc);
		return rc;
	}
	rc = cam_spi_wait(client, pg, addr_type);
		return rc;
}

int cam_spi_write(struct camera_io_master *client,
	uint32_t addr, uint32_t data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	struct cam_camera_spi_inst *pg =
		&client->spi_client->cmd_tbl.page_program;
	uint8_t header_len = sizeof(pg->opcode) + pg->addr_len + pg->dummy_len;
	uint16_t len = 0;
	char buf[CAMERA_SENSOR_I2C_TYPE_MAX];
	char *tx;
	int rc = -EINVAL;

	if ((addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (data_type != CAMERA_SENSOR_I2C_TYPE_MAX))
		return rc;

	CAM_DBG(CAM_EEPROM, "Data: 0x%x", data);
	len = header_len + (uint8_t)data_type;
	tx = kmalloc(len, GFP_KERNEL | GFP_DMA);
	if (!tx)
		goto NOMEM;

	if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
		buf[0] = data;
		CAM_DBG(CAM_EEPROM, "Byte %d: 0x%x", len, buf[0]);
	} else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
		buf[0] = (data >> BITS_PER_BYTE) & 0x00FF;
		buf[1] = (data & 0x00FF);
	} else if (data_type == CAMERA_SENSOR_I2C_TYPE_3B) {
		buf[0] = (data >> 16) & 0x00FF;
		buf[1] = (data >> 8) & 0x00FF;
		buf[2] = (data & 0x00FF);
	} else {
		buf[0] = (data >> 24) & 0x00FF;
		buf[1] = (data >> 16) & 0x00FF;
		buf[2] = (data >> 8) & 0x00FF;
		buf[3] = (data & 0x00FF);
	}

	rc = cam_spi_page_program(client, addr, buf,
		addr_type, data_type, tx);
	if (rc < 0)
		goto ERROR;
	goto OUT;
NOMEM:
	CAM_ERR(CAM_SENSOR, "memory allocation failed");
	return -ENOMEM;
ERROR:
	CAM_ERR(CAM_SENSOR, "error write");
OUT:
	kfree(tx);
	return rc;
}

int32_t cam_spi_write_seq(struct camera_io_master *client,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type, uint32_t num_byte)
{
	struct cam_camera_spi_inst *pg =
		&client->spi_client->cmd_tbl.page_program;
	const uint32_t page_size = client->spi_client->page_size;
	uint8_t header_len = sizeof(pg->opcode) + pg->addr_len + pg->dummy_len;
	uint16_t len;
	uint32_t cur_len, end;
	char *tx, *pdata = data;
	int rc = -EINVAL;

	if ((addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX) ||
		(addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID))
		return rc;
    /* single page write */
	if ((addr % page_size) + num_byte <= page_size) {
		len = header_len + num_byte;
		tx = kmalloc(len, GFP_KERNEL | GFP_DMA);
		if (!tx)
			goto NOMEM;
		rc = cam_spi_page_program(client, addr, data, addr_type,
			num_byte, tx);
		if (rc < 0)
			goto ERROR;
		goto OUT;
	}
	/* multi page write */
	len = header_len + page_size;
	tx = kmalloc(len, GFP_KERNEL | GFP_DMA);
	if (!tx)
		goto NOMEM;
	while (num_byte) {
		end = min(page_size, (addr % page_size) + num_byte);
		cur_len = end - (addr % page_size);
		CAM_ERR(CAM_SENSOR, "Addr: 0x%x curr_len: 0x%x pgSize: %d",
			addr, cur_len, page_size);
		rc = cam_spi_page_program(client, addr, pdata, addr_type,
			cur_len, tx);
		if (rc < 0)
			goto ERROR;
		addr += cur_len;
		pdata += cur_len;
		num_byte -= cur_len;
	}
	goto OUT;
NOMEM:
	pr_err("%s: memory allocation failed\n", __func__);
	return -ENOMEM;
ERROR:
	pr_err("%s: error write\n", __func__);
OUT:
	kfree(tx);
	return rc;
}

static int cam_spi_write_v1(struct camera_io_master *client,
	uint32_t addr, uint32_t data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	uint8_t len = 0;
	uint8_t i = 0;
	uint8_t retries = client->spi_client->retries;
	char *tx;
	int rc = -EINVAL;
	struct spi_device *spi = client->spi_client->spi_master;

	if ((addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (data_type != CAMERA_SENSOR_I2C_TYPE_MAX))
		return rc;

	CAM_DBG(CAM_SENSOR, "Addr: 0x%x Data: 0x%x", addr, data);
	len = (uint8_t)addr_type + (uint8_t)data_type;
	tx = kzalloc(len, GFP_KERNEL);
	if (!tx)
		goto NOMEM;

	cam_set_addr(addr, addr_type, addr_type, tx);
	memcpy(tx + addr_type, &data, data_type);

	if (client->spi_client->is_single_byte_txfr == 1) {
		for (i = 0; i < len; i++) {
			while ((rc = spi_write(spi, tx + i, 1)) && retries) {
				msleep(client->spi_client->retry_delay);
				retries--;
			}
		}
	} else {
		while ((rc = spi_write(spi, tx, len)) && retries) {
			msleep(client->spi_client->retry_delay);
			retries--;
		}
	}

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed %d", rc);
		goto OUT;
	}

NOMEM:
	CAM_ERR(CAM_SENSOR, "memory allocation failed");
	return -ENOMEM;

OUT:
	kfree(tx);
	return rc;
}

int cam_spi_write_table(struct camera_io_master *client,
	struct cam_sensor_i2c_reg_setting *write_setting)
{
	int i;
	int rc = -EFAULT;
	struct cam_sensor_i2c_reg_array *reg_setting;
	uint16_t client_addr_type;
	enum camera_sensor_i2c_type addr_type;
	struct spi_device *spi = NULL;

	if (!client || !write_setting)
		return rc;

	if ((write_setting->addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (write_setting->addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (write_setting->data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (write_setting->data_type >= CAMERA_SENSOR_I2C_TYPE_MAX))
		return rc;

	reg_setting = write_setting->reg_setting;
	client_addr_type = write_setting->addr_type;
	addr_type = write_setting->addr_type;
	for (i = 0; i < write_setting->size; i++) {
		CAM_DBG(CAM_SENSOR, "addr %x data %x",
			reg_setting->reg_addr, reg_setting->reg_data);
		if (client->spi_client->spi_communication_ver == SPI_COMM_VERSION_1) {
			if (write_setting->frequency != 0 &&
				(write_setting->frequency != client->spi_client->frequency)) {
				spi = client->spi_client->spi_master;
				spi->max_speed_hz = write_setting->frequency;
				spi_setup(spi);
				client->spi_client->frequency = spi->max_speed_hz;
			}
			rc = cam_spi_write_v1(client,
				reg_setting->reg_addr, reg_setting->reg_data,
				write_setting->addr_type, write_setting->data_type);
		} else {
			rc = cam_spi_write(client,
				reg_setting->reg_addr, reg_setting->reg_data,
				write_setting->addr_type, write_setting->data_type);
		}
		if (rc < 0)
			break;
		reg_setting++;
	}
		if (write_setting->delay > 20)
			msleep(write_setting->delay);
		else if (write_setting->delay)
			usleep_range(write_setting->delay * 1000,
			(write_setting->delay
			* 1000) + 1000);
	addr_type = client_addr_type;
	return rc;
}

static cam_spi_write_seq_v1(struct camera_io_master *client,
	struct cam_sensor_i2c_reg_setting *write_setting)
{
	int i;
	int32_t rc = 0;
	struct cam_sensor_i2c_reg_array *reg_setting;

	reg_setting = write_setting->reg_setting;
	for (i = 0; i < write_setting->size; i++) {
		reg_setting->reg_addr += i;
		rc = cam_spi_write_v1(client,
			reg_setting->reg_addr, reg_setting->reg_data,
			write_setting->addr_type, write_setting->data_type);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Sequential spi write failed: rc: %d", rc);
			break;
		}
		reg_setting++;
	}

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed rc: %d", rc);
	} else {
		if (write_setting->delay > 20)
			msleep(write_setting->delay);
		else if (write_setting->delay)
			usleep_range(write_setting->delay * 1000,
			(write_setting->delay * 1000) + 1000);
	}

	return rc;
}

static cam_spi_write_burst(struct camera_io_master *client,
	struct cam_sensor_i2c_reg_setting *write_setting)
{
	int i;
	uint8_t retries = client->spi_client->retries;
	int32_t rc = 0;
	uint32_t len = 0;
	char *buf = NULL;
	struct cam_sensor_i2c_reg_array *reg_setting;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	struct spi_device *spi = client->spi_client->spi_master;

	buf = kzalloc((write_setting->addr_type +
			(write_setting->size * write_setting->data_type)),
			GFP_KERNEL);

	if (!buf) {
		CAM_ERR(CAM_SENSOR, "BUF is NULL");
		return -ENOMEM;
	}

	reg_setting = write_setting->reg_setting;
	addr_type = write_setting->addr_type;
	data_type = write_setting->data_type;

	if (write_setting->write_only_data != 1) {
		if (addr_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
			buf[0] = reg_setting->reg_addr;
			CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", len, buf[len]);
			len = 1;
		} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
			buf[0] = reg_setting->reg_addr >> 8;
			buf[1] = reg_setting->reg_addr;
			CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", len, buf[len]);
			CAM_DBG(CAM_SENSOR, "byte %d: 0x%x", len+1, buf[len+1]);
			len = 2;
		} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_3B) {
			buf[0] = reg_setting->reg_addr >> 16;
			buf[1] = reg_setting->reg_addr >> 8;
			buf[2] = reg_setting->reg_addr;
			len = 3;
		} else if (addr_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
			buf[0] = reg_setting->reg_addr >> 24;
			buf[1] = reg_setting->reg_addr >> 16;
			buf[2] = reg_setting->reg_addr >> 8;
			buf[3] = reg_setting->reg_addr;
			len = 4;
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid I2C addr type");
			rc = -EINVAL;
			goto free_res;
		}
	}
	for (i = 0; i < write_setting->size; i++) {
		if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
			buf[len] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len, buf[len]);
			len += 1;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
			buf[len] = reg_setting->reg_data >> 8;
			buf[len+1] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len, buf[len]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len+1, buf[len+1]);
			len += 2;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_3B) {
			buf[len] = reg_setting->reg_data >> 16;
			buf[len + 1] = reg_setting->reg_data >> 8;
			buf[len + 2] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len, buf[len]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len+1, buf[len+1]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len+2, buf[len+2]);
			len += 3;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
			buf[len] = reg_setting->reg_data >> 24;
			buf[len + 1] = reg_setting->reg_data >> 16;
			buf[len + 2] = reg_setting->reg_data >> 8;
			buf[len + 3] = reg_setting->reg_data;
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len, buf[len]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len+1, buf[len+1]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len+2, buf[len+2]);
			CAM_DBG(CAM_SENSOR,
				"Byte %d: 0x%x", len+3, buf[len+3]);
			len += 4;
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid Data Type");
			rc = -EINVAL;
			goto free_res;
		}
		reg_setting++;
	}

	if (len > (write_setting->addr_type +
		(write_setting->size * write_setting->data_type))) {
		CAM_ERR(CAM_SENSOR, "Invalid Length: %u | Expected length: %u",
			len, (write_setting->addr_type +
			(write_setting->size * write_setting->data_type)));
		rc = -EINVAL;
		goto free_res;
	}

	if (client->spi_client->is_single_byte_txfr == 1) {
		for (i = 0; i < len; i++) {
			while ((rc = spi_write(spi, buf + i, 1)) && retries) {
				msleep(client->spi_client->retry_delay);
				retries--;
			}

		}
	} else {
		while ((rc = spi_write(spi, buf, len)) && retries) {
			msleep(client->spi_client->retry_delay);
			retries--;
		}
	}

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed rc: %d", rc);
	} else {
		if (write_setting->delay > 20)
			msleep(write_setting->delay);
		else if (write_setting->delay)
			usleep_range(write_setting->delay * 1000,
			(write_setting->delay * 1000) + 1000);
	}
free_res:
	kfree(buf);
	return rc;
}

int cam_spi_write_continuous_table(struct camera_io_master *client,
	struct cam_sensor_i2c_reg_setting *write_setting,
	uint8_t cam_sensor_i2c_write_flag)
{
	int rc = -EINVAL;
	struct spi_device *spi = NULL;

	if (!client || !write_setting)
		return rc;

	if ((write_setting->addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (write_setting->addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX)
		|| (write_setting->data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID)
		|| (write_setting->data_type >= CAMERA_SENSOR_I2C_TYPE_MAX))
		return rc;

	if (write_setting->frequency != 0 &&
		(write_setting->frequency != client->spi_client->frequency)) {
		spi = client->spi_client->spi_master;
		spi->max_speed_hz = write_setting->frequency;
		spi_setup(spi);
		client->spi_client->frequency = spi->max_speed_hz;
		CAM_DBG(CAM_SENSOR, "%s: update freq %d\n",
			__func__, write_setting->frequency);
	}

	if (cam_sensor_i2c_write_flag == CAM_SENSOR_I2C_WRITE_BURST)
		rc = cam_spi_write_burst(client, write_setting);
	else if (cam_sensor_i2c_write_flag == CAM_SENSOR_I2C_WRITE_SEQ)
		rc = cam_spi_write_seq_v1(client, write_setting);

	return rc;
}

int cam_spi_erase(struct camera_io_master *client, uint32_t addr,
	enum camera_sensor_i2c_type addr_type, uint32_t size)
{
	struct cam_camera_spi_inst *se = &client->spi_client->cmd_tbl.erase;
	int rc = 0;
	uint32_t cur;
	uint32_t end = addr + size;
	uint32_t erase_size = client->spi_client->erase_size;

	end = addr + size;
	for (cur = rounddown(addr, erase_size); cur < end; cur += erase_size) {
		CAM_ERR(CAM_SENSOR, "%s: erasing 0x%x size: %d\n",
			__func__, cur, erase_size);
		rc = cam_spi_write_enable(client, addr_type);
		if (rc < 0)
			return rc;
		rc = cam_spi_tx_helper(client, se, cur, NULL, addr_type, 0,
			NULL, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "%s: erase failed\n", __func__);
			return rc;
		}
		rc = cam_spi_wait(client, se, addr_type);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "%s: erase timedout\n", __func__);
			return rc;
		}
	}

	return rc;
}
