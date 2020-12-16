#include "dfu/dfu_target_peripheral.h"

#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <logging/log.h>

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <cJSON.h>
#include <cJSON_os.h>
#include <sys/base64.h>

LOG_MODULE_REGISTER(dfu_target_peripheral, CONFIG_DFU_TARGET_LOG_LEVEL);

#define DFU_MAGIC_LEN (8u)
#define DFU_MAGIC "XoPU24Tk"

#define DFU_PACKET_HEADER_LEN (DFU_MAGIC_LEN + 6u)
#define DFU_HEADER_FMT (DFU_MAGIC " %04d %s\n")

#define DFU_MAX_CHUNK_LEN (128u)

#define DFU_JSON_MAX_B64_DATA_LEN (4u * (DFU_MAX_CHUNK_LEN / 3u) + 20u)
#define DFU_JSON_MAX_LEN (DFU_JSON_MAX_B64_DATA_LEN + 100u)

#define DFU_PACKET_MAX_BUFFER_LEN                                              \
	(DFU_PACKET_HEADER_LEN + DFU_JSON_MAX_LEN + 2u)

#define CMD_START_DFU (7u)
#define CMD_RESTART_DFU (8u)
#define CMD_DFU_DATA (9u)

/* Local buffers */
static char json_str[DFU_JSON_MAX_LEN];
static uint8_t b64_buffer[DFU_JSON_MAX_B64_DATA_LEN];
static uint8_t pkt_buffer[DFU_PACKET_MAX_BUFFER_LEN];
static size_t pkt_buffer_len;

/* DFU state */
static size_t dfu_chunk_size = DFU_MAX_CHUNK_LEN;
static size_t dfu_msg_count;
static size_t dfu_image_len;
static size_t dfu_next_msg_id;
static size_t dfu_bytes_sent;
static dfu_target_callback_t dfu_callback;

static uint8_t chunk_buffer[DFU_MAX_CHUNK_LEN];
static size_t chunk_buffer_len;

/* UART driver */
static const struct device *uart_dev;

static volatile bool rx_processing_enabled;
K_SEM_DEFINE(response_sem, 0, 1);

/*
 * Packet structure:
 *     magic str "XoPU24Tk"
 *     \x20 'sp'
 *     JSON length (4 character string)
 *     \x20 'sp'
 *     JSON data
 *     \x0a '\n'
 */
static int send_pkt(const uint8_t *json_data, size_t len)
{
	int olen = snprintf((char *)pkt_buffer, sizeof(pkt_buffer) - 1,
			    DFU_HEADER_FMT, (int)len, (const char *)json_data);

	if (olen >= (sizeof(pkt_buffer) - 1)) {
		return -ENOMEM;
	}

	for (size_t i = 0; i < olen; i++) {
		uart_poll_out(uart_dev, pkt_buffer[i]);
	}

	return 0;
}

/*
 * {
 *     "cmd":CMD_START_DFU,
 *     "file_size":250000,     // bytes
 *     "max_chunk_size":2048,  // bytes
 *     "version":"1.0.0",      // string
 *     "total_msg_count":245   // Number of messages expected
 * }
 */
static int send_dfu_start_pkt(size_t file_size, size_t max_chunk_size,
			      uint32_t app_version, size_t total_msg_count)
{
	cJSON *obj = cJSON_CreateObject();

	cJSON_AddNumberToObject(obj, "cmd", CMD_START_DFU);
	cJSON_AddNumberToObject(obj, "file_size", file_size);
	cJSON_AddNumberToObject(obj, "max_chunk_size", max_chunk_size);
	cJSON_AddStringToObject(obj, "version", "1.0.0.0");
	cJSON_AddNumberToObject(obj, "total_msg_count", total_msg_count);

	if (!cJSON_PrintPreallocated(obj, json_str, sizeof(json_str), false)) {
		cJSON_Delete(obj);
		return -ENOMEM;
	}

	cJSON_Delete(obj);
	return send_pkt((const uint8_t *)json_str, strlen(json_str));
}

/*
 * {
 *     "cmd":CMD_DFU_DATA,
 *     "msg_id":145,               // ID of the message
 *     "chunk_size":2048,          // bytes
 *     "data":"BASE64ENCODEDDATA"  // String
 * }
 */
static int send_dfu_data_pkt(size_t message_id, size_t chunk_size,
			     const uint8_t *data, size_t data_len)
{
	cJSON *obj = cJSON_CreateObject();

	cJSON_AddNumberToObject(obj, "cmd", CMD_DFU_DATA);
	cJSON_AddNumberToObject(obj, "msg_id", message_id);
	cJSON_AddNumberToObject(obj, "chunk_size", chunk_size);

	{
		size_t olen = 0;
		int ret = base64_encode(b64_buffer, sizeof(b64_buffer), &olen,
					data, data_len);
		if (ret != 0) {
			LOG_ERR("***Failed to b64 encode data");
			LOG_ERR("***B64 needed %d, have %d", (int)olen,
				(int)sizeof(b64_buffer));
			cJSON_Delete(obj);
			return -ENOMEM;
		}

		cJSON_AddStringToObject(obj, "data", (const char *)b64_buffer);
	}

	if (!cJSON_PrintPreallocated(obj, json_str, sizeof(json_str), false)) {
		LOG_ERR("Failed to serialize JSON");
		cJSON_Delete(obj);
		return -ENOMEM;
	}

	cJSON_Delete(obj);
	return send_pkt((const uint8_t *)json_str, strlen(json_str));
}

static inline void start_rx_processing(void)
{
	/* Let the UART handler start looking for a packet*/
	k_sem_reset(&response_sem);
	pkt_buffer_len = 0;
	rx_processing_enabled = true;
}

static int wait_for_dfu_resp(k_timeout_t timeout)
{
	int rc = -1;

	/*
	 * Wait for the UART handler to receive a full message into pkt_buffer
	 */
	if (k_sem_take(&response_sem, timeout) == 0) {
		/*
		 * We don't need to look in the message. Since we got a
		 * response, we know the peripheral had no issues with the
		 * message was sent
		 */

		pkt_buffer_len = 0;
		rc = 0;
	}

	rx_processing_enabled = false;
	return rc;
}

static inline void trim_pkt_buffer(size_t idx)
{
	if (idx >= pkt_buffer_len) {
		pkt_buffer_len = 0u;
	} else if (idx > 0) {
		pkt_buffer_len = pkt_buffer_len - idx;
		memmove(pkt_buffer, pkt_buffer + idx, pkt_buffer_len);
	}
}

static inline void uart_rx_handler(uint8_t character)
{
	/* Don't process the character if we are not expecting a response */
	if (!rx_processing_enabled) {
		return;
	}

	pkt_buffer[pkt_buffer_len] = character;
	pkt_buffer_len += 1;

	do {
		/* Find the header, throw away other characters */
		if (pkt_buffer_len < DFU_PACKET_HEADER_LEN) {
			break;
		}

		bool has_header = false;
		do {
			has_header = (memcmp(pkt_buffer, DFU_MAGIC,
						  DFU_MAGIC_LEN) == 0);
			if (!has_header) {
				trim_pkt_buffer(1);
			}
		} while ((!has_header) &&
			 (pkt_buffer_len >= DFU_PACKET_HEADER_LEN));

		if (!has_header) {
			break;
		}

		/* We have a header, read out the length */
		size_t json_len = 0u;
		{
			char len_str[5];
			memcpy(len_str, &pkt_buffer[DFU_MAGIC_LEN + 1], 4);
			len_str[4] = '\0';

			long raw_len = strtol(len_str, NULL, 10);
			if ((raw_len == 0) || (raw_len > DFU_JSON_MAX_LEN)) {
				/*
				 * Invalid length field, thrown the
				 * mysteriously matching magic string
				 */
				trim_pkt_buffer(DFU_MAGIC_LEN);
				continue;
			}
			json_len = (size_t)raw_len;
		}

		size_t pkt_len = json_len + DFU_PACKET_HEADER_LEN;
		if (pkt_buffer_len < pkt_len) {
			/* Not enough bytes to extract the JSON data */
			break;
		}

		/*
		 * Complete response has been received. Trim the buffer to
		 * length and let the response parser know
		 */
		pkt_buffer_len = pkt_len;
		k_sem_give(&response_sem);

	} while (0);
}

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	uint8_t character;

	while (uart_fifo_read(dev, &character, 1)) {
		uart_rx_handler(character);
	}
}

int dfu_target_peripheral_init(size_t file_size, dfu_target_callback_t cb)
{
	LOG_INF("Starting peripheral DFU");

	dfu_chunk_size = DFU_MAX_CHUNK_LEN;

	dfu_msg_count = (file_size / dfu_chunk_size);
	if ((file_size % dfu_chunk_size) > 0) {
		dfu_msg_count += 1u;
	}

	dfu_image_len = file_size;

	dfu_next_msg_id = 0u;
	dfu_bytes_sent = 0u;

	dfu_callback = cb;

	chunk_buffer_len = 0u;

	/* Send the start packet */
	start_rx_processing();
	int rc = send_dfu_start_pkt(dfu_image_len, dfu_chunk_size, 0,
				    dfu_msg_count);

	if (rc != 0) {
		LOG_ERR("Failed to send start packet");
		return rc;
	}

	if (wait_for_dfu_resp(K_MSEC(5000)) != 0) {
		LOG_ERR("No response to CMD_START_DFU");
		dfu_callback(DFU_TARGET_EVT_TIMEOUT);
		return -EIO;
	}

	dfu_callback(DFU_TARGET_EVT_ERASE_DONE);
	return 0;
}

int dfu_target_peripheral_offset_get(size_t *offset)
{
	*offset = dfu_bytes_sent + chunk_buffer_len;
	return 0;
}

int dfu_target_peripheral_write(const void *const buf, size_t len)
{
	const uint8_t *buf_ptr = buf;

	do {
		size_t bytes_to_copy = dfu_chunk_size - chunk_buffer_len;
		if (len < bytes_to_copy) {
			bytes_to_copy = len;
		}
		memcpy(chunk_buffer + chunk_buffer_len, buf_ptr,
			bytes_to_copy);
		chunk_buffer_len += bytes_to_copy;
		buf_ptr += bytes_to_copy;
		len -= bytes_to_copy;

		bool is_last =
			(dfu_image_len <= (dfu_bytes_sent + chunk_buffer_len));

		if (is_last || (chunk_buffer_len >= dfu_chunk_size)) {
			LOG_INF("Sending [%d:%d] of %d bytes",
				(int)dfu_bytes_sent,
				(int)dfu_bytes_sent + chunk_buffer_len - 1,
				(int)dfu_image_len);

			start_rx_processing();
			int rc = send_dfu_data_pkt(dfu_next_msg_id,
						   dfu_chunk_size,
						   chunk_buffer,
						   chunk_buffer_len);
			dfu_bytes_sent += chunk_buffer_len;
			chunk_buffer_len = 0;

			if (rc != 0) {
				LOG_ERR("Failed to send DFU data");
				return rc;
			}

			if (is_last) {
				/*
				 * If this was the last message (chunk size
				 * aligned with the image size), then no
				 * response will be sent by the peripheral
				 * device
				 */
				LOG_INF("Last packet sent");
			} else if (wait_for_dfu_resp(K_MSEC(1000)) != 0) {
				LOG_ERR("No response to CMD_DFU_DATA");
				dfu_callback(DFU_TARGET_EVT_TIMEOUT);
				return -EIO;
			}

			dfu_next_msg_id += 1u;
		}

	} while (len > 0);

	return 0;
}

int dfu_target_peripheral_done(bool successful)
{
	int rc = 0;

	if (!successful) {
		/* Don't bother flushing the chunk buffer */
		LOG_INF("Peripheral DFU done, unsuccessful");
	} else {
		if (chunk_buffer_len > 0) {
			/* Final packet */
			LOG_INF("Peripheral DFU sending last packet");

			rc = send_dfu_data_pkt(dfu_next_msg_id, dfu_chunk_size,
					       chunk_buffer, chunk_buffer_len);
			if (rc != 0) {
				LOG_ERR("Failed to send final packet");
				dfu_callback(DFU_TARGET_EVT_TIMEOUT);
				return rc;
			}

			/* There is no final response message */
			k_sleep(K_MSEC(1000));

			dfu_next_msg_id += 1u;
			dfu_bytes_sent += chunk_buffer_len;
			chunk_buffer_len = 0;
		}

		if (dfu_bytes_sent < dfu_image_len) {
			LOG_ERR("Peripheral DFU was not given all the "
				"image data!");
		} else {
			LOG_INF("Peripheral DFU done, all data sent");
		}
	}

	dfu_msg_count = 0;
	dfu_image_len = 0;
	dfu_next_msg_id = 0;
	dfu_bytes_sent = 0;
	dfu_callback = NULL;

	return rc;
}

static int dfu_uart_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	uart_dev = device_get_binding(CONFIG_DFU_TARGET_PERIPHERAL_UART_NAME);
	if (uart_dev == NULL) {
		LOG_ERR("Cannot bind %s\n",
			CONFIG_DFU_TARGET_PERIPHERAL_UART_NAME);
		return -EINVAL;
	}

	int err;
	uint8_t dummy;
	uint32_t start_time = k_uptime_get_32();

	/* Wait for the UART line to become valid */
	do {
		err = uart_err_check(uart_dev);
		if (err) {
			if (k_uptime_get_32() - start_time > 500) {
				LOG_ERR("UART check failed: %d. "
					"UART initialization timed out.", err);
				return -EIO;
			}

			LOG_INF("UART check failed: %d. "
				"Dropping buffer and retrying.", err);

			while (uart_fifo_read(uart_dev, &dummy, 1)) {
				/* Do nothing with the data */
			}
			k_sleep(K_MSEC(10));
		}
	} while (err);

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);
	return 0;
}

SYS_INIT(dfu_uart_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
