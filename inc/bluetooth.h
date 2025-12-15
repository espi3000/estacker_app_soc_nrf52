#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
//#include <sys/printk.h>
//#include <sys/byteorder.h>
//#include <zephyr.h>
#include <soc.h>

/*
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/addr.h>
#include <bluetooth/gatt.h>
*/

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>

#define MY_SERVICE_UUID 0xd4, 0x86, 0x48, 0x24, 0x54, 0xB3, 0x43, 0xA1, \
			            0xBC, 0x20, 0x97, 0x8F, 0xC3, 0x76, 0xC2, 0x75

#define RX_CHARACTERISTIC_UUID  0xA6, 0xE8, 0xC4, 0x60, 0x7E, 0xAA, 0x41, 0x6B, \
			                    0x95, 0xD4, 0x9D, 0xCC, 0x08, 0x4F, 0xCF, 0x6A

#define TX_CHARACTERISTIC_UUID  0xED, 0xAA, 0x20, 0x11, 0x92, 0xE7, 0x43, 0x5A, \
			                    0xAA, 0xE9, 0x94, 0x43, 0x35, 0x6A, 0xD4, 0xD3

/** @brief Callback type for when new data is received. */
typedef void (*data_rx_cb_t)(uint8_t *data, uint8_t length);

/** @brief Callback struct used by the my_service Service. */
struct my_service_cb 
{
	/** Data received callback. */
	data_rx_cb_t    data_rx_cb;
};

int bt_init(void);

void bt_transmit(struct bt_conn *conn, const uint8_t *data, size_t len);

void bt_receive(struct bt_conn *conn, const uint8_t *data);

void bt_advertise(int err);

int communicate_samples(uint16_t *samples, size_t num_samples);

#endif // _BLUETOOTH_H_