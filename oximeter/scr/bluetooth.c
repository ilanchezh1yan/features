/**
 * @file bluetooth.c
 * @brief Bluetooth Low Energy communication module.
 *
 * Implements BLE services, characteristics, advertising, connection
 * management, and transmission of physiological measurement data
 * to external devices.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>

#include "bluetooth.h"

#define MIN_CONNECTION_INTERVAL 6     /*1.25 ms connection interval */
#define MAX_CONNECTION_INTERVAL 12     /*1.25ms connection interval */
#define SUPERVISOR_TIMEOUT 400          /* 4s connection timeout */

static struct bt_uuid_128 custom_Read_service_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                              0x00, 0x10, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00);  

static struct bt_uuid_128 live_graph_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe1, 0xff, 0x00, 0x00);    
static uint8_t data_packet[12]; 

volatile uint8_t BT_notify_enable;                                                       

/**
 * @brief Handle BLE notification configuration changes.
 *
 * Updates the notification status when a client enables or disables
 * notifications for the live data characteristic.
 *
 * @param attr Pointer to the characteristic attribute.
 * @param value New Client Characteristic Configuration value.
 *
 * @return None.
 */
void CCC_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
   BT_notify_enable = value;
}

BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_Read_service_uuid),
    BT_GATT_CHARACTERISTIC(&live_graph_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, data_packet),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/**
 * @brief BLE connection callback.
 *
 * Invoked when a remote BLE device successfully establishes a
 * connection with the PPG Recorder.
 *
 * @param conn Pointer to the active BLE connection.
 * @param err Connection status code.
 *
 * @return None.
 */
void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_le_conn_param *param = BT_LE_CONN_PARAM(MIN_CONNECTION_INTERVAL, MAX_CONNECTION_INTERVAL , 0, SUPERVISOR_TIMEOUT);
	bt_conn_le_param_update(conn, param);
}

/**
 * @brief BLE connection callback.
 *
 * Invoked when a remote BLE device successfully establishes a
 * connection with the PPG Recorder.
 *
 * @param conn Pointer to the active BLE connection.
 * @param err Connection status code.
 *
 * @return None.
 */
void disconnected(struct bt_conn *conn, uint8_t err)
{
    const struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, (BT_LE_AD_GENERAL), sizeof((BT_LE_AD_GENERAL))),
    };

    const struct bt_data sd[] = {
	    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};        

/**
 * @brief Initialize Bluetooth Low Energy services.
 *
 * Enables the Bluetooth stack, registers custom GATT services,
 * and prepares the device for advertising and data transmission.
 *
 * @return Initialization status.
 *         - 0 : Success
 *         - Non-zero : Initialization failed
 */
int ble_init (void) 
{
    int err;
    const struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, (BT_LE_AD_GENERAL), sizeof((BT_LE_AD_GENERAL))),
    };

    const struct bt_data sd[] = {
	    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    err = bt_enable(NULL);
    if (err) {
	    return err;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
	    return err;
    }
    return err;

}

/**
 * @brief Transmit data over Bluetooth Low Energy.
 *
 * Sends a packet through the custom BLE characteristic when
 * notifications are enabled by the connected client.
 *
 * @param data Pointer to the transmit buffer.
 * @param len Number of bytes to transmit.
 *
 * @return int Transmission status.
 *         - 0 : Success
 *         - Non-zero : Transmission failed
 */
int BT_send(uint8_t * RX_data, uint8_t size)
{
	int ret;

	memcpy(data_packet, RX_data, size);
	ret = bt_gatt_notify(NULL, &custom_service.attrs[1], data_packet, sizeof(data_packet));
	if(ret)
	 return 1;
	
	return 0;
}

                                                 

