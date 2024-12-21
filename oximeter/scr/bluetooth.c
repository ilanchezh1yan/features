#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>

#define MIN_CONNECTION_INTERVAL 8     /*1.25 ms connection interval */
#define MAX_CONNECTION_INTERVAL 8     /*1.25ms connection interval */
#define SUPERVISOR_TIMEOUT 400          /* 4s connection timeout */

static struct bt_uuid_128 custom_Read_service_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                              0x00, 0x10, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00);  

static struct bt_uuid_128 live_graph_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe1, 0xff, 0x00, 0x00);    
static uint8_t data_packet[12];                                                         

void CCC_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
   bool data = value ==  BT_GATT_CCC_NOTIFY;
}

BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_Read_service_uuid),
    BT_GATT_CHARACTERISTIC(&live_graph_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, data_packet),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#if 1
void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_le_conn_param *param = BT_LE_CONN_PARAM(MIN_CONNECTION_INTERVAL, MAX_CONNECTION_INTERVAL , 0, SUPERVISOR_TIMEOUT);
	bt_conn_le_param_update(conn, param);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
};
#endif        

int ble_init (void) 
{
    int err;
    static const struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, (BT_LE_AD_GENERAL), sizeof((BT_LE_AD_GENERAL))),
    };

    static const struct bt_data sd[] = {
	    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    
    err = bt_enable(NULL);
    if (err) {
	    return 1;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
	    return 1;
    }
    
    return 0;

}

int BT_send(unsigned char * RX_data)
{
	int ret;

	memcpy(data_packet, RX_data, 12);
	k_sleep(K_USEC(1000));
	ret = bt_gatt_notify(NULL, &custom_service.attrs[1], data_packet, sizeof(data_packet));
	if(ret)
	 return 1;
	
	return 0;
}
                                                 

