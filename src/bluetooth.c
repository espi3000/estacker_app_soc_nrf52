#include <zephyr/settings/settings.h>

#include "bluetooth.h"




#define BT_UUID_MY_SERVICE      BT_UUID_DECLARE_128(MY_SERVICE_UUID)
#define BT_UUID_MY_SERVICE_TX   BT_UUID_DECLARE_128(TX_CHARACTERISTIC_UUID)

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, MY_SERVICE_UUID),
};

struct bt_conn *BT_connection;
uint8_t BT_connected;    // 1: Bluetooth connection is established, 0: Disconnected
uint8_t BT_REQ_disconnect = 0;      // set flag when disconnecting is desired
extern uint16_t n_Samples_taken; // number of samples collected 
extern uint8_t Sensor_Type;          // MSB -> LSB: 0, 0, GPS, BIO, TOF, PMS, IMU, TMP

static void Bluetooth_Off(struct k_timer *timer_id);
K_TIMER_DEFINE(BT_Off_timer, Bluetooth_Off, NULL);

// Timer Callback
static void Bluetooth_Off(struct k_timer *timer_id) {
    BT_REQ_disconnect = 1;
    k_timer_stop(&BT_Off_timer);
}

bool bt_is_connected(void) {
    return BT_connected;
}

static void connected(struct bt_conn *conn, uint8_t err) {
    BT_connected = 1;

	struct bt_conn_info info; 
	//char addr[BT_ADDR_LE_STR_LEN];

	BT_connection = conn;

	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	} else if (bt_conn_get_info(conn, &info)) {
		printk("Could not parse connection info\n");
	} else {
		printk("Connection established!\n");
        /*
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("Connection established!		\n\
		Connected to: %s					\n\
		Role: %u							\n\
		Connection interval: %u				\n\
		Slave latency: %u					\n\
		Connection supervisory timeout: %u	\n"
		, addr, info.role, info.le.interval, info.le.latency, info.le.timeout);
        */
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    BT_connected = 0;
	printk("Disconnected (reason %u)\n", reason);
}


static struct bt_conn_cb conn_callbacks = {
	.connected				= connected,
	.disconnected   		= disconnected
};

void bt_advertise(int err) {
	if (err) {
		printk("BLE init failed with error code %d\n", err);
		return;
	}

	if (err) {
		printk("Failed to init LBS (err:%d)\n", err);
		return;
	}

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}



/* This function is called whenever a Notification has been sent by the TX Characteristic */
static void on_sent(struct bt_conn *conn, void *user_data) {
	ARG_UNUSED(user_data);

    const bt_addr_le_t * addr = bt_conn_get_dst(conn);
        
	printk("Data sent to Address 0x %02X %02X %02X %02X %02X %02X \n", 
        addr->a.val[0], addr->a.val[1], addr->a.val[2], 
        addr->a.val[3], addr->a.val[4], addr->a.val[5]);
}

/* This function is called whenever the CCCD register has been changed by the client*/
void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    switch(value) {
    case BT_GATT_CCC_NOTIFY: 
        // Start sending stuff!
        break;

    case BT_GATT_CCC_INDICATE: 
        // Start sending stuff via indications
        break;

    case 0: 
        // Stop sending stuff
        break;
        
    default: 
        printk("Error, CCCD has been set to an invalid value\n");     
    }
}


int bt_init(void) {
    int err;
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    settings_load(); 	// Required for bluetooth

    BT_connected = 0;

    k_msleep(500);

    //Configure connection callbacks
	bt_conn_cb_register(&conn_callbacks);

    return 0;
}
                        

/* LED Button Service Declaration and Registration */
BT_GATT_SERVICE_DEFINE(my_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MY_SERVICE),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_MY_SERVICE_TX,
	    BT_GATT_CHRC_NOTIFY,
	    BT_GATT_PERM_READ,
        NULL, NULL, NULL
    ),
    BT_GATT_CCC(
        on_cccd_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
    ),
);

/* This function sends a notification to a Client with the provided data,
given that the Client Characteristic Control Descripter has been set to Notify (0x1).
It also calls the on_sent() callback if successful*/
void bt_transmit(struct bt_conn *conn, const uint8_t *data, size_t len) {
    /* 
    The attribute for the TX characteristic is used with bt_gatt_is_subscribed 
    to check whether notification has been enabled by the peer or not.
    Attribute table: 0 = Service, 1 = Primary service, 2 = TX, 3 = CCC.
    */
    const struct bt_gatt_attr *attr = &my_service.attrs[2]; 

    struct bt_gatt_notify_params params = {
        .uuid   = BT_UUID_MY_SERVICE_TX,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = on_sent
    };

    // Check whether notifications are enabled or not
    if (!bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
        printk("Warning, notification not enabled on the selected attribute\n");
        return;
    }

    // Send the notification
	if (bt_gatt_notify_cb(conn, &params)) {
        printk("Error, unable to send notification\n");
        return;
    }
}


int communicate_samples(uint16_t *samples, size_t num_samples) {
    k_timer_start(&BT_Off_timer, K_SECONDS(3), K_SECONDS(3));
    BT_REQ_disconnect = 0;
    bt_advertise(0);
    //bt_conn_auth_cb_register(&auth_cb_display);     
    printk("Starting advertising\n");
    for (int adv_timer = 0; !bt_is_connected() && (adv_timer < 5000); adv_timer++) {
        k_msleep(1);
    }
    if (!bt_is_connected()) {
        // if Bluetooth connection cannot be established within the time limit: Stop advertising
        bt_le_adv_stop();
        printk("Error establishing Bluetooth connection!\n");
        return 1;
    }
    printk("Connection established!\n");
    k_sleep(K_SECONDS(2));

    size_t len = num_samples*sizeof(uint16_t);
    size_t num_packets = DIV_ROUND_UP(len, BT_GAP_ADV_MAX_ADV_DATA_LEN);
    for (size_t packet_idx = 0; packet_idx < num_packets; packet_idx++) {
        size_t offset = packet_idx*BT_GAP_ADV_MAX_ADV_DATA_LEN;
        size_t chunk_len = MIN(BT_GAP_ADV_MAX_ADV_DATA_LEN, len - offset);
        bt_transmit(BT_connection, (uint8_t*)&samples[offset], chunk_len);
    }
    
    printk("Notified\n");	
    while (BT_REQ_disconnect == 0) {
        k_msleep(1);
    }
    printk("Disconnecting\n");	
    bt_conn_disconnect(BT_connection,BT_HCI_ERR_REMOTE_LOW_RESOURCES);  
    bt_le_adv_stop();

    return 0;
}