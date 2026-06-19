/**
 * @file bluetooth.h
 * @brief Public interface for Bluetooth Low Energy communication.
 *
 * Defines BLE-related data structures, UUIDs, packet formats, and
 * function declarations used for wireless data transmission.
 */
#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

/**
 * @brief BLE packet format used for live physiological data transmission.
 *
 * Contains IR and RED PPG samples, calculated SpO₂ and heart rate values,
 * packet metadata, and checksum information for data integrity verification.
 */
struct ble_packet {
	unsigned char header1;
	unsigned char header2;
	unsigned char length;
	unsigned char IR_lsb;
	unsigned char IR_mid;
	unsigned char IR_msb;
	unsigned char RED_lsb;
	unsigned char RED_mid;
	unsigned char RED_msb;
	unsigned char SpO2;
	unsigned char HR;
	unsigned char CRC;
};

/**
 * @brief Initialize Bluetooth Low Energy services.
 *
 * Enables the Bluetooth stack, registers custom GATT services,
 * and prepares the device for advertising and data transmission.
 *
 * @return int Initialization status.
 *         - 0 : Success
 *         - Non-zero : Initialization failed
 */
int ble_init(void);

/**
 * @brief Transmit data through the BLE interface.
 *
 * Sends a packet over the custom BLE characteristic when
 * notifications are enabled by the connected client.
 *
 * @param data Pointer to the transmit buffer.
 * @param len Number of bytes to transmit.
 *
 * @return int Transmission status.
 *         - 0 : Success
 *         - Non-zero : Transmission failed
 */
int BT_send(unsigned char *data, unsigned char len);
	
