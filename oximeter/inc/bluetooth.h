#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

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

int ble_init (void);
int BT_send(unsigned char *, unsigned char);
#endif
	