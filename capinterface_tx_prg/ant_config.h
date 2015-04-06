/*
This file provides an easy location to change ANT settings.
*/

#ifndef ANT_SETTINGS_H
#define ANT_SETTINGS_H

//#define USER_USB_DEVICE_NUMBER	(0)	// USB ANT device number (0 is first device)
//#define USER_BAUDRATE			(57600)		// For AP1, use 50000; for AT3/AP2, use 57600

#define USER_RADIOFREQ			(66)		// RF Frequency + 2400 MHz (66 = default channel = 2466 MHz)

#define CHANNEL_0			(0)			// ANT channel to use
#define CHANNEL_0_CHAN_ID_DEV_NUM			(1)		// Device number
#define CHANNEL_0_CHAN_ID_DEV_TYPE			(1)			// Device type
#define CHANNEL_0_CHAN_ID_TRANS_TYPE			(2)			// Transmission type (2 = shared channel w/ 1 byte for ID)
#define CHANNEL_TYPE_USER CHANNEL_TYPE_SHARED_SLAVE    //cCHANNEL_TYPE_SHARED_SLAVE
#define ANT_EVENT_MSG_BUFFER_MIN_SIZE   (32)                  ///< Minimum size of ANT event message buffer


#define USER_CHANNEL_PD			(1638)	// Used for INITIAL channel decimation
#define USER_RX_TIMEOUT			(10)		// 2.5 sec per LSB
#define USER_TX_POWER				(3)			// 0, 1, 2, or 3 from lowest to highest

#define USER_NETWORK_KEY		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,}
#define ANT_CHANNEL_DEFAULT_NETWORK		(0)			// The network key is assigned to this network number

#define CHANNEL_0_ANT_EXT_ASSIGN			(0)			// Extended assign

#define BROADCAST_DATA_BUFFER_SIZE      (8) ///< Size of the broadcast data buffer

// Assign each device a unique device ID
//#define DEVICE_UNIQUE_ID		( (uint16_t) 0xbeef ) // must be between 0x0001 - 0xffff

#endif
