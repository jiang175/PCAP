#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "ant_config.h"
#include "nrf_delay.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"


//#include "PCAPint_main.c"

/**
 * @brief Setup ANT module ready for TX broadcast.
 *
 * Issues the following commands in a specific order:
 * - assign channel
 * - set channel ID   
 * - open channel 
 */
bool ant_channel_tx_broadcast_setup( void ) {
	uint32_t err_code;
	uint8_t ttype;
	
	/*// Initialize timer
	p_timer = timer_init( TIMER0 );
	if( p_timer == 0 ) {
		//return true; // error
	}*/
	
	// Set Channel Number
	err_code = sd_ant_channel_assign(CHANNEL_0, // channel
																	 CHANNEL_TYPE_USER, // channel type
																	 ANT_CHANNEL_DEFAULT_NETWORK, // network number
																	 CHANNEL_0_ANT_EXT_ASSIGN); // extended assign
	if( err_code != NRF_SUCCESS ) {
		return true; // error
	}

	// Set Channel ID. 
	err_code = sd_ant_channel_id_set(CHANNEL_0, // channel
																	 CHANNEL_0_CHAN_ID_DEV_NUM, // device #
																	 CHANNEL_0_CHAN_ID_DEV_TYPE,  // transaction type
																	 CHANNEL_0_CHAN_ID_TRANS_TYPE); // device type
	if( err_code != NRF_SUCCESS ) {
		return true; // error
	}
	
	// set radio freq
	err_code = sd_ant_channel_radio_freq_set( CHANNEL_0, // channel
																						USER_RADIOFREQ ); // freq
	if( err_code != NRF_SUCCESS ) {
		return true; // error
	}
	
	// set channel period
	err_code = sd_ant_channel_period_set( CHANNEL_0, // channel
																				USER_CHANNEL_PD ); // period
	if( err_code != NRF_SUCCESS ) {
		return true; // error
	}
	
	// set tx power
	//	err_code = sd_ant_channel_radio_tx_power_set( CHANNEL_0, // channel
			//																					USER_TX_POWER,0); // TX power

	if( err_code != NRF_SUCCESS ) {
		return true; // error
	}
	
	/*// set search timeouts
	if( channel_decimation == 0 ) {
		err_code = sd_ant_channel_rx_search_timeout_set(	USER_ANTCHANNEL, // channel
																											USER_RX_TIMEOUT ); // timeout value
	} else {
		err_code = sd_ant_channel_rx_search_timeout_set(	USER_ANTCHANNEL, // channel
																											channel_decimation ); // timeout value
	}
	if( err_code != NRF_SUCCESS ) {
		return true; // error
}*/
		
	return false; // no err
}