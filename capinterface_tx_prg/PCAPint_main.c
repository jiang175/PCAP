/* PCAP intialising & interfacing to nRF 51422 using SPI bus
 * 
 * Example extracted from Nordic Semiconductor's SPI master example. All Rights Reserved.
 *
 */

/** @file
* @brief SPI interface to PCAP.
* @defgroup spi_master_example SPI master loopback usage example
* @{
*
* @brief SPI master example.
*
* This example needs that the slave is configured to transmit the received bytes. That is the slave
* behaves as a loopback device for the master. The loopback can also be achieved without using a slave device at all by wiring MOSI and
* MISO pins of the spi master together. @ref TX_RX_MSG_LENGTH number of bytes are transmitted through the master and the received bytes are
* verified to be the same as transmitted. IF there is an error, gpio pin for relevant spi module is set high to show the error @sa ERROR_PIN_SPI0,
* @sa ERROR_PIN_SPI1. If there is error from both modules that is if both pins are set high, then this application loops for ever
*
*/

#include "spi_master.h"
#include "nrf_delay.h"
#include "common.h"
#include "spi_master_config.h"
#include "limits.h"
//#include "nRF6350.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "boards.h"

//ANT SETUP
#include "ant_config.h"
#include "ant_setup.c"
//PCAP
#include "PCAP.h"
#include "PCAP.c"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

//*****************************************
//DEVICE ID
#define DEVICE_ID 0x05;
//*******************************************


/* Static variables and buffers */
static uint8_t s_broadcast_data[BROADCAST_DATA_BUFFER_SIZE];	///< Primary data transmit buffer
//int rtc_flag = 1;


//static uint8_t s_counter = 1;																 ///< Counter to increment the ANT broadcast data payload

#define DELAY_MS							 100				/*!< Timer Delay in milli-seconds */
#define BUTTON2													2
#define LED7															 15
#define LED6															 14

/** 
 * @brief Handle error from application
 */
static void handle_error(void){while (1) {}}

/** 
 * @brief Stack Interrupt handler 
 *
 * Implemented to clear the pending flag when receiving 
 * an interrupt from the stack.
 */
void PROTOCOL_EVENT_IRQHandler(void){}

/**
 * @brief Handle softdevice asserts 
 *
 * @param[in] pc is the value of the program counter
 * @param[in] line_num is the line number which the assert occured
 * @param[in] p_file_name is the gile name of the file that it asserted
 *
 */
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t * p_file_name){while (1) {}}

/**** PCAP broadcast data prep function ***
		* Send read register data for ANT broadcast
		* @param add: Address/identifier to identify sent data
		* @param data: Data to be sent
		* @retval none
*/
static void pcap_broadcast_data(uint8_t add, uint32_t data1,uint32_t data2)
		{
				s_broadcast_data[0] = DEVICE_ID;
				s_broadcast_data[1] = add;
				for (uint8_t y = 2; y < (5); y++)
				{
						s_broadcast_data[y] = (data1 >> ((4)-y)*CHAR_BIT) & (0xFF) ;
				}
				for (uint8_t y = 5; y < (8); y++)
				{
						s_broadcast_data[y] = (data2 >> ((7)-y)*CHAR_BIT) & (0xFF) ;
				}
				
		}


/** Sets SPI interface.
 * @param lsb_first If true, least significant bits are transferred first
 * @param mod_num spi module to be used, either SPI0 or SPI1 from enumeration SPIModuleNumber
 * @retval Pointer to SPIaddress 
 * @retval false Error occurred
 * PCAP settings : 
 *							@SPImode 1 , @BITsequence , MSB first 
 */
 uint32_t* pcap_spi_set(SPIModuleNumber mod_num)
{
	uint32_t *PCAP_spi_address = spi_master_init(mod_num, SPI_MODE1, (bool) 0);

	return PCAP_spi_address;
}


/** 
 * @brief Handle ANT TX channel events 
 *
 * @param[in] event is the received ant event to handle
 *
 */
static void handle_channel_event(uint32_t event, uint8_t add, uint32_t data1,uint32_t data2)					
{
		uint32_t return_value;
		
		/* Prep broad cast data for transmit */
		pcap_broadcast_data(add, data1,data2);
						
		/* Broadcast the data */
		return_value = sd_ant_broadcast_message_tx(CHANNEL_0, BROADCAST_DATA_BUFFER_SIZE, s_broadcast_data);
		if (return_value != NRF_SUCCESS)
	 {
			/* Error */
			handle_error();
	 }
	 
		NRF_RTC1->CC[0] = 1*32768; 
	  rtc_flag = 1;
		NRF_RTC1->TASKS_START = 1;
		do 
    { 
       // Enter System ON sleep mode 
       __WFE();   
       // Make sure any pending events are cleared 
       __SEV(); 
       __WFE();                 
    }while(rtc_flag); 
		
    NRF_RTC1->TASKS_STOP = 1; 
    NRF_RTC1->TASKS_CLEAR = 1; 
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard. 
 */

void RTC1_IRQHandler(void) 
{ 
    // This handler will be run after wakeup from system ON (RTC wakeup) 
    if(NRF_RTC1->EVENTS_COMPARE[0]) 
    { 
       NRF_RTC1->EVENTS_COMPARE[0] = 0; 
       rtc_flag = 0;           
       NRF_RTC1->TASKS_CLEAR = 1; 
    } 
}



int main(void)

{
	// Variable declarations 
	bool ret0, ret1, ret2, ret3; // error checkers
	uint32_t sw = 1, sw2 = 1, sw3 =0, stat; 
	uint8_t CYC_ACTIVE, T_END_FLAG, RUNBIT, COMBI_ERR, CAP_ERR, CAP_ERR_PC, TEMP_ERR; // Status register error checkers
	//uint32_t	capref_t;
	uint8_t x, n;
	uint8_t	event;
	uint8_t	ant_channel;		
	uint32_t return_value;
  //uint32_t* PCAP_spi_address;
  char capref_s[16]; // String displays for LCD
  char cap1_s[7][16]; 
  float cap1, cap2, cap3, cap4, cap5, cap6, cap7, capref;
	int check = 0;
	
		 
	/* ANT event message buffer */
	static uint8_t event_message_buffer[ANT_EVENT_MSG_BUFFER_MIN_SIZE];
	
	/* Enable softdevice */
	return_value = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, softdevice_assert_callback);
	if (return_value != NRF_SUCCESS) 	{	while(true);}

  /*RTC Intialisation
	// Internal 32kHz RC 
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos; 
  
  // Start the 32 kHz clock, and wait for the start up to complete 
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0; 
  NRF_CLOCK->TASKS_LFCLKSTART = 1; 
  while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0); 
  */
	
  // Configure the RTC to run at 2 second intervals, and make sure COMPARE0 generates an interrupt (this will be the wakeup source) 
  NRF_RTC1->PRESCALER = 0; 
  NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk;  
  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;  
  NRF_RTC1->CC[0] = 5*32768; 
  NVIC_EnableIRQ(RTC1_IRQn); 
	/* Set application IRQ to lowest priority */
	//return_value = sd_nvic_SetPriority(SD_EVT_IRQn, NRF_APP_PRIORITY_LOW); 
	
	/* Enable application IRQ (triggered from protocol) */
  //return_value = sd_nvic_EnableIRQ(SD_EVT_IRQn);			
				/* Setup Channel_0 as a TX-RX Master Only */
	/*Modify this in the future to close & open channel */
	
	return_value = ant_channel_tx_broadcast_setup(); 
  if (return_value != false) while(true);
  
	/* SPI Intialisation */ 
	uint32_t *PCAP_spi_address = pcap_spi_set(SPI1); 
  pcap_dsp_write(PCAP_spi_address); 
	return_value = pcap_commcheck(PCAP_spi_address);
	while( return_value != true) 
		{
		 pcap_dsp_write(PCAP_spi_address); 
		 return_value = pcap_commcheck(PCAP_spi_address);
		}

	/* PCAP Config */
	ret0 = pcap_config(PCAP_spi_address);
	
	/* Main Loop */
	while(true)
	{	
		check = 0;
		
		/* PCAP Config */
		//needs implementation into loop to check if this needs to be reconfigured based on ANT recieved messages
		//ret0 = pcap_config(PCAP_spi_address);
		
		/* Capacitance Measurement */
		ret2 = pcap_measure(PCAP_spi_address);
		
		
		/* Prep broad cast data for transmit */
		//pcap_broadcast_data(read_stat, stat);
		
		/* Read cap values: */

		/* Reference Capacitor */
		//cap_t[0] = read_reg(PCAP_spi_address, read_reg0);
		/*

		
		cap_t[4] = read_reg(PCAP_spi_address, read_reg4);
		
		cap_t[5] = read_reg(PCAP_spi_address, read_reg5);
		
		cap_t[6] = read_reg(PCAP_spi_address, read_reg6);
		
		cap_t[7] = read_reg(PCAP_spi_address, read_reg7);
														
		*/
		ret3 = (stat == 0 && cap_t == 0 && cap_t[1] == 0); 
		
		/* Status bits */ 
		//configure later for further analysis 
		CYC_ACTIVE = (stat >> 23) & 1;
		T_END_FLAG = (stat >> 22) & 1;
		RUNBIT = (stat >> 19) & 1;
		COMBI_ERR = (stat >> 15) & 1;
		CAP_ERR = (stat >> 12) & 1;
		CAP_ERR_PC = (stat >> 5) & 0x0F;
		TEMP_ERR = (stat >> 3) & 1;
		
		/* Data extraction: */
		capref = data_extract(cap_t[0])*47 ;
		//capref = 5;
  //if (return_value != NRF_SUCCESS) while(true);	
				/*return_value = sd_ant_event_get(&ant_channel, &event, event_message_buffer);
			if (return_value == NRF_SUCCESS) 
			{

				switch (event)
				{
					case EVENT_RX:
					check = 1;
					break;
				}
			}		*/
	// Open channel. 
	return_value = sd_ant_channel_open(CHANNEL_0);
		// Get first data to send
	s_broadcast_data[0] = DEVICE_ID;
	return_value = sd_ant_broadcast_message_tx(CHANNEL_0, BROADCAST_DATA_BUFFER_SIZE, s_broadcast_data );


	if( return_value != NRF_SUCCESS ) {
		return true; // error
	}
		n = 0;
		//return_value = sd_ant_broadcast_message_tx(CHANNEL_0, BROADCAST_DATA_BUFFER_SIZE, s_broadcast_data);
		//nrf_delay_ms(2000);

		//return_value = sd_ant_event_get(&ant_channel, &event, event_message_buffer);

		/* send data */
		do
		{	
			/* Fetch the event */
			//return_value = sd_ant_event_get(&ant_channel, &event, event_message_buffer);
			
				/* Handle event */
						n++;
						//sd_ant_event_get(&ant_channel, &event, event_message_buffer);
					
						// HARD CODED FOR CMEAS_BITS equal to 4
						
						//uint16_t chk = (CMEAS_PORT_EN >> 2*n) & 0x03;
						//x = 0;

						/* Check for transmission and no of capacitors to be read*/
						if(1) 
						{
								switch (n)
								{
										case 1:
										memset(tx_data, 0, 8);
										memset(rx_data, 0, 8);
		
										/* Read Status register: */
										stat = read_reg(PCAP_spi_address, read_stat);
										/* Measure Capacitor 1  */
										//Needs a bit of refinement here ... this switch statement is useless but can be refined to handle registers
										cap_t[1] = read_reg(PCAP_spi_address, read_reg1);
										handle_channel_event(event, read_reg1, stat,cap_t[1]);
										break;
										
										case 2:
										/* Measure Capacitor 2 & 3 */
										//May need a memset
										cap_t[2] = read_reg(PCAP_spi_address, read_reg2);
										cap_t[3] = read_reg(PCAP_spi_address, read_reg3);
										handle_channel_event(event, read_reg2, cap_t[2],cap_t[3]);
										break;
																			
								}															 
						}

						/* Flag to ensure all capacitors values have been transmitted*/
						sw2 = 0;
		} while ( n < 3 );
		
		n = 0;			
		sw2 = 0;			
		sw3 = 1;			
		return_value = sd_ant_channel_close(CHANNEL_0);
		
	  NRF_RTC1->CC[0] = 1*32768; 
		rtc_flag = 1;
		NRF_RTC1->TASKS_START = 1;
		do 
    { 
       // Enter System ON sleep mode 
       __WFE();   
       // Make sure any pending events are cleared 
       __SEV(); 
       __WFE();                 
    }while(rtc_flag); 
		
    NRF_RTC1->TASKS_STOP = 1; 
    NRF_RTC1->TASKS_CLEAR = 1; 

		// sd_power_system_off(); 
}
} // int main(void)



/**
 Change Log 
 28/2/2013
 Included the start measurement command
 3/1/2013
 Changed the data extract function to extract the right bits.
 Changed the no of the recieved bit in the read_reg funtion	(i.e from 32 to 24) 
 Changed the decimal place division. 
 3/2/2013
 Fixed the tx_data function in the config reg
 Cleared tx_data using memset.
 3/3/2013
 Cleared rx_data using memset.
 Delay Function before transmssion.
 MSG_LEN is common for both transmit and recieve , the message lengths have been adjusted accordingly
 Debug portion has been commented out.
 implement delayed send of runbit
 3/4/2013
 Implemented parameter registers from 11 to 19
 Transmitting one configregister at a time
 NOTE ****** need to implement tranmsit check in pcap_config_write()
 3/6/2013
 Implemented runbit clearing before config register set
 removed all excess delay 
 reimplemented 32bit config register send
 3/7/2013
 Changing size of transmit and recieve registers from 48-8
 3/8/2013
 Implemented interactive interface on LCD screen and capacitance display based on open ports & configuration
 3/14 - 3/15/2013
 Implemented ANT transmission protocols adopted from broadcast_tx example
 Removed some excess button read commands and flags
 3/18/2013
 Changed memory location on target option to enable ARM IRQ
 Implemented delay to enable read of 2 data packet. 
 3/19/2013
 Removed Delay and implemented acknowledge transmission
 Changed all EVENT_TX to EVENT_TX_COMPLETED
 3/21/2013
 Changed location of Status transmit
 Implemented an extra EVENT_TX case to start loop
 Code seems to work!!
 5/16/2013
 Test Implementation of writing program 
 5/17/2013
 possible error in the program 
 msg_len in pcap_config_write was previoulsy 3 ... value should be 4. 
 5/18/2013
 implemented chk condition on soft exit IF statement to ensure sucessful transmission and prevent infinite looping.
 5/20/2013
 shifted SPI intialisation from within the loop to outside the loop.
 5/22/2013
 in pcapdsp_write changed the declaration of x & regadd to uint16
 change the prg_data to global variable and static
 This works!!!
 4/8/2015
 Implemented commcheck
 Removed 100ms delay in read reg
 moved memset function to same function
 4/9/2015
 Refined the minimum time for measuring... Hope it works
 **/
