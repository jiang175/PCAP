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
#include "pstorage.h"
#include "pstorage.c"
#include "softdevice_handler.h"
#include "softdevice_handler.c"
#include "nrf_assert.c"
#include "nrf_assert.h"
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))




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
//#include <time.h>  
#include <stdlib.h> 

//*****************************************
//DEVICE ID
#define DEVICE_ID 1;
//*******************************************


/* Static variables and buffers */
static uint8_t s_broadcast_data[BROADCAST_DATA_BUFFER_SIZE];	///< Primary data transmit buffer
static int8_t s_broadcast_temp[8];
static uint8_t ps_flag = 0;
uint8_t test_data[20] __attribute__((aligned(4)));
uint8_t test_data[20] = {0}; 
int a[3] = {0};
int b[3] = {0};
int programming = 1;
int cap[3] = {0};

uint8_t *ua;
uint8_t *ub;


pstorage_module_param_t flashparam;
pstorage_handle_t       flashhandle;
pstorage_handle_t       flash_block_handle;



//int rtc_flag = 1;


//static uint8_t s_counter = 1;																 ///< Counter to increment the ANT broadcast data payload

#define DELAY_MS							 100				/*!< Timer Delay in milli-seconds */
#define BUTTON2							 						2
#define LED7															 15
#define LED6															 14
//#define NEED_PSTORAGE 1

static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}



static void dm_pstorage_cb_handler(pstorage_handle_t * p_handle,
                                   uint8_t             op_code,
                                   uint32_t            result,
                                   uint8_t           * p_data,
                                   uint32_t            data_len)
{
    switch(op_code)
    {
        case PSTORAGE_ERROR_OP_CODE:
                ps_flag = 1;
            break;
        case PSTORAGE_STORE_OP_CODE: 
                ps_flag = 2;           
            break;
        case PSTORAGE_LOAD_OP_CODE:
                ps_flag = 3;           
            break;
        case PSTORAGE_CLEAR_OP_CODE:
                ps_flag = 4;            
            break;
        case PSTORAGE_UPDATE_OP_CODE:
                ps_flag = 5;
            break;
        default:
                ps_flag = 6;
            break;
    }
}

int pstorage_test(void)
{
		int check = 0;

    uint32_t retval;

    flashparam.block_size  = 20;
    flashparam.block_count = 1;
    flashparam.cb          = dm_pstorage_cb_handler;

    retval = pstorage_init();   
    retval = pstorage_register(&flashparam, &flashhandle);   

    //retval = pstorage_clear(&flashhandle, 20);  

    retval = pstorage_block_identifier_get(&flashhandle, 0, &flash_block_handle);      
    //retval = pstorage_store(&flash_block_handle, test_data, 4, 0);

    //retval = pstorage_block_identifier_get(&flashhandle, 0, &flash_block_handle);

    //uint8_t load_data[6];
    retval = pstorage_load(test_data, &flash_block_handle,20, 0); 
		if((test_data[0] == 0x01) | (test_data[1] == 0x01) | (test_data[2] == 0x01))
		{
			check = 1;
			a[0] = ((test_data[5] & 0xFF) | ((test_data[4] & 0xFF) << 8) | ((test_data[3] & 0x0F) << 16));
			b[0] = ((test_data[6] & 0xff) << 8) | (test_data[7] & 0xff);		
			a[1] = ((test_data[10] & 0xFF) | ((test_data[9] & 0xFF) << 8) | ((test_data[8] & 0x0F) << 16));
			b[1] = ((test_data[11] & 0xff) << 8) | (test_data[12] & 0xff);		
			a[2] = ((test_data[15] & 0xFF) | ((test_data[14] & 0xFF) << 8) | ((test_data[13] & 0x0F) << 16));
			b[2] = ((test_data[16] & 0xff) << 8) | (test_data[17] & 0xff);		
			//retval = pstorage_load(ua, &flash_block_handle, 4, 1); 
			//retval = pstorage_load(ub, &flash_block_handle, 4, 5); 
			//memcpy(&a,ua,sizeof(float));
			//memcpy(&b,ub,sizeof(float));
		}

		return check;
		
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {}
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
	
void rtc_delay(int second)
{
		NRF_RTC1->CC[0] = second*32768; 
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
static void pcap_broadcast_data(uint8_t add, int data1,int data2,int data3)
		{
			if(add == 5 )
			{
				s_broadcast_data[0] = DEVICE_ID;
				s_broadcast_data[1] = add;
				s_broadcast_data[2] = data1 >> 16;
				s_broadcast_data[3] = data1 >> 8;
				s_broadcast_data[4] = data1;
				s_broadcast_data[5] = 0;
				s_broadcast_data[6] = 0;
				s_broadcast_data[7] = 0;

			}
			else{
				s_broadcast_temp[0] = DEVICE_ID;
				s_broadcast_temp[1] = add;
				s_broadcast_temp[2] = data1 >> 8;
				s_broadcast_temp[3] = data1;
			  s_broadcast_temp[4] = data2 >> 8;
				s_broadcast_temp[5] = data2;
				s_broadcast_temp[6] = data3>> 8;
				s_broadcast_temp[7] = data3;
			  memcpy(s_broadcast_data,s_broadcast_temp,8);
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
static void handle_channel_event(uint32_t event, uint8_t add, int data1,int data2,int data3)					
{
		uint32_t return_value;
		
		/* Prep broad cast data for transmit */
		pcap_broadcast_data(add, data1,data2,data3);
	  
						
		/* Broadcast the data */
		return_value = sd_ant_broadcast_message_tx(CHANNEL_0, BROADCAST_DATA_BUFFER_SIZE, s_broadcast_data);
		if (return_value != NRF_SUCCESS)
	 {
			/* Error */
			handle_error();
	 }
	 
		NRF_RTC1->CC[0] = 2*32768; 
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
	int acknowledge_flag = 0;
	int event_flag = 0;
	int delay = 1;
	int pcap_flag = 1;
	int temp_flag = 0;
	int cy_time = 4;
	int rdc_sel = 6;
	int c_avg = 100;
	double temp1 = 0;
	double temp2 = 0;
	double temp3 = 0;
	int check_temp = 0;
	int deviceid = DEVICE_ID;
	int good = 0;

		 
	/* ANT event message buffer */
	static uint8_t event_message_buffer[ANT_EVENT_MSG_BUFFER_MIN_SIZE];
	
	/* Enable softdevice */
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false); //NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM
	softdevice_sys_evt_handler_set(sys_evt_dispatch); 
	//return_value = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, softdevice_assert_callback);//NRF_CLOCK_LFCLKSRC_XTAL_250_PPM   NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM
	//if (return_value != NRF_SUCCESS) 	{	while(true);}
	
  /*RTC Intialisation*/	
	
  NRF_RTC1->PRESCALER = 0; 
  NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk;  
  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;  
  NRF_RTC1->CC[0] = 5*32768; 
  NVIC_EnableIRQ(RTC1_IRQn); 
	
  sd_power_mode_set ( NRF_POWER_MODE_LOWPWR   );

  /*Random Delay*/
	NRF_RTC1->CC[0] = (10+rand()%10)*32768; 
  rtc_flag = 1;
  NRF_RTC1->TASKS_START = 1;
	//NRF_CLOCK->TASKS_HFCLKSTOP = 1;

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

	/*Persistent Storage*/
	check_temp = pstorage_test();

	

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
	
	/* Main Loop */
	while(true)
	{	
		if(pcap_flag)
		{
			pcap_flag = 0;
			/* PCAP Config */
	    ret0 = pcap_config(PCAP_spi_address,c_avg,temp_flag,cy_time,rdc_sel);
		}
		/* PCAP Config */
		/* Capacitance Measurement */
		ret2 = pcap_measure(PCAP_spi_address,c_avg,temp_flag,cy_time);
		
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
		
	  // Open channel. 
		return_value = sd_ant_channel_open(CHANNEL_0);

		/*Check Status make sure good data*/
		
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);

		stat = read_reg(PCAP_spi_address, read_stat);
		if(!(stat == 	0x100000 | stat == 	0x500000)) good = 0;
		else good = 1;
		/*Try to fix for 10 times*/
		if(!good)
		{
			for(int i = 0; i< 10; i++)
			{
				
				if(!good)
				{		
					  handle_channel_event(event, 5, stat,0,0);
						MSG_LEN = 8;
						memset(tx_data, 0, 8);
						memset(rx_data, 0, 8);
						tx_data[0] = 0x88; // full Reset 
					
						acknowledge_flag = 1;
						
						event_flag = 0;
						
						return_value = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
						pcap_dsp_write(PCAP_spi_address); 
						return_value = pcap_config(PCAP_spi_address,c_avg,temp_flag,cy_time,rdc_sel);
						return_value = pcap_measure(PCAP_spi_address,c_avg,temp_flag,cy_time);
						/*Check again*/
						stat = read_reg(PCAP_spi_address, read_stat);
						if(!(stat == 	0x100000 | stat == 	0x500000)) good = 0;
						else good = 1;
						rtc_delay(delay);
				
			
						}
		}
	}
		if(!good)
		{
			while(1)
			{
				handle_channel_event(event, 5, stat,0,0);
				rtc_delay(1);
			}
		}
		
		n = 0;
		/* send data */
		do
		{				
			/* Handle event */
			n++;
			if(1) 
				{
					switch (n)
					{
						case 1:
						memset(tx_data, 0, 8);
						memset(rx_data, 0, 8);
			
						/* Read Status register: */
											
						cap_t[1] = read_reg(PCAP_spi_address, read_reg1);		
						cap_t[2] = read_reg(PCAP_spi_address, read_reg2);		
						cap_t[3] = read_reg(PCAP_spi_address, read_reg3);		
											
						if(check_temp)
						{	
							temp1 = data_extract(cap_t[1])*1200;
							temp1 = (-(float)a[0]*temp1)/10000 + (float)b[0]/10;
							temp1 = temp1 * 100;
												
							temp2 = data_extract(cap_t[2])*1200;
							temp2 = (-(float)a[1]*temp2)/10000 + (float)b[1]/10;
							temp2 = temp2 * 100;
												
							temp3 = data_extract(cap_t[3])*1200;
							temp3 = (-(float)a[2]*temp3)/10000 + (float)b[2]/10;
							temp3 = temp3 * 100;

							handle_channel_event(event, 4, temp1,temp2,temp3);

						}
						else
						{
							cap[0] =  (int)((float)(data_extract(cap_t[1]))*10000);
							cap[1] =  (int)((float)(data_extract(cap_t[2]))*10000);
							cap[2] =  (int)((float)(data_extract(cap_t[3]))*10000);
							handle_channel_event(event, 4, cap[0],cap[1],cap[2]);
						}		
					break;
										
						case 2:
							if(temp_flag)
							{
									/*Temp Measure*/
									cap_t[4] = read_reg(PCAP_spi_address, read_reg10);
								  cap_t[5] = read_reg(PCAP_spi_address, read_reg11);
									//handle_channel_event(event, 3, cap_t[4],cap_t[5]);
							}										
							break;										
						}															 
			  }
						/* Flag to ensure all capacitors values have been transmitted*/
						sw2 = 0;
		} while ( n < 2 );
		
		n = 0;			
		sw2 = 0;			
		sw3 = 1;			
		
		acknowledge_flag = 1;
		
		event_flag = 0;
		while(!event_flag){
	  return_value = sd_ant_event_get(&ant_channel, &event, event_message_buffer);
			
		if (return_value == NRF_SUCCESS) 
		{
			switch (event)
			{
				case EVENT_RX:
					if( event_message_buffer[1u] ==  MESG_BROADCAST_DATA_ID) 
						{
						
						if(event_message_buffer[5] < 2)
						{
							delay = event_message_buffer[6];
							if(c_avg !=  (int)(event_message_buffer[8] << 8 |event_message_buffer[7] ))
							{
								c_avg = (int)(event_message_buffer[8] << 8 |event_message_buffer[7] );
								pcap_flag = 1;
							}	
							//temp_flag = event_message_buffer[6];
							if(event_message_buffer[5] != temp_flag)
							{
								temp_flag = event_message_buffer[5];
								pcap_flag = 1;
								//return_value =  pstorage_store(&flash_block_handle, test_data, 4, 0);
								//check_temp = 1;
							}
							if(event_message_buffer[9] != cy_time)
							{
								cy_time = event_message_buffer[9];
								pcap_flag = 1;
							}
							if(event_message_buffer[10] != rdc_sel)
							{
								rdc_sel = event_message_buffer[10];
								pcap_flag = 1;
							}
					}
					else if(event_message_buffer[3] == deviceid)
					{
							if((a[event_message_buffer[5] - 2] !=  ((event_message_buffer[8] & 0xFF) | ((event_message_buffer[7] & 0xFF) << 8) | ((event_message_buffer[6] & 0x0F) << 16))) | (b[event_message_buffer[5] - 2] !=  (((event_message_buffer[9] & 0xff) << 8) | (event_message_buffer[10] & 0xff))))
							{
								a[event_message_buffer[5] - 2] = ((event_message_buffer[8] & 0xFF) | ((event_message_buffer[7] & 0xFF) << 8) | ((event_message_buffer[6] & 0x0F) << 16));
								b[event_message_buffer[5] - 2] = ((event_message_buffer[9] & 0xff) << 8) | (event_message_buffer[10] & 0xff);
								test_data[event_message_buffer[5] - 2] = 0x01;
								test_data[3 + 5 * ((event_message_buffer[5]) - 2)] = event_message_buffer[6];
								test_data[3 + 5 * ((event_message_buffer[5]) - 2) + 1] = event_message_buffer[7];
								test_data[3 + 5 * ((event_message_buffer[5]) - 2) + 2] = event_message_buffer[8];
								test_data[3 + 5 * ((event_message_buffer[5]) - 2) + 3] = event_message_buffer[9];
								test_data[3 + 5 * ((event_message_buffer[5]) - 2) + 4] = event_message_buffer[10];
								if(check_temp){
									
									return_value =  pstorage_update(&flash_block_handle, test_data, 20, 0);
								}
								else
								{
									return_value =  pstorage_store(&flash_block_handle, test_data, 20, 0);
									check_temp = 1;
								}
							}
							/*test_data[0] = 0x01;
							return_value =  pstorage_store(&flash_block_handle, test_data, 1, 0);
							ua = (uint8_t *)&a;
							ub = (uint8_t *)&b;
							return_value =  pstorage_store(&flash_block_handle, ua, 4, 1);
							return_value =  pstorage_store(&flash_block_handle, ub, 4, 5);*/
						 // a = 10447;
						 //b = 12341;
					}
				}
					event_flag = 1;
					break;
				default:
					//acknowledge_flag = 0;
					event_flag = 0;
				break;
			}
		}
		else
		{
			return_value = sd_ant_channel_close(CHANNEL_0);
			NRF_RTC1->CC[0] = 5*32768; 
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
			
			// Open channel. 
	    return_value = sd_ant_channel_open(CHANNEL_0);
		  // Get first data to send
	    s_broadcast_data[0] = DEVICE_ID;
	    return_value = sd_ant_broadcast_message_tx(CHANNEL_0, BROADCAST_DATA_BUFFER_SIZE, s_broadcast_data );
			
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
			
			
		}
	}
		
		return_value = sd_ant_channel_close(CHANNEL_0);
	
		if(acknowledge_flag){
	  NRF_RTC1->CC[0] = delay*32768; 
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
