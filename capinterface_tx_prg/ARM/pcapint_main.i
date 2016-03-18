#line 1 "..\\PCAPint_main.c"




 














 

#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\spi_master.h"
 









 




#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 17 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\spi_master.h"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\spi_master.h"














 



 
typedef enum
{
    Freq_125Kbps = 0,         
    Freq_250Kbps,             
    Freq_500Kbps,             
    Freq_1Mbps,               
    Freq_2Mbps,               
    Freq_4Mbps,               
    Freq_8Mbps                
} SPIFrequency_t;



 
typedef enum
{
    SPI0 = 0,                
    SPI1                     
} SPIModuleNumber;



 
typedef enum
{
    
    SPI_MODE0 = 0,           
    SPI_MODE1,               
    
    SPI_MODE2,               
    SPI_MODE3                
} SPIMode;















 
uint32_t* spi_master_init(SPIModuleNumber module_number, SPIMode mode, _Bool lsb_first);
















 
_Bool spi_master_tx_rx(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data);



 
 
#line 23 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_delay.h"



#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf.h"




























 





 
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"

 








































 





 



 









 

typedef enum {
 
  Reset_IRQn                    = -15,               
  NonMaskableInt_IRQn           = -14,               
  HardFault_IRQn                = -13,               
  SVCall_IRQn                   =  -5,               
  DebugMonitor_IRQn             =  -4,               
  PendSV_IRQn                   =  -2,               
  SysTick_IRQn                  =  -1,               
 
  POWER_CLOCK_IRQn              =   0,               
  RADIO_IRQn                    =   1,               
  UART0_IRQn                    =   2,               
  SPI0_TWI0_IRQn                =   3,               
  SPI1_TWI1_IRQn                =   4,               
  GPIOTE_IRQn                   =   6,               
  ADC_IRQn                      =   7,               
  TIMER0_IRQn                   =   8,               
  TIMER1_IRQn                   =   9,               
  TIMER2_IRQn                   =  10,               
  RTC0_IRQn                     =  11,               
  TEMP_IRQn                     =  12,               
  RNG_IRQn                      =  13,               
  ECB_IRQn                      =  14,               
  CCM_AAR_IRQn                  =  15,               
  WDT_IRQn                      =  16,               
  RTC1_IRQn                     =  17,               
  QDEC_IRQn                     =  18,               
  LPCOMP_IRQn                   =  19,               
  SWI0_IRQn                     =  20,               
  SWI1_IRQn                     =  21,               
  SWI2_IRQn                     =  22,               
  SWI3_IRQn                     =  23,               
  SWI4_IRQn                     =  24,               
  SWI5_IRQn                     =  25                
} IRQn_Type;




 


 
 
 

 




   

#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
 







 

























 
























 




 


 

 













#line 110 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"


 







#line 145 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

#line 147 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 



#line 292 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"


#line 684 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 148 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 271 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 307 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 634 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 

#line 149 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"








 
#line 174 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

 






 
#line 190 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








#line 120 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\system_nrf51.h"




























 







#line 38 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\system_nrf51.h"


extern uint32_t SystemCoreClock;     









 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 121 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"


 
 
 




 


 

  #pragma push
  #pragma anon_unions
#line 148 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"


typedef struct {
  volatile uint32_t  CPU0;                               
  volatile uint32_t  SPIS1;                              
  volatile uint32_t  RADIO;                              
  volatile uint32_t  ECB;                                
  volatile uint32_t  CCM;                                
  volatile uint32_t  AAR;                                
} AMLI_RAMPRI_Type;

typedef struct {
  volatile  uint32_t  EN;                                 
  volatile  uint32_t  DIS;                                
} PPI_TASKS_CHG_Type;

typedef struct {
  volatile uint32_t  EEP;                                
  volatile uint32_t  TEP;                                
} PPI_CH_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[30];
  volatile  uint32_t  TASKS_CONSTLAT;                     
  volatile  uint32_t  TASKS_LOWPWR;                       
  volatile const  uint32_t  RESERVED1[34];
  volatile uint32_t  EVENTS_POFWARN;                     
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile uint32_t  RESETREAS;                          
  volatile const  uint32_t  RESERVED4[9];
  volatile const  uint32_t  RAMSTATUS;                          
  volatile const  uint32_t  RESERVED5[53];
  volatile  uint32_t  SYSTEMOFF;                          
  volatile const  uint32_t  RESERVED6[3];
  volatile uint32_t  POFCON;                             
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  GPREGRET;                          
 
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  RAMON;                              
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RESET;                             
 
  volatile const  uint32_t  RESERVED10[3];
  volatile uint32_t  RAMONB;                             
  volatile const  uint32_t  RESERVED11[8];
  volatile uint32_t  DCDCEN;                             
  volatile const  uint32_t  RESERVED12[291];
  volatile uint32_t  DCDCFORCE;                          
} NRF_POWER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_HFCLKSTART;                   
  volatile  uint32_t  TASKS_HFCLKSTOP;                    
  volatile  uint32_t  TASKS_LFCLKSTART;                   
  volatile  uint32_t  TASKS_LFCLKSTOP;                    
  volatile  uint32_t  TASKS_CAL;                          
  volatile  uint32_t  TASKS_CTSTART;                      
  volatile  uint32_t  TASKS_CTSTOP;                       
  volatile const  uint32_t  RESERVED0[57];
  volatile uint32_t  EVENTS_HFCLKSTARTED;                
  volatile uint32_t  EVENTS_LFCLKSTARTED;                
  volatile const  uint32_t  RESERVED1;
  volatile uint32_t  EVENTS_DONE;                        
  volatile uint32_t  EVENTS_CTTO;                        
  volatile const  uint32_t  RESERVED2[124];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[63];
  volatile const  uint32_t  HFCLKRUN;                           
  volatile const  uint32_t  HFCLKSTAT;                          
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  LFCLKRUN;                           
  volatile const  uint32_t  LFCLKSTAT;                          
  volatile const  uint32_t  LFCLKSRCCOPY;                      
 
  volatile const  uint32_t  RESERVED5[62];
  volatile uint32_t  LFCLKSRC;                           
  volatile const  uint32_t  RESERVED6[7];
  volatile uint32_t  CTIV;                               
  volatile const  uint32_t  RESERVED7[5];
  volatile uint32_t  XTALFREQ;                           
} NRF_CLOCK_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[330];
  volatile uint32_t  PERR0;                              
  volatile uint32_t  RLENR0;                             
  volatile const  uint32_t  RESERVED1[52];
  volatile uint32_t  PROTENSET0;                         
  volatile uint32_t  PROTENSET1;                         
  volatile uint32_t  DISABLEINDEBUG;                     
  volatile uint32_t  PROTBLOCKSIZE;                      
} NRF_MPU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[448];
  volatile uint32_t  REPLACEADDR[8];                     
  volatile const  uint32_t  RESERVED1[24];
  volatile uint32_t  PATCHADDR[8];                       
  volatile const  uint32_t  RESERVED2[24];
  volatile uint32_t  PATCHEN;                            
  volatile uint32_t  PATCHENSET;                         
  volatile uint32_t  PATCHENCLR;                         
} NRF_PU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[896];
  AMLI_RAMPRI_Type RAMPRI;                           
} NRF_AMLI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_TXEN;                         
  volatile  uint32_t  TASKS_RXEN;                         
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_DISABLE;                      
  volatile  uint32_t  TASKS_RSSISTART;                    
  volatile  uint32_t  TASKS_RSSISTOP;                     
  volatile  uint32_t  TASKS_BCSTART;                      
  volatile  uint32_t  TASKS_BCSTOP;                       
  volatile const  uint32_t  RESERVED0[55];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_ADDRESS;                     
  volatile uint32_t  EVENTS_PAYLOAD;                     
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_DISABLED;                    
  volatile uint32_t  EVENTS_DEVMATCH;                    
  volatile uint32_t  EVENTS_DEVMISS;                     
  volatile uint32_t  EVENTS_RSSIEND;                    
 
  volatile const  uint32_t  RESERVED1[2];
  volatile uint32_t  EVENTS_BCMATCH;                     
  volatile const  uint32_t  RESERVED2[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[61];
  volatile const  uint32_t  CRCSTATUS;                          
  volatile const  uint32_t  CD;                                 
  volatile const  uint32_t  RXMATCH;                            
  volatile const  uint32_t  RXCRC;                              
  volatile const  uint32_t  DAI;                                
  volatile const  uint32_t  RESERVED5[60];
  volatile uint32_t  PACKETPTR;                          
  volatile uint32_t  FREQUENCY;                          
  volatile uint32_t  TXPOWER;                            
  volatile uint32_t  MODE;                               
  volatile uint32_t  PCNF0;                              
  volatile uint32_t  PCNF1;                              
  volatile uint32_t  BASE0;                              
  volatile uint32_t  BASE1;                              
  volatile uint32_t  PREFIX0;                            
  volatile uint32_t  PREFIX1;                            
  volatile uint32_t  TXADDRESS;                          
  volatile uint32_t  RXADDRESSES;                        
  volatile uint32_t  CRCCNF;                             
  volatile uint32_t  CRCPOLY;                            
  volatile uint32_t  CRCINIT;                            
  volatile uint32_t  TEST;                               
  volatile uint32_t  TIFS;                               
  volatile const  uint32_t  RSSISAMPLE;                         
  volatile const  uint32_t  RESERVED6;
  volatile const  uint32_t  STATE;                              
  volatile uint32_t  DATAWHITEIV;                        
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  BCC;                                
  volatile const  uint32_t  RESERVED8[39];
  volatile uint32_t  DAB[8];                             
  volatile uint32_t  DAP[8];                             
  volatile uint32_t  DACNF;                              
  volatile const  uint32_t  RESERVED9[56];
  volatile uint32_t  OVERRIDE0;                          
  volatile uint32_t  OVERRIDE1;                          
  volatile uint32_t  OVERRIDE2;                          
  volatile uint32_t  OVERRIDE3;                          
  volatile uint32_t  OVERRIDE4;                          
  volatile const  uint32_t  RESERVED10[561];
  volatile uint32_t  POWER;                              
} NRF_RADIO_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile  uint32_t  TASKS_STOPRX;                       
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile  uint32_t  TASKS_STOPTX;                       
  volatile const  uint32_t  RESERVED0[3];
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile const  uint32_t  RESERVED1[56];
  volatile uint32_t  EVENTS_CTS;                         
  volatile uint32_t  EVENTS_NCTS;                        
  volatile uint32_t  EVENTS_RXDRDY;                      
  volatile const  uint32_t  RESERVED2[4];
  volatile uint32_t  EVENTS_TXDRDY;                      
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED4[7];
  volatile uint32_t  EVENTS_RXTO;                        
  volatile const  uint32_t  RESERVED5[46];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED6[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED7[93];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED8[31];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED9;
  volatile uint32_t  PSELRTS;                            
  volatile uint32_t  PSELTXD;                            
  volatile uint32_t  PSELCTS;                            
  volatile uint32_t  PSELRXD;                            
  volatile const  uint32_t  RXD;                               

 
  volatile  uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  BAUDRATE;                           
  volatile const  uint32_t  RESERVED11[17];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12[675];
  volatile uint32_t  POWER;                              
} NRF_UART_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[66];
  volatile uint32_t  EVENTS_READY;                       
  volatile const  uint32_t  RESERVED1[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELMISO;                           
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED7[681];
  volatile uint32_t  POWER;                              
} NRF_SPI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile const  uint32_t  RESERVED1[2];
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED2;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED3[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile uint32_t  EVENTS_RXDREADY;                    
  volatile const  uint32_t  RESERVED4[4];
  volatile uint32_t  EVENTS_TXDSENT;                     
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED6[4];
  volatile uint32_t  EVENTS_BB;                          
  volatile const  uint32_t  RESERVED7[3];
  volatile uint32_t  EVENTS_SUSPENDED;                   
  volatile const  uint32_t  RESERVED8[45];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED9[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED10[110];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED11[14];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  PSELSCL;                            
  volatile uint32_t  PSELSDA;                            
  volatile const  uint32_t  RESERVED13[2];
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED15[24];
  volatile uint32_t  ADDRESS;                            
  volatile const  uint32_t  RESERVED16[668];
  volatile uint32_t  POWER;                              
} NRF_TWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[9];
  volatile  uint32_t  TASKS_ACQUIRE;                      
  volatile  uint32_t  TASKS_RELEASE;                      
  volatile const  uint32_t  RESERVED1[54];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED2[8];
  volatile uint32_t  EVENTS_ACQUIRED;                    
  volatile const  uint32_t  RESERVED3[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED4[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED5[61];
  volatile const  uint32_t  SEMSTAT;                            
  volatile const  uint32_t  RESERVED6[15];
  volatile uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED7[47];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMISO;                           
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELCSN;                            
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RXDPTR;                             
  volatile uint32_t  MAXRX;                              
  volatile const  uint32_t  AMOUNTRX;                           
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  TXDPTR;                             
  volatile uint32_t  MAXTX;                              
  volatile const  uint32_t  AMOUNTTX;                           
  volatile const  uint32_t  RESERVED11;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  DEF;                                
  volatile const  uint32_t  RESERVED13[24];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED14[654];
  volatile uint32_t  POWER;                              
} NRF_SPIS_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_OUT[4];                       
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_IN[4];                       
  volatile const  uint32_t  RESERVED1[27];
  volatile uint32_t  EVENTS_PORT;                        
  volatile const  uint32_t  RESERVED2[97];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[129];
  volatile uint32_t  CONFIG[4];                          
  volatile const  uint32_t  RESERVED4[695];
  volatile uint32_t  POWER;                              
} NRF_GPIOTE_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  BUSY;                               
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_ADC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_COUNT;                        
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_SHUTDOWN;                     
  volatile const  uint32_t  RESERVED0[11];
  volatile  uint32_t  TASKS_CAPTURE[4];                   
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[126];
  volatile uint32_t  MODE;                               
  volatile uint32_t  BITMODE;                            
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED7[683];
  volatile uint32_t  POWER;                              
} NRF_TIMER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_TRIGOVRFLW;                   
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_TICK;                        
  volatile uint32_t  EVENTS_OVRFLW;                      
  volatile const  uint32_t  RESERVED1[14];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[109];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[13];
  volatile uint32_t  EVTEN;                              
  volatile uint32_t  EVTENSET;                          
 
  volatile uint32_t  EVTENCLR;                          
 
  volatile const  uint32_t  RESERVED4[110];
  volatile const  uint32_t  COUNTER;                            
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED5[13];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED6[683];
  volatile uint32_t  POWER;                              
} NRF_RTC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_DATARDY;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[127];
  volatile const  int32_t   TEMP;                               
  volatile const  uint32_t  RESERVED3[700];
  volatile uint32_t  POWER;                              
} NRF_TEMP_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_VALRDY;                      
  volatile const  uint32_t  RESERVED1[63];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[126];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  VALUE;                              
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_RNG_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTECB;                    

 
  volatile  uint32_t  TASKS_STOPECB;                     
 
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_ENDECB;                      
  volatile uint32_t  EVENTS_ERRORECB;                   
 
  volatile const  uint32_t  RESERVED1[127];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  ECBDATAPTR;                         
  volatile const  uint32_t  RESERVED3[701];
  volatile uint32_t  POWER;                              
} NRF_ECB_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                       
 
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_RESOLVED;                    
  volatile uint32_t  EVENTS_NOTRESOLVED;                 
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  NIRK;                               
  volatile uint32_t  IRKPTR;                             
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  ADDRPTR;                            
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED6[697];
  volatile uint32_t  POWER;                              
} NRF_AAR_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_KSGEN;                       
 
  volatile  uint32_t  TASKS_CRYPT;                       
 
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_ENDKSGEN;                    
  volatile uint32_t  EVENTS_ENDCRYPT;                    
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  MICSTATUS;                          
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  MODE;                               
  volatile uint32_t  CNFPTR;                             
  volatile uint32_t  INPTR;                              
  volatile uint32_t  OUTPTR;                             
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED5[697];
  volatile uint32_t  POWER;                              
} NRF_CCM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile const  uint32_t  RESERVED0[63];
  volatile uint32_t  EVENTS_TIMEOUT;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  RUNSTATUS;                          
  volatile const  uint32_t  REQSTATUS;                          
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  CRV;                                
  volatile uint32_t  RREN;                               
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED4[60];
  volatile  uint32_t  RR[8];                              
  volatile const  uint32_t  RESERVED5[631];
  volatile uint32_t  POWER;                              
} NRF_WDT_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_READCLRACC;                  
 
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_SAMPLERDY;                   
  volatile uint32_t  EVENTS_REPORTRDY;                  
 
  volatile uint32_t  EVENTS_ACCOF;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[125];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  LEDPOL;                             
  volatile uint32_t  SAMPLEPER;                          
  volatile const  int32_t   SAMPLE;                             
  volatile uint32_t  REPORTPER;                          
  volatile const  int32_t   ACC;                                
  volatile const  int32_t   ACCREAD;                           
 
  volatile uint32_t  PSELLED;                            
  volatile uint32_t  PSELA;                              
  volatile uint32_t  PSELB;                              
  volatile uint32_t  DBFEN;                              
  volatile const  uint32_t  RESERVED4[5];
  volatile uint32_t  LEDPRE;                             
  volatile const  uint32_t  ACCDBL;                             
  volatile const  uint32_t  ACCDBLREAD;                        
 
  volatile const  uint32_t  RESERVED5[684];
  volatile uint32_t  POWER;                              
} NRF_QDEC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_SAMPLE;                       
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_DOWN;                        
  volatile uint32_t  EVENTS_UP;                          
  volatile uint32_t  EVENTS_CROSS;                       
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  PSEL;                               
  volatile uint32_t  REFSEL;                             
  volatile uint32_t  EXTREFSEL;                          
  volatile const  uint32_t  RESERVED5[4];
  volatile uint32_t  ANADETECT;                          
  volatile const  uint32_t  RESERVED6[694];
  volatile uint32_t  POWER;                              
} NRF_LPCOMP_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  UNUSED;                             
} NRF_SWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[256];
  volatile const  uint32_t  READY;                              
  volatile const  uint32_t  RESERVED1[64];
  volatile uint32_t  CONFIG;                             
  volatile uint32_t  ERASEPAGE;                          
  volatile uint32_t  ERASEALL;                           
  volatile uint32_t  ERASEPROTECTEDPAGE;                 
  volatile uint32_t  ERASEUICR;                          
} NRF_NVMC_Type;


 
 
 




 

typedef struct {                                     
  PPI_TASKS_CHG_Type TASKS_CHG[4];                   
  volatile const  uint32_t  RESERVED0[312];
  volatile uint32_t  CHEN;                               
  volatile uint32_t  CHENSET;                            
  volatile uint32_t  CHENCLR;                            
  volatile const  uint32_t  RESERVED1;
  PPI_CH_Type CH[16];                                
  volatile const  uint32_t  RESERVED2[156];
  volatile uint32_t  CHG[4];                             
} NRF_PPI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile const  uint32_t  CODEPAGESIZE;                       
  volatile const  uint32_t  CODESIZE;                           
  volatile const  uint32_t  RESERVED1[4];
  volatile const  uint32_t  CLENR0;                             
  volatile const  uint32_t  PPFC;                               
  volatile const  uint32_t  RESERVED2;
  volatile const  uint32_t  NUMRAMBLOCK;                        
  
  union {
    volatile const  uint32_t  SIZERAMBLOCK[4];                 

 
    volatile const  uint32_t  SIZERAMBLOCKS;                    
  };
  volatile const  uint32_t  RESERVED3[5];
  volatile const  uint32_t  CONFIGID;                           
  volatile const  uint32_t  DEVICEID[2];                        
  volatile const  uint32_t  RESERVED4[6];
  volatile const  uint32_t  ER[4];                              
  volatile const  uint32_t  IR[4];                              
  volatile const  uint32_t  DEVICEADDRTYPE;                     
  volatile const  uint32_t  DEVICEADDR[2];                      
  volatile const  uint32_t  OVERRIDEEN;                         
  volatile const  uint32_t  NRF_1MBIT[5];                      
 
  volatile const  uint32_t  RESERVED5[10];
  volatile const  uint32_t  BLE_1MBIT[5];                      
 
} NRF_FICR_Type;


 
 
 




 

typedef struct {                                     
  volatile uint32_t  CLENR0;                             
  volatile uint32_t  RBPCONF;                            
  volatile uint32_t  XTALFREQ;                           
  volatile const  uint32_t  RESERVED0;
  volatile const  uint32_t  FWID;                               
  volatile uint32_t  BOOTLOADERADDR;                     
} NRF_UICR_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[321];
  volatile uint32_t  OUT;                                
  volatile uint32_t  OUTSET;                             
  volatile uint32_t  OUTCLR;                             
  volatile const  uint32_t  IN;                                 
  volatile uint32_t  DIR;                                
  volatile uint32_t  DIRSET;                             
  volatile uint32_t  DIRCLR;                             
  volatile const  uint32_t  RESERVED1[120];
  volatile uint32_t  PIN_CNF[32];                        
} NRF_GPIO_Type;


 

  #pragma pop
#line 1138 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"




 
 
 

#line 1179 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"


 
 
 

#line 1218 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51.h"


   
   
   








#line 38 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"




























 



 

#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
 







 

























 










#line 151 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"



#line 697 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"



#line 36 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
 

 






 






 






 
 

 






 






 






 
 

 



 
 

 





 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 





 
 

 






 
#line 184 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 192 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 201 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 






 
 

 



 
 

 






 
 

 
 

 
#line 243 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 255 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 267 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 279 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 291 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 303 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 315 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 327 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 342 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 354 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 366 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 378 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 390 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 402 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 414 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 426 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 441 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 453 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 465 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 477 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 489 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 501 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 513 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 525 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 540 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 552 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 564 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 576 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 588 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 600 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 612 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 624 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 639 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 651 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 663 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 675 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 687 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 699 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 711 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 723 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 738 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 750 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 762 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 774 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 786 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 798 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 810 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
#line 822 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
 

 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 





 
 

 






 
 

 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 





 
 

 





 
 

 





 






 
 

 






 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 
 

 






 






 
 

 






 
 

 
 

 





 
 

 



 



 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 
#line 2683 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 






 





 






 
 

 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 






 



 






 
 

 






 
 

 
 

 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 
 

 
#line 2950 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 2965 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 
 

 





 
 

 
 

 





 
 

 






 
 

 





 
 

 






 
 

 
 

 






 
 

 






 
 

 



 



 



 



 



 



 



 
 

 





 





 





 





 
 

 




 
 

 
#line 3739 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 





 
 

 



 
 

 





 





 





 





 
 

 





 
 

 





 





 





 





 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 
 

 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 







 
 

 
 

 





 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 
#line 4863 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 
#line 4885 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 
#line 5170 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 
#line 5181 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 



 



 



 
 

 





 





 



 



 



 
 

 



 



 



 



 
 

 



 



 



 



 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 





 
#line 5336 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 





 





 
 

 



 
 

 



 
 

 
#line 5395 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 



 



 



 



 



 



 



 





 





 





 





 





 





 





 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 



 
 

 






 
 

 
 

 





 
 

 






 
 

 






 
 

 





 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 





 





 





 





 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 



 
 

 



 
 

 
#line 5914 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 





 





 





 
 

 






 
 

 
 

 





 
 

 






 






 
 

 






 






 
 

 
#line 6002 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 
#line 6270 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 






 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 
#line 6647 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_bitfields.h"

 
 

 





 





 
 

 






 
 

 
 

 





 





 
 

 





 
 

 




 
 

 
 

 






 
 

 






 
 

 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 
 

 




 
 

 






 
#line 39 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_deprecated.h"




























 



 




 

 
 

 
 
 




 
 
 
 




 




 




 





 
 
 

 




 




 






 
 




 


 




 




 




 
 
#line 129 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_deprecated.h"
 
#line 162 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_deprecated.h"
 




 
#line 424 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf51_deprecated.h"



 



#line 40 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf.h"




#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\compiler_abstraction.h"




























 



 


    



    



    

  
#line 90 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\compiler_abstraction.h"

 

#line 45 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf.h"





#line 5 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_delay.h"

 

static __asm void __inline nrf_delay_us(uint32_t volatile number_of_us)
{
loop
        SUBS    R0, R0, #1
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        BNE    loop
        BX     LR
}
#line 71 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_delay.h"

void nrf_delay_ms(uint32_t volatile number_of_ms);

#line 24 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\common.h"
 









 




 

#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\common.h"
#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\common.h"




 







 




 
#line 25 "..\\PCAPint_main.c"
#line 1 "..\\spi_master_config.h"










 





 





 

















 





 





 


#line 26 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\limits.h"
 
 
 





 






     

     

     

     
#line 30 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\limits.h"
       

       






#line 45 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\limits.h"
     
     


     

     

     

     

     

     

     

     

     


       

       

       




 

#line 27 "..\\PCAPint_main.c"

#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_gpio.h"



#line 5 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_gpio.h"
#line 6 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_gpio.h"












 




 
typedef enum
{
    NRF_GPIO_PORT_DIR_OUTPUT,       
    NRF_GPIO_PORT_DIR_INPUT         
} nrf_gpio_port_dir_t;




 
typedef enum
{
    NRF_GPIO_PIN_DIR_INPUT,   
    NRF_GPIO_PIN_DIR_OUTPUT   
} nrf_gpio_pin_dir_t;




 
typedef enum
{
    NRF_GPIO_PORT_SELECT_PORT0 = 0,           
    NRF_GPIO_PORT_SELECT_PORT1,               
    NRF_GPIO_PORT_SELECT_PORT2,               
    NRF_GPIO_PORT_SELECT_PORT3,               
} nrf_gpio_port_select_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOPULL   = (0x00UL),                 
    NRF_GPIO_PIN_PULLDOWN = (0x01UL),                 
    NRF_GPIO_PIN_PULLUP   = (0x03UL),                   
} nrf_gpio_pin_pull_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOSENSE    = (0x00UL),              
    NRF_GPIO_PIN_SENSE_LOW  = (0x03UL),                   
    NRF_GPIO_PIN_SENSE_HIGH = (0x02UL),                  
} nrf_gpio_pin_sense_t;











 
static __inline void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | ((0x00UL) << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((1UL) << (0UL));
    }
}













 
static __inline void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
    }
}








 
static __inline void nrf_gpio_cfg_output(uint32_t pin_number)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                            | ((0x00UL) << (8UL))
                                            | ((0x00UL) << (2UL))
                                            | ((0UL) << (1UL))
                                            | ((1UL) << (0UL));
}










 
static __inline void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}











 
static __inline void nrf_gpio_cfg_sense_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config, nrf_gpio_pin_sense_t sense_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = (sense_config << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}








 
static __inline void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction)
{
    if(direction == NRF_GPIO_PIN_DIR_INPUT)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] =
          ((0x00UL) << (16UL))
        | ((0x00UL) << (8UL))
        | ((0x00UL) << (2UL))
        | ((0UL) << (1UL))
        | ((0UL) << (0UL));
    }
    else
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->DIRSET = (1UL << pin_number);
    }
}









 
static __inline void nrf_gpio_pin_set(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = (1UL << pin_number);
}









 
static __inline void nrf_gpio_pin_clear(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = (1UL << pin_number);
}









 
static __inline void nrf_gpio_pin_toggle(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUT ^= (1UL << pin_number);
}













 
static __inline void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value)
{
    if (value == 0)
    {
        nrf_gpio_pin_clear(pin_number);
    }
    else
    {
        nrf_gpio_pin_set(pin_number);
    }
}














 
static __inline uint32_t nrf_gpio_pin_read(uint32_t pin_number)
{
    return  ((((NRF_GPIO_Type *) 0x50000000UL)->IN >> pin_number) & 1UL);
}














 
static __inline void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value)
{
    *((volatile uint8_t*)(word_address) + byte_no) = value;
}













 
static __inline uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no)
{
    return (*((const volatile uint8_t*)(word_address) + byte_no));
}







 
static __inline void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir)
{
    if (dir == NRF_GPIO_PORT_DIR_OUTPUT)
    {
        nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->DIRSET, port, 0xFF);
    }
    else
    {
        nrf_gpio_range_cfg_input(port*8, (port+1)*8-1, NRF_GPIO_PIN_NOPULL);
    }
}







 
static __inline uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port)
{
    return nrf_gpio_word_byte_read(&((NRF_GPIO_Type *) 0x50000000UL)->IN, port);
}









 
static __inline void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUT, port, value);
}











 
static __inline void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTSET, port, set_mask);
}











 
static __inline void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR, port, clr_mask);
}

 

#line 29 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards.h"










 



#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards/nrf6310.h"










 



#line 16 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards/nrf6310.h"

#line 27 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards/nrf6310.h"

#line 39 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards/nrf6310.h"












#line 17 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards.h"
#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\boards.h"

#line 30 "..\\PCAPint_main.c"
#line 31 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_interface.h"








 
 



#line 15 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_interface.h"
#line 16 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_interface.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_svc.h"







#line 31 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_svc.h"

#line 17 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_interface.h"







 



 
enum {
   
  SVC_ANT_STACK_INIT = 0xC0,
   
  SVC_ANT_EVENT_GET,
   
  SVC_ANT_CHANNEL_ASSIGN,
  SVC_ANT_CHANNEL_UNASSIGN,
  SVC_ANT_CHANNEL_OPEN,
  SVC_ANT_CHANNEL_CLOSE,
  SVC_ANT_RX_SCAN_MODE_START,
   
  SVC_ANT_TX_BROADCAST_MESSAGE,
  SVC_ANT_TX_ACKNOWLEDGED_MESSAGE,
  SVC_ANT_BURST_HANDLER_REQUEST,
  SVC_ANT_PENDING_TRANSMIT_CLEAR,
  SVC_ANT_TRANSFER_STOP,
   
  SVC_ANT_NETWORK_KEY_SET,
  SVC_ANT_CHANNEL_RADIO_FREQ_SET,
  SVC_ANT_CHANNEL_RADIO_FREQ_GET,
  SVC_ANT_CHANNEL_RADIO_TX_POWER_SET,
  SVC_ANT_PROX_SEARCH_SET,
   
  SVC_ANT_CHANNEL_PERIOD_SET,
  SVC_ANT_CHANNEL_PERIOD_GET,
  SVC_ANT_CHANNEL_ID_SET,
  SVC_ANT_CHANNEL_ID_GET,
  SVC_ANT_SEARCH_WAVEFORM_SET,
  SVC_ANT_CHANNEL_RX_SEARCH_TIMEOUT_SET,
  SVC_ANT_SEARCH_CHANNEL_PRIORITY_SET,
  SVC_ANT_ACTIVE_SEARCH_SHARING_CYCLES_SET,
  SVC_ANT_ACTIVE_SEARCH_SHARING_CYCLES_GET,
  SVC_ANT_CHANNEL_LOW_PRIO_RX_SEARCH_TIMEOUT_SET,
  SVC_ANT_ADV_BURST_CONFIG_SET,
  SVC_ANT_ADV_BURST_CONFIG_GET,
  SVC_ANT_LIB_CONFIG_SET,
  SVC_ANT_LIB_CONFIG_CLEAR,
  SVC_ANT_LIB_CONFIG_GET,
  SVC_ANT_ID_LIST_ADD,
  SVC_ANT_ID_LIST_CONFIG,
  SVC_ANT_AUTO_FREQ_HOP_TABLE_SET,
  SVC_ANT_EVENT_FILTERING_SET,
  SVC_ANT_EVENT_FILTERING_GET,
   
  SVC_ANT_ACTIVE,
  SVC_ANT_CHANNEL_IN_PROGRESS,
  SVC_ANT_CHANNEL_STATUS_GET,
  SVC_ANT_PENDING_TRANSMIT,
   
  SVC_ANT_INIT_CW_TEST_MODE,
  SVC_ANT_CW_TEST_MODE,
   
  SVC_ANT_VERSION,
   
  SVC_ANT_CAPABILITIES,
   
  SVC_ANT_BURST_HANDLER_WAIT_FLAG_ENABLE,
  SVC_ANT_BURST_HANDLER_WAIT_FLAG_DISABLE,
   
  SVC_ANT_SDU_MASK_SET,
  SVC_ANT_SDU_MASK_GET,
  SVC_ANT_SDU_MASK_CONFIG,
  SVC_ANT_CRYPTO_CHANNEL_ENABLE,
  SVC_ANT_CRYPTO_KEY_SET,
  SVC_ANT_CRYPTO_INFO_SET,
  SVC_ANT_CRYPTO_INFO_GET,
  SVC_ANT_RFACTIVE_NOTIFICATION_CONFIG_SET,
  SVC_ANT_RFACTIVE_NOTIFICATION_CONFIG_GET,
  SVC_ANT_COEX_CONFIG_SET,
  SVC_ANT_COEX_CONFIG_GET,
   
  SVC_ANT_RESERVED0,
  SVC_ANT_RESERVED1,
  SVC_ANT_RESERVED2,
   
  SVC_ANT_EXTENDED0,
  SVC_ANT_EXTENDED1,
  SVC_ANT_EXTENDED2, 
};

 

 
 

 






 
uint32_t __svc(SVC_ANT_STACK_INIT) sd_ant_stack_reset (void);

 










 
uint32_t __svc(SVC_ANT_EVENT_GET) sd_ant_event_get (uint8_t *pucChannel, uint8_t *pucEvent, uint8_t *aucANTMesg);

 












 
uint32_t __svc(SVC_ANT_CHANNEL_ASSIGN) sd_ant_channel_assign (uint8_t ucChannel, uint8_t ucChannelType, uint8_t ucNetwork, uint8_t ucExtAssign);








 
uint32_t __svc(SVC_ANT_CHANNEL_UNASSIGN) sd_ant_channel_unassign (uint8_t ucChannel);








 
uint32_t __svc(SVC_ANT_CHANNEL_OPEN) sd_ant_channel_open(uint8_t ucChannel);









 
uint32_t __svc(SVC_ANT_CHANNEL_CLOSE) sd_ant_channel_close (uint8_t ucChannel);








 
uint32_t __svc(SVC_ANT_RX_SCAN_MODE_START) sd_ant_rx_scan_mode_start (uint8_t ucSyncChannelPacketsOnly);

 















 
uint32_t __svc(SVC_ANT_TX_BROADCAST_MESSAGE) sd_ant_broadcast_message_tx (uint8_t ucChannel, uint8_t ucSize, uint8_t *aucMesg);















 
uint32_t __svc(SVC_ANT_TX_ACKNOWLEDGED_MESSAGE) sd_ant_acknowledge_message_tx (uint8_t ucChannel, uint8_t ucSize, uint8_t *aucMesg);























 
uint32_t __svc(SVC_ANT_BURST_HANDLER_REQUEST) sd_ant_burst_handler_request(uint8_t ucChannel, uint16_t usSize, uint8_t *aucData, uint8_t ucBurstSegment);









 
uint32_t __svc(SVC_ANT_PENDING_TRANSMIT_CLEAR) sd_ant_pending_transmit_clear (uint8_t ucChannel, uint8_t *pucSuccess);






 
uint32_t __svc(SVC_ANT_TRANSFER_STOP) sd_ant_transfer_stop (void);

 









 
uint32_t __svc(SVC_ANT_NETWORK_KEY_SET) sd_ant_network_address_set (uint8_t ucNetwork, uint8_t *aucNetworkKey);








 
uint32_t __svc(SVC_ANT_CHANNEL_RADIO_FREQ_SET) sd_ant_channel_radio_freq_set (uint8_t ucChannel, uint8_t ucFreq);









 
uint32_t __svc(SVC_ANT_CHANNEL_RADIO_FREQ_GET) sd_ant_channel_radio_freq_get (uint8_t ucChannel, uint8_t *pucRfreq);









 
uint32_t __svc(SVC_ANT_CHANNEL_RADIO_TX_POWER_SET) sd_ant_channel_radio_tx_power_set (uint8_t ucChannel, uint8_t ucTxPower, uint8_t ucCustomTxPower);









 
uint32_t __svc(SVC_ANT_PROX_SEARCH_SET) sd_ant_prox_search_set (uint8_t ucChannel, uint8_t ucProxThreshold, uint8_t ucCustomProxThreshold);

 








 
uint32_t __svc(SVC_ANT_CHANNEL_PERIOD_SET) sd_ant_channel_period_set (uint8_t ucChannel, uint16_t usPeriod);









 
uint32_t __svc(SVC_ANT_CHANNEL_PERIOD_GET) sd_ant_channel_period_get (uint8_t ucChannel, uint16_t *pusPeriod);










 
uint32_t __svc(SVC_ANT_CHANNEL_ID_SET) sd_ant_channel_id_set (uint8_t ucChannel, uint16_t usDeviceNumber, uint8_t ucDeviceType, uint8_t ucTransmitType);











 
uint32_t __svc(SVC_ANT_CHANNEL_ID_GET) sd_ant_channel_id_get (uint8_t ucChannel, uint16_t *pusDeviceNumber, uint8_t *pucDeviceType, uint8_t *pucTransmitType);








 
uint32_t __svc(SVC_ANT_SEARCH_WAVEFORM_SET) sd_ant_search_waveform_set (uint8_t ucChannel, uint16_t usWaveform);








 
uint32_t __svc(SVC_ANT_CHANNEL_RX_SEARCH_TIMEOUT_SET) sd_ant_channel_rx_search_timeout_set (uint8_t ucChannel, uint8_t ucTimeout);








 
uint32_t __svc(SVC_ANT_SEARCH_CHANNEL_PRIORITY_SET) sd_ant_search_channel_priority_set (uint8_t ucChannel, uint8_t ucSearchPriority);








 
uint32_t __svc(SVC_ANT_ACTIVE_SEARCH_SHARING_CYCLES_SET) sd_ant_active_search_sharing_cycles_set (uint8_t ucChannel, uint8_t ucCycles);









 
uint32_t __svc(SVC_ANT_ACTIVE_SEARCH_SHARING_CYCLES_GET) sd_ant_active_search_sharing_cycles_get (uint8_t ucChannel, uint8_t *pucCycles);








 
uint32_t __svc(SVC_ANT_CHANNEL_LOW_PRIO_RX_SEARCH_TIMEOUT_SET) sd_ant_channel_low_priority_rx_search_timeout_set (uint8_t ucChannel, uint8_t ucTimeout);




















 
uint32_t __svc(SVC_ANT_ADV_BURST_CONFIG_SET) sd_ant_adv_burst_config_set (uint8_t *aucConfig, uint8_t ucSize);

























 
uint32_t __svc(SVC_ANT_ADV_BURST_CONFIG_GET) sd_ant_adv_burst_config_get (uint8_t ucRequestType, uint8_t *aucConfig);







 
uint32_t __svc(SVC_ANT_LIB_CONFIG_SET) sd_ant_lib_config_set (uint8_t ucANTLibConfig);






 
uint32_t __svc(SVC_ANT_LIB_CONFIG_CLEAR) sd_ant_lib_config_clear (uint8_t ucANTLibConfig);







 
uint32_t __svc(SVC_ANT_LIB_CONFIG_GET) sd_ant_lib_config_get (uint8_t *pucANTLibConfig);















 
uint32_t __svc(SVC_ANT_ID_LIST_ADD) sd_ant_id_list_add (uint8_t ucChannel, uint8_t *aucDevId, uint8_t ucListIndex);










 
uint32_t __svc(SVC_ANT_ID_LIST_CONFIG) sd_ant_id_list_config (uint8_t ucChannel, uint8_t ucIDListSize, uint8_t ucIncExcFlag);










 
uint32_t __svc(SVC_ANT_AUTO_FREQ_HOP_TABLE_SET) sd_ant_auto_freq_hop_table_set (uint8_t ucChannel, uint8_t ucFreq0, uint8_t ucFreq1, uint8_t ucFreq2);






 
uint32_t __svc(SVC_ANT_EVENT_FILTERING_SET) sd_ant_event_filtering_set (uint16_t usFilter);







 
uint32_t __svc(SVC_ANT_EVENT_FILTERING_GET) sd_ant_event_filtering_get (uint16_t *pusFilter);

 







 
uint32_t __svc(SVC_ANT_ACTIVE) sd_ant_active (uint8_t *pbAntActive);







 
uint32_t __svc(SVC_ANT_CHANNEL_IN_PROGRESS) sd_ant_channel_in_progress (uint8_t *pbChannelInProgress);









 
uint32_t __svc(SVC_ANT_CHANNEL_STATUS_GET) sd_ant_channel_status_get (uint8_t ucChannel, uint8_t *pucStatus);









 
uint32_t __svc(SVC_ANT_PENDING_TRANSMIT) sd_ant_pending_transmit (uint8_t ucChannel, uint8_t *pucPending);

 






 
uint32_t __svc(SVC_ANT_INIT_CW_TEST_MODE) sd_ant_cw_test_mode_init (void);








 
uint32_t __svc(SVC_ANT_CW_TEST_MODE) sd_ant_cw_test_mode (uint8_t ucRadioFreq, uint8_t ucTxPower, uint8_t ucCustomTxPower);







 
uint32_t __svc(SVC_ANT_VERSION) sd_ant_version_get (uint8_t* aucVersion);















 
uint32_t __svc(SVC_ANT_CAPABILITIES) sd_ant_capabilities_get (uint8_t* aucCapabilities);

 










 
uint32_t __svc(SVC_ANT_BURST_HANDLER_WAIT_FLAG_ENABLE) sd_ant_burst_handler_wait_flag_enable (uint8_t* pucWaitFlag);









 
uint32_t __svc(SVC_ANT_BURST_HANDLER_WAIT_FLAG_DISABLE) sd_ant_burst_handler_wait_flag_disable (void);

 









 
uint32_t __svc(SVC_ANT_SDU_MASK_SET) sd_ant_sdu_mask_set (uint8_t ucMask, uint8_t *aucMask);









 
uint32_t __svc(SVC_ANT_SDU_MASK_GET) sd_ant_sdu_mask_get (uint8_t ucMask, uint8_t *aucMask);








 
uint32_t __svc(SVC_ANT_SDU_MASK_CONFIG) sd_ant_sdu_mask_config (uint8_t ucChannel, uint8_t ucMaskConfig);


















 
uint32_t __svc(SVC_ANT_CRYPTO_CHANNEL_ENABLE) sd_ant_crypto_channel_enable (uint8_t ucChannel, uint8_t ucEnable, uint8_t ucKeyNum, uint8_t ucDecimationRate);









 
uint32_t __svc(SVC_ANT_CRYPTO_KEY_SET) sd_ant_crypto_key_set (uint8_t ucKeyNum, uint8_t *aucKey);

 







 
uint32_t __svc(SVC_ANT_CRYPTO_INFO_SET) sd_ant_crypto_info_set (uint8_t ucType, uint8_t *aucInfo);

 






 
uint32_t __svc(SVC_ANT_CRYPTO_INFO_GET) sd_ant_crypto_info_get (uint8_t ucType, uint8_t *aucInfo);

 









 
uint32_t __svc(SVC_ANT_RFACTIVE_NOTIFICATION_CONFIG_SET) sd_ant_rfactive_notification_config_set (uint8_t ucMode, uint16_t usTimeThreshold);

 






 
uint32_t __svc(SVC_ANT_RFACTIVE_NOTIFICATION_CONFIG_GET) sd_ant_rfactive_notification_config_get (uint8_t *pucMode, uint16_t *pusTimeThreshold);

 































 
uint32_t __svc(SVC_ANT_COEX_CONFIG_SET) sd_ant_coex_config_set (uint8_t ucChannel, uint8_t *aucCoexConfig, uint8_t *aucAdvCoexConfig);

 







 
uint32_t __svc(SVC_ANT_COEX_CONFIG_GET) sd_ant_coex_config_get (uint8_t ucChannel, uint8_t *aucCoexConfig, uint8_t *aucAdvCoexConfig);









 
uint32_t __svc(SVC_ANT_EXTENDED0) sd_ant_extended0 (uint8_t ucExtID, void *pvArg0, void *pvArg1, void *pvArg2);














 











 












 








 
#line 32 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"








 




#line 15 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"





 


 




 





 

#line 42 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 60 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 






 



 

#line 83 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 95 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 107 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"




 



 






 



 

#line 134 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 146 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 160 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 174 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 


 





 

#line 195 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 




 



 






 



 





 



 














 



 

#line 257 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 

#line 273 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 




 



 



 












 



 















 



 






 



 





 



 



 






 

#line 369 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"














#line 398 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"




 











 




#line 420 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"













#line 442 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"

















#line 466 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"







 




#line 500 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"

#line 530 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 



 
















 



 


 



 

#line 612 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 


 

 
typedef union
{
   uint8_t ucExtMesgBF;
   struct
   {
      uint8_t bExtFieldCont : 1;
      uint8_t               : 1;
      uint8_t               : 1;
      uint8_t               : 1;
      uint8_t               : 1;
      uint8_t bANTTimeStamp : 1;
      uint8_t bANTRssi      : 1;
      uint8_t bANTDeviceID  : 1;
   }stExtMesgBF;

} EXT_MESG_BF; 

typedef union
{
   uint32_t ulForceAlign; 
   uint8_t aucMessage[(((uint8_t)1) + ((uint8_t)1) + ((uint8_t)1) + (((uint8_t)8) + ((uint8_t)1) + (((uint8_t)4) + ((uint8_t)4) + ((uint8_t)2) + ((uint8_t)16))) + ((uint8_t)1))]; 
   struct
   {
      uint8_t ucSize; 
      union
      {
         uint8_t aucFramedData[(((uint8_t)1) + ((uint8_t)1) + (((uint8_t)8) + ((uint8_t)1) + (((uint8_t)4) + ((uint8_t)4) + ((uint8_t)2) + ((uint8_t)16))))]; 
         struct
         {
            uint8_t ucMesgID; 
            union
            {
               uint8_t aucMesgData[((((uint8_t)8) + ((uint8_t)1) + (((uint8_t)4) + ((uint8_t)4) + ((uint8_t)2) + ((uint8_t)16))) + ((uint8_t)1))]; 
               struct
               {
                  union
                  {
                     uint8_t ucChannel; 
                     uint8_t ucSubID; 
                  }uData0;
                  uint8_t aucPayload[((uint8_t)8)]; 
                  EXT_MESG_BF sExtMesgBF; 
                  uint8_t aucExtData[(((uint8_t)4) + ((uint8_t)4) + ((uint8_t)2) + ((uint8_t)16))]; 
               }stMesgData;
            }uMesgData;
         }stFramedData;
      }uFramedData;
      uint8_t ucCheckSum; 
   }stMessage;
   
} ANT_MESSAGE;
 

 

 
#line 688 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"
 





 
#line 33 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"






 







 




#line 21 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"
#line 22 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"
#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"
#line 24 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"
#line 25 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_error_soc.h"







 
 







 

 



#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_error.h"







 
 




 

 




 




 

#line 46 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_error.h"





 
#line 24 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_error_soc.h"

 


 




 




 


 







 
#line 26 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_soc.h"


 

 



 


 




















 


 

 
enum NRF_SOC_SVCS
{
  SD_FLASH_PAGE_ERASE = (0x20),
  SD_FLASH_WRITE,
  SD_FLASH_PROTECT,
  SD_MUTEX_NEW = (0x23),
  SD_MUTEX_ACQUIRE,
  SD_MUTEX_RELEASE,
  SD_NVIC_ENABLEIRQ,
  SD_NVIC_DISABLEIRQ,
  SD_NVIC_GETPENDINGIRQ,
  SD_NVIC_SETPENDINGIRQ,
  SD_NVIC_CLEARPENDINGIRQ,
  SD_NVIC_SETPRIORITY,
  SD_NVIC_GETPRIORITY,
  SD_NVIC_SYSTEMRESET,
  SD_NVIC_CRITICAL_REGION_ENTER,
  SD_NVIC_CRITICAL_REGION_EXIT,
  SD_RAND_APPLICATION_POOL_CAPACITY,
  SD_RAND_APPLICATION_BYTES_AVAILABLE,
  SD_RAND_APPLICATION_GET_VECTOR,
  SD_POWER_MODE_SET,
  SD_POWER_SYSTEM_OFF,
  SD_POWER_RESET_REASON_GET,
  SD_POWER_RESET_REASON_CLR,
  SD_POWER_POF_ENABLE,
  SD_POWER_POF_THRESHOLD_SET,
  SD_POWER_RAMON_SET,
  SD_POWER_RAMON_CLR,
  SD_POWER_RAMON_GET,
  SD_POWER_GPREGRET_SET,
  SD_POWER_GPREGRET_CLR,
  SD_POWER_GPREGRET_GET,
  SD_POWER_DCDC_MODE_SET,
  SD_APP_EVT_WAIT,
  SD_CLOCK_HFCLK_REQUEST,
  SD_CLOCK_HFCLK_RELEASE,
  SD_CLOCK_HFCLK_IS_RUNNING,
  SD_PPI_CHANNEL_ENABLE_GET,
  SD_PPI_CHANNEL_ENABLE_SET,
  SD_PPI_CHANNEL_ENABLE_CLR,
  SD_PPI_CHANNEL_ASSIGN,
  SD_PPI_GROUP_TASK_ENABLE,
  SD_PPI_GROUP_TASK_DISABLE,
  SD_PPI_GROUP_ASSIGN,
  SD_PPI_GROUP_GET,
  SD_RADIO_NOTIFICATION_CFG_SET,
  SD_ECB_BLOCK_ENCRYPT,
  SD_RADIO_SESSION_OPEN,
  SD_RADIO_SESSION_CLOSE,
  SD_RADIO_REQUEST,
  SD_EVT_GET,
  SD_TEMP_GET,
  SVC_SOC_LAST
};

 
enum NRF_MUTEX_VALUES
{
  NRF_MUTEX_FREE,
  NRF_MUTEX_TAKEN
};

 
enum NRF_APP_PRIORITIES
{
  NRF_APP_PRIORITY_HIGH = 1,
  NRF_APP_PRIORITY_LOW = 3
};

 
enum NRF_POWER_MODES
{
  NRF_POWER_MODE_CONSTLAT,   
  NRF_POWER_MODE_LOWPWR      
};


 
enum NRF_POWER_THRESHOLDS
{
  NRF_POWER_THRESHOLD_V21,   
  NRF_POWER_THRESHOLD_V23,   
  NRF_POWER_THRESHOLD_V25,   
  NRF_POWER_THRESHOLD_V27    
};


 
enum NRF_POWER_DCDC_MODES
{
  NRF_POWER_DCDC_MODE_OFF,           
  NRF_POWER_DCDC_MODE_ON,            
  NRF_POWER_DCDC_MODE_AUTOMATIC      
};

 
enum NRF_RADIO_NOTIFICATION_DISTANCES
{
  NRF_RADIO_NOTIFICATION_DISTANCE_NONE = 0,  
  NRF_RADIO_NOTIFICATION_DISTANCE_800US,     
  NRF_RADIO_NOTIFICATION_DISTANCE_1740US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_2680US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_3620US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_4560US,    
  NRF_RADIO_NOTIFICATION_DISTANCE_5500US     
};


 
enum NRF_RADIO_NOTIFICATION_TYPES
{
  NRF_RADIO_NOTIFICATION_TYPE_NONE = 0,         
  NRF_RADIO_NOTIFICATION_TYPE_INT_ON_ACTIVE,    
  NRF_RADIO_NOTIFICATION_TYPE_INT_ON_INACTIVE,  
  NRF_RADIO_NOTIFICATION_TYPE_INT_ON_BOTH,      
};

 
enum NRF_SOC_EVTS
{
  NRF_EVT_HFCLKSTARTED,                          
  NRF_EVT_POWER_FAILURE_WARNING,                 
  NRF_EVT_FLASH_OPERATION_SUCCESS,               
  NRF_EVT_FLASH_OPERATION_ERROR,                 
  NRF_EVT_RADIO_BLOCKED,                         
  NRF_EVT_RADIO_CANCELED,                        
  NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN,  
  NRF_EVT_RADIO_SESSION_IDLE,                    
  NRF_EVT_RADIO_SESSION_CLOSED,                  
  NRF_EVT_NUMBER_OF_EVTS
};

 


 



 
typedef volatile uint8_t nrf_mutex_t;

 
typedef uint8_t nrf_app_irq_priority_t;

 
typedef uint8_t nrf_power_mode_t;

 
typedef uint8_t nrf_power_failure_threshold_t;

 
typedef uint32_t nrf_power_dcdc_mode_t;

 
typedef uint8_t nrf_radio_notification_distance_t;

 
typedef uint8_t nrf_radio_notification_type_t;

 
enum NRF_RADIO_CALLBACK_SIGNAL_TYPE
{
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_START,             
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0,             
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO,              
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED,      
  NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED    
};





 
enum NRF_RADIO_SIGNAL_CALLBACK_ACTION
{
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,             
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,           
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_END,              
  NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END   
};

 
enum NRF_RADIO_HFCLK_CFG
{
  NRF_RADIO_HFCLK_CFG_DEFAULT,                       
  NRF_RADIO_HFCLK_CFG_FORCE_XTAL,                    
};

 
enum NRF_RADIO_PRIORITY
{
  NRF_RADIO_PRIORITY_HIGH,                           
  NRF_RADIO_PRIORITY_NORMAL,                         
};

 
enum NRF_RADIO_REQUEST_TYPE
{
  NRF_RADIO_REQ_TYPE_EARLIEST,                       
  NRF_RADIO_REQ_TYPE_NORMAL                          
};

 
typedef struct
{
  uint8_t       hfclk;                               
  uint8_t       priority;                            
  uint32_t      length_us;                           
  uint32_t      timeout_us;                          
} nrf_radio_request_earliest_t;

 
typedef struct
{
  uint8_t       hfclk;                               
  uint8_t       priority;                            
  uint32_t      distance_us;                         
  uint32_t      length_us;                           
} nrf_radio_request_normal_t;

 
typedef struct
{
  uint8_t                         request_type;      
  union
  {
    nrf_radio_request_earliest_t  earliest;          
    nrf_radio_request_normal_t    normal;            
  } params;
} nrf_radio_request_t;

 
typedef struct
{
  uint8_t               callback_action;             
  union
  {
    struct
    {
      nrf_radio_request_t * p_next;                  
    } request;                                       
    struct
    {
      uint32_t              length_us;               
    } extend;                                        
  } params;
} nrf_radio_signal_callback_return_param_t;












 
typedef nrf_radio_signal_callback_return_param_t * (*nrf_radio_signal_callback_t) (uint8_t signal_type);

 
typedef struct
{
  uint8_t key[(16)];                   
  uint8_t cleartext[(16)];       
  uint8_t ciphertext[((16))];     
} nrf_ecb_hal_data_t;

 


 






 
uint32_t __svc(SD_MUTEX_NEW) sd_mutex_new(nrf_mutex_t * p_mutex);







 
uint32_t __svc(SD_MUTEX_ACQUIRE) sd_mutex_acquire(nrf_mutex_t * p_mutex);






 
uint32_t __svc(SD_MUTEX_RELEASE) sd_mutex_release(nrf_mutex_t * p_mutex);











 
uint32_t __svc(SD_NVIC_ENABLEIRQ) sd_nvic_EnableIRQ(IRQn_Type IRQn);










 
uint32_t __svc(SD_NVIC_DISABLEIRQ) sd_nvic_DisableIRQ(IRQn_Type IRQn);











 
uint32_t __svc(SD_NVIC_GETPENDINGIRQ) sd_nvic_GetPendingIRQ(IRQn_Type IRQn, uint32_t * p_pending_irq);










 
uint32_t __svc(SD_NVIC_SETPENDINGIRQ) sd_nvic_SetPendingIRQ(IRQn_Type IRQn);










 
uint32_t __svc(SD_NVIC_CLEARPENDINGIRQ) sd_nvic_ClearPendingIRQ(IRQn_Type IRQn);













 
uint32_t __svc(SD_NVIC_SETPRIORITY) sd_nvic_SetPriority(IRQn_Type IRQn, nrf_app_irq_priority_t priority);











 
uint32_t __svc(SD_NVIC_GETPRIORITY) sd_nvic_GetPriority(IRQn_Type IRQn, nrf_app_irq_priority_t * p_priority);





 
uint32_t __svc(SD_NVIC_SYSTEMRESET) sd_nvic_SystemReset(void);










 
uint32_t __svc(SD_NVIC_CRITICAL_REGION_ENTER) sd_nvic_critical_region_enter(uint8_t * p_is_nested_critical_region);









 
uint32_t __svc(SD_NVIC_CRITICAL_REGION_EXIT) sd_nvic_critical_region_exit(uint8_t is_nested_critical_region);






 
uint32_t __svc(SD_RAND_APPLICATION_POOL_CAPACITY) sd_rand_application_pool_capacity_get(uint8_t * p_pool_capacity);






 
uint32_t __svc(SD_RAND_APPLICATION_BYTES_AVAILABLE) sd_rand_application_bytes_available_get(uint8_t * p_bytes_available);








 
uint32_t __svc(SD_RAND_APPLICATION_GET_VECTOR) sd_rand_application_vector_get(uint8_t * p_buff, uint8_t length);






 
uint32_t __svc(SD_POWER_RESET_REASON_GET) sd_power_reset_reason_get(uint32_t * p_reset_reason);






 
uint32_t __svc(SD_POWER_RESET_REASON_CLR) sd_power_reset_reason_clr(uint32_t reset_reason_clr_msk);







 
uint32_t __svc(SD_POWER_MODE_SET) sd_power_mode_set(nrf_power_mode_t power_mode);




 
uint32_t __svc(SD_POWER_SYSTEM_OFF) sd_power_system_off(void);









 
uint32_t __svc(SD_POWER_POF_ENABLE) sd_power_pof_enable(uint8_t pof_enable);







 
uint32_t __svc(SD_POWER_POF_THRESHOLD_SET) sd_power_pof_threshold_set(nrf_power_failure_threshold_t threshold);






 
uint32_t __svc(SD_POWER_RAMON_SET) sd_power_ramon_set(uint32_t ramon);






 
uint32_t __svc(SD_POWER_RAMON_CLR) sd_power_ramon_clr(uint32_t ramon);






 
uint32_t __svc(SD_POWER_RAMON_GET) sd_power_ramon_get(uint32_t * p_ramon);






 
uint32_t __svc(SD_POWER_GPREGRET_SET) sd_power_gpregret_set(uint32_t gpregret_msk);






 
uint32_t __svc(SD_POWER_GPREGRET_CLR) sd_power_gpregret_clr(uint32_t gpregret_msk);






 
uint32_t __svc(SD_POWER_GPREGRET_GET) sd_power_gpregret_get(uint32_t *p_gpregret);











 
uint32_t __svc(SD_POWER_DCDC_MODE_SET) sd_power_dcdc_mode_set(nrf_power_dcdc_mode_t dcdc_mode);










 
uint32_t __svc(SD_CLOCK_HFCLK_REQUEST) sd_clock_hfclk_request(void);









 
uint32_t __svc(SD_CLOCK_HFCLK_RELEASE) sd_clock_hfclk_release(void);









 
uint32_t __svc(SD_CLOCK_HFCLK_IS_RUNNING) sd_clock_hfclk_is_running(uint32_t * p_is_running);























 
uint32_t __svc(SD_APP_EVT_WAIT) sd_app_evt_wait(void);






 
uint32_t __svc(SD_PPI_CHANNEL_ENABLE_GET) sd_ppi_channel_enable_get(uint32_t * p_channel_enable);






 
uint32_t __svc(SD_PPI_CHANNEL_ENABLE_SET) sd_ppi_channel_enable_set(uint32_t channel_enable_set_msk);






 
uint32_t __svc(SD_PPI_CHANNEL_ENABLE_CLR) sd_ppi_channel_enable_clr(uint32_t channel_enable_clr_msk);









 
uint32_t __svc(SD_PPI_CHANNEL_ASSIGN) sd_ppi_channel_assign(uint8_t channel_num, const volatile void * evt_endpoint, const volatile void * task_endpoint);







 
uint32_t __svc(SD_PPI_GROUP_TASK_ENABLE) sd_ppi_group_task_enable(uint8_t group_num);







 
uint32_t __svc(SD_PPI_GROUP_TASK_DISABLE) sd_ppi_group_task_disable(uint8_t group_num);








 
uint32_t __svc(SD_PPI_GROUP_ASSIGN) sd_ppi_group_assign(uint8_t group_num, uint32_t channel_msk);








 
uint32_t __svc(SD_PPI_GROUP_GET) sd_ppi_group_get(uint8_t group_num, uint32_t * p_channel_msk);























 
uint32_t __svc(SD_RADIO_NOTIFICATION_CFG_SET) sd_radio_notification_cfg_set(nrf_radio_notification_type_t type, nrf_radio_notification_distance_t distance);









 
uint32_t __svc(SD_ECB_BLOCK_ENCRYPT) sd_ecb_block_encrypt(nrf_ecb_hal_data_t * p_ecb_data);









 
uint32_t __svc(SD_EVT_GET) sd_evt_get(uint32_t * p_evt_id);











 
uint32_t __svc(SD_TEMP_GET) sd_temp_get(int32_t * p_temp);


























 
uint32_t __svc(SD_FLASH_WRITE) sd_flash_write(uint32_t * const p_dst, uint32_t const * const p_src, uint32_t size);
























 
uint32_t __svc(SD_FLASH_PAGE_ERASE) sd_flash_page_erase(uint32_t page_number);













 
uint32_t __svc(SD_FLASH_PROTECT) sd_flash_protect(uint32_t protenset0, uint32_t protenset1);




















 
 uint32_t __svc(SD_RADIO_SESSION_OPEN) sd_radio_session_open(nrf_radio_signal_callback_t p_radio_signal_callback);











 
 uint32_t __svc(SD_RADIO_SESSION_CLOSE) sd_radio_session_close(void);

 





























 
 uint32_t __svc(SD_RADIO_REQUEST) sd_radio_request(nrf_radio_request_t * p_request );

 



 
#line 34 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_sdm.h"







 






 

 



#line 22 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_sdm.h"
#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_sdm.h"
#line 24 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_sdm.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_error_sdm.h"







 
 






 

 



#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_error_sdm.h"










 
#line 25 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\nrf_sdm.h"


 

 


 


 

 
enum NRF_SD_SVCS
{
  SD_SOFTDEVICE_ENABLE = (0x10),  
  SD_SOFTDEVICE_DISABLE,                
  SD_SOFTDEVICE_IS_ENABLED,             
  SD_SOFTDEVICE_VECTOR_TABLE_BASE_SET,  
  SVC_SDM_LAST                          
};

 
enum NRF_CLOCK_LFCLKSRCS
{
  NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM,                        
  NRF_CLOCK_LFCLKSRC_XTAL_500_PPM,                         
  NRF_CLOCK_LFCLKSRC_XTAL_250_PPM,                         
  NRF_CLOCK_LFCLKSRC_XTAL_150_PPM,                         
  NRF_CLOCK_LFCLKSRC_XTAL_100_PPM,                         
  NRF_CLOCK_LFCLKSRC_XTAL_75_PPM,                          
  NRF_CLOCK_LFCLKSRC_XTAL_50_PPM,                          
  NRF_CLOCK_LFCLKSRC_XTAL_30_PPM,                          
  NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,                          
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION,         
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_500MS_CALIBRATION,         
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_1000MS_CALIBRATION,        
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_2000MS_CALIBRATION,        
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION,        
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION,        
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_1000MS_CALIBRATION,   
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_2000MS_CALIBRATION,   
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION,   
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_8000MS_CALIBRATION,   
  NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_16000MS_CALIBRATION,  
};

 


 

 
typedef uint32_t nrf_clock_lfclksrc_t;













 
typedef void (*softdevice_assertion_handler_t)(uint32_t pc, uint16_t line_number, const uint8_t * p_file_name);

 


 


























 
uint32_t __svc(SD_SOFTDEVICE_ENABLE) sd_softdevice_enable(nrf_clock_lfclksrc_t clock_source, softdevice_assertion_handler_t assertion_handler);













 
uint32_t __svc(SD_SOFTDEVICE_DISABLE) sd_softdevice_disable(void);






 
uint32_t __svc(SD_SOFTDEVICE_IS_ENABLED) sd_softdevice_is_enabled(uint8_t * p_softdevice_enabled);








 
uint32_t __svc(SD_SOFTDEVICE_VECTOR_TABLE_BASE_SET) sd_softdevice_vector_table_base_set(uint32_t address);

 





 
#line 35 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_error.h"










 
 







 




#line 26 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_error.h"
#line 27 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_error.h"
#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_error.h"






 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);




 









     
#line 60 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_error.h"
    



     
#line 74 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_error.h"



 
#line 36 "..\\PCAPint_main.c"
#line 37 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\pstorage.h"










 











 




#line 1 "..\\..\\capinterface_tx_prg\\pstorage_platform.h"










 

  




 



#line 23 "..\\..\\capinterface_tx_prg\\pstorage_platform.h"






















 
typedef uint32_t pstorage_block_t;

typedef struct
{
    uint32_t            module_id;       
    pstorage_block_t    block_id;        
} pstorage_handle_t;

typedef uint16_t pstorage_size_t;       

 
void pstorage_sys_event_handler (uint32_t sys_evt);



 
 
#line 29 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\pstorage.h"











 






 






 





















 
typedef void (*pstorage_ntf_cb_t)(pstorage_handle_t *  p_handle,
                                  uint8_t              op_code,
                                  uint32_t             result,
                                  uint8_t *            p_data,
                                  uint32_t             data_len);


typedef struct
{
    pstorage_ntf_cb_t cb;              
    pstorage_size_t   block_size;     




 
    pstorage_size_t   block_count;     
} pstorage_module_param_t;

 









 






 
uint32_t pstorage_init(void);























 
uint32_t pstorage_register(pstorage_module_param_t * p_module_param,
                           pstorage_handle_t *       p_block_id);
























 
uint32_t pstorage_block_identifier_get(pstorage_handle_t * p_base_id,
                                       pstorage_size_t     block_num,
                                       pstorage_handle_t * p_block_id);

























 
uint32_t pstorage_store(pstorage_handle_t * p_dest,
                        uint8_t *           p_src,
                        pstorage_size_t     size,
                        pstorage_size_t     offset);
























 
uint32_t pstorage_update(pstorage_handle_t * p_dest,
                         uint8_t *           p_src,
                         pstorage_size_t     size,
                         pstorage_size_t     offset);



















 
uint32_t pstorage_load(uint8_t *           p_dest,
                       pstorage_handle_t * p_src,
                       pstorage_size_t     size,
                       pstorage_size_t     offset);
























 
uint32_t pstorage_clear(pstorage_handle_t * p_base_id, pstorage_size_t size);










 
uint32_t pstorage_access_status_get(uint32_t * p_count);

#line 368 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\pstorage.h"

 
 



#line 38 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"










 

#line 14 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
 
 
 




 
 



 













  


 








#line 46 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 75 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 96 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   



 

   




 
#line 115 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 415 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 503 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 532 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __pure int abs(int  );
   



 

extern __declspec(__nothrow) __pure div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __pure long int labs(long int  );
   



 




extern __declspec(__nothrow) __pure ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __pure __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __pure lldiv_t lldiv(__int64  , __int64  );
   











 
#line 613 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"



 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __pure __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 



 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 867 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


 

#line 15 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 16 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 185 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 201 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 224 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 239 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 262 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 494 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"



 

#line 17 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nordic_common.h"









  



 




 



 


 


 
    

 
    


#line 68 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nordic_common.h"




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_assert.h"







 



 




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_assert.h"


















 
void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name);

    
    



 
#line 57 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\nrf_assert.h"

#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 21 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 22 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"










 








 




#line 26 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"
#line 27 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"
#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"
#line 29 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"

 
typedef enum
{
    APP_IRQ_PRIORITY_HIGH = 1,
    APP_IRQ_PRIORITY_LOW  = 3
} app_irq_priority_t;

enum
{
    UNIT_0_625_MS = 625,                                 
    UNIT_1_25_MS  = 1250,                                
    UNIT_10_MS    = 10000                                
};



 

 
















 



 
typedef uint8_t uint16_le_t[2];

 
typedef uint8_t uint32_le_t[4];

 
typedef struct
{
    uint16_t  size;                  
    uint8_t * p_data;                
} uint8_array_t;






 
#line 104 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"
    





 
#line 123 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_util.h"
    






 








 





 












 











 
static __inline uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}
    






 
static __inline uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}






 
static __inline uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) | 
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}






 
static __inline uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}

    






 
static __inline uint8_t current_int_priority_get(void)
{
    uint32_t isr_vector_num = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICSR & (0x1FFUL << 0));
    if (isr_vector_num > 0)
    {
        int32_t irq_type = ((int32_t)isr_vector_num - 16);
        return (NVIC_GetPriority((IRQn_Type)irq_type) & 0xFF);
    }
    else
    {
        return 4;
    }
}




















 
static __inline uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}






 
static __inline _Bool is_word_aligned(void * p)
{
    return (((uint32_t)p & 0x00000003) == 0);
}



 
#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"












 



 









 
#line 56 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"




 
#line 67 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"



 








 








 








 






#line 117 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"

 






 
#line 134 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"

 



 
typedef enum
{
    STATE_INIT,                   
    STATE_DATA_TO_SWAP_WRITE,     
    STATE_DATA_ERASE,             
    STATE_HEAD_RESTORE,           
    STATE_TAIL_RESTORE,           
    STATE_NEW_BODY_WRITE,         
    STATE_SWAP_ERASE,             
    STATE_COMPLETE,               
    STATE_SWAP_DIRTY              
} swap_backup_state_t;







 
typedef struct
{
    pstorage_ntf_cb_t      cb;              
    pstorage_block_t       base_id;         
    pstorage_size_t        block_size;      
    pstorage_size_t        block_count;     
    pstorage_size_t        num_of_pages;    
} pstorage_module_table_t;


#line 182 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"







 
typedef struct
{
    uint8_t                op_code;        
    pstorage_size_t        size;           
    pstorage_size_t        offset;         
    pstorage_handle_t      storage_addr;   
    uint8_t *              p_data_addr;    
} cmd_queue_element_t;










 
typedef struct
{
    uint8_t                rp;                            
    uint8_t                count;                         
    _Bool                   flash_access;                  
    cmd_queue_element_t    cmd[30];  
} cmd_queue_t;


static cmd_queue_t         m_cmd_queue;                   
static pstorage_size_t     m_next_app_instance;           
static uint32_t            m_next_page_addr;              
static pstorage_size_t     m_round_val;                   
static _Bool                m_module_initialized = 0;  
static swap_backup_state_t m_swap_state;                  


static pstorage_module_table_t     m_app_table[2];  










 
static uint32_t cmd_process(void);






 
static void app_notify (uint32_t result);










 






 
static void cmd_queue_element_init(uint32_t index)
{
    
    m_cmd_queue.cmd[index].op_code                = 0x00;
    m_cmd_queue.cmd[index].size                   = 0;
    m_cmd_queue.cmd[index].storage_addr.module_id = 2;
    m_cmd_queue.cmd[index].storage_addr.block_id  = 0;
    m_cmd_queue.cmd[index].p_data_addr            = 0;
    m_cmd_queue.cmd[index].offset                 = 0;
}




 
static void cmd_queue_init (void)
{
    uint32_t cmd_index;

    m_round_val              = 0;
    m_swap_state             = STATE_INIT;
    m_cmd_queue.rp           = 0;
    m_cmd_queue.count        = 0;
    m_cmd_queue.flash_access = 0;

    for(cmd_index = 0; cmd_index < 30; cmd_index++)
    {
        cmd_queue_element_init(cmd_index);
    }
}














 
static uint32_t cmd_queue_enqueue(uint8_t             opcode,
                                  pstorage_handle_t * p_storage_addr,
                                  uint8_t *           p_data_addr,
                                  pstorage_size_t     size,
                                  pstorage_size_t     offset)
{
    uint32_t retval;
    uint8_t  write_index = 0;

    if (m_cmd_queue.count != 30)
    {
        
        write_index = m_cmd_queue.rp + m_cmd_queue.count;

        if (write_index >= 30)
        {
            write_index -= 30;
        }

        m_cmd_queue.cmd[write_index].op_code      = opcode;
        m_cmd_queue.cmd[write_index].p_data_addr  = p_data_addr;
        m_cmd_queue.cmd[write_index].storage_addr = (*p_storage_addr);
        m_cmd_queue.cmd[write_index].size         = size;
        m_cmd_queue.cmd[write_index].offset       = offset;
        retval                                    = ((0x0) + 0);
        if (m_cmd_queue.flash_access == 0)
        {
            retval = cmd_process();
            if (retval == ((0x0) + 17))
            {
                
                retval = ((0x0) + 0);
            }
        }
        m_cmd_queue.count++;
    }
    else
    {
        retval = ((0x0) + 4);
    }

    return retval;
}






 
static uint32_t cmd_queue_dequeue(void)
{
    uint32_t retval;
    retval = ((0x0) + 0);

    
    if (m_cmd_queue.count > 0)
    {
        retval = cmd_process();
        if (retval != ((0x0) + 0))
        {
            
            
            if (retval != ((0x0) + 17))
            {
                app_notify (retval);
            }
            else
            {
                
            }
        }
    }
    else
    {
        
    }

    return retval;
}






 
static void app_notify(uint32_t result)
{
    pstorage_ntf_cb_t  ntf_cb;
    uint8_t            op_code = m_cmd_queue.cmd[m_cmd_queue.rp].op_code;

#line 409 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
    {
        ntf_cb = m_app_table[m_cmd_queue.cmd[m_cmd_queue.rp].storage_addr.module_id].cb;
    }

    
    
    
    ntf_cb(&m_cmd_queue.cmd[m_cmd_queue.rp].storage_addr,
           op_code,
           result,
           m_cmd_queue.cmd[m_cmd_queue.rp].p_data_addr,
           m_cmd_queue.cmd[m_cmd_queue.rp].size);
}






 
void pstorage_sys_event_handler(uint32_t sys_evt)
{
    uint32_t retval = ((0x0) + 0);

    
    
    
    if (m_cmd_queue.flash_access == 1)
    {
        cmd_queue_element_t * p_cmd;

        m_cmd_queue.flash_access = 0;

        if (m_swap_state == STATE_SWAP_DIRTY)
        {
            if (sys_evt == NRF_EVT_FLASH_OPERATION_SUCCESS)
            {
                m_swap_state = STATE_INIT;
            }
            else
            {
                
                
                m_module_initialized = 0;
            }

            
            retval = cmd_queue_dequeue();
            if (retval != ((0x0) + 0))
            {
                app_notify(retval);
            }
            return;
        }

        switch (sys_evt)
        {
            case NRF_EVT_FLASH_OPERATION_SUCCESS:
                {
                    p_cmd = &m_cmd_queue.cmd[m_cmd_queue.rp];
                    m_round_val++;

                    const _Bool store_finished =
                        ((p_cmd->op_code == 0x02) &&
                        ((m_round_val * 1024) >= p_cmd->size));

                    const _Bool update_finished =
                        ((p_cmd->op_code == 0x05) &&
                        (m_swap_state == STATE_COMPLETE));

                    const _Bool clear_block_finished =
                        ((p_cmd->op_code == 0x04) &&
                        (m_swap_state == STATE_COMPLETE));

                    const _Bool clear_all_finished =
                        ((p_cmd->op_code == 0x04)         &&
                        ((m_round_val * 1024) >= p_cmd->size) &&
                        (m_swap_state == STATE_INIT));

                    if  (update_finished      ||
                         clear_block_finished ||
                         clear_all_finished   ||
                         store_finished)
                    {
                        m_swap_state = STATE_INIT;

                        app_notify(retval);

                        
                        cmd_queue_element_init(m_cmd_queue.rp);
                        m_round_val = 0;
                        m_cmd_queue.count--;
                        m_cmd_queue.rp++;
                        if (m_cmd_queue.rp >= 30)
                        {
                            m_cmd_queue.rp -= 30;
                        }
                    }
                    
                    retval = cmd_queue_dequeue();
                    if (retval != ((0x0) + 0))
                    {
                        app_notify(retval);
                    }
                }
                break;

            case NRF_EVT_FLASH_OPERATION_ERROR:
                app_notify(((0x0) + 13));
                break;

            default:
                
                break;

        }
    }
}
















 
static uint32_t swap_state_process(cmd_queue_element_t * p_cmd,
                                   uint32_t              page_number,
                                   uint32_t              head_word_size,
                                   uint32_t              tail_word_size)
{
    uint32_t retval = ((0x0) + 3);

    
    
    if (m_swap_state == STATE_INIT)
    {
        if ((head_word_size == 0) && (tail_word_size == 0))
        {
            
            m_swap_state = STATE_DATA_ERASE;
        }
        else
        {
            
            m_swap_state = STATE_DATA_TO_SWAP_WRITE;
        }
    }

    switch (m_swap_state)
    {
        case STATE_DATA_TO_SWAP_WRITE:
            
            retval = sd_flash_write((uint32_t *)(((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE))),
                                    (uint32_t *)(page_number * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)),
                                    ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE) / sizeof(uint32_t));
            if (retval == ((0x0) + 0))
            {
                m_swap_state = STATE_DATA_ERASE;
            }
            break;

        case STATE_DATA_ERASE:
            
            retval = sd_flash_page_erase(page_number);
            if (retval == ((0x0) + 0))
            {
                if (head_word_size == 0)
                {
                    if (tail_word_size == 0)
                    {
                        if (p_cmd->op_code == 0x04)
                        {
                            m_swap_state = STATE_COMPLETE;
                        }
                        else
                        {
                            m_swap_state = STATE_NEW_BODY_WRITE;
                        }
                    }
                    else
                    {
                        m_swap_state = STATE_TAIL_RESTORE;
                    }
                }
                else
                {
                    m_swap_state = STATE_HEAD_RESTORE;
                }
            }
            break;

        case STATE_HEAD_RESTORE:
            
            retval = sd_flash_write((uint32_t *)(page_number * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)),
                                    (uint32_t *)((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)),
                                    head_word_size);
            if (retval == ((0x0) + 0))
            {
                if (tail_word_size == 0)
                {
                    if (p_cmd->op_code == 0x04)
                    {
                        m_swap_state = STATE_SWAP_ERASE;
                    }
                    else
                    {
                        m_swap_state = STATE_NEW_BODY_WRITE;
                    }
                }
                else
                {
                    m_swap_state = STATE_TAIL_RESTORE;
                }
            }
            break;

        case STATE_TAIL_RESTORE:
            
            retval = sd_flash_write((uint32_t *)((page_number * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) +
                                                 (head_word_size * sizeof(uint32_t)) +
                                                 p_cmd->size),
                                    (uint32_t *)(((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) +
                                                 (head_word_size * sizeof(uint32_t)) +
                                                 p_cmd->size),
                                    tail_word_size);
            if (retval == ((0x0) + 0))
            {
                if (p_cmd->op_code == 0x04)
                {
                    m_swap_state = STATE_SWAP_ERASE;
                }
                else
                {
                    m_swap_state = STATE_NEW_BODY_WRITE;
                }
            }
            break;

        case STATE_NEW_BODY_WRITE:
            
            retval = sd_flash_write((uint32_t *)((page_number * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) +
                                                 (head_word_size * sizeof(uint32_t))),
                                    (uint32_t *)p_cmd->p_data_addr,
                                    p_cmd->size / sizeof(uint32_t));
            if (retval == ((0x0) + 0))
            {
                if ((head_word_size == 0) && (tail_word_size == 0))
                {
                    m_swap_state = STATE_COMPLETE;
                }
                else
                {
                    m_swap_state = STATE_SWAP_ERASE;
                }
            }
            break;

        case STATE_SWAP_ERASE:
            
            retval = sd_flash_page_erase(((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE));
            if (retval == ((0x0) + 0))
            {
                m_swap_state = STATE_COMPLETE;
            }
            break;

        default:
            break;
    }

    return retval;
}






 
static uint32_t cmd_process(void)
{
    uint32_t              retval;
    uint32_t              storage_addr;
    cmd_queue_element_t * p_cmd;

    retval = ((0x0) + 15);

    p_cmd = &m_cmd_queue.cmd[m_cmd_queue.rp];

    storage_addr = p_cmd->storage_addr.block_id;

    switch (p_cmd->op_code)
    {
        case 0x02:
            {
                uint32_t size;
                uint32_t offset;
                uint8_t * p_data_addr = p_cmd->p_data_addr;

                offset        = (m_round_val * 1024);
                size          = p_cmd->size - offset;
                p_data_addr  += offset;
                storage_addr += (p_cmd->offset + offset);

                if (size < 1024)
                {
                    retval = sd_flash_write(((uint32_t *)storage_addr),
                                             (uint32_t *)p_data_addr,
                                             size / sizeof(uint32_t));
                }
                else
                {
                    retval = sd_flash_write(((uint32_t *)storage_addr),
                                             (uint32_t *)p_data_addr,
                                             1024 / sizeof(uint32_t));
                }
            }
            break;

        case 0x04:
            {
                
                uint32_t page_number;

                pstorage_size_t  block_size =
                    m_app_table[p_cmd->storage_addr.module_id].block_size;

                pstorage_size_t  block_count =
                    m_app_table[p_cmd->storage_addr.module_id].block_count;

                pstorage_block_t base_address =
                    m_app_table[p_cmd->storage_addr.module_id].base_id;

                
                if (((base_address == storage_addr) && (block_size * block_count == p_cmd->size)) ||
                    (p_cmd->storage_addr.module_id == (2 + 1)))
                {
                    page_number = ((storage_addr / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) + m_round_val);

                    retval = sd_flash_page_erase(page_number);
                }
                
                else
                {
                    page_number = (storage_addr / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE));

                    uint32_t head_word_size =   (
                                                storage_addr -
                                                (page_number * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE))
                                            ) / sizeof(uint32_t);

                    uint32_t tail_word_size =   (
                                                ((page_number + 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) -
                                                (storage_addr + p_cmd->size)
                                            ) / sizeof(uint32_t);

                    retval = swap_state_process(p_cmd,
                                                page_number,
                                                head_word_size,
                                                tail_word_size);
                }
            }
            break;

        case 0x05:
            {
                uint32_t page_number =  (storage_addr / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE));

                uint32_t head_word_size =   (
                                                storage_addr + p_cmd->offset -
                                                (page_number * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE))
                                            ) / sizeof(uint32_t);

                uint32_t tail_word_size =   (
                                                ((page_number + 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) -
                                                (storage_addr + p_cmd->offset + p_cmd->size)
                                            ) / sizeof(uint32_t);

                retval = swap_state_process(p_cmd, page_number, head_word_size, tail_word_size);
            }
            break;

        default:
            
            break;
    }

    if (retval == ((0x0) + 0))
    {
       m_cmd_queue.flash_access = 1;
    }

    return retval;
}
 


uint32_t pstorage_init(void)
{
    uint32_t retval;

    cmd_queue_init();

    m_next_app_instance = 0;
    m_next_page_addr    = ((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 2 - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE));
    m_round_val         = 0;

    for (uint32_t index = 0; index < 2; index++)
    {
        m_app_table[index].cb           = 0;
        m_app_table[index].block_size   = 0;
        m_app_table[index].num_of_pages = 0;
        m_app_table[index].block_count  = 0;
    }

#line 842 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\app_common\\pstorage.c"
    m_swap_state = STATE_SWAP_DIRTY;

    
    retval = sd_flash_page_erase(((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE));
    if (retval == ((0x0) + 0))
    {
        m_cmd_queue.flash_access = 1;
        m_module_initialized     = 1;
    }


    return retval;
}


uint32_t pstorage_register(pstorage_module_param_t * p_module_param,
                           pstorage_handle_t       * p_block_id)
{
    uint16_t page_count;
    uint32_t total_size;

    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_module_param) == 0) { return ((0x0) + 14); };
    if ((p_block_id) == 0) { return ((0x0) + 14); };
    if ((p_module_param->cb) == 0) { return ((0x0) + 14); };
    if (((p_module_param->block_size) > ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) || ((p_module_param->block_size) < 0x0010)) { return ((0x0) + 7); };
    if (((p_module_param->block_count) == 0) || ((m_next_page_addr + ((p_module_param->block_count) *(p_module_param->block_size)) > ((((((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR != 0xFFFFFFFF) ? (((NRF_UICR_Type *) 0x10001000UL)->BOOTLOADERADDR / ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE)) : ((NRF_FICR_Type *) 0x10000000UL)->CODESIZE) - 1) * ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE))))) { return ((0x0) + 7); };

    
    if (!((p_module_param->block_size % sizeof(uint32_t)) == 0))
    {
        return ((0x0) + 7);
    }

    if (m_next_app_instance == 2)
    {
        return ((0x0) + 4);
    }

    p_block_id->module_id = m_next_app_instance;
    p_block_id->block_id  = m_next_page_addr;

    m_app_table[m_next_app_instance].base_id     = p_block_id->block_id;
    m_app_table[m_next_app_instance].cb          = p_module_param->cb;
    m_app_table[m_next_app_instance].block_size  = p_module_param->block_size;
    m_app_table[m_next_app_instance].block_count = p_module_param->block_count;

    
    page_count = 0;
    total_size = p_module_param->block_size * p_module_param->block_count;
    do
    {
        page_count++;
        if (total_size > ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE))
        {
            total_size -= ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE);
        }
        else
        {
            total_size = 0;
        }
        m_next_page_addr += ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE);
    }
    while (total_size >= ((uint16_t)((NRF_FICR_Type *) 0x10000000UL)->CODEPAGESIZE));

    m_app_table[m_next_app_instance].num_of_pages = page_count;
    m_next_app_instance++;

    return ((0x0) + 0);
}


uint32_t pstorage_block_identifier_get(pstorage_handle_t * p_base_id,
                                       pstorage_size_t     block_num,
                                       pstorage_handle_t * p_block_id)
{
    pstorage_handle_t temp_id;

    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_base_id) == 0) { return ((0x0) + 14); };
    if ((p_block_id) == 0) { return ((0x0) + 14); };
    if ((((p_base_id)->module_id) >= 2) || (m_app_table[(p_base_id)->module_id]. cb == 0)) { return ((0x0) + 7); };

    temp_id           = (*p_base_id);
    temp_id.block_id += (block_num * (m_app_table[(p_base_id)->module_id]. block_size));

    if (((&temp_id)->block_id) >= (m_app_table[(&temp_id)->module_id]. base_id + (m_app_table[(&temp_id)->module_id]. block_count * (m_app_table[(&temp_id)->module_id]. block_size)))) { return ((0x0) + 7); };

    (*p_block_id) = temp_id;

    return ((0x0) + 0);
}


uint32_t pstorage_store(pstorage_handle_t * p_dest,
                        uint8_t           * p_src,
                        pstorage_size_t     size,
                        pstorage_size_t     offset)
{
    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_src) == 0) { return ((0x0) + 14); };
    if ((p_dest) == 0) { return ((0x0) + 14); };
    if ((((p_dest)->module_id) >= 2) || (m_app_table[(p_dest)->module_id]. cb == 0)) { return ((0x0) + 7); };
    if (((p_dest)->block_id) >= (m_app_table[(p_dest)->module_id]. base_id + (m_app_table[(p_dest)->module_id]. block_count * (m_app_table[(p_dest)->module_id]. block_size)))) { return ((0x0) + 7); };
    if(((size) == 0) || ((size) > (m_app_table[(p_dest)->module_id]. block_size))) { return ((0x0) + 7); };
    if(((size) + (offset)) > (m_app_table[(p_dest)->module_id]. block_size)) { return ((0x0) + 7); };

    
    if ((!is_word_aligned(p_src)) || (!is_word_aligned((void *)(uint32_t)offset)))
    {
        return ((0x0) + 16);
    }

    if ((!is_word_aligned((uint32_t *)p_dest->block_id)))
    {
        return ((0x0) + 16);
    }

    return cmd_queue_enqueue(0x02, p_dest, p_src, size, offset);
}


uint32_t pstorage_update(pstorage_handle_t * p_dest,
                         uint8_t *           p_src,
                         pstorage_size_t     size,
                         pstorage_size_t     offset)
{
    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_src) == 0) { return ((0x0) + 14); };
    if ((p_dest) == 0) { return ((0x0) + 14); };
    if ((((p_dest)->module_id) >= 2) || (m_app_table[(p_dest)->module_id]. cb == 0)) { return ((0x0) + 7); };
    if (((p_dest)->block_id) >= (m_app_table[(p_dest)->module_id]. base_id + (m_app_table[(p_dest)->module_id]. block_count * (m_app_table[(p_dest)->module_id]. block_size)))) { return ((0x0) + 7); };
    if(((size) == 0) || ((size) > (m_app_table[(p_dest)->module_id]. block_size))) { return ((0x0) + 7); };
    if(((size) + (offset)) > (m_app_table[(p_dest)->module_id]. block_size)) { return ((0x0) + 7); };

    
    if ((!is_word_aligned(p_src)) || (!is_word_aligned((void *)(uint32_t)offset)))
    {
        return ((0x0) + 16);
    }

    if ((!is_word_aligned((uint32_t *)p_dest->block_id)))
    {
        return ((0x0) + 16);
    }

    return cmd_queue_enqueue(0x05, p_dest, p_src, size, offset);
}


uint32_t pstorage_load(uint8_t           * p_dest,
                       pstorage_handle_t * p_src,
                       pstorage_size_t     size,
                       pstorage_size_t     offset)
{
    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_src) == 0) { return ((0x0) + 14); };
    if ((p_dest) == 0) { return ((0x0) + 14); };
    if ((((p_src)->module_id) >= 2) || (m_app_table[(p_src)->module_id]. cb == 0)) { return ((0x0) + 7); };
    if (((p_src)->block_id) >= (m_app_table[(p_src)->module_id]. base_id + (m_app_table[(p_src)->module_id]. block_count * (m_app_table[(p_src)->module_id]. block_size)))) { return ((0x0) + 7); };
    if(((size) == 0) || ((size) > (m_app_table[(p_src)->module_id]. block_size))) { return ((0x0) + 7); };
    if(((size) + (offset)) > (m_app_table[(p_src)->module_id]. block_size)) { return ((0x0) + 7); };

    
    if ((!is_word_aligned(p_dest)) || (!is_word_aligned((void *)(uint32_t)offset)))
    {
        return ((0x0) + 16);
    }

    if ((!is_word_aligned((uint32_t *)p_src->block_id)))
    {
        return ((0x0) + 16);
    }

    memcpy(p_dest, (((uint8_t *)p_src->block_id) + offset), size);

    return ((0x0) + 0);
}


uint32_t pstorage_clear(pstorage_handle_t * p_dest, pstorage_size_t size)
{
    uint32_t retval;

    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_dest) == 0) { return ((0x0) + 14); };
    if ((((p_dest)->module_id) >= 2) || (m_app_table[(p_dest)->module_id]. cb == 0)) { return ((0x0) + 7); };
    if (((p_dest)->block_id) >= (m_app_table[(p_dest)->module_id]. base_id + (m_app_table[(p_dest)->module_id]. block_count * (m_app_table[(p_dest)->module_id]. block_size)))) { return ((0x0) + 7); };

    if ((!is_word_aligned((uint32_t *)p_dest->block_id)))
    {
        return ((0x0) + 16);
    }

    if (
           !(
                ((p_dest->block_id - m_app_table[p_dest->module_id].base_id) %
                m_app_table[p_dest->module_id].block_size) == 0
            )
        )
    {
        return ((0x0) + 7);
    }

    retval = cmd_queue_enqueue(0x04, p_dest, 0, size, 0);

    return retval;
}


uint32_t pstorage_access_status_get (uint32_t * p_count)
{
    do { if (!m_module_initialized) { return ((0x0) + 8); } } while(0);
    if ((p_count) == 0) { return ((0x0) + 14); };

    (*p_count) = m_cmd_queue.count;

    return ((0x0) + 0);
}

#line 39 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"










 


















 




#line 36 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"
#line 37 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"
#line 38 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_scheduler.h"










 


































 




#line 52 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_scheduler.h"
#line 53 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_scheduler.h"










 


            
 
typedef void (*app_sched_event_handler_t)(void * p_event_data, uint16_t event_size);












 
#line 90 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\app_common\\app_scheduler.h"


















 
uint32_t app_sched_init(uint16_t max_event_size, uint16_t queue_size, void * p_evt_buffer);





 
void app_sched_execute(void);










 
uint32_t app_sched_event_put(void *                    p_event_data,
                             uint16_t                  event_size,
                             app_sched_event_handler_t handler);



 
#line 39 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"
#line 40 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"










 








 






#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"






 








 




#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_ranges.h"






 


















 













































 
#line 22 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_types.h"






 







 




#line 21 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_types.h"


 


 


 



 
 
#line 46 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_types.h"
 


 
#line 56 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_types.h"
 



 



 




 
#line 110 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_types.h"
 

 




 




 




 



 



 

 
typedef struct
{ 
    unsigned char uuid128[16];
} ble_uuid128_t;

 
typedef struct
{
    uint16_t    uuid;  
    uint8_t     type;  
} ble_uuid_t;








 
#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"






 




 




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\nrf_svc.h"
#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"



 
enum BLE_GAP_SVCS
{
  SD_BLE_GAP_ADDRESS_SET  = 0x70,   
  SD_BLE_GAP_ADDRESS_GET,                       
  SD_BLE_GAP_ADV_DATA_SET,                      
  SD_BLE_GAP_ADV_START,                         
  SD_BLE_GAP_ADV_STOP,                          
  SD_BLE_GAP_CONN_PARAM_UPDATE,                 
  SD_BLE_GAP_DISCONNECT,                        
  SD_BLE_GAP_TX_POWER_SET,                      
  SD_BLE_GAP_APPEARANCE_SET,                    
  SD_BLE_GAP_APPEARANCE_GET,                    
  SD_BLE_GAP_PPCP_SET,                          
  SD_BLE_GAP_PPCP_GET,                          
  SD_BLE_GAP_DEVICE_NAME_SET,                   
  SD_BLE_GAP_DEVICE_NAME_GET,                   
  SD_BLE_GAP_AUTHENTICATE,                      
  SD_BLE_GAP_SEC_PARAMS_REPLY,                  
  SD_BLE_GAP_AUTH_KEY_REPLY,                    
  SD_BLE_GAP_SEC_INFO_REPLY,                    
  SD_BLE_GAP_CONN_SEC_GET,                      
  SD_BLE_GAP_RSSI_START,                         
  SD_BLE_GAP_RSSI_STOP,                          
};



 


 



 




 



 



 


 



 




 


 





 
#line 116 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"
 



 
#line 128 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"
 



 



  


 




 




 



 




 



 


 



 



 


 





 



 



 


 
#line 207 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"
 


 


 


 
#line 227 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gap.h"
 


 






 
 

 

 

 

 

 

 


 


 




 



 


 


 

 
typedef struct
{
  uint8_t addr_type;                     
  uint8_t addr[6];        
} ble_gap_addr_t;






 
typedef struct
{
  uint16_t min_conn_interval;          
  uint16_t max_conn_interval;          
  uint16_t slave_latency;              
  uint16_t conn_sup_timeout;           
} ble_gap_conn_params_t;












 
typedef struct
{
  uint8_t sm : 4;                      
  uint8_t lv : 4;                      

} ble_gap_conn_sec_mode_t;



 
typedef struct
{
  ble_gap_conn_sec_mode_t sec_mode;            
  uint8_t                 encr_key_size;       
} ble_gap_conn_sec_t;



 
typedef struct
{
  uint8_t irk[16];    
} ble_gap_irk_t;


 
typedef struct
{
  ble_gap_addr_t   ** pp_addrs;         
  uint8_t             addr_count;       
  ble_gap_irk_t    ** pp_irks;          
  uint8_t             irk_count;        
} ble_gap_whitelist_t;


 
typedef struct
{
  uint8_t               type;                  
  ble_gap_addr_t*       p_peer_addr;           
  uint8_t               fp;                    
  ble_gap_whitelist_t * p_whitelist;           
  uint16_t              interval;              
  uint16_t              timeout;               
} ble_gap_adv_params_t;


 
typedef struct
{
  uint8_t    filter;                     
  uint8_t    active    : 1;              
  uint8_t    selective : 1;              
  uint16_t   interval;                   
  uint16_t   window;                     
  uint16_t   timeout;                    
} ble_gap_scan_params_t;


 
typedef struct
{
  uint16_t   timeout;                    
  uint8_t    bond    : 1;                
  uint8_t    mitm    : 1;                
  uint8_t    io_caps : 3;                
  uint8_t    oob     : 1;                
  uint8_t    min_key_size;               
  uint8_t    max_key_size;               
} ble_gap_sec_params_t;


 
typedef struct
{
  uint16_t  div;                         
  uint8_t   ltk[16];    
  uint8_t   auth : 1;                    
  uint8_t   ltk_len : 7;                 
} ble_gap_enc_info_t;


 
typedef struct
{
  uint16_t  ediv;                        
  uint8_t   rand[8];                     
} ble_gap_master_id_t;


 
typedef struct
{
  ble_gap_addr_t  addr;                        
  uint8_t         irk[16];    
} ble_gap_id_info_t;


 
typedef struct
{
  uint8_t   csrk[16];  
} ble_gap_sign_info_t;






 
enum BLE_GAP_EVTS
{
  BLE_GAP_EVT_CONNECTED  = 0x10,     
  BLE_GAP_EVT_DISCONNECTED,                      
  BLE_GAP_EVT_CONN_PARAM_UPDATE,                 
  BLE_GAP_EVT_SEC_PARAMS_REQUEST,                
  BLE_GAP_EVT_SEC_INFO_REQUEST,                  
  BLE_GAP_EVT_PASSKEY_DISPLAY,                   
  BLE_GAP_EVT_AUTH_KEY_REQUEST,                  
  BLE_GAP_EVT_AUTH_STATUS,                       
  BLE_GAP_EVT_CONN_SEC_UPDATE,                   
  BLE_GAP_EVT_TIMEOUT,                           
  BLE_GAP_EVT_RSSI_CHANGED,                      
};


 
typedef struct
{
  ble_gap_addr_t        peer_addr;               
  uint8_t               irk_match :1;            
  uint8_t               irk_match_idx  :7;       
  ble_gap_conn_params_t conn_params;             
} ble_gap_evt_connected_t;


 
typedef struct
{
  uint8_t reason;                                
} ble_gap_evt_disconnected_t;


 
typedef struct
{
  ble_gap_conn_params_t conn_params;             
} ble_gap_evt_conn_param_update_t;


 
typedef struct
{
  ble_gap_sec_params_t peer_params;              
} ble_gap_evt_sec_params_request_t;


 
typedef struct
{
  ble_gap_addr_t peer_addr;                      
  uint16_t       div;                            
  uint8_t        enc_info  : 1;                  
  uint8_t        id_info   : 1;                  
  uint8_t        sign_info : 1;                  
} ble_gap_evt_sec_info_request_t;


 
typedef struct
{
  uint8_t passkey[6];                            
} ble_gap_evt_passkey_display_t;


 
typedef struct
{
  uint8_t key_type;                              
} ble_gap_evt_auth_key_request_t;




 
typedef struct
{
  uint8_t lv1 : 1;                               
  uint8_t lv2 : 1;                               
  uint8_t lv3 : 1;                               
} ble_gap_sec_levels_t;


 
typedef struct
{
  uint8_t ltk       : 1;                         
  uint8_t ediv_rand : 1;                         
  uint8_t irk       : 1;                         
  uint8_t address   : 1;                         
  uint8_t csrk      : 1;                         
} ble_gap_sec_keys_t;


 
typedef struct
{
  uint8_t               auth_status;             
  uint8_t               error_src;               
  ble_gap_sec_levels_t  sm1_levels;              
  ble_gap_sec_levels_t  sm2_levels;              
  ble_gap_sec_keys_t    periph_kex;              
  ble_gap_sec_keys_t    central_kex;             
  struct periph_keys_t
  {
    ble_gap_enc_info_t    enc_info;              
  } periph_keys;                                  
  struct central_keys_t
  {
    ble_gap_irk_t         irk;                   
    ble_gap_addr_t        id_info;               
  } central_keys;                                
} ble_gap_evt_auth_status_t;


 
typedef struct
{
  ble_gap_conn_sec_t conn_sec;                   
} ble_gap_evt_conn_sec_update_t;


 
typedef struct
{
  uint8_t src;                                   
} ble_gap_evt_timeout_t;


 
typedef struct
{
  int8_t  rssi;                                
} ble_gap_evt_rssi_changed_t;



 
typedef struct
{
  uint16_t conn_handle;                                      
  union                                                      
  {
    ble_gap_evt_connected_t          connected;              
    ble_gap_evt_disconnected_t       disconnected;           
    ble_gap_evt_conn_param_update_t  conn_param_update;      
    ble_gap_evt_sec_params_request_t sec_params_request;     
    ble_gap_evt_sec_info_request_t   sec_info_request;       
    ble_gap_evt_passkey_display_t    passkey_display;        
    ble_gap_evt_auth_key_request_t   auth_key_request;       
    ble_gap_evt_auth_status_t        auth_status;            
    ble_gap_evt_conn_sec_update_t    conn_sec_update;        
    ble_gap_evt_timeout_t            timeout;                
    ble_gap_evt_rssi_changed_t       rssi_changed;           
  } params;

} ble_gap_evt_t;










 
uint32_t __svc(SD_BLE_GAP_ADDRESS_SET) sd_ble_gap_address_set(ble_gap_addr_t const * const p_addr);








 
uint32_t __svc(SD_BLE_GAP_ADDRESS_GET) sd_ble_gap_address_get(ble_gap_addr_t * const p_addr);

























 
uint32_t __svc(SD_BLE_GAP_ADV_DATA_SET) sd_ble_gap_adv_data_set(uint8_t const * const p_data, uint8_t dlen, uint8_t const * const p_sr_data, uint8_t srdlen);












 
uint32_t __svc(SD_BLE_GAP_ADV_START) sd_ble_gap_adv_start(ble_gap_adv_params_t const * const p_adv_params);






 
uint32_t __svc(SD_BLE_GAP_ADV_STOP) sd_ble_gap_adv_stop(void);






















 
uint32_t __svc(SD_BLE_GAP_CONN_PARAM_UPDATE) sd_ble_gap_conn_param_update(uint16_t conn_handle, ble_gap_conn_params_t const * const p_conn_params);














 
uint32_t __svc(SD_BLE_GAP_DISCONNECT) sd_ble_gap_disconnect(uint16_t conn_handle, uint8_t hci_status_code);











 
uint32_t __svc(SD_BLE_GAP_TX_POWER_SET) sd_ble_gap_tx_power_set(int8_t tx_power);








 
uint32_t __svc(SD_BLE_GAP_APPEARANCE_SET) sd_ble_gap_appearance_set(uint16_t appearance);








 
uint32_t __svc(SD_BLE_GAP_APPEARANCE_GET) sd_ble_gap_appearance_get(uint16_t * const p_appearance);









 
uint32_t __svc(SD_BLE_GAP_PPCP_SET) sd_ble_gap_ppcp_set(ble_gap_conn_params_t const * const p_conn_params);








 
uint32_t __svc(SD_BLE_GAP_PPCP_GET) sd_ble_gap_ppcp_get(ble_gap_conn_params_t * const p_conn_params);












 
uint32_t __svc(SD_BLE_GAP_DEVICE_NAME_SET) sd_ble_gap_device_name_set(ble_gap_conn_sec_mode_t const * const p_write_perm, uint8_t const * const p_dev_name, uint16_t len);















 
uint32_t __svc(SD_BLE_GAP_DEVICE_NAME_GET) sd_ble_gap_device_name_get(uint8_t * const p_dev_name, uint16_t * const p_len);





















 
uint32_t __svc(SD_BLE_GAP_AUTHENTICATE) sd_ble_gap_authenticate(uint16_t conn_handle, ble_gap_sec_params_t const * const p_sec_params);


















 
uint32_t __svc(SD_BLE_GAP_SEC_PARAMS_REPLY) sd_ble_gap_sec_params_reply(uint16_t conn_handle, uint8_t sec_status, ble_gap_sec_params_t const * const p_sec_params);


















 
uint32_t __svc(SD_BLE_GAP_AUTH_KEY_REPLY) sd_ble_gap_auth_key_reply(uint16_t conn_handle, uint8_t key_type, uint8_t const * const key);

















 
uint32_t __svc(SD_BLE_GAP_SEC_INFO_REPLY) sd_ble_gap_sec_info_reply(uint16_t conn_handle, ble_gap_enc_info_t const * const p_enc_info, ble_gap_sign_info_t const * const p_sign_info);










 
uint32_t __svc(SD_BLE_GAP_CONN_SEC_GET) sd_ble_gap_conn_sec_get(uint16_t conn_handle, ble_gap_conn_sec_t * const p_conn_sec);











 
uint32_t __svc(SD_BLE_GAP_RSSI_START) sd_ble_gap_rssi_start(uint16_t conn_handle);












 
uint32_t __svc(SD_BLE_GAP_RSSI_STOP) sd_ble_gap_rssi_stop(uint16_t conn_handle);





 
#line 24 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_l2cap.h"






 




 




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_l2cap.h"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_l2cap.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_err.h"







 
 













 



#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\nrf_error.h"







 
 




 

 
#line 48 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\nrf_error.h"



 
#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_err.h"


 



 





 




 







 
#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_l2cap.h"
#line 21 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_l2cap.h"

 
enum BLE_L2CAP_SVCS 
{
  SD_BLE_L2CAP_CID_REGISTER = 0xB0,   
  SD_BLE_L2CAP_CID_UNREGISTER,                      
  SD_BLE_L2CAP_TX                                   
};


 


 

 

 


 


 


 


 

 
typedef struct
{
  uint16_t   len;                                  
  uint16_t   cid;                                  
} ble_l2cap_header_t;

 
enum BLE_L2CAP_EVTS 
{
  BLE_L2CAP_EVT_RX  = 0x70           
};


 
typedef struct
{
  ble_l2cap_header_t header;                       
  uint8_t    data[1];                              
} ble_l2cap_evt_rx_t;


 
typedef struct
{
  uint16_t conn_handle;                            
  union
  {
    ble_l2cap_evt_rx_t rx;                         
  } params;
} ble_l2cap_evt_t;












 
uint32_t __svc(SD_BLE_L2CAP_CID_REGISTER) sd_ble_l2cap_cid_register(uint16_t cid);










 
uint32_t __svc(SD_BLE_L2CAP_CID_UNREGISTER) sd_ble_l2cap_cid_unregister(uint16_t cid);


















 
uint32_t __svc(SD_BLE_L2CAP_TX) sd_ble_l2cap_tx(uint16_t conn_handle, ble_l2cap_header_t const * const p_header, uint8_t const * const p_data);






 
#line 25 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatt.h"






 
 



 




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatt.h"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatt.h"



 

 


 



 



 

 


 
#line 47 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatt.h"
 


 


 


 



 


 
#line 95 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatt.h"
 




 
#line 129 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatt.h"
 



 


 

 

 
typedef struct
{
   
  uint8_t broadcast       :1;  
  uint8_t read            :1;  
  uint8_t write_wo_resp   :1;  
  uint8_t write           :1;  
  uint8_t notify          :1;  
  uint8_t indicate        :1;  
  uint8_t auth_signed_wr  :1;  
} ble_gatt_char_props_t;

 
typedef struct
{
   
  uint8_t reliable_wr     :1;  
  uint8_t wr_aux          :1;  
} ble_gatt_char_ext_props_t;






 
#line 26 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gattc.h"






 




 




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gattc.h"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gattc.h"
#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gattc.h"
#line 21 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gattc.h"


 
enum BLE_GATTC_SVCS
{
  SD_BLE_GATTC_PRIMARY_SERVICES_DISCOVER = 0x90,  
  SD_BLE_GATTC_RELATIONSHIPS_DISCOVER,                          
  SD_BLE_GATTC_CHARACTERISTICS_DISCOVER,                        
  SD_BLE_GATTC_DESCRIPTORS_DISCOVER,                            
  SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ,                         
  SD_BLE_GATTC_READ,                                            
  SD_BLE_GATTC_CHAR_VALUES_READ,                                
  SD_BLE_GATTC_WRITE,                                           
  SD_BLE_GATTC_HV_CONFIRM                                       
};


 


 

 

 


 

 
typedef struct
{
  uint16_t          start_handle;  
  uint16_t          end_handle;    
} ble_gattc_handle_range_t;


 
typedef struct
{
  ble_uuid_t               uuid;           
  ble_gattc_handle_range_t handle_range;   
} ble_gattc_service_t;


 
typedef struct
{
  uint16_t            handle;            
  ble_gattc_service_t included_srvc;     
} ble_gattc_include_t;


 
typedef struct
{
  ble_uuid_t              uuid;                  
  ble_gatt_char_props_t   char_props;            
  uint8_t                 char_ext_props : 1;    
  uint16_t                handle_decl;           
  uint16_t                handle_value;          
} ble_gattc_char_t;


 
typedef struct
{
  uint16_t          handle;          
  ble_uuid_t        uuid;            
} ble_gattc_desc_t;


 
typedef struct
{
  uint8_t    write_op;                  
  uint16_t   handle;                    
  uint16_t   offset;                    
  uint16_t   len;                       
  uint8_t*   p_value;                   
  uint8_t    flags;                     
} ble_gattc_write_params_t;




 
enum BLE_GATTC_EVTS
{
  BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP = 0x30,   
  BLE_GATTC_EVT_REL_DISC_RSP,                              
  BLE_GATTC_EVT_CHAR_DISC_RSP,                             
  BLE_GATTC_EVT_DESC_DISC_RSP,                             
  BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP,                 
  BLE_GATTC_EVT_READ_RSP,                                  
  BLE_GATTC_EVT_CHAR_VALS_READ_RSP,                        
  BLE_GATTC_EVT_WRITE_RSP,                                 
  BLE_GATTC_EVT_HVX,                                       
  BLE_GATTC_EVT_TIMEOUT                                    
};

 
typedef struct
{
  uint16_t             count;            
  ble_gattc_service_t services[1];       
} ble_gattc_evt_prim_srvc_disc_rsp_t;

 
typedef struct
{
  uint16_t             count;            
  ble_gattc_include_t includes[1];       
} ble_gattc_evt_rel_disc_rsp_t;

 
typedef struct
{
  uint16_t            count;           
  ble_gattc_char_t    chars[1];        
} ble_gattc_evt_char_disc_rsp_t;

 
typedef struct
{
  uint16_t            count;           
  ble_gattc_desc_t    descs[1];        
} ble_gattc_evt_desc_disc_rsp_t;

 
typedef struct 
{
  uint16_t            handle;           
  uint8_t             *p_value;        

 
} ble_gattc_handle_value_t;

 
typedef struct
{
  uint16_t                  count;             
  uint16_t                  value_len;         
  ble_gattc_handle_value_t  handle_value[1];   
} ble_gattc_evt_char_val_by_uuid_read_rsp_t;

 
typedef struct
{
  uint16_t            handle;          
  uint16_t            offset;          
  uint16_t            len;             
  uint8_t             data[1];         
} ble_gattc_evt_read_rsp_t;

 
typedef struct
{
  uint16_t            len;             
  uint8_t             values[1];       
} ble_gattc_evt_char_vals_read_rsp_t;

 
typedef struct
{
  uint16_t            handle;            
  uint8_t             write_op;          
  uint16_t            offset;            
  uint16_t            len;               
  uint8_t             data[1];           
} ble_gattc_evt_write_rsp_t;

 
typedef struct
{
  uint16_t            handle;          
  uint8_t             type;            
  uint16_t            len;             
  uint8_t             data[1];         
} ble_gattc_evt_hvx_t;

 
typedef struct
{
  uint8_t          src;                        
} ble_gattc_evt_timeout_t;

 
typedef struct
{
  uint16_t            conn_handle;                 
  uint16_t            gatt_status;                 
  uint16_t            error_handle;                
  union
  {
    ble_gattc_evt_prim_srvc_disc_rsp_t          prim_srvc_disc_rsp;          
    ble_gattc_evt_rel_disc_rsp_t                rel_disc_rsp;                
    ble_gattc_evt_char_disc_rsp_t               char_disc_rsp;               
    ble_gattc_evt_desc_disc_rsp_t               desc_disc_rsp;               
    ble_gattc_evt_char_val_by_uuid_read_rsp_t   char_val_by_uuid_read_rsp;   
    ble_gattc_evt_read_rsp_t                    read_rsp;                    
    ble_gattc_evt_char_vals_read_rsp_t          char_vals_read_rsp;          
    ble_gattc_evt_write_rsp_t                   write_rsp;                   
    ble_gattc_evt_hvx_t                         hvx;                         
    ble_gattc_evt_timeout_t                     timeout;                     
  } params;                                                                  
} ble_gattc_evt_t;


















 
uint32_t __svc(SD_BLE_GATTC_PRIMARY_SERVICES_DISCOVER) sd_ble_gattc_primary_services_discover(uint16_t conn_handle, uint16_t start_handle, ble_uuid_t const * const p_srvc_uuid);















 
uint32_t __svc(SD_BLE_GATTC_RELATIONSHIPS_DISCOVER) sd_ble_gattc_relationships_discover(uint16_t conn_handle, ble_gattc_handle_range_t const * const p_handle_range);

















 
uint32_t __svc(SD_BLE_GATTC_CHARACTERISTICS_DISCOVER) sd_ble_gattc_characteristics_discover(uint16_t conn_handle, ble_gattc_handle_range_t const * const p_handle_range);














 
uint32_t __svc(SD_BLE_GATTC_DESCRIPTORS_DISCOVER) sd_ble_gattc_descriptors_discover(uint16_t conn_handle, ble_gattc_handle_range_t const * const p_handle_range);















 
uint32_t __svc(SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ) sd_ble_gattc_char_value_by_uuid_read(uint16_t conn_handle, ble_uuid_t const * const p_uuid, ble_gattc_handle_range_t const * const p_handle_range);
















 
uint32_t __svc(SD_BLE_GATTC_READ) sd_ble_gattc_read(uint16_t conn_handle, uint16_t handle, uint16_t offset);














 
uint32_t __svc(SD_BLE_GATTC_CHAR_VALUES_READ) sd_ble_gattc_char_values_read(uint16_t conn_handle, uint16_t const * const p_handles, uint16_t handle_count);





















 
uint32_t __svc(SD_BLE_GATTC_WRITE) sd_ble_gattc_write(uint16_t conn_handle, ble_gattc_write_params_t const * const p_write_params);












 
uint32_t __svc(SD_BLE_GATTC_HV_CONFIRM) sd_ble_gattc_hv_confirm(uint16_t conn_handle, uint16_t handle);







 
#line 27 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"






 




 




#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
#line 21 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
#line 22 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"




 
enum BLE_GATTS_SVCS
{
  SD_BLE_GATTS_SERVICE_ADD = 0xA0,  
  SD_BLE_GATTS_INCLUDE_ADD,                       
  SD_BLE_GATTS_CHARACTERISTIC_ADD,                
  SD_BLE_GATTS_DESCRIPTOR_ADD,                    
  SD_BLE_GATTS_VALUE_SET,                         
  SD_BLE_GATTS_VALUE_GET,                         
  SD_BLE_GATTS_HVX,                               
  SD_BLE_GATTS_SERVICE_CHANGED,                   
  SD_BLE_GATTS_RW_AUTHORIZE_REPLY,                 
  SD_BLE_GATTS_SYS_ATTR_SET,                        
  SD_BLE_GATTS_SYS_ATTR_GET,                      
};



 

 



 


 


 


 


 



 



 
#line 80 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
 



 
#line 92 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble_gatts.h"
 


 




 


 



 


 

 
typedef struct
{
  ble_gap_conn_sec_mode_t read_perm;        
  ble_gap_conn_sec_mode_t write_perm;       
  uint8_t                 vlen       :1;    
  uint8_t                 vloc       :2;    
  uint8_t                 rd_auth    :1;     
  uint8_t                 wr_auth    :1;    
} ble_gatts_attr_md_t;


 
typedef struct
{
  ble_uuid_t*          p_uuid;           
  ble_gatts_attr_md_t* p_attr_md;        
  uint16_t             init_len;         
  uint16_t             init_offs;        
  uint16_t             max_len;          
  uint8_t*             p_value;         

 
} ble_gatts_attr_t;


 
typedef struct
{
  ble_uuid_t           srvc_uuid;        
  ble_uuid_t           char_uuid;        
  ble_uuid_t           desc_uuid;        
  uint16_t             srvc_handle;      
  uint16_t             value_handle;     
  uint8_t              type;             
} ble_gatts_attr_context_t;


 
typedef struct
{
  uint8_t          format;       
  int8_t           exponent;     
  uint16_t         unit;         
  uint8_t          name_space;   
  uint16_t         desc;         
} ble_gatts_char_pf_t;


 
typedef struct
{
  ble_gatt_char_props_t       char_props;                
  ble_gatt_char_ext_props_t   char_ext_props;            
  uint8_t*                    p_char_user_desc;          
  uint16_t                    char_user_desc_max_size;   
  uint16_t                    char_user_desc_size;        
  ble_gatts_char_pf_t*        p_char_pf;                 
  ble_gatts_attr_md_t*        p_user_desc_md;            
  ble_gatts_attr_md_t*        p_cccd_md;                 
  ble_gatts_attr_md_t*        p_sccd_md;                 
} ble_gatts_char_md_t;


 
typedef struct
{
  uint16_t          value_handle;        
  uint16_t          user_desc_handle;    
  uint16_t          cccd_handle;         
  uint16_t          sccd_handle;         
} ble_gatts_char_handles_t;


 
typedef struct
{
  uint16_t          handle;              
  uint8_t           type;                
  uint16_t          offset;              
  uint16_t*         p_len;               
  uint8_t*          p_data;              
} ble_gatts_hvx_params_t;

 
typedef struct
{
  uint16_t          gatt_status;         
  uint8_t           update : 1;          
  uint16_t          offset;              
  uint16_t          len;                 
  uint8_t*          p_data;              
} ble_gatts_read_authorize_params_t;

 
typedef struct
{
  uint16_t          gatt_status;         
} ble_gatts_write_authorize_params_t;

 
typedef struct
{
  uint8_t                               type;    
  union {
    ble_gatts_read_authorize_params_t   read;    
    ble_gatts_write_authorize_params_t  write;   
  } params;
} ble_gatts_rw_authorize_reply_params_t;




 
enum BLE_GATTS_EVTS
{
  BLE_GATTS_EVT_WRITE = 0x50,        
  BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST,              
  BLE_GATTS_EVT_SYS_ATTR_MISSING,                  
  BLE_GATTS_EVT_HVC,                               
  BLE_GATTS_EVT_SC_CONFIRM,                        
  BLE_GATTS_EVT_TIMEOUT                            
};


 
typedef struct
{
  uint16_t                    handle;              
  uint8_t                     op;                  
  ble_gatts_attr_context_t    context;             
  uint16_t                    offset;              
  uint16_t                    len;                 
  uint8_t                     data[1];             
} ble_gatts_evt_write_t;

 
typedef struct
{
  uint16_t                    handle;              
  ble_gatts_attr_context_t    context;             
  uint16_t                    offset;              
} ble_gatts_evt_read_t;

 
typedef struct
{
  uint8_t                     type;              
  union {
    ble_gatts_evt_read_t      read;              
    ble_gatts_evt_write_t     write;             
  } request;
} ble_gatts_evt_rw_authorize_request_t;

 
typedef struct
{
  uint8_t hint;
} ble_gatts_evt_sys_attr_missing_t;


 
typedef struct
{
  uint16_t          handle;                        
} ble_gatts_evt_hvc_t;

 
typedef struct
{
  uint8_t          src;                        
} ble_gatts_evt_timeout_t;


 
typedef struct
{
  uint16_t conn_handle;                                        
  union
  {
    ble_gatts_evt_write_t                 write;               
    ble_gatts_evt_rw_authorize_request_t  authorize_request;   
    ble_gatts_evt_sys_attr_missing_t      sys_attr_missing;    
    ble_gatts_evt_hvc_t                   hvc;                 
    ble_gatts_evt_timeout_t               timeout;             
  } params;
} ble_gatts_evt_t;
















 
uint32_t __svc(SD_BLE_GATTS_SERVICE_ADD) sd_ble_gatts_service_add(uint8_t type, ble_uuid_t const*const p_uuid, uint16_t *const p_handle);



















 
uint32_t __svc(SD_BLE_GATTS_INCLUDE_ADD) sd_ble_gatts_include_add(uint16_t service_handle, uint16_t inc_srvc_handle, uint16_t *const p_include_handle);























 
uint32_t __svc(SD_BLE_GATTS_CHARACTERISTIC_ADD) sd_ble_gatts_characteristic_add(uint16_t service_handle, ble_gatts_char_md_t const*const p_char_md, ble_gatts_attr_t const*const p_attr_char_value, ble_gatts_char_handles_t *const p_handles);

















 
uint32_t __svc(SD_BLE_GATTS_DESCRIPTOR_ADD) sd_ble_gatts_descriptor_add(uint16_t char_handle, ble_gatts_attr_t const * const p_attr, uint16_t* const p_handle);














 
uint32_t __svc(SD_BLE_GATTS_VALUE_SET) sd_ble_gatts_value_set(uint16_t handle, uint16_t offset, uint16_t* const p_len, uint8_t const * const p_value);
















 
uint32_t __svc(SD_BLE_GATTS_VALUE_GET) sd_ble_gatts_value_get(uint16_t handle, uint16_t offset, uint16_t *const p_len, uint8_t* const p_data);



































 
uint32_t __svc(SD_BLE_GATTS_HVX) sd_ble_gatts_hvx(uint16_t conn_handle, ble_gatts_hvx_params_t const*const p_hvx_params);




















 
uint32_t __svc(SD_BLE_GATTS_SERVICE_CHANGED) sd_ble_gatts_service_changed(uint16_t conn_handle, uint16_t start_handle, uint16_t end_handle);














 
uint32_t __svc(SD_BLE_GATTS_RW_AUTHORIZE_REPLY) sd_ble_gatts_rw_authorize_reply(uint16_t conn_handle, ble_gatts_rw_authorize_reply_params_t const*const p_rw_authorize_reply_params);



























  
uint32_t __svc(SD_BLE_GATTS_SYS_ATTR_SET) sd_ble_gatts_sys_attr_set(uint16_t conn_handle, uint8_t const*const p_sys_attr_data, uint16_t len); 

 

















  
uint32_t __svc(SD_BLE_GATTS_SYS_ATTR_GET) sd_ble_gatts_sys_attr_get(uint16_t conn_handle, uint8_t * const p_sys_attr_data, uint16_t* const p_len); 






 
#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s310\\ble.h"



 
enum BLE_COMMON_SVCS
{
  SD_BLE_EVT_GET  = 0x60,        
  SD_BLE_TX_BUFFER_COUNT_GET,            
  SD_BLE_UUID_VS_ADD,                    
  SD_BLE_UUID_DECODE,                    
  SD_BLE_UUID_ENCODE,                    
  SD_BLE_VERSION_GET,                    
  SD_BLE_USER_MEM_REPLY,                 
};


 



 


 


 




 
enum BLE_COMMON_EVTS
{
  BLE_EVT_TX_COMPLETE  = 0x01,   
  BLE_EVT_USER_MEM_REQUEST,              
  BLE_EVT_USER_MEM_RELEASE               
};

 
typedef struct
{
  uint8_t*          p_mem;       
  uint16_t          len;         
} ble_user_mem_block_t;



 
typedef struct
{
  uint8_t count;                         
} ble_evt_tx_complete_t;

 
typedef struct
{
  uint8_t                     type;      
} ble_evt_user_mem_request_t;

 
typedef struct
{
  uint8_t                     type;        
  ble_user_mem_block_t        mem_block;   
} ble_evt_user_mem_release_t;


 
typedef struct
{
  uint16_t conn_handle;                  
  union
  {
    ble_evt_tx_complete_t           tx_complete;         
    ble_evt_user_mem_request_t      user_mem_request;    
    ble_evt_user_mem_release_t      user_mem_release;    
  } params;
} ble_common_evt_t;

 
typedef struct
{
  uint16_t evt_id;                       
  uint16_t evt_len;                      
} ble_evt_hdr_t;

 
typedef struct
{
  ble_evt_hdr_t header;                  
  union
  {
    ble_common_evt_t  common_evt;          
    ble_gap_evt_t     gap_evt;             
    ble_l2cap_evt_t   l2cap_evt;           
    ble_gattc_evt_t   gattc_evt;           
    ble_gatts_evt_t   gatts_evt;           
  } evt;
} ble_evt_t;




 
typedef struct
{
  uint8_t   version_number;              
  uint16_t  company_id;                  
  uint16_t  subversion_number;           
} ble_version_t;


























 
uint32_t __svc(SD_BLE_EVT_GET) sd_ble_evt_get(uint8_t* p_dest, uint16_t *p_len);



































 
uint32_t __svc(SD_BLE_TX_BUFFER_COUNT_GET) sd_ble_tx_buffer_count_get(uint8_t* p_count);



























 
uint32_t __svc(SD_BLE_UUID_VS_ADD) sd_ble_uuid_vs_add(ble_uuid128_t const * const p_vs_uuid, uint8_t * const p_uuid_type);



















                                                  
uint32_t __svc(SD_BLE_UUID_DECODE) sd_ble_uuid_decode(uint8_t uuid_le_len, uint8_t const * const p_uuid_le, ble_uuid_t * const p_uuid);













 
uint32_t __svc(SD_BLE_UUID_ENCODE) sd_ble_uuid_encode(ble_uuid_t const * const p_uuid, uint8_t * const p_uuid_le_len, uint8_t * const p_uuid_le);











 
uint32_t __svc(SD_BLE_VERSION_GET) sd_ble_version_get(ble_version_t * p_version);












 
uint32_t __svc(SD_BLE_USER_MEM_REPLY) sd_ble_user_mem_reply(uint16_t conn_handle, ble_user_mem_block_t *p_block);






 
#line 29 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"
#line 30 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"
#line 31 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"
#line 32 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"
#line 33 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"




 
typedef void (*ble_evt_handler_t) (ble_evt_t * p_ble_evt);














 
uint32_t softdevice_ble_evt_handler_set(ble_evt_handler_t ble_evt_handler);

#line 62 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ble_stack_handler_types.h"



 
#line 41 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ant_stack_handler_types.h"










 








 






#line 28 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ant_stack_handler_types.h"




 
typedef struct
{
    uint8_t channel;                                                                       
    uint8_t event;                                                                         
    uint8_t evt_buffer[32];                                        
} ant_evt_t;

 
typedef void (*ant_evt_handler_t) (ant_evt_t * p_ant_evt);














 
uint32_t softdevice_ant_evt_handler_set(ant_evt_handler_t ant_evt_handler);

#line 66 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\ant_stack_handler_types.h"



 
#line 42 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"




 
typedef uint32_t (*softdevice_evt_schedule_func_t) (void);

 
typedef void (*sys_evt_handler_t) (uint32_t evt_id);
















 
   
#line 86 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"




























 
uint32_t softdevice_handler_init(nrf_clock_lfclksrc_t              clock_source,
                                 void *                            p_evt_buffer,
                                 uint16_t                          evt_buffer_size,
                                 softdevice_evt_schedule_func_t    evt_schedule_func);






 
uint32_t softdevice_handler_sd_disable(void);















 
uint32_t softdevice_sys_evt_handler_set(sys_evt_handler_t sys_evt_handler);



 
void intern_softdevice_events_execute(void);

static __inline void softdevice_evt_get(void * p_event_data, uint16_t event_size)
{
    do { const _Bool LOCAL_BOOLEAN_VALUE = (event_size == 0); if (!LOCAL_BOOLEAN_VALUE) { do { app_error_handler((0), 152, (uint8_t*) "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\sd_common\\softdevice_handler.h"); } while (0); } } while (0);
    intern_softdevice_events_execute();
}

static __inline uint32_t softdevice_evt_schedule(void)
{
    return app_sched_event_put(0, 0, softdevice_evt_get);
}
 



 
#line 40 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"










 

#line 14 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
#line 15 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
#line 16 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
#line 17 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
#line 18 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
#line 19 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
#line 20 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"

#line 23 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"







static softdevice_evt_schedule_func_t m_evt_schedule_func;               



static uint8_t *                      m_evt_buffer;                      



static uint16_t                       m_ble_evt_buffer_size;             


static volatile _Bool                  m_softdevice_enabled = 0;      


static ble_evt_handler_t              m_ble_evt_handler;                 



static ant_evt_handler_t              m_ant_evt_handler;                 


static sys_evt_handler_t              m_sys_evt_handler;                 










 
void softdevice_assertion_handler(uint32_t pc, uint16_t line_num, const uint8_t * file_name)
{
    ((void)(pc));
    assert_nrf_callback(line_num, file_name);
}


void intern_softdevice_events_execute(void)
{
    if (!m_softdevice_enabled)
    {
        
        

        return;
    }

    _Bool no_more_soc_evts = (m_sys_evt_handler == 0);

    _Bool no_more_ble_evts = (m_ble_evt_handler == 0);


    _Bool no_more_ant_evts = (m_ant_evt_handler == 0);


    for (;;)
    {
        uint32_t err_code;

        if (!no_more_soc_evts)
        {
            uint32_t evt_id;

            
            err_code = sd_evt_get(&evt_id);
            
            if (err_code == ((0x0) + 5))
            {
                no_more_soc_evts = 1;
            }
            else if (err_code != ((0x0) + 0))
            {
                do { app_error_handler((err_code), 105, (uint8_t*) "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"); } while (0);
            }
            else
            {
                
                m_sys_evt_handler(evt_id);
            }
        }


        
        if (!no_more_ble_evts)
        {
            
            uint16_t evt_len = m_ble_evt_buffer_size;

            err_code = sd_ble_evt_get(m_evt_buffer, &evt_len);
            if (err_code == ((0x0) + 5))
            {
                no_more_ble_evts = 1;
            }
            else if (err_code != ((0x0) + 0))
            {
                do { app_error_handler((err_code), 128, (uint8_t*) "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"); } while (0);
            }
            else
            {
                
                m_ble_evt_handler((ble_evt_t *)m_evt_buffer);
            }
        }



        
        if (!no_more_ant_evts)
        {
            
            err_code = sd_ant_event_get(&((ant_evt_t *)m_evt_buffer)->channel,
                                        &((ant_evt_t *)m_evt_buffer)->event,
                                        ((ant_evt_t *)m_evt_buffer)->evt_buffer);
            if (err_code == ((0x0) + 5))
            {
                no_more_ant_evts = 1;
            }
            else if (err_code != ((0x0) + 0))
            {
                do { app_error_handler((err_code), 152, (uint8_t*) "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"); } while (0);
            }
            else
            {
                
                m_ant_evt_handler((ant_evt_t *)m_evt_buffer);
            }
        }


        if (no_more_soc_evts)
        {
            

            
            if (no_more_ble_evts && no_more_ant_evts)
            {
                break;
            }
#line 188 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"
        }
    }
}


uint32_t softdevice_handler_init(nrf_clock_lfclksrc_t           clock_source,
                                 void *                         p_evt_buffer,
                                 uint16_t                       evt_buffer_size,
                                 softdevice_evt_schedule_func_t evt_schedule_func)
{
    uint32_t err_code;

    

    
    if (p_evt_buffer == 0)
    {
        return ((0x0) + 7);
    }
    
    
    if (!is_word_aligned(p_evt_buffer))
    {
        return ((0x0) + 7);
    }

    m_evt_buffer = (uint8_t *)p_evt_buffer;







    m_ble_evt_buffer_size = evt_buffer_size;




    
    m_evt_schedule_func = evt_schedule_func;

    
    err_code = sd_softdevice_enable(clock_source, softdevice_assertion_handler);
    if (err_code != ((0x0) + 0))
    {
        return err_code;
    }

    m_softdevice_enabled = 1;

    
    return sd_nvic_EnableIRQ(SWI2_IRQn);
}


uint32_t softdevice_handler_sd_disable(void)
{
    uint32_t err_code = sd_softdevice_disable();
 
    m_softdevice_enabled = !(err_code == ((0x0) + 0));

    return err_code;
}



uint32_t softdevice_ble_evt_handler_set(ble_evt_handler_t ble_evt_handler)
{
    if (ble_evt_handler == 0)
    {
        return ((0x0) + 14);
    }

    m_ble_evt_handler = ble_evt_handler;

    return ((0x0) + 0);
}




uint32_t softdevice_ant_evt_handler_set(ant_evt_handler_t ant_evt_handler)
{
    if (ant_evt_handler == 0)
    {
        return ((0x0) + 14);
    }

    m_ant_evt_handler = ant_evt_handler;

    return ((0x0) + 0);
}



uint32_t softdevice_sys_evt_handler_set(sys_evt_handler_t sys_evt_handler)
{
    if (sys_evt_handler == 0)
    {
        return ((0x0) + 14);
    }

    m_sys_evt_handler = sys_evt_handler;

    return ((0x0) + 0);
}





 
void SWI2_IRQHandler(void)
{
    if (m_evt_schedule_func != 0)
    {
        uint32_t err_code = m_evt_schedule_func();
        do { const uint32_t LOCAL_ERR_CODE = (err_code); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 306, (uint8_t*) "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\sd_common\\softdevice_handler.c"); } while (0); } } while (0);
    }
    else
    {
        intern_softdevice_events_execute();
    }
}
#line 41 "..\\PCAPint_main.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\nrf_assert\\nrf_assert.c"










 
#line 13 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Source\\nrf_assert\\nrf_assert.c"


void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
  (void) file_name;  
  (void) line_num;  
 
  while (1) ;
}
#line 42 "..\\PCAPint_main.c"
#line 43 "..\\PCAPint_main.c"






#line 1 "..\\ant_config.h"


 









#line 19 "..\\ant_config.h"
















#line 50 "..\\PCAPint_main.c"
#line 1 "..\\ant_setup.c"
#line 2 "..\\ant_setup.c"
#line 3 "..\\ant_setup.c"
#line 4 "..\\ant_setup.c"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 5 "..\\ant_setup.c"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"




 





 












 








 






#line 48 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"

#line 62 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"

   




 















 
#line 93 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"











 





extern __softfp unsigned __ARM_dcmp4(double  , double  );
extern __softfp unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __softfp int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __softfp int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __softfp int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __softfp int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __softfp int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __softfp int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __softfp int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __softfp int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __softfp int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __softfp int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 211 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"



   
  typedef float float_t;
  typedef double double_t;







extern const int math_errhandling;



extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __pure double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __pure double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }




    inline float _sqrtf(float __x) { return (float)sqrt(__x); }

    



 

extern __declspec(__nothrow) __pure double ceil(double  );
    
    
extern __declspec(__nothrow) __pure double fabs(double  );
    
    

extern __declspec(__nothrow) __pure double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __pure double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __pure float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 445 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __pure double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __pure float _fabsf(float);  
inline __declspec(__nothrow) __pure float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __pure float ceilf(float  );
extern __declspec(__nothrow) __pure float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 769 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );
inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }

extern __declspec(__nothrow) __pure double fmax(double  , double  );
extern __declspec(__nothrow) __pure float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __pure double fmin(double  , double  );
extern __declspec(__nothrow) __pure float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );
inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }

extern __declspec(__nothrow) __int64 llrint(double  );
extern __declspec(__nothrow) __int64 llrintf(float  );
inline __declspec(__nothrow) __int64 llrintl(long double __x)     { return llrint((double)__x); }

extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );
inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }

extern __declspec(__nothrow) __int64 llround(double  );
extern __declspec(__nothrow) __int64 llroundf(float  );
inline __declspec(__nothrow) __int64 llroundl(long double __x)     { return llround((double)__x); }

extern __declspec(__nothrow) __pure double nan(const char * );
extern __declspec(__nothrow) __pure float nanf(const char * );
inline __declspec(__nothrow) __pure long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 808 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"
extern __declspec(__nothrow) __pure double nearbyint(double  );
extern __declspec(__nothrow) __pure float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int * );
extern  float remquof(float  , float  , int * );
inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }

extern __declspec(__nothrow) __pure double round(double  );
extern __declspec(__nothrow) __pure float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __pure double trunc(double  );
extern __declspec(__nothrow) __pure float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 980 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"











#line 1182 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"



 

#line 6 "..\\ant_setup.c"

#line 8 "..\\ant_setup.c"
#line 9 "..\\ant_setup.c"
#line 10 "..\\ant_setup.c"
#line 1 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"








 

#line 406 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"








 

#line 691 "C:\\Keil\\ARM\\Device\\Nordic\\nrf51422\\Include\\s210\\ant_parameters.h"



 
#line 11 "..\\ant_setup.c"
#line 12 "..\\ant_setup.c"











 
_Bool ant_channel_tx_broadcast_setup( void ) {
	uint32_t err_code;
	uint8_t ttype;
	
	



 
	
	
	err_code = sd_ant_channel_assign((0), 
																	 ((uint8_t) 0x20), 
																	 (0), 
																	 (0)); 
	if( err_code != ((0x0) + 0) ) {
		return 1; 
	}

	
	err_code = sd_ant_channel_id_set((0), 
																	 (1), 
																	 (1),  
																	 (2)); 
	if( err_code != ((0x0) + 0) ) {
		return 1; 
	}
	
	
	err_code = sd_ant_channel_radio_freq_set( (0), 
																						(66) ); 
	if( err_code != ((0x0) + 0) ) {
		return 1; 
	}
	
	
	err_code = sd_ant_channel_period_set( (0), 
																				(819) ); 
	if( err_code != ((0x0) + 0) ) {
		return 1; 
	}
	
	
	
			

	if( err_code != ((0x0) + 0) ) {
		return 1; 
	}
	
	









 
		
	return 0; 
}
#line 51 "..\\PCAPint_main.c"

#line 1 "..\\PCAP.h"


 
 
#line 16 "..\\PCAP.h"












 

 
#line 44 "..\\PCAP.h"

 


















 


#line 73 "..\\PCAP.h"










#line 95 "..\\PCAP.h"


#line 104 "..\\PCAP.h"




 

#line 53 "..\\PCAPint_main.c"
#line 1 "..\\PCAP.c"
#line 2 "..\\PCAP.c"
#line 3 "..\\PCAP.c"
#line 4 "..\\PCAP.c"
#line 5 "..\\PCAP.c"
#line 6 "..\\PCAP.c"
#line 7 "..\\PCAP.c"
#line 1 "..\\PCAP.h"


 
 
#line 16 "..\\PCAP.h"












 

 
#line 44 "..\\PCAP.h"

 


















 


#line 73 "..\\PCAP.h"










#line 95 "..\\PCAP.h"


#line 104 "..\\PCAP.h"




 

#line 8 "..\\PCAP.c"


static uint32_t cap_t[8];  
static uint8_t tx_data[8];  
static uint8_t rx_data[8];  
static uint8_t MSG_LEN;
static uint8_t prgdata[4096] = {0x0, 0x0, 0x0, 0x62, 0x63, 0x0, 0x65, 0xBE, 0x1, 0x20, 0x26, 0x42, 0x5C, 0x48, 0xA0, 0x3, 0x21, 0xE4, 0x20, 0x31, 0xA1, 0x3, 0x21, 0xE4, 0x20, 0x31, 0x84, 0x1, 0x23, 0x63, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0B, 0x43, 0x58, 0xC0, 0xFE, 0x43, 0xC0, 0x44, 0x7A, 0x7E, 0x20, 0x0B, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0xED, 0x44, 0xC0, 0xC0, 0xC0, 0xF6, 0xFF, 0x43, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x41, 0x23, 0x94, 0xD0, 0x43, 0xEE, 0x44, 0xD2, 0x43, 0xEF, 0x44, 0x20, 0x5A, 0x70, 0x60, 0x71, 0x61, 0x78, 0x68, 0x2, 0x7A, 0xF3, 0x43, 0xC7, 0xFE, 0x41, 0xEB, 0x45, 0x5A, 0x21, 0xDF, 0x46, 0x46, 0x46, 0x46, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0x55, 0xED, 0x45, 0xEC, 0x51, 0xF4, 0x41, 0x23, 0x88, 0xEA, 0x45, 0xF5, 0x41, 0x23, 0x88 , 0xE9, 0x45, 0x1D, 0x41, 0x43, 0x58, 0xEA, 0x21, 0x99, 0xE9, 0x50, 0x46, 0xEB, 0x44, 0xA9, 0x2, 0xEB, 0x59, 0x43, 0xCA, 0xFE, 0x41, 0x5C, 0xA8, 0x3, 0xC0, 0x5A, 0xEB, 0x45, 0xEB, 0x41, 0xF2, 0x45, 0xF6, 0x41, 0x23, 0x88, 0xEA, 0x45, 0xF7, 0x41, 0x23, 0x88, 0xE9, 0x45, 0x1F, 0x41, 0x43, 0x58, 0xEA, 0x21, 0x99, 0xE9, 0x50, 0x46, 0xEB, 0x44, 0xA9, 0x2, 0xEB, 0x59, 0x43, 0xCA, 0xFE, 0x41, 0x5C, 0xA8, 0x3, 0xC0, 0x5A, 0xEB, 0x45, 0xEB, 0x41, 0xF3, 0x45, 0x2, 0xFF, 0xFF, 0xFF, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x7C, 0x7D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x7C, 0x6D, 0x45, 0x41, 0x6C, 0x7D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x6C, 0x7D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x7C, 0x6D, 0x45, 0x41, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0xF0, 0x7C, 0x6D, 0x45, 0x41, 0x2, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x2, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F, 0x2, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x2, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x2, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x5E, 0x2, 0x6A, 0xFD, 0x43, 0x40, 0x4F, 0x4F, 0x4F, 0xEB, 0x45, 0x7A, 0xF9, 0x41, 0x43, 0x58, 0xEB, 0x21, 0x99, 0xEA, 0x44, 0xC0, 0xC0, 0xC0, 0xF1, 0xFF, 0x43, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x41, 0xED, 0x45, 0xC0, 0x41, 0xC0, 0xC0, 0xC0, 0xF8, 0xFF, 0x43, 0xE9, 0x44, 0x6A, 0x1D, 0x43, 0xAB, 0x1, 0xEA, 0x58, 0x8E, 0x3, 0xEC, 0x53, 0x1D, 0x50, 0x1F, 0x44, 0xEC, 0x53, 0xED, 0x53, 0xE9, 0x43, 0xEC, 0x58, 0xAC, 0xE6, 0x8E, 0x26, 0xC0, 0xC0, 0xC0, 0xF9, 0xFF, 0x43, 0xEC, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0xED, 0x44, 0xC0, 0x41, 0xC0, 0xC0, 0xC0, 0xFC, 0xFF, 0x43, 0xE9, 0x44, 0x1D, 0x43, 0x1F, 0x59, 0xE9, 0x43, 0xED, 0x53, 0xEC, 0x53, 0x58, 0xAC, 0xF2, 0x7A, 0xC0, 0xC0, 0xC0, 0xC9, 0xFF, 0x43, 0xEC, 0x44, 0xE7, 0x44, 0xE8, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0xED, 0x44, 0x1F, 0x43, 0x4E, 0x4E, 0x4E, 0x44, 0xC0, 0xC0, 0xC0, 0xCF, 0xFF, 0x43, 0xE9, 0x44, 0x8E, 0x7, 0xC0, 0xC0, 0xC0, 0xCB, 0xFF, 0x43, 0xE9, 0x44, 0x40, 0x5D, 0x1D, 0x43, 0x1F, 0x21, 0xCA, 0xE8, 0x43, 0xEC, 0x44, 0x1D, 0x45, 0xF8, 0x43, 0xAB, 0x0C, 0xC0, 0x41, 0xED, 0x53, 0x53, 0x1F, 0x43, 0x4E, 0x4E, 0x4E, 0x44, 0xE7, 0x53, 0xC0, 0x41, 0xE8, 0x53, 0xE7, 0x53, 0x41, 0xEC, 0x45, 0xE9, 0x43, 0x5C, 0xAC, 0xD3, 0xC0, 0xC0, 0xC0, 0xCF, 0xFF, 0x43, 0xE9, 0x44, 0xE8, 0x41, 0xE9, 0x43, 0x5C, 0xA8, 0x0C, 0xC0, 0x41, 0xE8, 0x43, 0x53, 0xEC, 0x44, 0x1D, 0x44, 0x59, 0x43, 0xAB, 0xEB, 0xC8, 0x43, 0x46, 0x46, 0x46, 0x44, 0x7A, 0x8A, 0x1B, 0xC0, 0x43, 0x40, 0x5D, 0x5D, 0x90, 0x15, 0xC8, 0x45, 0xC9, 0x45, 0xF8, 0x43, 0xAA, 0x0B, 0xCA, 0x45, 0xCB, 0x45, 0xCC, 0x45, 0xCD, 0x45, 0xCE, 0x45, 0xCF, 0x45, 0x0, 0x2, 0xC0, 0x43, 0x4E, 0x4E, 0xEA, 0x44, 0xE9, 0x44, 0x8E, 0x6, 0xC0, 0x43, 0x4E, 0xEA, 0x44, 0xE9, 0x44, 0xF8, 0x43, 0xAB, 0x13, 0xC0, 0x43, 0x4E, 0xEA, 0x44, 0x4E, 0x50, 0xE9, 0x44, 0x8E, 0x8, 0xC0, 0x43, 0xEA, 0x44, 0x4E, 0x4E, 0x50, 0xE9, 0x44, 0xC0, 0xC0, 0xC0, 0xC8, 0xFF, 0x43, 0x4E, 0x4E, 0xE9, 0x51, 0x91, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x92, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x93, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x94, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x95, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x96, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x97, 0x1, 0x23, 0x5A, 0xEA, 0x43, 0xE9, 0x51, 0x2, 0xE9, 0x43, 0x46, 0x46, 0xEC, 0x44, 0x1D, 0x45, 0x2, 0x7A, 0xFA, 0x41, 0x4F, 0x4F, 0x4F, 0xE7, 0x45, 0x5A, 0xFB, 0x43, 0xE7, 0x75, 0x21, 0xCA, 0x65, 0xD0, 0x45, 0x5A, 0xFC, 0x43, 0xE7, 0x21, 0xCA, 0xD1, 0x45, 0x5A, 0xFD, 0x43, 0xE7, 0x21, 0xCA, 0xD2, 0x45, 0x79, 0x69, 0x2, 0xD7, 0xFE, 0x43, 0xE7, 0x45, 0x5D, 0xAD, 0x1, 0x5D, 0x45, 0x41, 0x2, 0x1F, 0x43, 0x1D, 0x44, 0xC0, 0x43, 0xEC, 0x51, 0xED, 0x51, 0x5D, 0xAA, 0xF2, 0x2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2, 0x1, 0x3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
int rtc_flag = 0;
	



 

uint32_t pack(uint32_t a, uint32_t b, uint8_t nb, uint32_t c, uint8_t nc, uint32_t d, uint8_t nd, uint32_t e, uint8_t ne)
    {
        uint32_t p = a;
        p = (p << nb)| b ;
        p = (p << nc)| c ;
        p = (p << nd)| d ;
        p = (p << ne)| e ;
        return p;
    }









 
_Bool pcap_spi_tx_rx(uint32_t *PCAP_spi_address, uint8_t MSG_LEN, uint8_t *tx_data)
{
    _Bool PCAP_spi_chk = spi_master_tx_rx(PCAP_spi_address, MSG_LEN, (const uint8_t *)tx_data, (uint8_t *)rx_data);
  
  if (PCAP_spi_chk)
    return 1;
    else 
        return 0;
}




 
_Bool pcap_dsp_write(uint32_t *PCAP_spi_address) 
{
    
    uint16_t regadd = 0;
    uint16_t x;
    uint8_t y;
    _Bool b;
    uint32_t p;
    MSG_LEN = 4;

    
    
        for (x = 0; x < 4096; x++)
            {
                regadd = x;                                 
                p = 0x09;      
                p = (p << 12)|regadd;   
                p = (p << 8)|prgdata[x];  
                
                
                for (y = 0; y < (3); y++)
                    {
                        tx_data[y] = (p >> ((2)-y)*8)&(0xFF);
                    }
                b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); 
                memset(tx_data, 0, 8);
                memset(rx_data, 0, 8);
                
            }
            return b;
}
    




 
    
_Bool pcap_config_write(uint32_t *PCAP_spi_address, uint32_t *regdata) 
    {
        
        
        uint8_t regadd = 0;
				uint8_t x, y;
        uint32_t p;
        _Bool b; 
        
        MSG_LEN = 4;

          
         
        memset(tx_data, 0, 8);
				memset(rx_data, 0, 8);
        p = 0x03; 
        p = (p << 6)|20; 
        p = (p << 24)|(0); 
        for (y = 0; y < (4); y++)
            {
                tx_data[y] = (p >> ((3)-y)*8) & (0xFF) ;
            }
        b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);

         
        for (x = 0; x < 20; x++)
            {
                regadd = x;                                 
                p = 0x03;      
                p = (p << 6)|regadd;   
                p = (p << 24)|regdata[x];  
                
                
                for (y = 0; y < (4); y++)
                    {
                        tx_data[y] = (p >> ((3)-y)*8)&(0xFF) ;
                    }
                b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); 
                
                
                
                memset(tx_data, 0, 8);
                memset(rx_data, 0, 8);
                
            }
            
						
			((NRF_RTC_Type *) 0x40011000UL)->CC[0] = 3276; 
			rtc_flag = 1;
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
			do 
			{ 
				
				__wfe();   
				
				__sev(); 
				__wfe();                 
			}while(rtc_flag); 
			
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1; 

            
            p = 0x03; 
            p = (p << 6)|20; 
            p = (p << 24)|(1); 
            for (y = 0; y < (4); y++)
                {
                    tx_data[y] = (p >> ((3)-y)*8) & (0xFF) ;
                    b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
                }
        b = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
        
        
        return b;
    }
    




 
_Bool config_reg_set(uint32_t *PCAP_spi_address,int c_avg,int onoff, int cy_time, int rdc_sel) 
    { 
        uint32_t config_reg_d[20];
        uint8_t DSP_PRESET, PG_PRESET;
        _Bool w; 
        
         
        config_reg_d[0] = pack((0x04), (((uint8_t) 0) << 2)|2, 4, ((uint8_t) 0x00), 8, ((uint8_t) 0x0F), 4, ((uint8_t) 0x0F),4);
                
         
        config_reg_d[1] = 0x201022;
        
         
        config_reg_d[2] = pack(((uint8_t) 0xFF), ((uint8_t) 4), 4, rdc_sel, 4, 0x0B, 8, 0, 0);
        
         
        config_reg_d[3] = pack(((uint8_t) 0), ((uint8_t) 0x0D), 6, ((uint8_t) 0), 3, c_avg, 13 , 0, 0);
        
         
        config_reg_d[4] = pack(((uint8_t) 0), ((uint8_t) 0), 2, cy_time, 10, ((uint8_t) 0 ), 4, ((((uint8_t) 0) << 2)|onoff), 4);
        
         
        config_reg_d[5] = pack(((uint8_t) 1), ((uint32_t) 1), 22, 0, 0, 0, 0, 0 ,0);
        
            
         
        config_reg_d[6] = pack(0, ((uint8_t) 0), 1, ((uint8_t) 0x0E), 7, 0x40, 8, 0, 0);
        
        
         
        config_reg_d[7] = 0x1F0000 ;
        
         
        DSP_PRESET = pack(((uint8_t) 1), ((uint8_t) 0), 1, ((uint8_t) 0), 1, ((uint8_t) 0), 1, ((uint8_t) 0), 4);
        PG_PRESET = pack(((uint8_t) 0), ((uint8_t) 0), 1, ((uint8_t) 0), 1, 0, 0, 0, 0);
        
        config_reg_d[8] = pack(DSP_PRESET, ((uint8_t) 0x00), 4, (((uint8_t) 0) << 2)|((uint8_t) 0), 4, ((uint8_t) 3), 4, PG_PRESET, 4);
        
        
         
        config_reg_d[9] = pack(((uint8_t) 0x0F), ((uint8_t) 0x0F), 4, ((uint8_t) 0x00), 4, (((uint8_t) 0x00) << 4)| ((uint8_t) 0x00), 8, (((uint8_t) 3) << 2) | ((uint8_t) 3), 4); 
        
        
         
        config_reg_d[10] = pack(0x18, 00, 8, ((uint8_t) 0x87), 8,0,0,0,0);
        
        
         
        config_reg_d[11] = 0x000000 ;
        
         
        config_reg_d[12] = 0x000000 ;
        
         
        config_reg_d[13] = 0x000000 ;
        
         
        config_reg_d[14] = 0x000000 ;
        
         
        config_reg_d[15] = 0x000000 ;

         
        config_reg_d[16] = 0x000000 ;
        
         
        config_reg_d[17] = 0x000000 ;
        
         
        config_reg_d[18] = 0x000000 ;
        
         
        config_reg_d[19] = 0x200000 ; 
                
        w = pcap_config_write(PCAP_spi_address, config_reg_d);
        return w;
    }

    






 

uint32_t read_reg(uint32_t *PCAP_spi_address, uint8_t wr_reg_add) 
    { 
        _Bool meas;
        uint32_t rx_reg_data;
        uint8_t p;
        
        MSG_LEN = 4; 
    
        
        p = 0x01;      
        p = (p << 6)|wr_reg_add;   
        tx_data[0] = p; 
        
        
        meas = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data); 
        
        rx_reg_data = pack(rx_data[1], rx_data[2], 8, rx_data[3], 8, 0, 0, 0, 0 );
            
        
        if (meas == 0)
            return meas;
        else
            return rx_reg_data;
				memset(tx_data, 0, 8);
				memset(rx_data, 0, 8);
    }





  
float data_extract(uint32_t data)
{
    
    
    
    float p = data;
    float ext= p/(pow(2,21));
    return ext;
}





 
_Bool pcap_commcheck(uint32_t *PCAP_spi_address)
{
		_Bool w; 
		 
		MSG_LEN = 8;
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);
		tx_data[0] = 0x10;	
		tx_data[1] = 0x08;				
	
		w = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
	
		if (rx_data[2] == 0x01)
		{
			return 1;
		}
		else
		{
			return 0;
		}
		
}





 
_Bool 	pcap_config(uint32_t *PCAP_spi_address, int c_avg,int onoff, int cy_time, int rdc_sel)
{
		_Bool w,w1,w2; 
		 
		w1 = config_reg_set(PCAP_spi_address, c_avg, onoff, cy_time, rdc_sel);
		
		 
		MSG_LEN = 8;
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);
		tx_data[0] = 0x8A; 
		
		
		w2 = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
	
		if (w1 && w2 == 1)
		{
			w = 1;
			return w ;
		}
		else
		{
			w = 0;
			return w = 0;
		}
}
	




 
_Bool 	pcap_measure(uint32_t *PCAP_spi_address,int c_avg, int onoff, int cy_time)
{
	_Bool w;
	uint8_t cap_n, n, pul_n;
	 
		MSG_LEN = 8;
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);
		tx_data[0] = 0x8C; 
		
		
		w = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
		 
		
		
		switch(((uint8_t) 4))
			{
				case 1: 
					n = 1;
					cap_n = 1;
					do
					{
						uint16_t chk = (((uint8_t) 0xFF) >> n) & 0x01;					
						if(chk == 1) 
						{
							cap_n++;
						}
							n++;
					}while(n < 8);
					pul_n = cap_n*2 + 1;
					break;
				case 4: 
					n = 1;
					cap_n = 1;
					do
					{
						uint16_t chk = (((uint8_t) 0xFF) >> 2*n) & 0x03;

						 
						if(chk == 3) 
						{
							cap_n++;
						}
						n++;
					} while(n < 4);
					pul_n = cap_n*3 + 1;
					 
					break;
			}
		switch(onoff)
		{
			case 0:
			
			
			float time = ((float)pul_n*(float)((float)(cy_time+1)*0.02*(float)c_avg)+250)/1000; 
			((NRF_RTC_Type *) 0x40011000UL)->CC[0] = time*32768; 
			rtc_flag = 1;
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
			do 
			{ 
				
				__wfe();   
				
				__sev(); 
				__wfe();                 
			}while(rtc_flag); 
			
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1; 
			break;
			
			case 1:
			
			
			
			time = ((float)pul_n*(float)((float)(cy_time+1)*0.02*(float)c_avg)+250)/1000; 
			((NRF_RTC_Type *) 0x40011000UL)->CC[0] = time*32768; 
			rtc_flag = 1;
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
			do 
			{ 
				
				__wfe();   
				
				__sev(); 
				__wfe();                 
			}while(rtc_flag); 
			
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
			((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1; 
			break;			
		}
		return w;
}
#line 54 "..\\PCAPint_main.c"

#line 56 "..\\PCAPint_main.c"
#line 57 "..\\PCAPint_main.c"
#line 58 "..\\PCAPint_main.c"
#line 59 "..\\PCAPint_main.c"
#line 60 "..\\PCAPint_main.c"

#line 62 "..\\PCAPint_main.c"







 
static uint8_t s_broadcast_data[(8)];	
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
        case 0x01:
                ps_flag = 1;
            break;
        case 0x02: 
                ps_flag = 2;           
            break;
        case 0x03:
                ps_flag = 3;           
            break;
        case 0x04:
                ps_flag = 4;            
            break;
        case 0x05:
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

    

    retval = pstorage_block_identifier_get(&flashhandle, 0, &flash_block_handle);      
    

    

    
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
			
			
			
			
		}

		return check;
		
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {}


 
static void handle_error(void){while (1) {}}






 
void PROTOCOL_EVENT_IRQHandler(void){}








 
void softdevice_assert_callback(uint32_t pc, uint16_t line_num, const uint8_t * p_file_name){while (1) {}}






 
	
void rtc_delay(int second)
{
		((NRF_RTC_Type *) 0x40011000UL)->CC[0] = second*32768; 
	  rtc_flag = 1;
		((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
		do 
    { 
       
       __wfe();   
       
       __sev(); 
       __wfe();                 
    }while(rtc_flag); 
		
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1;
}
static void pcap_broadcast_data(uint8_t add, int data1,int data2,int data3)
		{
			if(add == 5 )
			{
				s_broadcast_data[0] = 1;;
				s_broadcast_data[1] = add;
				s_broadcast_data[2] = data1 >> 16;
				s_broadcast_data[3] = data1 >> 8;
				s_broadcast_data[4] = data1;
				s_broadcast_data[5] = 0;
				s_broadcast_data[6] = 0;
				s_broadcast_data[7] = 0;

			}
			else{
				s_broadcast_temp[0] = 1;;
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









 
 uint32_t* pcap_spi_set(SPIModuleNumber mod_num)
{
	uint32_t *PCAP_spi_address = spi_master_init(mod_num, SPI_MODE1, (_Bool) 0);

	return PCAP_spi_address;
}







 
static void handle_channel_event(uint32_t event, uint8_t add, int data1,int data2,int data3)					
{
		uint32_t return_value;
		
		 
		pcap_broadcast_data(add, data1,data2,data3);
	  
						
		 
		return_value = sd_ant_broadcast_message_tx((0), (8), s_broadcast_data);
		if (return_value != ((0x0) + 0))
	 {
			 
			handle_error();
	 }
	 
		((NRF_RTC_Type *) 0x40011000UL)->CC[0] = 2*32768; 
	  rtc_flag = 1;
		((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
		do 
    { 
       
       __wfe();   
       
       __sev(); 
       __wfe();                 
    }while(rtc_flag); 
		
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1;
}




 

void RTC1_IRQHandler(void) 
{ 
    
    if(((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[0]) 
    { 
       ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[0] = 0; 
       rtc_flag = 0;           
       ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1; 
    } 
}



int main(void)

{
	
	_Bool ret0, ret1, ret2, ret3; 
	uint32_t sw = 1, sw2 = 1, sw3 =0, stat; 
	uint8_t CYC_ACTIVE, T_END_FLAG, RUNBIT, COMBI_ERR, CAP_ERR, CAP_ERR_PC, TEMP_ERR; 
	
	uint8_t x, n;
	uint8_t	event;
	uint8_t	ant_channel;		
	uint32_t return_value;
  
  char capref_s[16]; 
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
	int deviceid = 1;;
	int good = 0;

		 
	 
	static uint8_t event_message_buffer[(32)];
	
	 
	do { static uint32_t EVT_BUFFER[(((((((((sizeof(ble_evt_t) + (23))) < ((sizeof(ant_evt_t))) ? ((sizeof(ant_evt_t))) : ((sizeof(ble_evt_t) + (23))))) < (sizeof(uint32_t)) ? (sizeof(uint32_t)) : ((((sizeof(ble_evt_t) + (23))) < ((sizeof(ant_evt_t))) ? ((sizeof(ant_evt_t))) : ((sizeof(ble_evt_t) + (23))))))) - 1) / (sizeof(uint32_t))) + 1)]; uint32_t ERR_CODE; ERR_CODE = softdevice_handler_init((NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION), EVT_BUFFER, sizeof(EVT_BUFFER), (0) ? softdevice_evt_schedule : 0); do { const uint32_t LOCAL_ERR_CODE = (ERR_CODE); if (LOCAL_ERR_CODE != ((0x0) + 0)) { do { app_error_handler((LOCAL_ERR_CODE), 362, (uint8_t*) "..\\PCAPint_main.c"); } while (0); } } while (0); } while (0); 
	softdevice_sys_evt_handler_set(sys_evt_dispatch); 
	
	
	
   	
	
  ((NRF_RTC_Type *) 0x40011000UL)->PRESCALER = 0; 
  ((NRF_RTC_Type *) 0x40011000UL)->EVTENSET = (0x1UL << (16UL));  
  ((NRF_RTC_Type *) 0x40011000UL)->INTENSET = (0x1UL << (16UL));  
  ((NRF_RTC_Type *) 0x40011000UL)->CC[0] = 5*32768; 
  NVIC_EnableIRQ(RTC1_IRQn); 
	
  sd_power_mode_set ( NRF_POWER_MODE_LOWPWR   );

   
	((NRF_RTC_Type *) 0x40011000UL)->CC[0] = (10+rand()%10)*32768; 
  rtc_flag = 1;
  ((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
	

	do 
  { 
	  
		__wfe();   
		
		__sev(); 
		__wfe();                 
  }while(rtc_flag); 
	
	((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
	((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1; 

	 
	check_temp = pstorage_test();

	

	 
	
	
	 
  
				 
	 
	
	return_value = ant_channel_tx_broadcast_setup(); 
  if (return_value != 0) while(1);
  
	  
	uint32_t *PCAP_spi_address = pcap_spi_set(SPI1); 
  pcap_dsp_write(PCAP_spi_address); 
	
	 
	while(1)
	{	
		if(pcap_flag)
		{
			pcap_flag = 0;
			 
	    ret0 = pcap_config(PCAP_spi_address,c_avg,temp_flag,cy_time,rdc_sel);
		}
		 
		 
		ret2 = pcap_measure(PCAP_spi_address,c_avg,temp_flag,cy_time);
		
		ret3 = (stat == 0 && cap_t == 0 && cap_t[1] == 0); 
		
		  
		
		CYC_ACTIVE = (stat >> 23) & 1;
		T_END_FLAG = (stat >> 22) & 1;
		RUNBIT = (stat >> 19) & 1;
		COMBI_ERR = (stat >> 15) & 1;
		CAP_ERR = (stat >> 12) & 1;
		CAP_ERR_PC = (stat >> 5) & 0x0F;
		TEMP_ERR = (stat >> 3) & 1;
		
	  
		return_value = sd_ant_channel_open((0));

		 
		
		memset(tx_data, 0, 8);
		memset(rx_data, 0, 8);

		stat = read_reg(PCAP_spi_address, (8));
		if(!(stat == 	0x100000 | stat == 	0x500000)) good = 0;
		else good = 1;
		 
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
						tx_data[0] = 0x88; 
					
						acknowledge_flag = 1;
						
						event_flag = 0;
						
						return_value = pcap_spi_tx_rx(PCAP_spi_address, MSG_LEN, tx_data);
						pcap_dsp_write(PCAP_spi_address); 
						return_value = pcap_config(PCAP_spi_address,c_avg,temp_flag,cy_time,rdc_sel);
						return_value = pcap_measure(PCAP_spi_address,c_avg,temp_flag,cy_time);
						 
						stat = read_reg(PCAP_spi_address, (8));
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
		 
		do
		{				
			 
			n++;
			if(1) 
				{
					switch (n)
					{
						case 1:
						memset(tx_data, 0, 8);
						memset(rx_data, 0, 8);
			
						 
											
						cap_t[1] = read_reg(PCAP_spi_address, (1));		
						cap_t[2] = read_reg(PCAP_spi_address, (2));		
						cap_t[3] = read_reg(PCAP_spi_address, (3));		
											
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
									 
									cap_t[4] = read_reg(PCAP_spi_address, (13));
								  cap_t[5] = read_reg(PCAP_spi_address, (14));
									
							}										
							break;										
						}															 
			  }
						 
						sw2 = 0;
		} while ( n < 2 );
		
		n = 0;			
		sw2 = 0;			
		sw3 = 1;			
		
		acknowledge_flag = 1;
		
		event_flag = 0;
		while(!event_flag){
	  return_value = sd_ant_event_get(&ant_channel, &event, event_message_buffer);
			
		if (return_value == ((0x0) + 0)) 
		{
			switch (event)
			{
				case ((uint8_t)0x80):
					if( event_message_buffer[1u] ==  ((uint8_t)0x4E)) 
						{
						
						if(event_message_buffer[5] < 2)
						{
							delay = event_message_buffer[6];
							if(c_avg !=  (int)(event_message_buffer[8] << 8 |event_message_buffer[7] ))
							{
								c_avg = (int)(event_message_buffer[8] << 8 |event_message_buffer[7] );
								pcap_flag = 1;
							}	
							
							if(event_message_buffer[5] != temp_flag)
							{
								temp_flag = event_message_buffer[5];
								pcap_flag = 1;
								
								
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
							




 
						 
						 
					}
				}
					event_flag = 1;
					break;
				default:
					
					event_flag = 0;
				break;
			}
		}
		else
		{
			return_value = sd_ant_channel_close((0));
			((NRF_RTC_Type *) 0x40011000UL)->CC[0] = 5*32768; 
	    rtc_flag = 1;
		  ((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
			do 
		  { 
				
				__wfe();   
				
				__sev(); 
				__wfe();                 
		  }while(rtc_flag); 
			
			
	    return_value = sd_ant_channel_open((0));
		  
	    s_broadcast_data[0] = 1;;
	    return_value = sd_ant_broadcast_message_tx((0), (8), s_broadcast_data );
			
		  ((NRF_RTC_Type *) 0x40011000UL)->CC[0] = 1*32768; 
	    rtc_flag = 1;
		  ((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
			do 
		  { 
				
				__wfe();   
				
				__sev(); 
				__wfe();                 
		  }while(rtc_flag); 
			
			
		}
	}
		
		return_value = sd_ant_channel_close((0));
	
		if(acknowledge_flag){
	  ((NRF_RTC_Type *) 0x40011000UL)->CC[0] = delay*32768; 
		rtc_flag = 1;
		((NRF_RTC_Type *) 0x40011000UL)->TASKS_START = 1;
		do 
    { 
       
       __wfe();   
       
       __sev(); 
       __wfe();                 
    }while(rtc_flag); 
		
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP = 1; 
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1; 
	}
		
}
} 
































































 
