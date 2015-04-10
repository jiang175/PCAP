

/***************   PCAP  CONFIGURATION ********************/
/* PCAP config register address definitions */
#define conf_reg0      (0)
#define conf_reg1      (1)
#define conf_reg2      (2)
#define conf_reg3      (3)
#define conf_reg4      (4)
#define conf_reg5      (5)
#define conf_reg6      (6)
#define conf_reg7      (7)
#define conf_reg8      (8)
#define conf_reg9      (9)
#define conf_reg10      (10)

/*
#define conf_reg11      (11)
#define conf_reg12      (12)
#define conf_reg13      (13)
#define conf_reg14      (14)
#define conf_reg15      (15)
#define conf_reg16      (16)
#define conf_reg17      (17)
#define conf_reg18      (18)
#define conf_reg19      (19)
#define conf_reg20      (20)
*/

/* PCAP  read register address definitions ** ask matthew if this is the right intialisation ** */
#define read_reg0      (0)   // C0 LSB
#define read_reg1      (1)  // C1/C0
#define read_reg2      (2)  // C2/C0
#define read_reg3      (3)  // C3/C0
#define read_reg4      (4)  // C4/C0
#define read_reg5      (5)  // C5/C0
#define read_reg6      (6)  // C6/C0
#define read_reg7      (7)  // C7/C0
#define read_stat      (8)  // Status register
#define read_reg8      (11) // unused
#define read_reg9      (12) // unused
#define read_reg10      (13) // R0/Rref
#define read_reg11      (14) // R0/Rref

/* Config registration parameters*/
// reconfig all the uint to be of 8 16 32 there is no 24 or 4 or 2

//Register 0 
#define MEMCOMP ((uint8_t) 0) // 0 = disable , 1 = 5 byte , 2 = 33 byte 3 = byte
#define ECC_MODE ((uint8_t) 0x00) // OTP internal error detection and repair 0x00 disable , 0x0F Double, 0xF0 Quad
#define AUTOBOOT_DIS ((uint8_t) 0x0F) // 0xF slave operation, 0x0 stand alone operation
#define MEM_LOCK_DIS ((uint8_t) 0x0F) // OxO activatin the memory read-out blocker 0xF Readoutremain un-blocled 

// Register 2
#define CMEAS_PORT_EN ((uint8_t) 0xFF) // Each bit activates individual PCx ports
#define CMEAS_BITS ((uint8_t) 4) // 1 = grounded cap , 4 =floating single capacitance, 8 = floating differential capacitances
#define RDCHG_INT_SEL ((uint8_t) 6) // 4 = 180 kohm , 5 = 90kohm , 6 = 30 kohm, 7 = 10 kohm

// Register 3
#define CY_CLK_SEL ((uint8_t) 0) // 0 = 20us 2 = 1us 3 = 0.25us 
#define SEQ_TIME ((uint8_t) 0x0D) // 0 = off otherwise s = seq_time  then trig perious will 20us*2^(s+1) 
#define CMEAS_FAKE ((uint8_t) 0) // no of fake block measurements 
#define C_AVRG  ((uint16_t) 100) // Averaging the CDC results
 

// Register 4
#define CMEAS_STARTPIN ((uint8_t) 0) // 0 = PG0, 1 = PG1 , .... 
#define CMEAS_TRIG_SEL ((uint8_t) 0) // 0 = softwaretrigger only , 1 = continuous mode, 2 = timer-triggered mode , 3 = pulse-triggered mode
#define CMEAS_CYTIME ((uint16_t) 4) // CDC cycle time = (CMEAS_CYTIME+1)*clock_period
#define TMEAS_CYTIME ((uint8_t) 0 ) // 0 = 140 us , 1 = 280us
#define TMEAS_STARTPIN ((uint8_t) 0) // 0 = PG0, 1 = PG1 ... pin for pulse triggered temperature measurement.
#define TMEAS_TRIG_SEL ((uint8_t) 0) // trigger source for the temperature measurement 0 = off/opcode triggered , 1 = cmeas-triggered 2 = timer-triggered mode 3= pulse-triggered mode. 

//Register 5
#define T_AVRG ((uint8_t) 1) // 0( no averaging), 1 (4 fold averaging) , 2 (8-fold averaging) , 3(16-fold averaging) 
#define TMEAS_TRIG_PREDIV ((uint32_t) 0) //zero counts as one, set zero for hygrometers etc 

//Register 6 
#define TMEAS_FAKE ((uint8_t) 0) // 0 = 2 dummy measuremtns,  1 = 8 dummy measurement
#define TMEAS_7BITS ((uint8_t) 0) 

//Register 8 
#define DSP_SRAM_SEL ((uint8_t) 1) // 0 = OTP , 1 = SRAM 
#define DSP_START ((uint8_t) 0) // start command
#define DSP_STARTONOVL ((uint8_t) 0) // 0 = default is mandatory 
#define DSP_STARTONTEMP ((uint8_t) 0) // 0 = default, mandatory with standard fimware 03.01.xx
#define DSP_STARTPIN ((uint8_t) 0) // 0 = PG0, 1 = PG1, 2 = PG2, 3 = PG3
#define DSP_FF_IN ((uint8_t) 0x00) // Bit 12 = PG0 , Bit 13 = PG1, ....
#define DSP_WATCHDOG_LENGTH ((uint8_t) 0) //
#define DSP_MOFLO_EN ((uint8_t) 0) //bit 9 for PG1 bit8 for PG0
#define DSP_SPEED ((uint8_t) 3) // 1 = standard (fast), 3 = low-current (slow)
#define INT2PG2 ((uint8_t) 0) /// interrupt pin reroute to PG2
#define PG1_X_G3 ((uint8_t) 0) //pulse codes reroute from PG1 to PG3
#define PG0_X_G2 ((uint8_t) 0) //pulse codes reroute from PG0 to PG2

//Register 9 
#define PG_DIR_IN ((uint8_t) 0x0F) // toggles outputs to input(PG3/bit23 to PG0/bit 20)
#define PG_PULL_UP ((uint8_t) 0x0F) // Activates pull-up resistor in PG0 to PG3 lines
#define PI_EN ((uint8_t) 0x00) // enables pulse-density or pulse0width mode code generation. PWM0/PDM0 can be output at ports PG0 or PG2. PWM1/PDM1 can be output at ports PG1 or PG3.
#define PI1_CLK_SEL ((uint8_t) 0x00) //Base frequency for the pulse code interfaces based on low-freq or external high-freq oscillator
#define PI0_CLK_SEL ((uint8_t) 0x00)
#define PI1_RES ((uint8_t) 3) // Resolution of pulse code interfaces: 0 = 7 bit, 1 = 8 bit , 2 = 9 bit, 3 = 10 bit.
#define PI0_RES ((uint8_t) 3) // Resolution of pulse code interfaces (see above)

//Register 10 
#define V_CORE_CTL ((uint8_t) 0x87) // 0x87 = Low-current , 0x47 standard 

/********************************************************/

