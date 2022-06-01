/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_LEVEL_3
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    OFF
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_FRC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff


#include <xc.h>
#include <inttypes.h>
#include <stdio.h>


void set_performance_mode()
{
	unsigned int cp0;

    // Unlock Sequence
    asm volatile("di"); // Disable all interrupts
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;

    // PB1DIV
    // Peripheral Bus 1 cannot be turned off, so there's no need to turn it on
    PB1DIVbits.PBDIV = 1; // Peripheral Bus 1 Clock Divisor Control (PBCLK1 is SYSCLK divided by 2)

    // PB2DIV
    PB2DIVbits.ON = 1; // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB2DIVbits.PBDIV = 1; // Peripheral Bus 2 Clock Divisor Control (PBCLK2 is SYSCLK divided by 2)

    // PB3DIV
    PB3DIVbits.ON = 1; // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB3DIVbits.PBDIV = 1; // Peripheral Bus 3 Clock Divisor Control (PBCLK3 is SYSCLK divided by 2)

    // PB4DIV
    PB4DIVbits.ON = 1; // Peripheral Bus 4 Output Clock Enable (Output clock is enabled)
    while (!PB4DIVbits.PBDIVRDY); // Wait until it is ready to write to
    PB4DIVbits.PBDIV = 0; // Peripheral Bus 4 Clock Divisor Control (PBCLK4 is SYSCLK divided by 1)

    // PB5DIV
    PB5DIVbits.ON = 1; // Peripheral Bus 5 Output Clock Enable (Output clock is enabled)
    PB5DIVbits.PBDIV = 1; // Peripheral Bus 5 Clock Divisor Control (PBCLK5 is SYSCLK divided by 2)

    // PB7DIV
    PB7DIVbits.ON = 1; // Peripheral Bus 7 Output Clock Enable (Output clock is enabled)
    PB7DIVbits.PBDIV = 0; // Peripheral Bus 7 Clock Divisor Control (PBCLK7 is SYSCLK divided by 1)

    // PB8DIV
    PB8DIVbits.ON = 1; // Peripheral Bus 8 Output Clock Enable (Output clock is enabled)
    PB8DIVbits.PBDIV = 1; // Peripheral Bus 8 Clock Divisor Control (PBCLK8 is SYSCLK divided by 2)

    // PRECON - Set up prefetch
    PRECONbits.PFMSECEN = 0; // Flash SEC Interrupt Enable (Do not generate an interrupt when the PFMSEC bit is set)
    PRECONbits.PREFEN = 0b11; // Predictive Prefetch Enable (Enable predictive prefetch for any address)
    PRECONbits.PFMWS = 0b010; // PFM Access Time Defined in Terms of SYSCLK Wait States (Two wait states)

    // Set up caching
    cp0 = _mfc0(16, 0);
    cp0 &= ~0x07;
    cp0 |= 0b011; // K0 = Cacheable, non-coherent, write-back, write allocate
    _mtc0(16, 0, cp0);

    // Lock Sequence
    SYSKEY = 0x33333333;
    asm volatile("ei"); // Enable all interrupts
}

////////////////////////////////////////////////////////////////////////////// CORNELL Library ////////////////////////////////////////////////////////////////////////////////
#define FSCK 100000.0 // the I2C communication speed we want, 100kHz
#define PBCLK 50000000.0 // the speed of the peripheral bus clock
#define TPGD 0.000000104 // propagation delay, defined in the PIC32MZ EF datasheet as 104ns

//This function will initialize the I2C(1) peripheral.
void Setup_I2C(void)
{
	ANSELGbits.ANSG7 = 0;
    ANSELGbits.ANSG8 = 0;
    TRISGbits.TRISG7 = 1;
	TRISGbits.TRISG8 = 1;

    I2C4CON = 0;            // Turn off I2C4 module
    I2C4CONbits.DISSLW = 1; // Disable slew rate for 100kHz

    //Set the I2C BRG Baud Rate.
//    float BRG = ((1 / (2 * FSCK)) - TPGD) * (PBCLK) - 2;
//    I2C4BRG = (int)BRG;     // Set baud rate
//    char buf[30];
//    sprintf(buf, "%f\n", BRG);
//	uart1_txwrite_str(buf);

//    I2C4BRG = 75;  // test with a lower baud rate --> WORKS for MCP23016
    I2C4BRG = 0x01ED;  // test with a lower baud rate

	//Now we will initialise the I2C peripheral for Master Mode, No Slew Rate
	//Control, SMbus levels, and leave the peripheral switched off.

	I2C4CONbits.I2CEN = 0;
	I2C4CONbits.I2CSIDL = 0;
	I2C4CONbits.SCLREL = 1;
	I2C4CONbits.IPMIEN = 0;
	I2C4CONbits.A10M = 0;
	I2C4CONbits.DISSLW = 1;
	I2C4CONbits.SMEN = 0;
	I2C4CONbits.GCEN = 0;
	I2C4CONbits.STREN = 0;
	I2C4CONbits.ACKDT = 0;
	I2C4CONbits.ACKEN = 0;
	I2C4CONbits.RCEN = 0;
	I2C4CONbits.PEN = 0;
	I2C4CONbits.RSEN = 0;
	I2C4CONbits.SEN = 0;

	//Clearing the receive and transmit buffers
	I2C4RCV = 0x0000;
	I2C4TRN = 0x0000;

	//Now we can enable the peripheral
	I2C4CONbits.I2CEN = 1;
	uart1_txwrite_str("I2C Setup Complete\n");
}


/*********************************************************************
* Function:        IdleI2C()
* Overview:		Waits for bus to become Idle
********************************************************************/
unsigned int IdleI2C(void)
{
	while (I2C4STATbits.TRSTAT);		//Wait for bus Idle
}

unsigned int StartI2C(void)
{
	//This function generates an I2C start condition and returns status
	//of the Start.
    IdleI2C();
	I2C4CONbits.SEN = 1;		//Generate Start Condition
	//Nop();
	while (I2C4CONbits.SEN);	//Wait for Start Condition
	//return(I2C4STATbits.S);	//Optionally return status
}


/*********************************************************************
* Function:        RestartI2C()
* Overview:		Generates a restart condition and optionally returns status
********************************************************************/
unsigned int RestartI2C(void)
{
	//This function generates an I2C Restart condition and returns status
	//of the Restart.

	I2C4CONbits.RSEN = 1;		//Generate Restart
	//Nop();
	while (I2C4CONbits.RSEN);	//Wait for restart
	//return(I2C4STATbits.S);	//Optional - return status
}


/*********************************************************************
* Function:        StopI2C()
* Overview:		Generates a bus stop condition
********************************************************************/
unsigned int StopI2C(void)
{
	//This function generates an I2C stop condition and returns status
	//of the Stop.

	I2C4CONbits.PEN = 1;		//Generate Stop Condition
	//Nop();
	while (I2C4CONbits.PEN);	//Wait for Stop
	//return(I2C4STATbits.P);	//Optional - return status
}


/*********************************************************************
* Function:        WriteI2C()
* Overview:		Writes a byte out to the bus
********************************************************************/
unsigned int WriteI2C(unsigned char byte)
{
	//This function transmits the byte passed to the function
	//while (I2C4STATbits.TRSTAT);	//Wait for bus to be idle
	I2C4TRN = byte;					//Load byte to I2C4 Transmit buffer
	//Nop();
	while (I2C4STATbits.TBF);		//wait for data transmission
}


/*********************************************************************
* Function:        ACKStatus()
* Output:		Acknowledge Status.
* Overview:		Return the Acknowledge status on the bus (1 = ACK received, 0 = ACK not received)
********************************************************************/
unsigned int ACKStatus(void)
{
	return (!I2C4STATbits.ACKSTAT);		//Return Ack Status
}


/*********************************************************************
* Function:        NotAckI2C()
* Overview:		Generates a NO Acknowledge on the Bus
********************************************************************/
unsigned int NotAckI2C(void)
{
	I2C4CONbits.ACKDT = 1;			//Set for NotACk
	I2C4CONbits.ACKEN = 1;
	while(I2C4CONbits.ACKEN);		//wait for ACK to complete
	I2C4CONbits.ACKDT = 0;			//Set for NotACk
}


/*********************************************************************
* Function:        AckI2C()
* Overview:		Generates an Acknowledge.
********************************************************************/
unsigned int AckI2C(void)
{
	I2C4CONbits.ACKDT = 0;			//Set for ACk
	I2C4CONbits.ACKEN = 1;
	while(I2C4CONbits.ACKEN);		//wait for ACK to complete
}

/*********************************************************************
* Function:        getI2C()
* Input:		None.
* Output:		contents of I2C4 receive buffer.
* Overview:		Read a single byte from Bus
********************************************************************/
unsigned char getI2C(void)
{
	I2C4CONbits.RCEN = 1;			// Enable Master receive
	//Nop();
	while(!I2C4STATbits.RBF);		// Wait for receive buffer to be full
	return(I2C4RCV);				// Return data in buffer
}

////////////////////////////////////////////////////////////////////////////// CORNELL Library ////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// I2C for MCP23016 ///////////////////////////////////////////////////////////////////////////////
//#define MCP23016_ADDRESS 0x20            // The address of MCP23016 when the AD0 pin is connected to ground
//
//#define GP0 0x00
//#define GP1 0x01
//#define OLAT0 0x02
//#define OLAT1 0x03
//#define IPOL0 0x04
//#define IPOL1 0x05
//#define IODIR0 0x06
//#define IODIR1 0x07
//#define INTCAP0 0x08
//#define INTCAP1 0x09
//#define IOCON0 0x0A
//#define IOCON1 0x0B
//
//void MCP23016_read(unsigned char *value_lsb, unsigned char *value_msb){
//    StartI2C();                            /* Send start condition */
////    uart1_txwrite_str("TST1\t");
//    WriteI2C(MCP23016_ADDRESS << 1);    /* Send MCP23016's address, read/write bit not set (AD + R) */
//    IdleI2C();
//    while(!ACKStatus());
////    uart1_txwrite_str("TST2\t");
//    WriteI2C(GP0);                      /* Send the command byte */
//    IdleI2C();
//    while(!ACKStatus());
////    uart1_txwrite_str("TST3\t");
//    RestartI2C();
////    uart1_txwrite_str("TST4\t");
//    WriteI2C(MCP23016_ADDRESS << 1 | 0x1);  /* Send MCP23016's address, read/write bit set (AD + W) */
////    uart1_txwrite_str("TST5\t");
//    while(!ACKStatus());
//    IdleI2C();
//    *value_lsb = getI2C();                     /* Read value from the I2C bus */
////    uart1_txwrite_str("TST6\t");
//    AckI2C();
//    IdleI2C();
//    *value_msb = getI2C();                     /* Read value from the I2C bus */
//    NotAckI2C();
//    StopI2C();                             /* Send stop condition */
//}
//
//
//void MCP23016_write(unsigned char reg, unsigned char value1, unsigned char value2){
//    StartI2C();                         /* Send start condition */
//    WriteI2C(MCP23016_ADDRESS << 1);    /* Send MCP23016's address, read/write bit not set (AD + R) */
//    IdleI2C();
//    while(!ACKStatus());
//    WriteI2C(reg);                      /* Send the command byte */
//    IdleI2C();
//    while(!ACKStatus());
//    WriteI2C(value1);                   /* Send the message */
//    IdleI2C();
//    while(!ACKStatus());
//    WriteI2C(value2);                   /* Send the message */
//    IdleI2C();
//    while(!ACKStatus());
//    StopI2C();                          /* Send stop condition */
//}
//
//void main(){
//    char *buf_test = "\nTEST Main\n";
//    char buf_r1[100], buf_r2[100];
////    unsigned char value_w = 0x2e;
//
//    unsigned char value_lsb, value_msb;
//    set_performance_mode();
//
//    uart1_init(9600);
//    uart1_txwrite_str(buf_test);
//    Setup_I2C();
//    MCP23016_write(IODIR0, 0xff, 0xff); // Set all as inputs - for reading
////    MCP23016_write(IODIR0, 0x00, 0x00); // Set all as outputs - for writing
////    uart1_txwrite_str("After Write ");
//    while(1){
//        /* WRITE TEST --> WORKS!! */
////        MCP23016_write(GP0, 0x11, 0xf1);
////        uart1_txwrite_str("After Write 2 ");
//        /* READ TEST --> WORKS!!!!!!!!! */
//        MCP23016_read(&value_lsb, &value_msb);
//        sprintf(buf_r1, "0x%x\t",  value_msb);
//        sprintf(buf_r2, "0x%x\n",  value_lsb);
//        uart1_txwrite_str(buf_r1);
//        uart1_txwrite_str(buf_r2);
//    }
//}
////////////////////////////////////////////////////////////////////////////// I2C for MCP23016 ///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////// I2C for MCP23008 ///////////////////////////////////////////////////////////////////////////////
//#define MCP23008_ADDRESS 0x20            // The address of MCP23016 when the AD0 pin is connected to ground
//
//#define IODIR 0x00
//#define IPOL 0x01
//#define GPINTEN 0x02
//#define DEFVAL 0x03
//#define INTCON_MCP 0x04
//#define IOCON 0x05
//#define GPPU 0x06
//#define INTF 0x07
//#define INTCAP 0x08
//#define GPIO 0x09
//#define OLAT 0x0A
//
//void MCP23008_read(unsigned char *value){
//    StartI2C();                            /* Send start condition */
////    uart1_txwrite_str("TST1\t");
//    WriteI2C(MCP23008_ADDRESS << 1);    /* Send MCP23008's address, read/write bit not set (AD + R) */
//    IdleI2C();
//    while(!ACKStatus());
////    uart1_txwrite_str("TST2\t");
//    WriteI2C(GPIO);                      /* Send the command byte */
//    IdleI2C();
//    while(!ACKStatus());
////    uart1_txwrite_str("TST3\t");
//    RestartI2C();
////    uart1_txwrite_str("TST4\t");
//    WriteI2C(MCP23008_ADDRESS << 1 | 0x1);  /* Send MCP23008's address, read/write bit set (AD + W) */
////    uart1_txwrite_str("TST5\t");
//    while(!ACKStatus());
//    IdleI2C();
//    *value = getI2C();                     /* Read value from the I2C bus */
//    NotAckI2C();
//    StopI2C();                             /* Send stop condition */
//}
//
//void MCP23008_write(unsigned char reg, unsigned char value1){
//    StartI2C();                         /* Send start condition */
////    uart1_txwrite_str("TST1\t");
//    WriteI2C(MCP23008_ADDRESS << 1);    /* Send MCP23016's address, read/write bit not set (AD + R) */
////    uart1_txwrite_str("TST2\t");
//    IdleI2C();
////    uart1_txwrite_str("TST3\t");
//    while(!ACKStatus());
//    uart1_txwrite_str("TST4\t");
//    WriteI2C(reg);                      /* Send the command byte */
//    IdleI2C();
//    uart1_txwrite_str("TST5\t");
//    while(!ACKStatus());
//    uart1_txwrite_str("TST6\t");
//    WriteI2C(value1);                   /* Send the message */
//    uart1_txwrite_str("TST7\t");
//    IdleI2C();
//    uart1_txwrite_str("TST8\t");
//    while(!ACKStatus());
//    uart1_txwrite_str("TST9\t");
//    StopI2C();                          /* Send stop condition */
//}
//
//void main(){
//    char *buf_test = "\nTEST Main\n";
//    char buf_r1[100], buf_r2[100];
////    unsigned char value_w = 0x2e;
//
//    unsigned char value;
////    set_performance_mode();
//
//    uart1_init(9600);
//    uart1_txwrite_str(buf_test);
//    Setup_I2C();
//    MCP23008_write(IODIR, 0xff); // Set all as inputs - for reading
//    uart1_txwrite_str("After Write ");
//    while(1){
//        /* WRITE TEST --> WORKS!! */
////        MCP23016_write(GP0, 0x11, 0xf1);
////        uart1_txwrite_str("After Write 2 ");
//        /* READ TEST --> WORKS!!!!!!!!! */
//        MCP23008_read(&value);
//        sprintf(buf_r1, "0x%x\t",  value);
//        uart1_txwrite_str(buf_r1);
//    }
//}
////////////////////////////////////////////////////////////////////////////// I2C for MCP23008 ///////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////// I2C for NCD9830 ///////////////////////////////////////////////////////////////////////////////
#define NCD9830_ADDRESS 0x4A
//#define NCD9830_ADDRESS 0x4B

#define CH0 0b10001000
#define CH1 0b11000100
#define CH2 0b10010100
#define CH3 0b11010100
#define CH4 0b10100100
#define CH5 0b11100100
#define CH6 0b10110100
#define CH7 0b11110100


void NCD9830_read(unsigned char channel, unsigned char *value){
    StartI2C();                            /* Send start condition */
    WriteI2C(NCD9830_ADDRESS << 1);    /* Send NCD9830's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(channel);                      /* Send the command byte */
    IdleI2C();
    while(!ACKStatus());
    RestartI2C();
    WriteI2C(NCD9830_ADDRESS << 1 | 0x1);  /* Send NCD9830's address, read/write bit set (AD + W) */
    while(!ACKStatus());
    IdleI2C();
    *value = getI2C();                     /* Read value from the I2C bus */
    NotAckI2C();
    StopI2C();                             /* Send stop condition */
}


void main(){
    char *buf_test = "\nTEST Main\n";
    char buf[10];

    unsigned char value;
    set_performance_mode();

    uart1_init(9600);
    uart1_txwrite_str(buf_test);
    Setup_I2C();
    while(1){
        /* READ TEST */
//        uart1_txwrite_str("TST0\t");
//        StartI2C();                            /* Send start condition */
//        uart1_txwrite_str("TST1\t");
//        WriteI2C(NCD9830_ADDRESS << 1);    /* Send NCD9830's address, read/write bit not set (AD + R) */
//        uart1_txwrite_str("TST2\t");
//        IdleI2C();
//        uart1_txwrite_str("TST3\n");
//        StopI2C();
//        while(!ACKStatus());
//        uart1_txwrite_str("TST4\t");

        NCD9830_read(CH7, &value);
        uart1_txwrite_str("AFTER READ\t");
        sprintf(buf, "%d\n",  value);
        uart1_txwrite_str(buf);
    }
}
////////////////////////////////////////////////////////////////////////////// I2C for NCD9830 ///////////////////////////////////////////////////////////////////////////////
