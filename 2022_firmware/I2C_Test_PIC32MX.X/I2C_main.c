#pragma config FNOSC = FRCPLL, POSCMOD = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20
#pragma config FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, FSOSCEN = OFF

#include <xc.h>
#include <stdio.h>


#define FCY     40000000UL

////////////////////////////////////////////////////////////////////////////// CORNELL Library ////////////////////////////////////////////////////////////////////////////////

void Setup_I2C(void)
{	
	TRISBbits.TRISB9 = 1;
	TRISBbits.TRISB8 = 1;
	//This function will initialize the I2C(1) peripheral.
	
	//Set the I2C(1) BRG Baud Rate.
	//(((1/400KHz)-130ns)x40MIPs)-2 = 93
	I2C1BRG = 93;
	
	//Now we will initialise the I2C peripheral for Master Mode, No Slew Rate
	//Control, SMbus levels, and leave the peripheral switched off.
	
	I2C1CONbits.I2CEN = 0;
	I2C1CONbits.I2CSIDL = 0;
	I2C1CONbits.SCLREL = 1;
	I2C1CONbits.IPMIEN = 0;
	I2C1CONbits.A10M = 0;
	I2C1CONbits.DISSLW = 1;
	I2C1CONbits.SMEN = 0;
	I2C1CONbits.GCEN = 0;
	I2C1CONbits.STREN = 0;
	I2C1CONbits.ACKDT = 0;
	I2C1CONbits.ACKEN = 0;
	I2C1CONbits.RCEN = 0;
	I2C1CONbits.PEN = 0;
	I2C1CONbits.RSEN = 0;
	I2C1CONbits.SEN = 0;
	
	//Clearing the receive and transmit buffers
	I2C1RCV = 0x0000;
	I2C1TRN = 0x0000;
	
	//Now we can enable the peripheral	
	I2C1CONbits.I2CEN = 1;
	
	uart1_txwrite_str("\nI2C Setup Complete");
}

unsigned int StartI2C(void)
{
	//This function generates an I2C start condition and returns status 
	//of the Start.

	I2C1CONbits.SEN = 1;		//Generate Start Condition
	//Nop();
	while (I2C1CONbits.SEN);	//Wait for Start Condition
	//return(I2C1STATbits.S);	//Optionally return status
}


/*********************************************************************
* Function:        RestartI2C()
* Overview:		Generates a restart condition and optionally returns status
********************************************************************/
unsigned int RestartI2C(void)
{
	//This function generates an I2C Restart condition and returns status 
	//of the Restart.

	I2C1CONbits.RSEN = 1;		//Generate Restart		
	//Nop();
	while (I2C1CONbits.RSEN);	//Wait for restart	
	//return(I2C1STATbits.S);	//Optional - return status
}


/*********************************************************************
* Function:        StopI2C()
* Overview:		Generates a bus stop condition
********************************************************************/
unsigned int StopI2C(void)
{
	//This function generates an I2C stop condition and returns status 
	//of the Stop.

	I2C1CONbits.PEN = 1;		//Generate Stop Condition
	//Nop();
	while (I2C1CONbits.PEN);	//Wait for Stop
	//return(I2C1STATbits.P);	//Optional - return status
}


/*********************************************************************
* Function:        WriteI2C()
* Overview:		Writes a byte out to the bus
********************************************************************/
unsigned int WriteI2C(unsigned char byte)
{
	//This function transmits the byte passed to the function
	//while (I2C1STATbits.TRSTAT);	//Wait for bus to be idle
	I2C1TRN = byte;					//Load byte to I2C1 Transmit buffer
	//Nop();
	while (I2C1STATbits.TBF);		//wait for data transmission
}


/*********************************************************************
* Function:        IdleI2C()
* Overview:		Waits for bus to become Idle
********************************************************************/
unsigned int IdleI2C(void)
{
	while (I2C1STATbits.TRSTAT);		//Wait for bus Idle
}

/*********************************************************************
* Function:        ACKStatus()
* Output:		Acknowledge Status.
* Overview:		Return the Acknowledge status on the bus (1 = ACK received, 0 = ACK not received)
********************************************************************/
unsigned int ACKStatus(void)
{
	return (!I2C1STATbits.ACKSTAT);		//Return Ack Status
}


/*********************************************************************
* Function:        NotAckI2C()
* Overview:		Generates a NO Acknowledge on the Bus
********************************************************************/
unsigned int NotAckI2C(void)
{
	I2C1CONbits.ACKDT = 1;			//Set for NotACk
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		//wait for ACK to complete
	I2C1CONbits.ACKDT = 0;			//Set for NotACk
}


/*********************************************************************
* Function:        AckI2C()
* Overview:		Generates an Acknowledge.
********************************************************************/
unsigned int AckI2C(void)
{
	I2C1CONbits.ACKDT = 0;			//Set for ACk
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		//wait for ACK to complete
}

/*********************************************************************
* Function:        getI2C()
* Input:		None.
* Output:		contents of I2C1 receive buffer.
* Overview:		Read a single byte from Bus
********************************************************************/
unsigned char getI2C(void)
{
	I2C1CONbits.RCEN = 1;			//Enable Master receive
	//Nop();
	while(!I2C1STATbits.RBF);		//Wait for receive buffer to be full
	return(I2C1RCV);				//Return data in buffer
}

////////////////////////////////////////////////////////////////////////////// CORNELL Library ////////////////////////////////////////////////////////////////////////////////

#define PCF8574_ADDRESS 0x20            // The address of PCF8574 when the AD0 pin is connected to ground

void i2c_master_read(unsigned char *value){
    StartI2C();                            /* Send start condition */  
    WriteI2C(PCF8574_ADDRESS << 1 | 0x1);  /* Send PCF8574's address, read/write bit set (AD + W) */  
    while(!ACKStatus());
    IdleI2C();
//    RestartI2C();
    *value = getI2C();                     /* Read value from the I2C bus */
    NotAckI2C();
    StopI2C();                             /* Send stop condition */
}

void i2c_master_write(unsigned char value){
    StartI2C();                        /* Send start condition */  
    WriteI2C(PCF8574_ADDRESS << 1);    /* Send PCF8574's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(value);                    /* Send the message */
    IdleI2C();
    while(!ACKStatus());
    StopI2C();                         /* Send stop condition */  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void main(){
    char *buf_test = "\nTEST Main\n";
    char buf_r[10];
    unsigned char value_w = 0x2e;
    unsigned char value_r;
    
    uart1_init(9600);
    uart1_txwrite_str(buf_test);
    
    Setup_I2C();
    while(1){
        /* WRITE TEST */
//        i2c_master_write(value_w);
        /* READ TEST */
        i2c_master_read(&value_r);
        sprintf(buf_r, "0x%x\n", value_r);
        uart1_txwrite_str(buf_r);
    }
}
