#include "i2c4.h"

//This function will initialize the I2C4 peripheral.
void Setup_I2C(void)
{	
	ANSELGbits.ANSG7 = 0;
    ANSELGbits.ANSG8 = 0;
    TRISGbits.TRISG7 = 1;
	TRISGbits.TRISG8 = 1;
	
    I2C4CON = 0;            // Turn off I2C1 module
    I2C4CONbits.DISSLW = 1; // Disable slew rate for 100kHz
	
    //Set the I2C BRG Baud Rate.
    I2C4BRG = 0x01ED;  // for a 100kHz i2c communication

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


///////////////////////////////////////// I2C for NCD9830 /////////////////////////////////////////
void NCD9830_read(unsigned char i2c_address, unsigned char channel, unsigned char *value){
    StartI2C();                            /* Send start condition */  
    WriteI2C(i2c_address << 1);    /* Send NCD9830's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(channel);                      /* Send the command byte */
    IdleI2C();
    while(!ACKStatus());
    RestartI2C();
    WriteI2C(i2c_address << 1 | 0x1);  /* Send NCD9830's address, read/write bit set (AD + W) */  
    while(!ACKStatus());
    IdleI2C();
    *value = getI2C();                     /* Read value from the I2C bus */
    NotAckI2C();
    StopI2C();                             /* Send stop condition */
}
///////////////////////////////////////// I2C for NCD9830 /////////////////////////////////////////

///////////////////////////////////////// I2C for MCP23008 /////////////////////////////////////////
void MCP23008_read(unsigned char i2c_address, unsigned char *value){
    StartI2C();                            /* Send start condition */  
    WriteI2C(i2c_address << 1);    /* Send MCP23008's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(GPIO);                      /* Send the command byte */
    IdleI2C();
    while(!ACKStatus());
    RestartI2C();
    WriteI2C(i2c_address << 1 | 0x1);  /* Send MCP23008's address, read/write bit set (AD + W) */  
    while(!ACKStatus());
    IdleI2C();
    *value = getI2C();                     /* Read value from the I2C bus */
    NotAckI2C();
    StopI2C();                             /* Send stop condition */
}

void MCP23008_write(unsigned char i2c_address, unsigned char reg, unsigned char value1){
    StartI2C();                         /* Send start condition */  
    WriteI2C(i2c_address << 1);    /* Send MCP23016's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(reg);                      /* Send the command byte */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(value1);                   /* Send the message */
    IdleI2C();
    while(!ACKStatus());
    StopI2C();                          /* Send stop condition */  
}
///////////////////////////////////////// I2C for MCP23008 /////////////////////////////////////////

///////////////////////////////////////// I2C for MCP23016 /////////////////////////////////////////
void MCP23016_read(unsigned char i2c_address, unsigned char *value_lsb, unsigned char *value_msb){
    StartI2C();                            /* Send start condition */  
    WriteI2C(i2c_address << 1);    /* Send MCP23016's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(GP0);                      /* Send the command byte */
    IdleI2C();
    while(!ACKStatus());
    RestartI2C();
    WriteI2C(i2c_address << 1 | 0x1);  /* Send MCP23016's address, read/write bit set (AD + W) */  
    while(!ACKStatus());
    IdleI2C();
    *value_lsb = getI2C();                     /* Read value from the I2C bus */
    AckI2C();
    IdleI2C();
    *value_msb = getI2C();                     /* Read value from the I2C bus */
    NotAckI2C();
    StopI2C();                             /* Send stop condition */
}


void MCP23016_write(unsigned char i2c_address, unsigned char reg, unsigned char value1, unsigned char value2){
    StartI2C();                         /* Send start condition */  
    WriteI2C(i2c_address << 1);    /* Send MCP23016's address, read/write bit not set (AD + R) */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(reg);                      /* Send the command byte */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(value1);                   /* Send the message */
    IdleI2C();
    while(!ACKStatus());
    WriteI2C(value2);                   /* Send the message */
    IdleI2C();
    while(!ACKStatus());
    StopI2C();                          /* Send stop condition */  
}
///////////////////////////////////////// I2C for MCP23016 /////////////////////////////////////////