#include <stdbool.h>
#include <string.h>
#include <xc.h>

// Setup the I2C Master
void i2c_master_setup(void)
{
    I2C1BRG = 487; //I2CBRG = [1/(2*Fsck) - PGD]*Pbick - 2
                   // Fsck is the freq (100 kHz here), PGD = 104 ns
                   // Pbickis Peripheral clock freq (100 MHz)

    I2C1CONbits.ON = 1; // turn on the I2C1 module
}

// Start a transmission on the I2C bus. 
void i2c_master_start(void) 
{
    i2c_master_idle();   //Check for Bus Status
    I2C1CONbits.SEN = 1; // send the start bit 

    // wait for the start bit to be sent 
    while(I2C1CONbits.SEN) { ; } 
} 

// Stop a transmission on the I2C bus. 
void i2c_master_stop(void) 
{
    i2c_master_idle();   // Check for Bus Status
    // comm is complete and master relinquishes bus 
    I2C1CONbits.PEN = 1; 
    // wait for STOP to complete 
    while(I2C1CONbits.PEN) { ; } 
} 

// Restart a transmission on the I2C bus. 
void i2c_master_restart(void) 
{
    i2c_master_idle();    //Check for Bus Status
    I2C1CONbits.RSEN = 1; // send a restart 
    // wait for the restart to clear 
    while(I2C1CONbits.RSEN) { ; } 
} 

// Check the bus status 
void i2c_master_idle(void)
{ 
    // Acknowledge sequence not in progress 
    // Receive sequence not in progress 
    // Stop condition not in progress 
    // Repeated Start condition not in progress 
    // Start condition not in progress 
    while(I2C1CON & 0x1F); 

    // Bit = 0 ? Master transmit is not in progress    
    while(I2C1STATbits.TRSTAT); 

    return;
} 

// Send an acknowledge to the Slave
void i2c_master_ack_nack(int val) 
{ 
    //To sends ACK = 0 (slave should send another byte) or NACK = 1 (no more bytes requested from slave) 

    i2c_master_idle();   // Check for Bus Status
    // store ACK HACK in ACKDT 
    // send ACKDT 
    // wait for ACK HACK to be sent 
    I2C1CONbits.ACKDT = val; 
    I2C1CONbits.ACKEN = 1; 
    while(I2C1CONbits.ACKEN) { ; } 
}

// Send one byte data to the slave and check the acknowledged 
bool i2c_master_send(unsigned char byte) 
{
    // send a byte to slave 
    I2C1TRN = byte; // if an address, bit 0 = 0 for write, 1 for read
    // wait for the transmission to finish 
    while(I2C1STATbits.TRSTAT) { ; } 
    if(I2C1STATbits.ACKSTAT) 
    { // if this is high, slave has not acknowledged 
        return false; 
    } 
    else 
    { 
        return true; 
    }
}

// Receive a byte from the slave 
unsigned char i2c_master_recv(void) 
{
//    // Clear the previous data from temp. Rx buffer. 
//    memset(bRxTemp_Buf, 0, sizeof(bRxTemp_Buf)); 
//
//    I2C1CONbits.RCEN = 1;          // start receiving data
//    while(!I2C1STATbits.RBF) { ; } // wait to receive the data
//    // Convert the DWORD to BYTE 
//    dword_to_buf( I2C1RCV, &bRxTemp_Buf[0]); 
//    return I2C1RCV; // read and return the data
//    
    I2C4CONbits.RCEN = 1;               // Receive enable
    while (I2C4CONbits.RCEN);           // Wait until RCEN is cleared (automatic)  
    while (!I2C4STATbits.RBF);          // Wait until Receive Buffer is Full (RBF flag)  
    return (unsigned char) I2C4RCV;     // Retrieve value from I2C4RCV
}