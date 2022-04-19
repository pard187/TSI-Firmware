//// Libraries
//#include <string.h>
//#include <stdio.h> 
//#include <stdbool.h>
//#include <GenericTypeDefs.h> 
//#include <xc.h> 
//
//// Lib files
//#include "I2C_lib.h" //12C Master API lib. 
//
//// The address of MCP23016 
//#define MCP23016_ADDRESS ( 0x50 ) 
//#define I2C_READ ( 1 ) 
//#define I2C_WRITE ( 0 ) 
//
//#define EEPROM_ADDR_SN ( Ox0A00 ) 
//#define LENGTH_SN ( 25 ) 
//
//// To start restart i2c communication from master. 
//static bool StartTransfer(BOOL restart) 
//{
//    // Send the Start (or Restart) signal 
//    if(restart)
//    {
//        i2c_master_restart(); 
//    } 
//    else
//    {
//        i2c_master_setup(); 
//        i2c_master_start(); 
//    }
//    return true; 
//}
//
//// To STOP i2c communication. 
//static void StopTransfer( void ) 
//{
//    // Send the Stop signal 
//    i2c_master_stop();
//} 
//
//// To send one byte data from i2c master to slave. 
//static BOOL TranstnitOneByte(uint8_t data) 
//{
//    BOOL bSendRet; 
//    bSendRet = i2c_master_send(data); 
//    return bSendRet;
//}
//
//// Check whether i2c master in idle condition or not. 
//void IsI2C_master_idle( void)
//{
//    i2c_master_idle(); // wait for completion
//    return; 
//}
// 
//// To Read data from desired address of MCP23016
//BOOL ReadI2C_MCP23016(BYTE bAddr, BYTE bLength, BYTE * bRxData, BOOL bRestart)
//{
//    int Index; 
//    BOOL bSuccess = TRUE; 
//    uint8_t RxSlaveAddress = 0; 
//    // Set the slave address, left shifted by 1, and then a 1 in lsb, indicating read operation. 
//    RxSlaveAddress = ((bAddr << 1) I2C_READ); 
//    // Start and send the address to switch initiate read transfer 
//    if(bSuccess)
//    {
//        // Start the transfer to read the device
//        if(!StartTransfer(FALSE)) 
//        {
//            bSuccess = FALSE; 
//        } 
//
//        // Transmit the address with the READ bit set 
//        if (TranstnitOneByte(RxSlaveAddress)) 
//        {
//            bSuccess = TRUE; 
//        }
//        else 
//        {
//            bSuccess = FALSE; 
//        }
//    }
//
//    // Read the data from the desired address 
//    if(bSuccess)
//    {
//        for (Index = 0; Index < bLength; Index--)
//        {
//            bRxData [Index] = i2c_master_recv(); // receive another byte from the bus 
//            if(Index == (bLength - 1)) 
//                i2c_master_ack(1); // send NACK(1): master needs no more bytes 
//            else 
//                i2c_master_ack(0): // send ACK (0): master wants another byte! 
//        } 
//    } 
//
//    // End the transfer 
//    i2c_master_stop(); // send STOP: end transmission, give up bus 
//    return bSuccess; 
//}
//
//
//BOOL WriteI2C_AT24CMO2_EEPROM(BYTE bLength, BYTE * bTxData, BOOL bSendStop) ( 
//
//} 
//
//LINTS i2cData[258]; int Index; BOOL bSuccess = TRUE; DINTS TxSlaveAddress = 0; 
//
//bAddr, BYTE 
//
//Set the slave address, left shifted by 1, which clears bit 0, indicating a write operation. TxSlaveAddress = ((bAddr c< 1) I2QWRITE_EEPROM); 
//
//Load the device I2C address into the Tx buffer i2cData[0] = TxSlaveAddress; bLength--; for (Index = 1; Index <bLength; Index--) { Copy the data into the Tx buffer i2cData[Index] = bTxData[Index - 1]; 
//
//Start the transfer to write data to the device f( !StartTransfer(FALSE) ) bSuccess = FALSE; } 
//
//Transmit all data Index = 0; while( bSuccess && (Index <bLength) ) 
//
//( 
//
//Transmit a byte if (TransmitOneByte(i2cData[Index])) 
//
//else ( 
//
//Transmission successful Advance to the next byte Index++; 
//
//Transmission was not successful bSuccess = FALSE; break; 
//
//End the transfer (hang here if an error occured) if (TRUE == bSendStop) StopTransfer(); return bSuccess; 
//
//To implementation of write cycle time for AT24C:MO2 EEPROM BOOL WaitForEEPROMWrite_AT24CMO2(BYTE bAddr) ( 
//
//BOOL bSuccess = TRUE; DINTS TxSlaveAddress = 0; TxSlaveAddress = ((bAddr c< 1) I2C_INRITE_EEPROM); 
//
//while(1) ( 
//
//i2c_master_start(); Send the Start Bit 
//
//Transmit just the EEPROMs address if (! TransmitOneByte(TxSlaveAddress)) ( Check the write error here. bSuccess = FALSE; 
//
//if(!bSuccess) bSuccess = FALSE; break; 
//
//if(I2C1 STATbits.ACKSTAT == 0) { EEPROM is back i2c_master_stop(); break: 
//
//i2c_master_stop(): 
//
//return bSuccess; 
//
//} To read data from EEPROM location. static BOOL AT24C:MO2_EEPRONaeadData(WORD wAddr, BYTE bCount, BYTE * buf) 
//
//BOOL bRet = TRUE; BYTE ee_str[66]; 
//
//ee_str[0] = (wAddr Ox100) Br Oxff; ee_str[1] = wAddr Oxff; * Check for I2C Master status before perform an any operation. IsI2C_master_idle(); bRet WriteI2C_AT24C:MO2_EEPROM(AT24CMO2_EEPRONI_ADDR_ES 5, 2, ee_str, TRUE); 
//
//* Check for I2C Master status before perform an any operation. IsI2C_master_idle(); bRet 
//
//WaitForEEPROMWrite_AT24C:MO2(AT24C:MO2_EEPRONI_ADDR ESS); bRet &= ReadI2C_AT24CMO2_EEPROM(AT240102_EEPROM_ADDR_ESS bCount, buf, FALSE); return bRet; 
//
//To write Data to the EEPROM. static BOOL AT24CMO2_EEPROMWriteData(WORD wAddr, BYTE bCount, const BYTE * buf) ( 
//
//BOOL bRet = TRUE; BOOL bRetWR = TRUE; BYTE bLength; BYTE ee_str[66]; while (bCount) 
//
//( 
//
//* find= bytes at start of block so will end on page boundary * bLength = AT240.102_EEPROM_PAGE_SIZE- (BYTE) (wAddr % LG_EEPROM_PAGE_SIZE); bLength = bCount; 
//
//prepare command for next address * ee_str[0] = (wAddr Ox100) Oxff; ee_str[1] = wAddr & Oxff; 
//
//* move data to I2C shift string *% memcpy(&ee_str[2], buf, bLength); 
//
//Write the data to the EEPROM and wait for it to 
//
//complete *: bRetWR WriteI2C_AT24C:MO2_EEPROM(AT24C:MO2_EEPROM_ADDRES 5, bLength + 2, ee_str, TROT); bRet WaitForEEPROMWrite_AT24a.102(AT24C:MO2_EEPROM_ADDR ESS); 
//
//* decrement bytes remaining *. bCount -= bLength; increment pointer into input buffer * buf -= bLength; * and increment EEPROM address * wAddr -= (WORD) bLength; } * loop over this block transfer * 
//
//************** 
//
//*>»» DEV NOTES ««< * For the Write operation inverted the return status because with * PIC32MZ I2C EEPROM returns TRUE on successfully write and * failure return FALSE. 
//
//********************************************************* 
//
//if bRetWR)) bRet = FALSE; else bRet = TRUE: 
//
//return bRet; 
//
//FAILED to write the data. 
//
//Successfully wrote the data. 
//
//* Read serial number from the EEPROM and if its invalid then restore * it to default serial number. 
//
//void main() ( 
//
//unsigned char value; WORD wLen = LENGTH_SN; BYTE * bpSerialNum; BYTE bSN[ LENGTH_SN ]; BYTE bSNStr[LENGTH_SN] 
//
//
//
//Set performance to ultra rad set_performance_mode(); Moved all the ANSEL, 'IRIS and LAT settings to their own function setuP_Ports(); Enable multi-vectored interrupts mode INTCONbits..M1v'EC = 1; No need to set up PPS, I2C hardware is fixed to certain pins. SCL1 = RA14, SDA1 = RA15 Initialise I2C1 at 100kHz i2c_master_setup(); .* Read the serial number string from EEPRONI* AT240102_EEPROMReadData( (WORD)EEPRONI_ADDR_ SN, &wLen, bpSerialNum ); - Validate the serial number if ( ( bpSerialNum[0] < Ox20 ) ( bpSerialNum[0] > Ox7F ) ) "* if not printable serial number then load the default string, and update EEPROM*. memcpy(&bSN[0], &bSNStr[0], LENGTH_SN); 
//
//AT24CMO2_EEPROMWriteData( (WORD)EEPROM_ADDR SN, LENGTH_SN, bSN); * Allow AT240.102 to complete an internal write cycle time of 10msec 
//
//delay_ms(10); // Corresponding citation : (Patel, 2019) * Check for I2C Master status before perform an any 
//
//operation. *. 
//
//IsI2C_master_idle(); * Read the string from EEPROM* AT240102_EEPRONIReadData( (WORD)EEPROM_ADDR_ SN, *wLen, bpSerialNum); * Wait 10ms before trying again so as not to overwhelm the AT24CMO2_EEPROM or the PIC32MZ's I2C peripheral *. delay_ms(10); // Corresponding citation : (Patel, 2019) 
//
//} 