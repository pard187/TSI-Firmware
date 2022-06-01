#ifndef I2C4_H
#define I2C4_H

#ifdef __cplusplus
extern "C" {
#endif

#include <xc.h>
#include <inttypes.h>
#include <stdio.h>

#define FSCK 100000.0 // the I2C communication speed we want, 100kHz
#define PBCLK 50000000.0 // the speed of the peripheral bus clock
#define TPGD 0.000000104 // propagation delay, defined in the PIC32MZ EF datasheet as 104ns

extern void Setup_I2C(void);

///////////////////////////////////////// I2C for NCD9830 /////////////////////////////////////////
#define U25_NCD9830_ADDRESS 0x4A            // The address of NCD9830 when AD1 is connected to VDD and AD0 is connected to GND
#define U43_NCD9830_ADDRESS 0x4B            // The address of NCD9830 when AD0 and AD1 are both connected to VDD

#define CH0 0b10001000
#define CH1 0b11000100
#define CH2 0b10010100
#define CH3 0b11010100
#define CH4 0b10100100
#define CH5 0b11100100
#define CH6 0b10110100
#define CH7 0b11110100

extern void NCD9830_read(unsigned char i2c_address, unsigned char channel, unsigned char *value);
///////////////////////////////////////// I2C for NCD9830 /////////////////////////////////////////


///////////////////////////////////////// I2C for MCP23008 /////////////////////////////////////////
#define U47_MCP23008_ADDRESS 0x20            // The address of MCP23016 when the AD0 pin is connected to ground

#define IODIR 0x00
#define IPOL 0x01
#define GPINTEN 0x02
#define DEFVAL 0x03
#define INTCON_MCP 0x04
#define IOCON 0x05
#define GPPU 0x06
#define INTF 0x07
#define INTCAP 0x08
#define GPIO 0x09
#define OLAT 0x0A

extern void MCP23008_read(unsigned char i2c_address, unsigned char *value);
///////////////////////////////////////// I2C for MCP23008 /////////////////////////////////////////


///////////////////////////////////////// I2C for MCP23016 /////////////////////////////////////////
#define MCP23016_ADDRESS 0x20            // The address of MCP23016 when the AD0 pin is connected to ground

#define GP0 0x00
#define GP1 0x01
#define OLAT0 0x02
#define OLAT1 0x03
#define IPOL0 0x04
#define IPOL1 0x05
#define IODIR0 0x06
#define IODIR1 0x07
#define INTCAP0 0x08
#define INTCAP1 0x09
#define IOCON0 0x0A
#define IOCON1 0x0B

extern void MCP23008_read(unsigned char i2c_address, unsigned char *value);
///////////////////////////////////////// I2C for MCP23016 /////////////////////////////////////////
    
    
#ifdef __cplusplus
}
#endif

#endif /* UART1_H */