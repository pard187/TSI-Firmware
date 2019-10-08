/*
This module is for external ADC and I/O Expander Data communication.
Not tested yet
*/
#include "app_dataio.h"

APP_DATAIO_DATA app_dataioData;
// NCD9830CH: ADC
uint8_t NCD9830CH_READ_FLAG[8];
uint8_t NCD9830CH_WRITE[8];
uint8_t NCD9830CH_READ[8];
// MCP23016GPB: I/O Expander
uint8_t MCP23016GPB_READ_FLAG[8];
uint8_t MCP23016GPB_WRITE[1];
uint8_t MCP23016GPB_READ[2];
uint8_t MCP23016_CONFIG[2];
uint8_t MCP23016_CONFIG_INV[2];
uint8_t buffer_read[1];
uint8_t buffer_write[1];
DRV_I2C_BUFFER_HANDLE h2;

//uint8_t NCD9830CH0_WRITE[1];
//uint8_t NCD9830CH0_READ[1];
//uint8_t NCD9830CH1_WRITE[1];
//uint8_t NCD9830CH1_READ[1];
//uint8_t NCD9830CH2_WRITE[1];
//uint8_t NCD9830CH2_READ[1];
//uint8_t NCD9830CH3_WRITE[1];
//uint8_t NCD9830CH3_READ[1];
//uint8_t NCD9830CH4_WRITE[1];
//uint8_t NCD9830CH4_READ[1];
//uint8_t NCD9830CH5_WRITE[1];
//uint8_t NCD9830CH5_READ[1];
//uint8_t NCD9830CH6_WRITE[1];
//uint8_t NCD9830CH6_READ[1];
//uint8_t NCD9830CH7_WRITE[1];
//uint8_t NCD9830CH7_READ[1];

void APP_DATAIO_Initialize ( void )
{
    app_dataioData.state = APP_DATAIO_STATE_INIT;
    NCD9830CH_WRITE[0] = 0b10000100;
    NCD9830CH_WRITE[1] = 0b11000100;
    NCD9830CH_WRITE[2] = 0b10010100;
    NCD9830CH_WRITE[3] = 0b11010100;
    NCD9830CH_WRITE[4] = 0b10100100;
    NCD9830CH_WRITE[5] = 0b11100100;
    NCD9830CH_WRITE[6] = 0b10110100;
    NCD9830CH_WRITE[7] = 0b11110100;
    
    MCP23016GPB_WRITE[0] = 1;
//    NCD9830CH0_WRITE[0] = 0b10000100;
//    NCD9830CH1_WRITE[0] = 0b10010100;
//    NCD9830CH2_WRITE[0] = 0b10100100;
//    NCD9830CH3_WRITE[0] = 0b10110100;
//    NCD9830CH4_WRITE[0] = 0b11000100;
//    NCD9830CH5_WRITE[0] = 0b11010100;
//    NCD9830CH6_WRITE[0] = 0b11100100;
//    NCD9830CH7_WRITE[0] = 0b11110100;
    
    MCP23016_CONFIG[0] = 0x06; // configure I/O
    MCP23016_CONFIG[1] = 0xFF; // configure I/O
    MCP23016_CONFIG_INV[0] = 0x04; // configure Inverting
    MCP23016_CONFIG_INV[1] = 0xFF; // configure Inverting
    
    buffer_write[0] = 0b10000100;
}


void APP_DATAIO_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_dataioData.state )
    {
        /* Application's initial state. */
        case APP_DATAIO_STATE_INIT:
        {
            bool appInitialized = true;      
            if (appInitialized)
            {
                h2 = DRV_I2C0_Transmit(0x40,MCP23016_CONFIG,2,NULL);
                h2 = DRV_I2C0_Transmit(0x40,MCP23016_CONFIG_INV,2,NULL);
//                h2 = DRV_I2C0_Receive (0x40,buffer_read,1,NULL);            
                app_dataioData.state = APP_DATAIO_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_DATAIO_STATE_SERVICE_TASKS:
        {
            int i;
            for (i = 0; i < 8; i++) {
                if (NCD9830CH_READ_FLAG[i]) {
//                    h2 = DRV_I2C0_Transmit(0x94,buffer_write,1,NULL);
//                    h2 = DRV_I2C0_Receive (0x94,buffer_read,1,NULL); 
//                    h2 = DRV_I2C0_Transmit(0x94,NCD9830CH_WRITE+i,1,NULL);
//                    h2 = DRV_I2C0_Receive (0x94,NCD9830CH_READ+i,1,NULL); 
                    h2 = DRV_I2C0_TransmitThenReceive(0x94,NCD9830CH_WRITE+i,1,NCD9830CH_READ+i,1,NULL); 
                    NCD9830CH_READ_FLAG[i] = 0;
                };
            }

            if (MCP23016GPB_READ_FLAG[0]) {
                h2 = DRV_I2C0_TransmitThenReceive(0x40,MCP23016GPB_WRITE,1,MCP23016GPB_READ,2,NULL); 
                MCP23016GPB_READ_FLAG[0] = 0;
            }; 
            break;
        }
        
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/*Global Functions*/
void set_NCD9830_READ_flag(int ch) {
    NCD9830CH_READ_FLAG[ch] = 1;
}

void set_MCP23016_READ_flag(int ch) {
    MCP23016GPB_READ_FLAG[ch] = 1;
}

int get_MCP23016B() {
    return (int)MCP23016GPB_READ[0];
}

//int get_NCD9830_reading(int ch) {
//    return (int)NCD9830CH_READ[ch];
//}

uint32_t get_NCD9830_reading(int ch) {
    return (uint32_t)NCD9830CH_READ[ch];
}

uint32_t get_ADCCh(int ch){
    DRV_ADC0_Open();
    DRV_ADC1_Open();
    DRV_ADC2_Open();
    DRV_ADC3_Open();
    DRV_ADC_Start();
    uint32_t res = DRV_ADC_SamplesRead(ch);
    DRV_ADC_Stop();
    return res;
}
/*******************************************************************************
 End of File
 */
