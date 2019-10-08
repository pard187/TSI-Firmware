#include "app_can.h"
#include "app_uart.c"
#include "TSIconfig.h"

APP_CAN_DATA app_canData;

uint8_t message_array[(CAN_MESSAGE_SEND_ARRAY_LENGTH)][(CAN_MESSAGE_SEND_BYTES)];
uint16_t addr_array[(CAN_MESSAGE_SEND_ARRAY_LENGTH)];
uint8_t array_count; // from 0 to CAN_MESSAGE_SEND_ARRAY_LENGTH;
uint8_t current_ptr; // from 0 to CAN_MESSAGE_SEND_ARRAY_LENGTH;


void APP_CAN_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_canData.state = APP_CAN_STATE_INIT;
}


void APP_CAN_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_canData.state )
    {
        /* Application's initial state. */
        case APP_CAN_STATE_INIT:
        {
            bool appInitialized = true;
            array_count = 0;
            current_ptr = 0;
            if (appInitialized)
            {
                app_canData.state = APP_CAN_STATE_WAIT;
            }
            break;
        }

        case APP_CAN_STATE_WAIT: // waiting for can message to be put into send array by other apps
        {
            if (current_ptr != array_count) { // messages are added to send array
                app_canData.state = APP_CAN_STATE_SEND;
            }
            else { 
                // all messages are sent in send array, clear the pointer
                current_ptr = 0;
                array_count = 0;
            }
            break;
        }

        case APP_CAN_STATE_SEND: 
        {
            if (!CAN_SEND_TASK()) app_canData.state = APP_CAN_STATE_WAIT; //keep sending message until all messages in send array are sent;
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

// send messages out
int CAN_SEND_TASK() {
    if (current_ptr == array_count) {
        return 0; // All message are sent, APP_CAN_STATE change to WAIT state
    }
    else {
        // what??? if statement with no action??
        if(DRV_CAN0_ChannelMessageTransmit(CAN_CHANNEL0, addr_array[current_ptr], CAN_MESSAGE_SEND_BYTES,
                message_array[current_ptr]) == false); // blocking can transmit function 
        current_ptr++;
        return 1; // message sent, stay in APP_CAN_STATE_SEND
    }
}


// store message from other software system and be ready to send out
/*  Send array in bytes form
 *  1 will be returned if can message is added successfully into send array;
 *  0 will be returned if can message is not added.
 */
int can_send_bytes(uint16_t send_addr, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, 
                    uint8_t b5, uint8_t b6, uint8_t b7) {
    
    if (array_count < CAN_MESSAGE_SEND_ARRAY_LENGTH) { // available place in can send array
        
        addr_array[array_count] = send_addr; // store array value
        
        int i;
        for (i = 0; i < CAN_MESSAGE_SEND_BYTES; i++) { // put message in queue
            switch(i) {
                case 0: message_array[array_count][0] = b0;
                case 1: message_array[array_count][1] = b1;
                case 2: message_array[array_count][2] = b2;
                case 3: message_array[array_count][3] = b3;
                case 4: message_array[array_count][4] = b4;
                case 5: message_array[array_count][5] = b5;
                case 6: message_array[array_count][6] = b6;
                case 7: message_array[array_count][7] = b7;
            }
        }
        array_count++; // increment to be sent array
        return 1;
    }
    else {
        return 0;
    }
}