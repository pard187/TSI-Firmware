//#include <stdio.h>
//#include <stdlib.h>
//#include <linux/i2c-dev.h>
//#include <sys/ioctl.h>
//#include <fcntl.h>
//#include <math.h>
//
//void main()
//{
//    // Create I2C bus
//    int file;
//    char *bus = "/dev/i2c-1";
//    if((file = open(bus, O_RDWR)) < 0)
//    {
//        printf("Failed to open the bus. \n");
//        exit(1);
//    }
//    // Get I2C device, PCF8574 I2C address is 0x20(32)
//    ioctl(file, I2C_SLAVE, 0x20);
//    
//    // Set all pins as INPUT(0xFF)
//    char config[1] = {0};
//    config[0] = 0xFF;
//    write(file, config, 1);
//    printf("All Pins State are HIGH \n");
//    sleep(1);
//    
//    // Read 1 byte of data
//    char data[1] = {0};
//    if(read(file, data, 1) != 1)
//    {
//        printf("Error : Input/output Error \n");
//    }
//    else
//    {
//        // Output to screen
//        int data1 = (data[0] & 0xFF);
//        
//        for(int i=0; i<8; i++)
//        {
//            if((data1 & ((int)Math.pow(2, i))) == 0)
//            {
//                printf("I/O Pin %d State is LOW \n", i);
//            }
//            else
//            {
//                printf("I/O Pin %d State is HIGH \n", i);
//                sleep(0.5);
//            }
//        }
//    }
//}