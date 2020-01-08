/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <mbed_printf.h>
#include "mbed.h"
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"
// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "board.h"
#include "MAX30205.h"
#include "MAX30102.h"
#include "algorithm.h"

using namespace events;

#define LORAWAN_APP_DATA_SIZE 40

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[LORAWAN_APP_DATA_SIZE];
uint8_t rx_buffer[LORAWAN_APP_DATA_SIZE];

int BuffPtr=0;
uint8_t APP_PORT = 2; 

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        5000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     2

/**
 * I2C Instance
 */
#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9
I2C I2c(I2C_SDA, I2C_SCL);

/**
 * Body Temperature Sensor class object
 */
uint8_t address = 0x48;
MAX30205 bodyTempSensor(I2c, address);
static uint16_t rawTemp; 


/*
Heart Beat sensor
*/
uint32_t aun_ir_buffer[500];     //IR LED sensor data
int32_t n_ir_buffer_length;      //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02;                  //SPO2 value
int8_t ch_spo2_valid;            //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;            //heart rate value
int8_t  ch_hr_valid;             //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
int32_t temp_n_sp02 = 99;
int32_t temp_n_heart_rate = 80;
DigitalIn INT(PB_15);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS * EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it down the radio object.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

//#define HAS_DISPLAY 1


const int buffer_size = 255;
// might need to increase buffer size for high baud rates
char gps_buffer[buffer_size+1];

volatile int rx_in=0;
#if MBED_CONF_APP_LORAWAN_ENABLED

#ifdef DEVICE_SPI
/**
 * Entry point for application
 */
int main (void)
{
    // setup tracing
    // int i;
    //

    int i;
    
    setup_trace();
    BoardInit();
    mbed_printf("Starting\n");
    maxim_max30102_reset();
    maxim_max30102_read_reg(0,&uch_dummy); //read and clear status register
    maxim_max30102_init();  //initializes the MAX30102
    n_ir_buffer_length=100;
    

    // while(1){
    //     uint16_t packet_len;
    //     int16_t retcode;
    //     BuffPtr = 0;
    //     // bodyTempSensor.readTemperature(rawTemp);
    //     // int temperature = bodyTempSensor.toCelsius(rawTemp)*100;
    //     // mbed_printf("\r\n Temperature  : %d \r\n", temperature);
    //     for(i=0;i<n_ir_buffer_length;i++)
    //         {
    //         aun_red_buffer[i-100]=aun_red_buffer[i];
    //         aun_ir_buffer[i-100]=aun_ir_buffer[i];
    //         }
    //         //take 100 sets of samples before calculating the heart rate.
    //         for(i=25;i<n_ir_buffer_length;i++)
    //         {
    //             while(INT.read()==1);
    //             maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
    //         }
    //         maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
    //         if (n_heart_rate != -999 && n_sp02 != -999){
    //             // tx_buffer[BuffPtr++] = n_heart_rate >> 8;
    //             // tx_buffer[BuffPtr++] = n_heart_rate & 0xFF;
    //             // tx_buffer[BuffPtr++] = n_sp02 >> 8;
    //             // tx_buffer[BuffPtr++] = n_sp02 & 0xFF;
    //             // packet_len=BuffPtr;
    //             // retcode = lorawan.send(4, tx_buffer, packet_len, MSG_CONFIRMED_FLAG);
    //             // if (retcode < 0) 
    //             // {
    //             //     retcode == LORAWAN_STATUS_WOULD_BLOCK ? mbed_printf("send - WOULD BLOCK\r\n") : mbed_printf("\r\n send() - Error code %d \r\n", retcode);
    //             //     return;
    //             // }
    //             // memset(tx_buffer, 0, sizeof(tx_buffer));
    //             mbed_printf("\r\n Heart Rate  : %d \r\n", n_heart_rate);
    //             mbed_printf("\r\n SPO2  : %d \r\n", n_sp02);
    //             }
    //             else 
    //             {
    //                 mbed_printf(".\n\r");   
    //             }
    // }

    // maxim_max30102_reset();
    // maxim_max30102_read_reg(0,&uch_dummy); //read and clear status register
    // maxim_max30102_init();  //initializes the MAX30102
    // n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    
        
    //Gps.service();
    //Gps.verbose= true ;

#if 0
    // To test that data is received from GPS this you must make m_uart public
    Gps.m_uart.baud(9600);
    NVIC_DisableIRQ(USART3_IRQn);

    mbed_printf("GPS Echo\n");    
    for (int j=0;j<20000;j++) {
        while ( Gps.m_uart.readable()) {
            gps_buffer[rx_in] = Gps.m_uart.getc();
            mbed_printf("%c",gps_buffer[rx_in]);
            //if (rx_buffer[rx_in]==13 || rx_buffer[rx_in]==10)
        }
    }
    NVIC_EnableIRQ(USART3_IRQn);
#endif

    // Test powecycle gps
    //Gps.enable(0);
    //wait(0.5)
    Gps.enable(1);
    // make sure IRQ is enabled
    NVIC_EnableIRQ(USART3_IRQn);
    Gps.service();
    // mbed_printf(" LIS3DH dev id is %d \n", acc.read_id());   
    // if (acc.read_id() == I_AM_LIS3DH){
    //         mbed_printf(" LIS3DH found\n");   
    // } else 
    // {
    //     mbed_printf(" LIS3DH not found!\n");           
    // }
    // CHECK we got the pins right from mbed
    if (PA_7 != MBED_CONF_APP_LORA_SPI_MOSI)
       printf("MOSI %d\n",MBED_CONF_APP_LORA_SPI_MOSI);

    if (PA_6 != MBED_CONF_APP_LORA_SPI_MISO)
       printf("MISO %d\n",MBED_CONF_APP_LORA_SPI_MISO);

    if (PA_5 != MBED_CONF_APP_LORA_SPI_SCLK)
       printf("SCLK %d\n",MBED_CONF_APP_LORA_SPI_SCLK);

    if (PB_0 != MBED_CONF_APP_LORA_CS)
       printf("CS %d\n",MBED_CONF_APP_LORA_CS);

    if (PH_1 != MBED_CONF_APP_LORA_TCXO)
       printf("TXCO %d\n",MBED_CONF_APP_LORA_TCXO);


    // End check


    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        mbed_printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }
    mbed_printf("\r\n New Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
                                          != LORAWAN_STATUS_OK) {
        mbed_printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }
    mbed_printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        mbed_printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    mbed_printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
    Gps.service( );
    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        mbed_printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    mbed_printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(INT.read()==1); 
        
        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
                wait(0.01);
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
    return 0;
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
    int i;
    
    BuffPtr = 0;
    Gps.service( );
    switch(APP_PORT)
    {

        case 2:
        {
            #if    1
            // Cceck data
                NVIC_DisableIRQ(USART3_IRQn);

                printf("GPS Echo\n");    
                for (int j=0;j<30000;j++) 
                {
                    while ( Gps.m_uart.readable())
                    {
                        gps_buffer[rx_in] = Gps.m_uart.getc();
                        printf("%c",gps_buffer[rx_in]);
                    }
                }

                NVIC_EnableIRQ(USART3_IRQn);
            #endif
            APP_PORT = 3;
            if (Gps.have_fix) 
            {
                 // change port for next transmission
                uint16_t altitudeGps = atoi( Gps.NmeaGpsData.NmeaAltitude );
                double hdopGps;
                double latitude=Gps.Latitude;
                double longitude=Gps.Longitude;
                altitudeGps =  atoi( Gps.NmeaGpsData.NmeaAltitude ); // in m
                hdopGps =  atof( Gps.NmeaGpsData.NmeaHorizontalDilution);
                int32_t lat = ((latitude + 90) / 180.0) * 16777215;
                int32_t lon = ((longitude + 180) / 360.0) * 16777215;
                int16_t alt = altitudeGps;
                int8_t hdev =  hdopGps; 
                tx_buffer[BuffPtr++] = lat >> 16;
                tx_buffer[BuffPtr++] = lat >> 8;
                tx_buffer[BuffPtr++] = lat;
                tx_buffer[BuffPtr++] = lon >> 16;
                tx_buffer[BuffPtr++] = lon >> 8;
                tx_buffer[BuffPtr++] = lon;
                tx_buffer[BuffPtr++] = alt >> 8;
                tx_buffer[BuffPtr++] = alt;
                tx_buffer[BuffPtr++] = hdev;
                tx_buffer[BuffPtr++] = atoi( Gps.NmeaGpsData.NmeaFixQuality);
                tx_buffer[BuffPtr++] = atoi( Gps.NmeaGpsData.NmeaSatelliteTracked);
                packet_len=BuffPtr;
                retcode = lorawan.send(2, tx_buffer, packet_len, MSG_CONFIRMED_FLAG);

                if (retcode < 0) 
                {
                    retcode == LORAWAN_STATUS_WOULD_BLOCK ? mbed_printf("send - WOULD BLOCK\r\n") : mbed_printf("\r\n send() - Error code %d \r\n", retcode);
                    // lorawan.cancel_sending();
                    return;
                }
                mbed_printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
                memset(tx_buffer, 0, sizeof(tx_buffer));
            }
            else 
            {
                
                mbed_printf("No GPS Fix yet\n");
                
            }
        }
        break;
        case 3:
        {
            APP_PORT = 4; // change port for next transmission
            // @TODO: Read body temperature
            bodyTempSensor.readTemperature(rawTemp);
            int temperature = bodyTempSensor.toCelsius(rawTemp)*100;
            tx_buffer[BuffPtr++] = temperature >> 8;
            tx_buffer[BuffPtr++] = temperature & 0xFF; // BuffPtr init to Zero
            packet_len=BuffPtr;
            retcode = lorawan.send(3, tx_buffer, packet_len, MSG_CONFIRMED_FLAG);
            if (retcode < 0) 
            {
                retcode == LORAWAN_STATUS_WOULD_BLOCK ? mbed_printf("send - WOULD BLOCK\r\n") : mbed_printf("\r\n send() - Error code %d \r\n", retcode);
                // lorawan.cancel_sending();
                return;
            }
            mbed_printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
            mbed_printf("\r\n Temperature  : %d \r\n", temperature);
            memset(tx_buffer, 0, sizeof(tx_buffer));
        }
        break;
        case 4:
        {
            
            APP_PORT = 2; // change port for next transmission
            // @TODO: Read heart beat
            for(i=0;i<n_ir_buffer_length;i++)
            {
            aun_red_buffer[i]=aun_red_buffer[i];
            aun_ir_buffer[i]=aun_ir_buffer[i];
            }
            //take 100 sets of samples before calculating the heart rate.
            for(i=25;i<n_ir_buffer_length;i++)
            {
                while(INT.read()==1);
                maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
            }
	        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
            // if (n_heart_rate != -999){ // || n_sp02 != -999
                // n_sp02 = temp_n_sp02;
                // n_heart_rate = temp_n_heart_rate;
                tx_buffer[BuffPtr++] = n_heart_rate >> 8;
                tx_buffer[BuffPtr++] = n_heart_rate & 0xFF;
                tx_buffer[BuffPtr++] = n_sp02 >> 8;
                tx_buffer[BuffPtr++] = n_sp02 & 0xFF;
                packet_len=BuffPtr;
                retcode = lorawan.send(4, tx_buffer, packet_len, MSG_CONFIRMED_FLAG);
                if (retcode < 0) 
                {
                    retcode == LORAWAN_STATUS_WOULD_BLOCK ? mbed_printf("send - WOULD BLOCK\r\n") : mbed_printf("\r\n send() - Error code %d \r\n", retcode);
                    // lorawan.cancel_sending();
                    return;
                }
                memset(tx_buffer, 0, sizeof(tx_buffer));
                mbed_printf("\r\n Heart Rate  : %d \r\n", n_heart_rate);
                mbed_printf("\r\n SPO2  : %d \r\n", n_sp02);
                // }
            
        }
        break;
    }

}

/**
 * Receive a message from the Network Server
 */
// static void receive_message()
// {
//     int16_t retcode;
//     retcode = lorawan.receive(MBED_CONF_LORA_APP_PORT, rx_buffer, sizeof(rx_buffer), MSG_CONFIRMED_FLAG|MSG_UNCONFIRMED_FLAG);
//     if (retcode < 0) 
//     {
//         mbed_printf("\r\n receive() - Error code %d \r\n", retcode);
//         return;
//     }
//     mbed_printf(" Data:");
//     for (uint8_t i = 0; i < retcode; i++) 
//     {
//         mbed_printf("%x", rx_buffer[i]);
//     }
//     mbed_printf("\r\n Data Length: %d\r\n", retcode);
//     memset(rx_buffer, 0, sizeof(rx_buffer));
// }

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    Gps.service( );

    switch (event) {
        case CONNECTED:
            mbed_printf("\r\n Connection - Successful \r\n");
            
            
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            mbed_printf("\r\n Disconnected Successfully \r\n");
            

            break;
        case TX_DONE:
            mbed_printf("\r\n Message Sent to Network Server \r\n");
            

            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            mbed_printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            

            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            mbed_printf("\r\n Received message from Network Server \r\n");
            //receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            mbed_printf("\r\n Error in reception - Code = %d \r\n", event);            

            break;
        case JOIN_FAILURE:
            mbed_printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

#endif //DEVICE_SPI

#else
int main (void)
{
    return 0;
}
#endif //MBED_CONF_APP_LORAWAN_ENABLED
// EOF
