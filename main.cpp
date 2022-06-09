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

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "Wait.h"

#include "AM2315.h"

AnalogIn   Masse(PA_0); // PA4 dispo comme port
DigitalIn  User_Button(PB_2);
double SoustractionMasse = 0.0029304032;
double ResultatMasse;
#define SOUSTRACTIONMASSE 0.0029304032

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing (temps entre chaque message, de base 3000, 30s = 30000)
 */
#define TX_TIMER                        8500

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

/**
 * Dummy sensor class object
 */
//DS1820  ds1820(PC_9);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Entry point for application
 */

static void sleeping();

static void sleep_for();

int Start = 0;

AM2315::AM2315(PinName SDA , PinName SCL ):i2c(SDA, SCL)
{
}
 
 
 
int main(void)
{    
    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}


// I2C Temperature and Humidity Sensor AM2315
//
bool AM2315::read()
{

  char data_write[5];
  char data_read[10];
  int i =0;
  for(i=0; i<8; i++)
    data_read[i]=0;

  // Wake up the sensor
  // write single byte twice to wake up
  // single write is not enough
  data_write[0] = 0x00;
  i2c.write(AM2315_ADDR,data_write,1,0); 
  i2c.write(AM2315_ADDR,data_write,1,0);
  
  // Read temperature and humidity register
  // send request to AM2315
  data_write[0] = AM2315_REG_READ;
  data_write[1] = 0x00;  // read from adr 0x00
  data_write[2] = 0x04;  // read 4 bytes
  i2c.write(AM2315_ADDR, data_write, 3, 0); // with stop
  
  // wait 2ms before we start to read reg
  wait_ns(2); 
  
  i2c.read(AM2315_ADDR, data_read, 8, 1);

  if (data_read[0] != AM2315_REG_READ) 
    return false;
  // check numbers of bytes read
  if (data_read[1] != 4) 
    return false; 
    
  humidity = data_read[2];
  humidity *= 256;
  humidity += data_read[3];
  humidity /= 10;
  
  celsius = data_read[4] & 0x7F;
  celsius *= 256;
  celsius += data_read[5];
  celsius /= 10;

  if (data_read[4] >> 7) 
    celsius = -(celsius);

  // return celsius;
  return true;
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{  
    AM2315 hydrotemp;
    float celsiusMain;
    float humidityMain;
    hydrotemp.read();
    humidityMain = hydrotemp.humidity;
    celsiusMain = hydrotemp.celsius;
    
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;
    
    uint32_t PoidsNormalized = 0;
    uint32_t PoidsLoop = 0;

    // Masse analog entre 0 et 1
    for(int i = 0;i<1000;i++){
        wait_us(1000);
        PoidsLoop = Masse.read_u16(); // Prend la mesure immédiate du poids. (entre 0 et 65535)
        PoidsNormalized += PoidsLoop; // Prend la mesure immédiate du poids. (entre 0 et 65535)
        }
    PoidsNormalized=PoidsNormalized/1000;

    printf("\r\n Sensor Value Celsius = %.10f \r\n", celsiusMain); // TEST printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
    printf("\r\n Sensor Value Humidity = %.10f \r\n", humidityMain);
    printf("\r\n Sensor Value Weight normalized = 0x%04X \n", PoidsNormalized);
    printf("\r\n Sensor Value Weight in Kg = %.3f \r\n", (71*(float)PoidsNormalized)/(0.228*65536));

    packet_len = sprintf((char *) tx_buffer, "%d|%.2f|%.2f",(int)humidityMain,celsiusMain,(71*(float)PoidsNormalized)/(0.228*65536)); /*  Message envoyé en Base64 à "frm_payload" dans les Forward Uplink Data (ANCIEN CODE : 71*(float)Masse)/0.2388278544) */
    /*  Humidité en int : Temérature en float 0.X : Masse en 100g float */
    /* 100|0.1|50.1 */
    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message); // (30000 = 30s et 3000 = 3s)
            }
        }
        return;
    }
    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                sleeping();
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            
            sleeping();
            
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

static void sleeping()
{
    if (Start<=1) // METTRE 1 POUR QUE CA MARCHE (envoie 2 message de suite mais marche)
    {
        send_message();
        Start++;
    }
    else
    {
        printf("\r\n Sleeping for 4 hours \r\n");

        ThisThread::sleep_for(14400000); // Sleep pour 4 heures (30s = 30000) 14400s
        
        printf("\r\n Wake up \r\n");
        send_message();
        Start=0;
    }
}

static void sleep_for()
{
    printf("\r\n Sleeping for 4 hours \r\n");
    ThisThread::sleep_for(14400000); // Sleep pour 4 heures (30s = 30000) 14400s
        
    printf("\r\n Wake up \r\n");
}

// EOF
