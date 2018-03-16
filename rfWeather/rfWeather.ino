/*****************************************************************************************
* FILENAME :        rfWeather.ino
*
* DESCRIPTION : Main function for the ATTiny85 433MHz temperature and humidity sensor
*  
*
* NOTES :
*
*
* Copyright (c) [2017] [Stephan Wink]
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
vAUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* AUTHOR :    Stephan Wink        START DATE :    01.10.2017
*
*****************************************************************************************/

/****************************************************************************************/
/* Include Interfaces */
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <RCSwitch.h>
#include <RCSwitch.h>
#include "dht.h" //From Rob Tillaart
#include "RfProcl.h"
#include "RfDevices.h"

/****************************************************************************************/
/* Local constant defines */
#define MY_NODE_ID                FROM_NODE_ID_BATH_1
#define MY_SERVER_ID              TO_NODE_ID_1FLOOR

#define DIGI_PIN_D0               0u 
#define DIGI_PIN_D1               1u  
#define DIGI_PIN_D2               2u  
#define DIGI_PIN_D3               3u  
#define DIGI_PIN_D4               4u  
#define DIGI_PIN_D5               5u 

#define TX_PIN                    DIGI_PIN_D2
#define DHT22PIN                  DIGI_PIN_D3

// time until the watchdog wakes the mc in seconds
#define WATCHDOG_TIME             8 // 1, 2, 4 or 8
 
// after how many watchdog wakeups we should collect and send the data
#define WATCHDOG_WAKEUPS_TARGET   7 // 8 * 7 = 56 seconds between each data collection

/****************************************************************************************/
/* Local function like makros */

/****************************************************************************************/
/* Local type definitions (enum, struct, union) */

/****************************************************************************************/
/* Static Data instantiation */
static RCSwitch mySwitch         = RCSwitch();
static dht DHT22;
static float humidity_f32        = 0.0f;
static float temperature_f32     = 0.0f;
static int16_t txTemperature_s16 = 0;
static int16_t txHumidity_s16    = 0;
static uint16_t counter_u16      = 0u;
static msg_t  myMessage_s;


/****************************************************************************************/
/* Public functions (unlimited visibility) */

/**---------------------------------------------------------------------------------------
 * @brief     Verifies the message by checking the pre/post amble and checksum
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
void setup() {
  
  // Transmitter is connected to Arduino Pin #10  
  mySwitch.enableTransmit(TX_PIN);
  
  // Optional set protocol (default is 1, will work for most outlets)
  // mySwitch.setProtocol(2);

  // Optional set pulse length.
  // mySwitch.setPulseLength(320);
  
  // Optional set number of transmission repetitions.
  // mySwitch.setRepeatTransmit(15);

  // initialize message 
  RfProcl::InitializeMessage(&myMessage_s);
  RfProcl::SetFromNodeId(&myMessage_s, MY_NODE_ID);
  RfProcl::SetToNodeId(&myMessage_s, MY_SERVER_ID);

  counter_u16 = 0u;

  // enable the watchdog
  //enableWatchdog();
}

/**---------------------------------------------------------------------------------------
 * @brief     Verifies the message by checking the pre/post amble and checksum
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
void loop() 
{
  int chk         = DHT22.read22(DHT22PIN);
  humidity_f32    = DHT22.humidity;
  temperature_f32 = DHT22.temperature;
  bool succ_bol = true;
  uint8_t data_u8a[4];
  uint16_t data_u16a[2];
  uint32_t data_u32;

  RfProcl::InitializeMessage(&myMessage_s);
  RfProcl::SetFromNodeId(&myMessage_s, FROM_NODE_ID_03);
  RfProcl::SetToNodeId(&myMessage_s, TO_NODE_ID_03);
  RfProcl::SetMsgTypeId(&myMessage_s, MSG_ID_03);
  RfProcl::SetMsgData(&myMessage_s, 0xa5a5);
  RfProcl::CalculateChkSum(&myMessage_s);

  succ_bol = RfProcl::VerifyMessage(&myMessage_s);
  if(FROM_NODE_ID_03 != RfProcl::GetFromNodeId(&myMessage_s))
  {
      succ_bol = false;
      mySwitch.send(0xf1, 8);
      delay(1000);
  }
  if(TO_NODE_ID_03 != RfProcl::GetToNodeId(&myMessage_s))
  {
      succ_bol = false;
      mySwitch.send(0xf2, 8);
      delay(1000);
  }
  if(MSG_ID_03 != RfProcl::GetMsgTypeId(&myMessage_s))
  {
      succ_bol = false;
      mySwitch.send(0xf3, 8);
      delay(1000);
  }
  if(0xa5a5 != RfProcl::GetMsgData(&myMessage_s))
  {
      succ_bol = false;
      mySwitch.send(0xf4, 8);
      delay(1000);
  }
  if(0xffa5a549 != RfProcl::GetRawData(&myMessage_s))
  {
      succ_bol = false;
      mySwitch.send(0xf5, 8);
      delay(1000);
  }
  
  if(true == succ_bol)
  {
    mySwitch.send(RfProcl::GetRawData(&myMessage_s), 32);
    delay(2000);
  }
  else
  {
    mySwitch.send(myMessage_s.header_u8, 8);
    delay(1000);
    mySwitch.send(myMessage_s.data_u16, 16);
    delay(1000);
    mySwitch.send(myMessage_s.chkSum_u8, 8);
    delay(2000);
    data_u8a[0] = (uint8_t)((RfProcl::GetRawData(&myMessage_s) & 0xFF000000) >> 24);
    data_u8a[1] = (uint8_t)((RfProcl::GetRawData(&myMessage_s) & 0x00FF0000) >> 16);
    data_u8a[2] = (uint8_t)((RfProcl::GetRawData(&myMessage_s) & 0x0000FF00) >> 8);
    data_u8a[3] = (uint8_t)((RfProcl::GetRawData(&myMessage_s) & 0x000000FF));
    mySwitch.send(data_u8a[0], 16);
    delay(1000);
    mySwitch.send(data_u8a[1], 16);
    delay(1000);
    mySwitch.send(data_u8a[2], 16);
    delay(1000);
    mySwitch.send(data_u8a[3], 16);
    delay(2000);

    data_u16a[0] = (uint16_t)((RfProcl::GetRawData(&myMessage_s) & 0xFFFF0000) >> 16);
    data_u16a[1] = (uint16_t)((RfProcl::GetRawData(&myMessage_s) & 0x0000FFFF));
    mySwitch.send(data_u16a[0], 16);
    delay(1000);
    mySwitch.send(data_u16a[1], 16);
    delay(2000);

    data_u16a[0] = (uint16_t)((((uint16_t)myMessage_s.header_u8) << 8) + ((myMessage_s.data_u16 & 0xFF00) >> 8));
    data_u16a[1] = (uint16_t)(((myMessage_s.data_u16 & 0x00FF) << 8) + myMessage_s.chkSum_u8);
    mySwitch.send(data_u16a[0], 16);
    delay(1000);
    mySwitch.send(data_u16a[1], 16);
    delay(2000);

    data_u32 = ((uint32_t)(data_u16a[0]) << 16) + data_u16a[1];
    mySwitch.send(data_u32, 32);
    delay(2000);
  }
  /*
  RfProcl::SetMsgTypeId(&myMessage_s, MSG_ID_CNT);
  RfProcl::SetMsgData(&myMessage_s, counter_u16);
  RfProcl::CalculateChkSum(&myMessage_s);
  mySwitch.send(RfProcl::GetRawData(&myMessage_s), 32);
  counter_u16++;
  delay(1000);*/
  /*
  txTemperature_s16 = (int16_t)(temperature_f32 * 100.0); 
  RfProcl::SetMsgTypeId(&myMessage_s, MSG_ID_TEMP);
  RfProcl::SetMsgData(&myMessage_s, (uint16_t)txTemperature_s16);
  RfProcl::CalculateChkSum(&myMessage_s);
  mySwitch.send(RfProcl::GetRawData(&myMessage_s), 32);
  delay(500);
  
  txHumidity_s16 = (int16_t)(humidity_f32 * 100.0); 
  RfProcl::SetMsgTypeId(&myMessage_s, MSG_ID_HUM);
  RfProcl::SetMsgData(&myMessage_s, (uint16_t)txTemperature_s16);
  RfProcl::CalculateChkSum(&myMessage_s);
  mySwitch.send(RfProcl::GetRawData(&myMessage_s), 32);
  delay(500); */ 

  mySwitch.send(4444, 24);
  delay(3000);
  // deep sleep
  /*for(uint8_t i=0;i < WATCHDOG_WAKEUPS_TARGET;i++)
  {
    enterSleep();
  }*/
}

/****************************************************************************************/
/* Private functions: */
/**---------------------------------------------------------------------------------------
 * @brief     Verifies the message by checking the pre/post amble and checksum
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
void enableWatchdog()
{
  cli();
  
  // clear the reset flag
  MCUSR &= ~(1<<WDRF);
  
  // set WDCE to be able to change/set WDE
  WDTCR |= (1<<WDCE) | (1<<WDE);
 
  // set new watchdog timeout prescaler value
  #if WATCHDOG_TIME == 1
    WDTCR = 1<<WDP1 | 1<<WDP2;
  #elif WATCHDOG_TIME == 2
    WDTCR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2;
  #elif WATCHDOG_TIME == 4
    WDTCR = 1<<WDP3;
  #elif WATCHDOG_TIME == 8
    WDTCR = 1<<WDP0 | 1<<WDP3;
  #else
    #error WATCHDOG_TIME must be 1, 2, 4 or 8!
  #endif
  
  // enable the WD interrupt to get an interrupt instead of a reset
  WDTCR |= (1<<WDIE);
  
  sei();
}

/**---------------------------------------------------------------------------------------
 * @brief     Verifies the message by checking the pre/post amble and checksum
 * @author    winkste
 * @date      06. Mar. 2018
 * @param     msg_p     pointer to message buffer
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
// function to go to sleep
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
}
