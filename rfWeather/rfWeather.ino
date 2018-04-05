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
#include <RfProcl.h>
#include <RfDevices.h>


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

#define TX_PIN                    DIGI_PIN_D3
#define DHT22PIN                  DIGI_PIN_D2

// time until the watchdog wakes the mc in seconds
#define WATCHDOG_TIME             8u // 1, 2, 4 or 8
 
// after how many watchdog wakeups we should collect and send the data
#define WATCHDOG_WAKEUPS_TARGET   1u // 8 * 7 = 56 seconds between each data collection

// after how many data collections we should get the battery status
#define BAT_CHECK_INTERVAL        30u
 
// min max values for the ADC to calculate the battery percent
#define BAT_ADC_MIN               0u  // ~0V
#define BAT_ADC_MAX               1023u // ~10V
#define BAT_ADC_REF_VOLTAGE       5.0f
#define BAT_ADC_VOLT_DIVID        2u

/****************************************************************************************/
/* Local function like makros */

/****************************************************************************************/
/* Local type definitions (enum, struct, union) */

/****************************************************************************************/
/* Static Data instantiation */
static RCSwitch mySwitch_sts      = RCSwitch();
static dht dht22_sts;
static float humidity_f32s        = 0.0f;
static float temperature_f32s     = 0.0f;
static int16_t txTemperature_s16s = 0;
static int16_t txHumidity_s16s    = 0;
static uint16_t counter_u16s      = 0u;
static uint16_t isrCounter_u16s   = 0u;
static uint16_t batVal_u16s       = 0u;
static msg_t  myMessage_sts;

// counter for the battery check, starting at BAT_CHECK_INTERVAL to 
// transmit the battery status on first loop
static uint8_t batCheckCount_u8s  = BAT_CHECK_INTERVAL;

/****************************************************************************************/
/* Local functions (reduced visibility) */
static void EnableWatchdog(void);
static void EnterSleep(void);
static void SetupAdc(void);
static uint16_t BatCheck(void);

/****************************************************************************************/
/* Public functions (unlimited visibility) */

/**---------------------------------------------------------------------------------------
 * @brief     ARDUINO Setup function
 * @author    winkste
 * @date      06. Mar. 2018
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
void setup() 
{
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW);
    
    // setup the input voltage measurement
    SetupAdc();
    
    // Transmitter is connected to Arduino Pin #10  
    mySwitch_sts.enableTransmit(TX_PIN);
    
    // Optional set protocol (default is 1, will work for most outlets)
    // mySwitch_sts.setProtocol(2);
    
    // Optional set pulse length.
    // mySwitch_sts.setPulseLength(320);
    
    // Optional set number of transmission repetitions.
    // mySwitch_sts.setRepeatTransmit(15);
    
    // initialize message 
    RfProcl::InitializeMessage(&myMessage_sts);
    RfProcl::SetFromNodeId(&myMessage_sts, MY_NODE_ID);
    RfProcl::SetToNodeId(&myMessage_sts, MY_SERVER_ID);
    
    counter_u16s = 0u;
    
    // enable the watchdog
    EnableWatchdog();
}

/**---------------------------------------------------------------------------------------
 * @brief     ARDUINO loop function
 * @author    winkste
 * @date      06. Mar. 2018
 * @return    n/a
*//*-----------------------------------------------------------------------------------*/
void loop() 
{
    int chk         = dht22_sts.read22(DHT22PIN);
    humidity_f32s    = dht22_sts.humidity;
    temperature_f32s = dht22_sts.temperature;
    
      // battery check/status
    /*batCheckCount_u8s++;
    if(batCheckCount_u8s >= BAT_CHECK_INTERVAL)
    {
      BatCheck();
      batCheckCount_u8s = 0;
    }*/
    
    RfProcl::SetMsgTypeId(&myMessage_sts, MSG_ID_CNT);
    RfProcl::SetMsgData(&myMessage_sts, counter_u16s);
    RfProcl::CalculateChkSum(&myMessage_sts);
    mySwitch_sts.send(RfProcl::GetRawData(&myMessage_sts), 32);
    counter_u16s++;
    delay(1000);
    
    txTemperature_s16s = (int16_t)(temperature_f32s * 100.0); 
    RfProcl::SetMsgTypeId(&myMessage_sts, MSG_ID_TEMP);
    RfProcl::SetMsgData(&myMessage_sts, (uint16_t)txTemperature_s16s);
    RfProcl::CalculateChkSum(&myMessage_sts);
    mySwitch_sts.send(RfProcl::GetRawData(&myMessage_sts), 32);
    delay(1000);
    
    txHumidity_s16s = (int16_t)(humidity_f32s * 100.0); 
    RfProcl::SetMsgTypeId(&myMessage_sts, MSG_ID_HUM);
    RfProcl::SetMsgData(&myMessage_sts, (uint16_t)txHumidity_s16s);
    RfProcl::CalculateChkSum(&myMessage_sts);
    mySwitch_sts.send(RfProcl::GetRawData(&myMessage_sts), 32);
    delay(1000); 
    
    batVal_u16s = BatCheck();
    RfProcl::SetMsgTypeId(&myMessage_sts, MSG_ID_BAT);
    RfProcl::SetMsgData(&myMessage_sts, batVal_u16s);
    RfProcl::CalculateChkSum(&myMessage_sts);
    mySwitch_sts.send(RfProcl::GetRawData(&myMessage_sts), 32);
    delay(1000); 
    
    // deep sleep
    for(uint8_t idx_u8 = 0u; idx_u8 < WATCHDOG_WAKEUPS_TARGET; idx_u8++)
    {
        EnterSleep();
    }
}

/****************************************************************************************/
/* Private functions: */
/**---------------------------------------------------------------------------------------
 * @brief     Configures and enables the Wotchdog
 * @author    winkste
 * @date      06. Mar. 2018
 * @return    n/a
*//*-----------------------------------------------------------------------------------*/
void EnableWatchdog(void)
{
    cli();
    
    // clear the reset flag
    MCUSR &= ~(1 << WDRF);
    
    // set WDCE to be able to change/set WDE
    WDTCR |= (1 << WDCE) | (1 << WDE);
    
    // set new watchdog timeout prescaler value
    #if WATCHDOG_TIME == 1
      WDTCR = 1 << WDP1 | 1 << WDP2;
    #elif WATCHDOG_TIME == 2
      WDTCR = 1 << WDP0 | 1 << WDP1 | 1 << WDP2;
    #elif WATCHDOG_TIME == 4
      WDTCR = 1 << WDP3;
    #elif WATCHDOG_TIME == 8
      WDTCR = 1 << WDP0 | 1 << WDP3;
    #else
      #error WATCHDOG_TIME must be 1, 2, 4 or 8!
    #endif
    
    // enable the WD interrupt to get an interrupt instead of a reset
    WDTCR |= (1 << WDIE);
    
    sei();
}

/**---------------------------------------------------------------------------------------
 * @brief     function to go to sleep
 * @author    winkste
 * @date      06. Mar. 2018
 * @return    n/a
*//*-----------------------------------------------------------------------------------*/
void EnterSleep(void)
{
    /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   
    sleep_enable();
    
    /* Now enter sleep mode. */
    sleep_mode();
    
    /* The program will continue from here after the WDT timeout*/
    sleep_disable(); /* First thing to do is disable sleep. */
}

/**---------------------------------------------------------------------------------------
 * @brief     Setup of the A to D measurement for battery voltage at pin PB4
 * @author    winkste
 * @date      06. Mar. 2018
 * @return    true, if message is compliant to checks
*//*-----------------------------------------------------------------------------------*/
void SetupAdc(void)
{
      // setup the ADC
    ADMUX =
      (1 << ADLAR) | // left shift result
      (0 << REFS1) | // Sets ref. voltage to VCC, bit 0
      (0 << REFS0) | // Sets ref. voltage to VCC, bit 0
      (0 << MUX3)  | // use ADC2 for input (PB4), MUX bit 3
      (0 << MUX2)  | // use ADC2 for input (PB4), MUX bit 2
      (1 << MUX1)  | // use ADC2 for input (PB4), MUX bit 1
      (0 << MUX0);   // use ADC2 for input (PB4), MUX bit 0
    ADCSRA =
      (1 << ADEN)  | // enable ADC
      (1 << ADPS2) | // set prescaler to 64, bit 2
      (1 << ADPS1) | // set prescaler to 64, bit 1
      (0 << ADPS0);  // set prescaler to 64, bit 0
    
    // disable ADC for powersaving
    ADCSRA &= ~(1<<ADEN);
    
    // disable analog comperator for powersaving
    ACSR |= (1<<ACD);
}

/**---------------------------------------------------------------------------------------
 * @brief     function to read and send the battery status
 * @author    winkste
 * @date      06. Mar. 2018
 * @return    returns the measured voltage at Pon pb4
*//*-----------------------------------------------------------------------------------*/
uint16_t BatCheck(void)
{
    uint16_t rawAdcVal_u16;
    float measuredVoltage_f32;
    
    // enable the ADC
    ADCSRA |= (1 << ADEN);
    
    // short delay
    _delay_ms(10);
    
    ADCSRA |= (1 << ADSC); // start ADC measurement
    while ( ADCSRA & (1 << ADSC) ); // wait till conversion complete
    
    rawAdcVal_u16 =  ADCH << 2;
    rawAdcVal_u16 += ADCL >> 6;
    
    // clear the ADIF bit by writing 1 to it
    ADCSRA |= (1 << ADIF);
    
    // disable the ADC
    ADCSRA &= ~(1 << ADEN);
    
    // calc the battery voltage based on the reference voltage
    measuredVoltage_f32 = (float)rawAdcVal_u16;
    measuredVoltage_f32 = measuredVoltage_f32 / 1024.0f;
    measuredVoltage_f32 = measuredVoltage_f32 * BAT_ADC_REF_VOLTAGE;
    measuredVoltage_f32 = measuredVoltage_f32 / BAT_ADC_VOLT_DIVID;
    measuredVoltage_f32 = measuredVoltage_f32 * 100.0f;
    return ((uint16_t) measuredVoltage_f32);
    //return(rawAdcVal_u16);
}

/**---------------------------------------------------------------------------------------
 * @brief     Interrupt Service Routine for watchdog interrupt vector
 * @author    winkste
 * @date      06. Mar. 2018
 * @WDT_vect  watchdog interrupt vector
*//*-----------------------------------------------------------------------------------*/
// watchdog ISR
ISR(WDT_vect)
{
    // nothing to do here, just wake up
    isrCounter_u16s++;
}
