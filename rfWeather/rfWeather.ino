/*
  Example for different sending methods
  
  https://github.com/sui77/rc-switch/
  
*/

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <RCSwitch.h>
#include "dht.h" //From Rob Tillaart

#define DIGI_PIN_D0            0u 
#define DIGI_PIN_D1            1u  
#define DIGI_PIN_D2            2u  
#define DIGI_PIN_D3            3u  
#define DIGI_PIN_D4            4u  
#define DIGI_PIN_D5            5u 

#define TX_PIN                 DIGI_PIN_D2
//#define DIGI_PIN_LED           DIGI_PIN_D1
#define DHT22PIN               DIGI_PIN_D3

// time until the watchdog wakes the mc in seconds
#define WATCHDOG_TIME 8 // 1, 2, 4 or 8
 
// after how many watchdog wakeups we should collect and send the data
#define WATCHDOG_WAKEUPS_TARGET 7 // 8 * 7 = 56 seconds between each data collection

RCSwitch mySwitch     = RCSwitch();
dht DHT22;
float humidity_f32        = 0.0f;
float temperature_f32     = 0.0f;
int16_t txTemperature_s16 = 0;
int16_t txHumidity_s16    = 0;
uint32_t txData_u32       = 0;

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

void setup() {
  
  // Transmitter is connected to Arduino Pin #10  
  mySwitch.enableTransmit(TX_PIN);
  
  // Optional set protocol (default is 1, will work for most outlets)
  // mySwitch.setProtocol(2);

  // Optional set pulse length.
  // mySwitch.setPulseLength(320);
  
  // Optional set number of transmission repetitions.
  // mySwitch.setRepeatTransmit(15);

  // enable the watchdog
  //enableWatchdog();
  
}

void loop() 
{
  int chk         = DHT22.read22(DHT22PIN);
  humidity_f32    = DHT22.humidity;
  temperature_f32 = DHT22.temperature;
  
  mySwitch.send(5396, 24);
  delay(1000);
  txTemperature_s16 = (int16_t)(temperature_f32 * 100.0); 
  txData_u32 = txTemperature_s16;
  mySwitch.send(txData_u32, 24);
  delay(1000);
  txHumidity_s16 = (int16_t)(humidity_f32 * 100.0); 
  txData_u32 = txHumidity_s16;
  mySwitch.send(txData_u32, 24);
  delay(1000);
  /* Same switch as above, but using decimal code */
  mySwitch.send(5396, 24);
  delay(2000);  

  // deep sleep
  /*for(uint8_t i=0;i < WATCHDOG_WAKEUPS_TARGET;i++)
  {
    enterSleep();
  }*/
}

// watchdog ISR
ISR(WDT_vect)
{
  // nothing to do here, just wake up
}
