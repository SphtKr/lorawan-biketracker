#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include "cubecell_SSD1306Wire.h"

#include "LoRaWan_Gps_feedback.h"

#include "secrets.h"

//when gps wakes, if within GPS_UPDATE_TIMEOUT gps is not fixed then into low power mode
#define GPS_UPDATE_TIMEOUT 120000

//once fixed, GPS_CONTINUE_TIME later into low power mode
#define GPS_CONTINUE_TIME 1000
/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

#define INT_VIBR_GPIO GPIO5
#define INT_SHOCK_GPIO GPIO6
#define INT_INACTIVE_SAMPLES_BEFORE_SLEEP 3
uint8_t inactiveSamples = 0;
uint8_t vibrInterruptActionEnabled = 1;
bool resetShocksOnInactivity = true;
uint8_t shocks = 0;

/* OTAA para*/
uint8_t devEui[] = DEVICE_EUI;
uint8_t appEui[] = APP_EUI;
uint8_t appKey[] = APP_KEY;

/* ABP para*/ // Not used but expected by external code to be populated here.
uint8_t nwkSKey[] = NWK_SKEY;
uint8_t appSKey[] = APP_SKEY;
uint32_t devAddr =  ( uint32_t )DEV_ADDR;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]=CHANNEL_MASK;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 50000;// 120000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

// Much of this is based on the example code provided with the HTCC-AB02S,
// with mainly some data payload changes.
static void prepareTxFrame( uint8_t port )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  float lat, lon, alt, hdop, sats;
  
  Serial.println("Waiting for GPS FIX ...");

  Serial.println("GPS Searching...");
  
  Air530.begin();

  uint32_t start = millis();
  while( (millis()-start) < GPS_UPDATE_TIMEOUT )
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
    // gps fixed in a second
    if( Air530.location.age() < 1000 )
    {
      break;
    }
  }
  
  //if gps fixed,  GPS_CONTINUE_TIME later stop GPS into low power mode, and every 1 second update gps, print and display gps info
  if(Air530.location.age() < 1000)
  {
    start = millis();
    uint32_t printinfo = 0;
    while( (millis()-start) < GPS_CONTINUE_TIME )
    {
      while (Air530.available() > 0)
      {
        Air530.encode(Air530.read());
      }

      if( (millis()-start) > printinfo )
      {
        printinfo += 1000;
        printGPSInfo();
      }
    }
  }
  else
  {
    Serial.println("No GPS signal");
    //delay(2000);
  }
  Air530.end(); 
  
  lat = Air530.location.lat();
  lon = Air530.location.lng();
  alt = Air530.altitude.meters();
  sats = Air530.satellites.value();
  hdop = Air530.hdop.hdop();

  uint16_t batteryVoltage = getBatteryVoltage();

  unsigned char *puc;

  appDataSize = 0;
  puc = (unsigned char *)(&lat);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&lon);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&alt);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  puc = (unsigned char *)(&hdop);
  appData[appDataSize++] = puc[0];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[3];
  appData[appDataSize++] = inactiveSamples;
  appData[appDataSize++] = shocks;
  appData[appDataSize++] = (uint8_t)(batteryVoltage >> 8);
  appData[appDataSize++] = (uint8_t)batteryVoltage;

  Serial.print(" BatteryVoltage:");
  Serial.println(batteryVoltage);
}

// Shock Sensor functions
void onShockInterrupt()
{
  shocks++;
  inactiveSamples = 0;
  if(deviceState == DEVICE_STATE_SLEEP){
    deviceState = DEVICE_STATE_SEND;
  } else if(deviceState == DEVICE_STATE_SEND || deviceState == DEVICE_STATE_CYCLE){
    LoRaWAN.cycle(1); // restart the send cycle ASAP
    // ^ Actually this seems bad... causes frame count increment errors, at least
    //   if it happens at the wrong point in the state machine.
  } //TODO: Any corner cases where we could fall into a hole here?
  ClearPinInterrupt(INT_SHOCK_GPIO);
  //Serial.printf("Shock interrupt! shocks: %d\r\n", shocks);
}

//Vibration Sensor functions
void onSamplingInterrupt()
{
  if(vibrInterruptActionEnabled != 0){
    //Serial.printf("GPIO interrupt during sampling. Count inactiveSamples reset to zero!\r\n");
    inactiveSamples = 0;
  }
  ClearPinInterrupt(INT_VIBR_GPIO);
}
void onLongSleepInterrupt()
{
  if(vibrInterruptActionEnabled != 0){
    //Serial.printf("Woke up by GPIO during long sleep!\r\n");
    inactiveSamples = 0;
    detachInterrupt(INT_VIBR_GPIO);
    
    // Pick up where we left off?
    //txDutyCycleTime = 0;
    //LoRaWAN.cycle(txDutyCycleTime);
    //deviceState = DEVICE_STATE_SLEEP;
    deviceState = DEVICE_STATE_SEND; // try this... if problems, do the above.
  }
  ClearPinInterrupt(INT_VIBR_GPIO);
}

void setup() {
  boardInitMcu();
  Serial.begin(115200);

#if(AT_SUPPORT)
  enableAt();
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

  //sleep interrupt code
  pinMode(INT_VIBR_GPIO,INPUT_PULLUP); // https://github.com/HelTecAutomation/CubeCell-Arduino/issues/23
  digitalWrite(INT_VIBR_GPIO, HIGH);
  pinMode(INT_SHOCK_GPIO,INPUT_PULLUP);
  digitalWrite(INT_SHOCK_GPIO, HIGH);
  attachInterrupt(INT_SHOCK_GPIO,onShockInterrupt,FALLING);
}

uint8_t iasTemp = 0;

void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
      Serial.println("LORAWAN_DEVEUI_AUTO");
      LoRaWAN.generateDeveuiByChipID();
#if(LORAWAN_DEVEUI_AUTO)
#endif
#if(AT_SUPPORT)
      getDevParam();
#endif
      printDevParam();
      LoRaWAN.init(loraWanClass,loraWanRegion);
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      Serial.printf("DEVICE_STATE_SEND enter (%d)\r\n", inactiveSamples);
      attachInterrupt(INT_VIBR_GPIO,onSamplingInterrupt,FALLING);
      inactiveSamples++;
      if(inactiveSamples >= INT_INACTIVE_SAMPLES_BEFORE_SLEEP && resetShocksOnInactivity){
        shocks = 0;
      }
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      Serial.printf("DEVICE_STATE_SEND exit\r\n");
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      Serial.printf("DEVICE_STATE_CYCLE enter\r\n");
      detachInterrupt(INT_VIBR_GPIO); // Done sampling
      // Schedule next packet transmission
      if(inactiveSamples < INT_INACTIVE_SAMPLES_BEFORE_SLEEP){
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
      } else {
        attachInterrupt(INT_VIBR_GPIO,onLongSleepInterrupt,FALLING);
        Serial.printf("  Attached onLongSleepInterrupt\r\n");
        // Don't schedule a timer wakeup/duty cycle
      }
      deviceState = DEVICE_STATE_SLEEP;
      Serial.printf("DEVICE_STATE_CYCLE exit\r\n");
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      Serial.printf("DEVICE_STATE_SLEEP enter\r\n");
      LoRaWAN.sleep();
      Serial.printf("DEVICE_STATE_SLEEP exit\r\n");
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
