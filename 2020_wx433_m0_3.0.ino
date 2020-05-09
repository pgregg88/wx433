
#define DEBUG  // Controls print statements. Comment out for production
//#define DEBUG_BAT  // Controls LED blinks for battery operation debug. Comment out for production
//#define MY_DEBUG  //Controls logging of protocol messages. Comment out for production
//#define MY_DEBUG_VERBOSE_RFM69 //Controls logging  of radio messages. Comment out for production
//#define LOWPOWER_SLEEP // enables lowpower sleep. Don't use if you need to listen for radio messages
//#define ENABLE_MYSENSORS

#ifdef ENABLE_MYSENSORS
  // Configure RFM69 Radio
  #define MY_RADIO_RFM69
  #define MY_RFM69_NEW_DRIVER
  #define MY_IS_RFM69HW
  #define MY_RFM69_FREQUENCY RFM69_433MHZ
  //#define MY_ENCRYPTION_SIMPLE_PASSWD "some16characters"

  // Configure hardware pins
  #define MY_RFM69_IRQ_PIN 9
  #define MY_RFM69_IRQ_NUM 9
  #define MY_RFM69_CS_PIN A2

  // Configure MySensors Node and its children
  // Parent is always my gateway.  No need to search
  #define MY_PARENT_NODE_ID 0
  #define MY_PARENT_NODE_IS_STATIC
  #define MY_NODE_ID 71 // Manually assigned node ID for this device.  AUTO not supported if using MQTT
  #define BARO_CHILD 1
  #define TEMP_CHILD 2
  #define HUM_CHILD 3
  #define VOLT_CHILD 4
#endif

#define BATTERY_PIN A5
#define LED_PIN LED_BUILTIN

// All defines must be before any library inclusions

#include <SPI.h>
#ifdef ENABLE_MYSENSORS
  #include <MySensors.h>
#endif
#include <Wire.h>
#include <SparkFunBME280.h>
#ifdef LOWPOWER_SLEEP
  #include <ArduinoLowPower.h>
#endif
// Define variables to use later
bool initialValueSent = false;
volatile bool awakeFlag = false; 
#ifdef DEBUG_BAT
  unsigned long sleepTime = 5000; // Update every 5 seconds
#else
  unsigned long sleepTime = 300000; // Update every 5 minutes
#endif
float lastTemperature = 0;
float lastPressure = 0;
float lastHumidity = 0;
float lastBattery = 0;

#ifdef ENABLE_MYSENSORS
  // Only use one variable type per child
  // Customize HA to display icon and units
  MyMessage tempMsg(TEMP_CHILD, V_TEMP);
  MyMessage humMsg(HUM_CHILD, V_HUM);
  MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
  MyMessage voltageMsg(VOLT_CHILD, V_VOLTAGE);
#endif

BME280 myBME280; //Global sensor object

//replace Serial with SerialUSB
#if defined (MOTEINO_M0)
  #if defined(SERIAL_PORT_USBVIRTUAL)
    #define Serial SERIAL_PORT_USBVIRTUAL // Required for Serial on Zero based boards
  #endif
#endif

void setup()
{
  /*
    Used for RFM69 CS (NSS)
    I don't think that the MySensors.h anticipates using analog
    input as an output.
    The radio does not work without the following pinMode statement.
  */
  pinMode(A2, OUTPUT);  //Used for RFM69 CS (NSS)
  //Setup ADC. Arduino default is 10 bits
  analogReadResolution(12);

  #ifdef LOWPOWER_SLEEP
    LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, awake, CHANGE);
  #endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //SerialUSB does not work when sleeping the Moteino M0 board.
  // Blinking LED was used in debug mode.
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("In setup function.");
#endif
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  myBME280.beginI2C();
  // Use same settings as Micro Python version
  myBME280.settings.filter = 3;
  myBME280.settings.tempOverSample = 8;
  myBME280.settings.pressOverSample = 4;
  myBME280.settings.humidOverSample = 2;

  myBME280.setMode(MODE_SLEEP); //Sleep for now
}

#ifdef ENABLE_MYSENSORS
  void presentation()
  {
  #ifdef DEBUG
    Serial.println("Sending Presentation Info");
  #endif
    sendSketchInfo("windx", "1.0");   //Max 25 characters.  Optional to do.
    // Use one child per measurement for consistent HA entity naming.
    present(TEMP_CHILD, S_TEMP);
    present(HUM_CHILD, S_HUM);
    present(BARO_CHILD, S_BARO);
    present(VOLT_CHILD, S_MULTIMETER);
  }
#endif

void loop()
{
  myBME280.setMode(MODE_FORCED); //Wake up sensor and take reading
  #ifdef DEBUG
  long startTime = millis();
  #endif
  while (myBME280.isMeasuring() == false) ; //Wait for sensor to start measurment
  while (myBME280.isMeasuring() == true) ; //Hang out while sensor completes the reading
#ifdef DEBUG
  long endTime = millis();

  //Sensor is back from sleep so we can now get the data

  Serial.print(" Measure time(ms): ");
  Serial.println(endTime - startTime);
#endif
  float temperature = myBME280.readTempC();
  float humidity = myBME280.readFloatHumidity();
  float pressure = myBME280.readFloatPressure() / 100.0;
  float battery = analogRead(BATTERY_PIN);
  //Reads approx 620 per volt, 4095/3.3/2
  battery = battery / 625.0; //My calibration

#ifdef DEBUG
  Serial.print(" Temp: ");
  Serial.print(temperature);
  Serial.print(" Humidity: ");
  Serial.print(humidity);
  Serial.print(" Pressure: ");
  Serial.print(pressure);
  Serial.print(" Battery Voltage ");
  Serial.println(battery);
#endif
  // Only attempt to send if link is up else sleep again.
  #ifdef ENABLE_MYSENSORS
  if (transportCheckUplink(true))
  {
    #ifdef DEBUG
      Serial.println("Uplink OK ");
      send(tempMsg.set(temperature, 1));
    #endif
    #ifdef DEBUG_BAT
      send(tempMsg.set(temperature, 1));
      send(pressureMsg.set(pressure, 1));
      send(voltageMsg.set(battery, 2));
      send(humMsg.set(humidity, 2));
    #endif   
    // To Save power only send message on changed values
    if (abs(temperature - lastTemperature) > 0.5)
    {
    send(tempMsg.set(temperature, 1));
    lastTemperature = temperature;
    }
    if (abs(humidity - lastHumidity) > 1.0)
    {
    send(humMsg.set(humidity, 2));
    lastHumidity = humidity;
    }

    if (abs(pressure - lastPressure) > 1.0)
    {
    send(pressureMsg.set(pressure, 1));
    lastPressure = pressure;
    }

    if (abs(battery - lastBattery) > 0.1)
    {
    send(voltageMsg.set(battery, 2));
    lastBattery = battery;
    }
  } else {
    #ifdef DEBUG_BAT
       blink();
    #endif   
   #ifdef DEBUG
      Serial.println("Uplink Down ");
    #endif
  }
  #endif

#ifdef DEBUG
  endTime = millis();
  Serial.print(" Total time(ms): ");
  Serial.println(endTime - startTime);
#endif

  //Read current mode of radio
  // byte rfmOpMode = readRegister(0x01);

#ifdef LOWPOWER_SLEEP

  // Sleeping Radio
  writeRegister(0x01, 0x00);
  // Arduino Sleep
#ifdef DEBUG
  wait(5000); //Use this to debug radio sleep only. Time is in mS
#else
  wait(10); //Some settle time before sleeping
  LowPower.deepSleep(sleepTime);
  wait(10);
#endif
  // Restoring Radio to Listen Mode
  writeRegister(0x01, 0x10);
  
  if (awakeFlag) // Flash LED twice when waking up from sleep.
  {
    #ifdef DEBUG_BAT
      blink2();
    #endif
    awakeFlag = false;
    
  }
  #endif
}

void awake()
{
  awakeFlag = true;
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}

//Read from RFM69HW register.  Not used
/*
byte readRegister(byte dataToSend)
{
  byte result;   // result to return
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
  // return the result:
  return (result);
}
*/
 
 #ifdef LOWPOWER_SLEEP
//Sends a write command to RFM69HW
void writeRegister(byte thisRegister, byte thisValue)
{
  // now combine the register address and the write command into one byte:
  byte dataToSend = thisRegister | 0x80;
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
}


void blink2()
{
  digitalWrite(LED_PIN, HIGH);
  
  (100);
  digitalWrite(LED_PIN, LOW);
  wait(1000);
  digitalWrite(LED_PIN, HIGH);
  wait(100);
  digitalWrite(LED_PIN, LOW);
}
void blink()
{
  digitalWrite(LED_PIN, HIGH);
  wait(100);
  digitalWrite(LED_PIN, LOW);
}
#endif