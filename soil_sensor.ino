#include "main.h"

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t taskEvent = NULL;

/** Timer to wakeup task frequently and send message */
SoftwareTimer taskWakeupTimer;

/** Buffer for received LoRaWan data */
uint8_t rcvdLoRaData[256];
/** Length of received data */
uint8_t rcvdDataLen = 0;

#include <Wire.h>
#include "ClosedCube_BME680.h"

ClosedCube_BME680 bme680;


/**
 * @brief Flag for the event type
 * -1 => no event
 * 0 => LoRaWan data received
 * 1 => Timer wakeup
 * 2 => tbd
 * ...
 */
uint8_t eventType = -1;

/**
 * @brief Timer event that wakes up the loop task frequently
 * 
 * @param unused 
 */
void periodicWakeup(TimerHandle_t unused)
{
  // Switch on blue LED to show we are awake
  digitalWrite(LED_CONN, HIGH);
  eventType = 1;
  // Give the semaphore, so the loop task will wake up
  xSemaphoreGiveFromISR(taskEvent, pdFALSE);
}

/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void)
{
  // Create the LoRaWan event semaphore
  taskEvent = xSemaphoreCreateBinary();
  // Initialize semaphore
  xSemaphoreGive(taskEvent);

  // Initialize the built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize the connection status LED
  pinMode(LED_CONN, OUTPUT);
  digitalWrite(LED_CONN, LOW);

#ifndef MAX_SAVE
  // Initialize Serial for debug output
  Serial.begin(115200);

  time_t timeout = millis();
  // On nRF52840 the USB serial is not available immediately
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else
    {
      break;
    }
  }
#endif

  digitalWrite(LED_BUILTIN, LOW);

  // initialize Hygro / Temp sensor
  Wire.begin();     
  bme680.init(0x76); // I2C address: 0x76 or 0x77
  bme680.reset();
  #ifndef MAX_SAVE
  Serial.print("Chip ID=0x");
  Serial.println(bme680.getChipID(), HEX);
  #endif
  
  // oversampling: humidity = x1, temperature = x2, pressure = x16
  bme680.setOversampling(BME680_OVERSAMPLING_X1, BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X16);
  bme680.setIIRFilter(BME680_FILTER_3);
  bme680.setForcedMode();
    
  // Initialize LoRaWan and start join request
  int8_t loraInitResult = initLoRaWan();

#ifndef MAX_SAVE
  if (loraInitResult != 0)
  {
    switch (loraInitResult)
    {
    case -1:
      Serial.println("SX126x init failed");
      break;
    case -2:
      Serial.println("LoRaWan init failed");
      break;
    case -3:
      Serial.println("Subband init error");
      break;
    case -4:
      Serial.println("LoRa Task init error");
      break;
    default:
      Serial.println("LoRa init unknown error");
      break;
    }

    // Without working LoRa we just stop here
    while (1)
    {
      Serial.println("Nothing I can do, just loving you");
      delay(5000);
    }
  }
  Serial.println("LoRaWan init success");
#endif

  // Take the semaphore so the loop will go to sleep until an event happens
  xSemaphoreTake(taskEvent, 10);
}

/**
 * @brief Arduino loop task. Called in a loop from the FreeRTOS task handler
 * 
 */
void loop(void)
{
  // Switch off blue LED to show we go to sleep
  digitalWrite(LED_BUILTIN, LOW);

  // Sleep until we are woken up by an event
  if (xSemaphoreTake(taskEvent, portMAX_DELAY) == pdTRUE)
  {
    // Switch on blue LED to show we are awake
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500); // Only so we can see the blue LED

    // Check the wake up reason
    switch (eventType)
    {
    case 0: // Wakeup reason is package downlink arrived
#ifndef MAX_SAVE
      Serial.println("Received package over LoRaWan");
#endif
      if (rcvdLoRaData[0] > 0x1F)
      {
#ifndef MAX_SAVE
        Serial.printf("%s\n", (char *)rcvdLoRaData);
#endif
      }
      else
      {
#ifndef MAX_SAVE
        for (int idx = 0; idx < rcvdDataLen; idx++)
        {
          Serial.printf("%X ", rcvdLoRaData[idx]);
        }
        Serial.println("");
#endif
      }

      break;
    case 1: { // Wakeup reason is timer
#ifndef MAX_SAVE
      Serial.println("Timer wakeup");
#endif
      // Wake up by timer expiration

      // Read the T et RNH sensor values
      float Temperature = 0.0, Humidity = 0.0, Pressure = 0.0;
      ClosedCube_BME680_Status status = bme680.readStatus();
      if (status.newDataFlag) {
        Temperature = bme680.readTemperature();
        Humidity = bme680.readHumidity();
        Pressure = bme680.readPressure();
      } else {
#ifndef MAX_SAVE
        Serial.printf("Error reading T & H\r\n");        
#endif
      }
      bme680.setForcedMode();
#ifndef MAX_SAVE
      Serial.printf("Temperature %f\r\n",Temperature);
      Serial.printf("Humidity %f\r\n",Humidity);
      Serial.printf("Pressure %f\r\n",Pressure);
#endif

      // Read the soil moisture value
      uint16_t soil = analogRead(A0);
      if (soil < 640 ) soil = 640;
      if (soil > 940 ) soil = 940;
      float HumidySoil = (soil-640) / 3.0;
#ifndef MAX_SAVE
        Serial.printf("Soil value %f\r\n",HumidySoil);            
#endif
      // Send the data packageÒ
      if (sendLoRaFrame(Temperature, Humidity, Pressure, HumidySoil))
      {
#ifndef MAX_SAVE
        Serial.println("LoRaWan package sent successfully");
#endif
      }
      else
      {
#ifndef MAX_SAVE
        Serial.println("LoRaWan package send failed");
        /// \todo maybe you need to retry here?
#endif
      }
    }
      break;
    default:
#ifndef MAX_SAVE
      Serial.println("This should never happen ;-)");
#endif
      break;
    }
    // Go back to sleep
    xSemaphoreTake(taskEvent, 10);
  }
}
