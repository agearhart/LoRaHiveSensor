#include "arduino_secrets.h"

#include "ArduinoJson.h"

#include "avr/sleep.h"

// https://www.airspayce.com/mikem/arduino/RadioHead/
#include "libraries/RadioHead/RH_RF69.h"
#include "SPI.h"

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
#include "DHT.h"

// #define LOOP_WAIT 1800000 // 30 minutes for production
#define LOOP_WAIT 5000  // 5 seconds for testing

#define RF69_FREQ 915.0  // America license free ISM band 915MHz

#define RFM69_CS      8  // output pin
#define RFM69_INT     7  // input pin
#define RFM69_RST     4  // reset
#define LED           13  // blinker

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1 to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND (if your sensor has 3 pins)
// Connect pin 4 (on the right) of the sensor to GROUND and leave the pin 3 EMPTY (if your sensor has 4 pins)
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

#define DHTTYPE DHT11   // DHT 11
#define DHTPIN 2 // pin we're connected to

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Instance of our temperature & humidity sensor
DHT dht(DHTPIN, DHTTYPE);

// comes from arduino_secrets.h
char loraEncryptKey[] = SECRET_LORA_ENCRYPTION_KEY;
String sensorId[] = SECRET_SENSOR_ID;

void setup()
{
    Serial.begin(115200);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    // manually reset the radio
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if (!rf69.init()) {
        Serial.println("RFM69 radio init failed");
        sleep_mode();
    }

    /* Defaults after init are:
    *    434.0MHz,
    *    modulation GFSK_Rb250Fd250,
    *    +13dbM (for low power module)
    *    No encryption
    */
    if (!rf69.setFrequency(RF69_FREQ)) {
        Serial.println("setFrequency failed");
        sleep_mode();;
    }

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    rf69.setEncryptionKey( (uint8_t *) loraEncryptKey );

    pinMode(LED, OUTPUT);

    dht.begin();
}

void loop()
{
    // Wait a few seconds between measurements.
    delay(LOOP_WAIT);

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();

    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(F("Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));

    DynamicJsonDocument sensorReading(1024);

    sensorReading["sensor"]    = SECRET_SENSOR_ID;
    sensorReading["humidity"]  = h;
    sensorReading["tempC"]     = t;
    sensorReading["heatIndex"] = hic;

    int sensorReadingLength = measureJson(sensorReading) + 1;

    char sensorReadingString[sensorReadingLength];
    serializeJson(sensorReading, sensorReadingString, sensorReadingLength);

    Serial.print("Sending "); Serial.println(*sensorReadingString);

    // Send a message!
    rf69.send((uint8_t *)sensorReadingString, sensorReadingLength);
    rf69.waitPacketSent();
}
