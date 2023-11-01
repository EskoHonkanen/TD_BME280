/**
* @file TD_BME280_test.ino
* @brief
* This simple code show how to use TD_BME280 library to read temperature,
* humidity and pressure from MBE280 sensor. Forced mode is used.

* Interface:
* Sensor         Arduino Uno Board
* --------------------------------
* Vin (3.3V)      3.3V
* Gnd             Gnd
* SDA             A4
* SCK             A5
* --------------------------------
*
* Written by Honee52.
 */

#include <TD_BME280.h>

/**
 * ----------------------------------------------------------------------------
 * Define BME280 and variables.
 * ----------------------------------------------------------------------------
 */
TD_BME280 bme(0x76);
char str_humidity[8];
char str_temperature[8];
char str_pressure[8];
float temperat_o, pressure_o, humidity_o;
uint8_t retval;

/**
 * ----------------------------------------------------------------------------
 * Setup
 * ----------------------------------------------------------------------------
*/
void setup() {
	/* Initialize serial and wait for port to open: */
	Serial.begin(9600);
	while (!Serial) {
		; // Wait for serial port. Remove wait if not native USB port.
	}
  delay(1000);

  /**
  * -----------------------------------------------------
  * BME280 begin.
  * status = bme.begin(0x76, &Wire)
  * status = bme.begin(0x77, &Wire)
  * -----------------------------------------------------
  */
  bme.begin();
  if (bme.init() != NO_ERROR) {
    //digitalWrite(LED_RED, HIGH);
    Serial.println("bme.init() failed!");
  } 	

  /**
  * -----------------------------------------------------
  * Set BME280 into Forced Mode.
  * -----------------------------------------------------
  */
  bme.setSampling(
    MODE_FORCED,
    OSR_X1,         // Temperature
    OSR_X1,         // Pressure
    OSR_X1,         // Humidity
    FILTER_OFF);
  delay(100); 

}

/**
 * ----------------------------------------------------------------------------
 * Main loop.
 * ----------------------------------------------------------------------------
*/
void loop() {
        
  retval = measureValues();
  //
  if (retval == NO_ERROR) {
    dtostrf(temperat_o,  4, 2, str_temperature);
    dtostrf(humidity_o,  4, 2, str_humidity);
    dtostrf(pressure_o,  4, 2, str_pressure);
    //
    Serial.println("Measurements:");
    Serial.print("Temperature: ");
    Serial.println(str_temperature);
    Serial.print("Humidity:    ");
    Serial.println(str_humidity);
    Serial.print("Pressure:    ");
    Serial.println(str_pressure);
  }
  else {
    Serial.print("Error: 0b");
    Serial.println(retval, BIN);
  }
  Serial.println(" ");

  delay(10000); /* Wait */
}

/**
 * ----------------------------------------------------------------------------
 * Measure temperature, humidity and pressure from BME280.
 * @param no
 * @return error code
 * ----------------------------------------------------------------------------
*/
uint8_t measureValues() 
{
  uint8_t ret_val;
  ret_val = NO_ERROR;

  ret_val = bme.startForced();
  ret_val |= bme.readTemperature(&temperat_o);
  ret_val |= bme.readHumidity(&humidity_o);
  ret_val |= bme.readPressure(&pressure_o);
  pressure_o = pressure_o / 100.0F; 

  return ret_val;
}