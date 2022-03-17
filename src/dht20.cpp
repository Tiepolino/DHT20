/*******************************************************************************
 * dht20.cpp
 *
 * Interface for the DHT20 Temperature and Humidity sensor
 *
 * Author				: Tiebolt van der Linden
 * Created			: 2022-03-17 / 20.28
 * Last Changed	: 2022-03-17 / 20.28
 *
 * History
 *		20220317 - Initial Version
 */

#include <dht20.h>

/**
 * @brief Construct a new DHT20::DHT20 object
 * 
 * @param wire 
 */
DHT20::DHT20(TwoWire *wire) : _wire(wire) {
  _temperature = 0.0;
  _humidity = 0.0;
}

#if defined (ESP8266) || defined (ESP32)
/**
 * @brief Startup the DHT20 sensor
 * 
 * @param i2cAddress  - optional - i2c Address of the sensor
 * @param sda         - optional - SDA pin
 * @param scl         - optional - SCL pin
 * @return true       - On success
 * @return false      - On failure
 */
bool DHT20::begin(uint8_t i2cAddress, uint8_t sda, uint8_t scl) {
  uint8_t cmd[3] = {0x71};                                                      // Get status command
  uint8_t result;

  // ---- Check if we need to setup I2C pins ----
  if ((sda < 0xff) && (scl << 0xff)) {
    _wire.begin(sda, scl);
  } else {
    _wire.begin();
  }

  _i2cAddress = i2cAddress;                                                     // Set the i2c address

  _writeCommand(cmd, 1);                                                        // Send the command to te sensor
  _readData(&result, 1);                                                        // Read the result

  // ---- Sensor should respond with 0x18 ----
  if ((result & 0x18) != 0x18) {
    _lastError = DHT20_ERROR_NOSENSOR;
    return false;
  }

  return true;
}
#else
/**
 * @brief Startup the DHT20 sensor
 * 
 * @param i2cAddress  - optional - i2c address of the sensor
 * @return true       - On success
 * @return false      - On failure
 */
bool DHT20::begin(uint8_t i2cAddress) {
  uint8_t cmd[3] = {0x71};                                                      // Get status command
  uint8_t result;

  _i2cAddress = i2cAddress;                                                     // Set the i2c address

  _wire->begin();                                                               // Startup the i2c bus
  _writeCommand(cmd, 1);                                                        // Send the command to the sensor
  _readData(&result, 1);                                                        // Read the result

  // ---- Sensor should respond with 0x18 ----
  if ((result & 0x18) != 0x18) {
    _lastError = DHT20_ERROR_NOSENSOR;
    return false;
  }
  
  return true;
}
#endif

/**
 * @brief Start a measurement and process the results
 * 
 * @return true   - On successfull processing of the result
 * @return false  - On failure
 */
bool DHT20::readSensorData(void) {
  uint8_t cmd[3] = { 0xac, 0x33, 0x00 };                                        // Command to initiate a measurement
  uint8_t data[7] = {0};                                                        // Buffer to store the results
  uint32_t rawData = 0x00;                                                      // Initialise the rawData
  uint8_t retries = DHT20_RETRIES;                                              // Set the number of times to try and read data from the sensor

  _lastError = DHT20_ERROR_NONE;                                                // We are just starting so no errors jet

  _writeCommand(cmd, 3);                                                        // Send the measurement command to the sensor
  delay(DHT20_MEASURETIME);                                                     // Wait for the measurement to complete
  while (retries--) {
    delay(10);
    _readData(&data, 7);                                                        // Read the data from the sensor (6 Data Bytes + 1 CRC Byte)
    if ((data[0] >> 7) == 0)                                                    // Check if measurement is completed
      break;
  }
  
  // ---- Check if the measurement succeeded ----
  if (retries <= 0) {
    _lastError = DHT20_ERROR_MFAIL;
    return false;
  }

  // ---- Get the temperature component ----
  rawData = (data[3] & 0x0f); rawData <<= 8;
  rawData += data[4]; rawData <<= 8;
  rawData += data[5];
  _temperature = (float) (((float) rawData / 0x100000) * 200) - 50;             // Calculate the Temperature in °C

  // ---- Get the humidity component ----
  rawData = data[1]; rawData <<= 8;
  rawData += data[2]; rawData <<= 4;
  rawData += (data[3] >> 4);
  _humidity = (float) ((float) rawData / 0x100000) * 100;                       // Calculate the Relative Humidity in %RH

  return true;
}

/**
 * @brief Returns the last error that has occured
 * 
 * @return uint8_t - The error that has occured
 */
uint8_t DHT20::getLastError(void) {
  return _lastError;
}

/**
 * @brief Returns the measured Temperature in the requested scale
 * 
 * @param scale   - Scale to present the temperature in
 * @return float  - The temperature value in seleced scale
 */
float DHT20::getTemperature(uint8_t scale) {
  _lastError = DHT20_ERROR_NONE;                                                // No error jet

  switch(scale) {
    case _CELCIUS:
      return _temperature;
    case _FAHRENHEIT:
      return (_temperature * 1.8) + 32;
    case _KELVIN:
      return _temperature + 273.15;
    default:
      _lastError = DHT20_ERROR_SCALE;                                           // Unknown scale was presented
      return 255.0;                                                             // Returning 255.0 to indicate an error
  }
}

/**
 * @brief Returns the measured relative humidity in %RH
 * 
 * @return float  - The relative humidity value
 */
float DHT20::getHumidity(void) {
  return _humidity;
}

/**
 * @brief Writes a command to the sensor
 * 
 * @param cmd     - The command to be written
 * @param size    - The number of bytes in the command
 * @return true   - On succes
 * @return false  - On Failure
 */
bool DHT20::_writeCommand(const void *cmd, size_t size) {
  _lastError = DHT20_ERROR_NONE;                                                // No error jet

  uint8_t *buf = (uint8_t *) cmd;                                               // Create a pointer to the beginning of the command
  _wire->beginTransmission(_i2cAddress);                                        // Start the communication with the sensor
  delay(10);
  for (uint8_t byte = 0; byte < size; byte++) {                                 // Send every byte of the command to the sensor
    _wire->write(buf[byte]);
  }
  
  // ---- Check if transmission was successfull ----
  if (_wire->endTransmission() != 0) {
    _lastError = DHT20_ERROR_I2CWRITE;
    return false;
  }

  return true;
}

/**
 * @brief Reads the result from the sensor
 * 
 * @param data    - Buffer where the data can be stored
 * @param size    - Size of the buffer
 * @return true   - On success
 * @return false  - On Failure
 */
bool DHT20::_readData(void *data, size_t size) {
  _lastError = DHT20_ERROR_NONE;                                                // No error jet

  uint8_t *buf = (uint8_t *) data;                                              // Create a pointer to the beginnen of the data buffer
  if (_wire->requestFrom(_i2cAddress, size) != size) {                          // Request 'size' bytes from the sensor
    _lastError = DHT20_ERROR_I2CREAD;                                           // If we get less then 'size' bytes an error occured
    return false;
  }
  delay(10);
  for (uint8_t byte = 0; byte < size; byte++) {                                 // Store the result in the data buffer
    buf[byte] = _wire->read();
  }

  uint8_t crc = _crc8(buf, 6);                                                  // Calculate the checksum over the result

  if (crc != buf[6]) {                                                          // Check if the cecksum is correct
    _lastError = DHT20_ERROR_CRC;
    return false;
  }
  
  return true;
}

/**
 * @brief Calculate a checksum over the result from the sensor
 * 
 * @param ptr       - Pointer to the data buffer containing the result
 * @param size      - Number of bytes in the buffer
 * @return uint8_t  - The calculated CRC8 value for this buffer
 */
uint8_t DHT20::_crc8(uint8_t *ptr, size_t size) {
  uint8_t crc = 0xFF;                                                           // CRC base value

  while (size--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc <<= 1;
        crc ^= 0x31;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}
