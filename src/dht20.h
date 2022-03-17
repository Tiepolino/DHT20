#ifndef __SENSENET_DHT20__
#define __SENSENET_DHT20__

#include <Arduino.h>
#include <Wire.h>

// ---- Sensor Settings ----
#define DHT20_I2C_ADDRESS     0x38                                              // Address of this sensor
#define DHT20_RETRIES            3                                              // Number of retries to wait for measurement
#define DHT20_MEASURETIME       85                                              // Time in ms to wait for a measurement to finish

// ---- Temperature Scales ----
#define _CELCIUS              0x01
#define _FAHRENHEIT           0x02
#define _KELVIN               0x03

// ---- DHT20 Error States ----
#define DHT20_ERROR_NONE      0x00                                              // No error
#define DHT20_ERROR_NOSENSOR  0x01                                              // Sensor not found
#define DHT20_ERROR_CRC       0x02                                              // CRC error
#define DHT20_ERROR_MFAIL     0x03                                              // Measurement failed
#define DHT20_ERROR_SCALE     0x04                                              // Scale unknown
#define DHT20_ERROR_I2CWRITE  0x05                                              // I2C Write error
#define DHT20_ERROR_I2CREAD   0x06                                              // I2C Read error

class DHT20 {
  public:
    float _temperature, _humidity;

    DHT20(TwoWire *wire = &Wire);
#if defined (ESP8266) || defined (ESP32)
    bool begin(uint8_t i2cAddress = DHT20_I2C_ADDRESS, uint8_t sda = 0xff, uint8_t scl = 0xff);
#endif
    bool begin(uint8_t i2cAddress = DHT20_I2C_ADDRESS);                         // Starts the sensor
    bool readSensorData(void);                                                  // Initiate a measurement
    uint8_t getLastError(void);                                                 // Returns the last error encountered
    float getTemperature(uint8_t scale = _CELCIUS);                             // Returns the temperature in the requested scale
    float getHumidity(void);                                                    // Returns the humidity in %RH

  private:
    TwoWire *_wire;                                                             // Handle to I2C class
    uint8_t _i2cAddress;                                                        // i2c Address for this sensor
    uint8_t _lastError = 0x00;                                                  // Clear the lastError status

    bool _writeCommand(const void *cmd, size_t size);                           // Writes a command to the sensor
    bool _readData(void *data, size_t size);                                    // Reads the result from the sensor
    uint8_t _crc8(uint8_t *ptr, size_t size);                                   // Calculate the CRC
};

#endif // __SENSENET_DHT20__