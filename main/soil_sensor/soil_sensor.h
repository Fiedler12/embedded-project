#include "driver/i2c.h"
#include <stdint.h>

/**
 * Configure the soil sensor, setting up the required data structures on the ESP32-C3
 * @param sda_pin The GPIO pin on which the SDA signal of the I2C module is connected (white wire)
 * @param scl_pin The GPIO pin on which the SCL signal of the I2C module is connected (green wire)
 *
 * @return An i2c_port_t that must be given to read_soil_sensor to read the value. Returns -1 if something went wrong
*/
i2c_port_t setup_i2c(int sda_pin, int scl_pin);

/**
 * Read a value from the soil sensor
 * @param port The i2c_port_t returned from setup_soil_sensor
 * @return An unsigned short value, representing the moisture level of the soil sensor
*/
unsigned short read_soil_sensor(i2c_port_t port);

/**
 * An example of how to use the soil sensor
*/
void i2c_example();