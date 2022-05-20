// -----------------------------------------------------------------------------
// SHT3X Sensor over I2C (Wemos)
// Copyright (C) 2017-2019 by Xose Pérez <xose dot perez at gmail dot com>
// -----------------------------------------------------------------------------

#if SENSOR_SUPPORT && SHT3X_I2C_SUPPORT

#pragma once


#include "I2CSensor.h"
#include "../utils.h"

class SHT3XI2CSensor : public I2CSensor<> {

    public:

        // ---------------------------------------------------------------------
        // Public
        // ---------------------------------------------------------------------

        SHT3XI2CSensor() {
            _sensor_id = SENSOR_SHT3X_I2C_ID;
            _count = 2;
        }

        // ---------------------------------------------------------------------
        // Sensor API
        // ---------------------------------------------------------------------

        // Initialization method, must be idempotent
        void begin() {

            if (!_dirty) return;

            // I2C auto-discover
            unsigned char addresses[] = {0x44,0x45};
            _address = _begin_i2c(_address, sizeof(addresses), addresses);
            if (_address == 0) return;
            i2c_write_uint8(_address, 0x30, 0xA2); // Soft reset to ensure sensor in default state
            espurna::time::blockingDelay(
                espurna::duration::Milliseconds(500));
            _ready = true;
            _dirty = false;

        }

        // Descriptive name of the sensor
        String description() {
            char buffer[25];
            snprintf(buffer, sizeof(buffer), "SHT3X @ I2C (0x%02X)", _address);
            return String(buffer);
        }

        // Type for slot # index
        unsigned char type(unsigned char index) {
            if (index == 0) return MAGNITUDE_TEMPERATURE;
            if (index == 1) return MAGNITUDE_HUMIDITY;
            return MAGNITUDE_NONE;
        }

        // Pre-read hook (usually to populate registers with up-to-date data)
        void pre() {

            _error = SENSOR_ERROR_OK;

            unsigned char buffer[6];
            i2c_write_uint8(_address, 0x2C, 0x06); // Measurement High Repeatability with Clock Stretch Enabled
            espurna::time::blockingDelay(
                espurna::duration::Milliseconds(500));
            i2c_read_buffer(_address, buffer, 6);
            if ((CRC8(buffer[0],buffer[1],buffer[2])) && (CRC8(buffer[3],buffer[4],buffer[5]))) {
                // cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc
                _temperature = ((((buffer[0] * 256.0) + buffer[1]) * 175) / 65535.0) - 45;
                _humidity = ((((buffer[3] * 256.0) + buffer[4]) * 100) / 65535.0);
            }
            else {
                _error = SENSOR_ERROR_CRC;
            }
            status_register();
        }
        
        // Read the status register and output to Debug log
        void status_register() {

            unsigned char buffer[3];
            bool crc, cmd, htr;
            i2c_write_uint8(_address, 0xF3, 0x2D); // Read status register
            espurna::time::blockingDelay(
                espurna::duration::Milliseconds(500));
            i2c_read_buffer(_address, buffer, 3);
            if (CRC8(buffer[0],buffer[1],buffer[2])) {
                // see https://sensirion.com/resource/datasheet/sht3x-d
                crc = buffer[1] & 0b00000001;
                cmd = buffer[1] & 0b00000010;
                htr = buffer[0] & 0b00100000;
                DEBUG_MSG_P(PSTR("[SHT3X] Status crc:%u cmd:%u htr:%u\n"), crc, cmd, htr);
            }
            else {
                DEBUG_MSG_P(PSTR("[SHT3X] Checksum error\n"));
            }
            
            i2c_write_uint8(_address, 0x30, 0x41); // Clear status register
            espurna::time::blockingDelay(
                espurna::duration::Milliseconds(500));
        }

        // Current value for slot # index
        double value(unsigned char index) {
            if (index == 0) return _temperature;
            if (index == 1) return _humidity;
            return 0;
        }

    protected:

        double _temperature = 0;
        unsigned char _humidity = 0;
        
        bool 	CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC) {
          	/*
            * adapted from https://github.com/Risele/SHT3x/blob/master/SHT3x.cpp
          	*	Name  : CRC-8
          	*	Poly  : 0x31	x^8 + x^5 + x^4 + 1
          	*	Init  : 0xFF
          	*	Revert: false
          	*	XorOut: 0x00
          	*	Check : for 0xBE,0xEF CRC is 0x92
          	*/
          	uint8_t crc = 0xFF;
          	uint8_t i;
          	crc ^= MSB;

          	for (i = 0; i < 8; i++)
          			crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

          	crc ^= LSB;
          	for (i = 0; i < 8; i++)
          			crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

          	if (crc == CRC) return true;
          	return false;
        }

};

#endif // SENSOR_SUPPORT && SHT3X_I2C_SUPPORT
