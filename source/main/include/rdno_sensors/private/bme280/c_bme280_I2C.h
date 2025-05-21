#ifndef __RDNO_SENSORS_BME_280_I2C_H__
#define __RDNO_SENSORS_BME_280_I2C_H__

#include "rdno_sensors/private/bme280/c_bme280.h"

/// BME280I2C - I2C Implementation of BME280.
class BME280I2C : public BME280
{
public:
    enum I2CAddr
    {
        I2CAddr_0x76 = 0x76,
        I2CAddr_0x77 = 0x77
    };

    struct Settings : public BME280::Settings
    {
        Settings(OSR _tosr = OSR_X1, OSR _hosr = OSR_X1, OSR _posr = OSR_X1, Mode _mode = Mode_Forced, StandbyTime _st = StandbyTime_1000ms, Filter _filter = Filter_16, SpiEnable _se = SpiEnable_False, I2CAddr _addr = I2CAddr_0x76)
            : BME280::Settings(_tosr, _hosr, _posr, _mode, _st, _filter, _se)
            , bme280Addr(_addr)
        {
        }

        I2CAddr bme280Addr;
    };

    /// Constructor used to create the class. All parameters have
    /// default values.
    BME280I2C(const Settings& settings = Settings());

    virtual void setSettings(const Settings& settings);
    const Settings& getSettings() const;

protected:

private:
    Settings m_settings;

    /// Write values to BME280 registers.
    virtual bool WriteRegister(uint8_t addr, uint8_t data);

    /// Read values from BME280 registers.
    virtual bool ReadRegister(uint8_t addr, uint8_t data[], uint8_t length);
};


#endif  // __RDNO_SENSORS_BME_280_I2C_H__
