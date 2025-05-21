

#include "Arduino.h"
#include "Wire.h"

#include "rdno_sensors/private/bme280/c_bme280_I2C.h"

BME280I2C::BME280I2C(const Settings& settings)
    : BME280(settings)
    , m_settings(settings)
{
}

void BME280I2C::setSettings(const Settings& settings)
{
    m_settings = settings;
    BME280::setSettings(settings);
}

const BME280I2C::Settings& BME280I2C::getSettings() const { return m_settings; }

bool BME280I2C::WriteRegister(uint8_t addr, uint8_t data)
{
    Wire.beginTransmission(m_settings.bme280Addr);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission();

    return true;  // TODO: Check return values from wire calls.
}

bool BME280I2C::ReadRegister(uint8_t addr, uint8_t data[], uint8_t length)
{
    uint8_t ord(0);

    Wire.beginTransmission(m_settings.bme280Addr);
    Wire.write(addr);
    Wire.endTransmission();

    Wire.requestFrom(static_cast<uint8_t>(m_settings.bme280Addr), length);

    while (Wire.available())
    {
        data[ord++] = Wire.read();
    }

    return ord == length;
}
