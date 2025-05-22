#ifndef __RDNO_SENSORS_BME280_H__
#define __RDNO_SENSORS_BME280_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    class alloc_t;

    namespace nsensors
    {
        bool initBME280(alloc_t* allocator, u8 i2c_address);

        // Pressure in hPa
        // Temperature in °C
        // Humidity in %
        void updateBME280(f32& outPressure, f32& outTemperature, f32& outHumidity);
        
    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_BME280_H__
