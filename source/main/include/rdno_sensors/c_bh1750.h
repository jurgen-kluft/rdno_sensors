#ifndef __RDNO_SENSORS_BH1750_H__
#define __RDNO_SENSORS_BH1750_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    class alloc_t;

    namespace nsensors
    {
        bool initBH1750(u8 i2c_address = 0x23);

        // outLuxValue = light intensity in lux
        bool updateBH1750(u16& outLuxValue);
    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_BH1750_H__
