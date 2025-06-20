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
        bool initBH1750(alloc_t* allocator, u8 i2c_address);

        // outLuxValue = light intensity in lux
        bool updateBH1750(s32& outLuxValue);
    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_BH1750_H__
