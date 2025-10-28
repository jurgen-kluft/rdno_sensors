#ifndef __RDNO_SENSORS_RD03D_H__
#define __RDNO_SENSORS_RD03D_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            struct target_t
            {
                s16 x;  // X coordinate in mm (left/right)
                s16 y;  // Y coordinate in mm (distance)
                s16 v;  // Speed of the target in cm/s
            };

            void begin(u8 rxPin, u8 txPin);
            bool update();
            bool getTarget(s8 i, target_t& t);  // get target i (0..2)

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_RD03D_H__
