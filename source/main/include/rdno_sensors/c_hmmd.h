#ifndef __RDNO_SENSORS_HMMD_H__
#define __RDNO_SENSORS_HMMD_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        bool initHMMD(u8 rxPin, u8 txPin);
        bool readHMMD(f32* outDistance);

    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_HMMD_H__
