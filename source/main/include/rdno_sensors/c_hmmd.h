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
        bool readHMMD(s8* outDetection, u16* outDistanceInCm);
        bool readHMMD2(s8* outDetection, u16* outDistanceInCm);

    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_HMMD_H__
