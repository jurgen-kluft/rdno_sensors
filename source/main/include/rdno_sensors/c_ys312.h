#ifndef __RDNO_SENSORS_YS312_H__
#define __RDNO_SENSORS_YS312_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nys312
        {
            bool read(s8 pin, u16* outValue);
        }
    }  // namespace nsensors
}  // namespace ncore

#endif  // __RDNO_SENSORS_YS312_H__
