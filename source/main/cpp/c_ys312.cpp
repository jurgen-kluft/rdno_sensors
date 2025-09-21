#include "rdno_sensors/c_ys312.h"
#include "rdno_core/c_timer.h"

#ifdef TARGET_ESP32
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nys312
        {
#ifdef TARGET_ESP32

            // NOTE: Cannot get this to work!

            // 1. The PIR generates an interrupt signal approximately every 16ms (pulling the output pin high).
            // 2. After detecting the PIR interrupt signal, maintain the pin for 120µs to 5ms before forcing it low.
            // 3. The MCU IO outputs a low level for 4µs to 8µs, then a high level for 4µs to 8µs.
            //    The MCU IO switches to input mode and reads 1 bit of data (high bits are read first).
            //    The time it takes to pull the pin low + the time it takes to pull the pin high + the time it takes
            //    to read 1 bit of data must not exceed 31.25µs (recommended: no more than 25µs).
            // 4. Repeat step 3 until all 19 bits of data have been read.
            // 5. After reading the data, the MCU IO outputs a low level and switches back to input mode.

#    define PIR_THRESHOLD \
        40  // PIR感应距离阈值设置,阈值越低感应距离越远,一般设置在35~60,根据实际使用确定,阈值设置越低，需要重点观察误报情况！！！
            // 插件探头建议设置35以上，TO贴片建议设置50以上，塑封贴片建议设置60以上

            u16 pirDataLast;  // 上一个PIR数据
            u16 difference;   // 差值
            u8  pirNormalCnt, pirReactionCnt;
            u64 pirLastReadTimeInMillis = 0;

#    define DISINT noInterrupts
#    define ENINT  interrupts

            u8 Timer1ms = 0;

            bool read(s8 PIR_PIN, u16* outValue)
            {
                u16 pirData;
                u8  checkCode;
                u8  i;
                u8  returnValue;

                if (digitalRead(PIR_PIN) == HIGH)  // PIR数据更新产生高电平开始读取
                {
                    pirLastReadTimeInMillis = ntimer::millis();  // 记录PIR最后一次读取时间

                    DISINT();                // 关闭总中断
                    delayMicroseconds(120);  // 延时120us
                    pirData      = 0;
                    checkCode    = 0;
                    returnValue  = 0;
                    for (i = 0; i < 19; i++)  // 读取19位数据
                    {
                        // PIR_PIN = 0;  // PIR引脚输出低电平(先修改寄存器确定输出低电平，后切换为输出模式)
                        // PIR_PIN_OUT;  // 切换输出模式
                        digitalWrite(PIR_PIN, LOW);  // PIR引脚输出低电平(先修改寄存器确定输出低电平，后切换为输出模式)
                        pinMode(PIR_PIN, OUTPUT);    // 切换输出模式

                        delayMicroseconds(4);  // 延时4us~8us

                        digitalWrite(PIR_PIN, HIGH);  // PIR引脚输出高电平
                        delayMicroseconds(4);         // 延时4us~8us

                        pinMode(PIR_PIN, INPUT);  // 切换输入模式
                        delayMicroseconds(4);     // 延时4us~8us

                        // 读取数据
                        if (i < 2 || i > 17)
                        {
                            checkCode <<= 1;
                            if (digitalRead(PIR_PIN) == HIGH)
                            {
                                checkCode |= 0x01;
                            }
                        }
                        else
                        {
                            pirData <<= 1;
                            if (digitalRead(PIR_PIN) == HIGH)
                            {
                                pirData |= 0x01;
                            }
                        }
                    }

                    digitalWrite(PIR_PIN, LOW);  // PIR引脚输出低电平
                    pinMode(PIR_PIN, OUTPUT);    // 切换输出模式

                    delayMicroseconds(4);     // 延时4us~8us
                    pinMode(PIR_PIN, INPUT);  // 切换输入模式

                    ENINT();  // 开启总中断

                    if (checkCode == 0x04)  // 校验头码尾码
                    {
                        if ((pirData & (u16)0x8000) != 0)  // 校验负数
                        {
                            pirData = (~pirData) + 1;  // 转换为正数
                        }

                        if (pirData < 5000)
                        {
                            if (pirData > pirDataLast)  // 数据上升
                            {
                                difference = pirData - pirDataLast;  // 计算前后两次数据差值
                            }
                            else if (pirData < pirDataLast)  // 数据下降
                            {
                                difference = pirDataLast - pirData;  // 计算前后两次数据差值
                            }

                            if (difference < 200)
                            {
                                if (pirNormalCnt < 255)
                                {
                                    pirNormalCnt++;
                                }

                                if (pirData > PIR_THRESHOLD && pirNormalCnt > 20)  // 大于阈值且连续 20 次符合差值小于 200
                                {
                                    if (pirReactionCnt < 3)
                                    {
                                        pirReactionCnt++;
                                    }
                                    else
                                    {
                                        pirReactionCnt = 0;
                                        returnValue    = 1;  // 连续 4 次大于阈值，返回感应触发结果
                                    }
                                }
                                else
                                {
                                    pirReactionCnt = 0;
                                }
                            }
                            else
                            {
                                pirNormalCnt = 0;
                            }
                        }

                        pirDataLast = pirData;
                    }

                    *outValue = pirData;
                    return true;
                }
                else
                {
                    if ((ntimer::millis() - pirLastReadTimeInMillis) > 16)
                    {
                        pirLastReadTimeInMillis = ntimer::millis();  // 记录PIR最后一次读取时间

                        digitalWrite(PIR_PIN, HIGH);  // PIR引脚输出高电平
                        pinMode(PIR_PIN, OUTPUT);     // 切换输出模式
                        delayMicroseconds(4);         // 延时4us左右
                        pinMode(PIR_PIN, INPUT);      // 切换输入模式
                        delayMicroseconds(4);         // 延时4us左右
                    }

                    *outValue = 0;
                    return false;
                }
            }
#else
            bool read(s8 pin, u16* outValue)
            {
                outValue = 0;
                return false;
            }
#endif

        }  // namespace nys312
    }  // namespace nsensors
}  // namespace ncore