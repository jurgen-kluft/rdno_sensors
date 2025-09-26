#include "rdno_sensors/c_rd03d.h"
#include "rdno_core/c_log.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"

#ifdef TARGET_ESP32
#    include "Arduino.h"
#    include "Wire.h"
#endif

#if 0
namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            static void LOG_ERR(const char* msg)
            {
                ncore::nlog::log_error(msg);
            }

#    define RD03D_TX_BUF_MAX_LEN 18
#    define RD03D_RX_BUF_MAX_LEN 64
#    define RD03D_UART_BAUD_RATE 256000
#    define RD03D_MAX_TARGETS    3

/* Arbitrary max duration to wait on a semaphore */
#    define RD03D_SEMA_MAX_WAIT  K_SECONDS(1)

            enum sensor_channel_rd03d
            {
                /**
                 * Channels to configure the sensor
                 */
                SENSOR_CHAN_RD03D_CONFIG_DISTANCE = SENSOR_CHAN_PRIV_START,
                SENSOR_CHAN_RD03D_CONFIG_FRAMES,
                SENSOR_CHAN_RD03D_CONFIG_DELAY_TIME,
                SENSOR_CHAN_RD03D_CONFIG_DETECTION_MODE,
                SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE,
                /*
                 * Return the X (in val1), Y (in val2) of the target (in mm)
                 */
                SENSOR_CHAN_RD03D_POS,
                /*
                 * Return the speed of the target (val1, in cm/s)
                 */
                SENSOR_CHAN_RD03D_SPEED,
                /*
                 * Return the distance to the target (val1, in mm)
                 */
                SENSOR_CHAN_RD03D_DISTANCE,
            };

            enum sensor_attribute_rd03d
            {
                SENSOR_ATTR_RD03D_TARGETS = SENSOR_ATTR_PRIV_START,

                SENSOR_ATTR_RD03D_TARGET_0,
                SENSOR_ATTR_RD03D_TARGET_1,
                SENSOR_ATTR_RD03D_TARGET_2,

                SENSOR_ATTR_RD03D_CONFIG_VALUE,
                SENSOR_ATTR_RD03D_CONFIG_MINIMUM,
                SENSOR_ATTR_RD03D_CONFIG_MAXIMUM,
            };

            enum rd03d_protocol_cmd_idx
            {
                RD03D_CMD_IDX_OPEN_CMD_MODE,
                RD03D_CMD_IDX_CLOSE_CMD_MODE,
                RD03D_CMD_IDX_DEBUGGING_MODE,
                RD03D_CMD_IDX_REPORTING_MODE,
                RD03D_CMD_IDX_RUNNING_MODE,

                RD03D_CMD_IDX_SET_MIN_DISTANCE,
                RD03D_CMD_IDX_SET_MAX_DISTANCE,
                RD03D_CMD_IDX_SET_MIN_FRAMES,  // Min number of frames considered as appearing
                RD03D_CMD_IDX_SET_MAX_FRAMES,  // Max number of frames considered as dissapearing
                RD03D_CMD_IDX_SET_DELAY_TIME,  // The delay time for considering as dissapeared

                RD03D_CMD_IDX_GET_MIN_DISTANCE,
                RD03D_CMD_IDX_GET_MAX_DISTANCE,
                RD03D_CMD_IDX_GET_MIN_FRAMES,
                RD03D_CMD_IDX_GET_MAX_FRAMES,
                RD03D_CMD_IDX_GET_DELAY_TIME,

                RD03D_CMD_IDX_SINGLE_TARGET_MODE,  // Detection mode single target
                RD03D_CMD_IDX_MULTI_TARGET_MODE,   // Detection mode multiple targets
            };

            enum rd03d_detection_mode
            {
                RD03D_DETECTION_MODE_SINGLE_TARGET,
                RD03D_DETECTION_MODE_MULTI_TARGET,
            };

            enum rd03d_operation_mode
            {
                RD03D_OPERATION_MODE_CMD    = 0x80,
                RD03D_OPERATION_MODE_DEBUG  = 0x00,
                RD03D_OPERATION_MODE_REPORT = 0x01,
                RD03D_OPERATION_MODE_RUN    = 0x02,
            };

            enum rd03d_property
            {
                RD03D_PROP_MIN_DISTANCE,
                RD03D_PROP_MAX_DISTANCE,
                RD03D_PROP_MIN_FRAMES,
                RD03D_PROP_MAX_FRAMES,
                RD03D_PROP_DELAY_TIME,
                RD03D_PROP_DETECTION_MODE,
                RD03D_PROP_OPERATION_MODE,
            };

            struct rd03d_data_t
            {
                u8 tx_bytes;    /* Number of bytes send so far */
                u8 tx_data_len; /* Number of bytes to send */
                u8 tx_data[RD03D_TX_BUF_MAX_LEN];

                u8 rx_enabled;     /* Flag to indicate if RX is enabled */
                u8 rx_bytes;       /* Number of bytes received so far */
                u8 rx_frame_start; /* Start of an ACK or Report in the buffer */
                u8 rx_data_len;    /* Number of bytes to receive */
                u8 rx_data[RD03D_RX_BUF_MAX_LEN];

                u8 operation_mode;
                u8 detection_mode;

                target_t targets[RD03D_MAX_TARGETS];

                u16 min_distance;
                u16 max_distance;
                u16 min_frames;
                u16 max_frames;
                u16 delay_time;
            };

            rd03d_data_t data;

            struct rd03d_cfg_t
            {
                u16 min_distance;
                u16 max_distance;
                u16 min_frames;
                u16 max_frames;
                u16 delay_time;
            };

            rd03d_cfg_t config;

            struct device_t
            {
                rd03d_cfg_t    *config;
                rd03d_data_t   *data;
                HardwareSerial *serial;
            };

            HardwareSerial serial;

            device_t device = {&config, &data, &serial};

            // Endianness is Little Endian

            static bool rd03d_rx_frame(const device_t *dev);
            static void rd03d_tx_data(const device_t *dev);

            // clang-format off

            // Protocol data frame format, head and tail
            static const u8 RD03D_CMD_HEAD[]    = {0xFD, 0xFC, 0xFB, 0xFA};
            static const u8 RD03D_CMD_TAIL[]    = {0x04, 0x03, 0x02, 0x01};
            static const u8 RD03D_FRAME_HEAD[]  = {0xF4, 0xF3, 0xF2, 0xF1};
            static const u8 RD03D_FRAME_TAIL[]  = {0xF8, 0xF7, 0xF6, 0xF5};
            static const u8 RD03D_REPORT_HEAD[] = {0xAA, 0xFF, 0x03, 0x00};
            static const u8 RD03D_REPORT_TAIL[] = {0x55, 0xCC};

            // Note: So in the comments you may read Word which means that this is a register value. 
            //       The byte order of the command is thus swapped compared to an indicated Word value.

            // Send command protocol frame format
            // |----------------------------------------------------------------------------------
            // | Frame header | Intra-frame data length  |  Intra-frame data  |   End of frame   |
            // | FD FC FB FA  |  2 bytes                 |    See table 4     |   04 03 02 01    |
            // |----------------------------------------------------------------------------------

            // Table 4 
            // Send intra-frame data format
            // |----------------------------------------------------------
            // |  Command Word (2 bytes)   |    Command value (N bytes)  |
            // |----------------------------------------------------------

            // 	RD03D_CMD_IDX_OPEN_CMD_MODE      = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_CLOSE_CMD_MODE     = { RD03D_CMD_HEADER_BEGIN, 0x02, 0x00, 0xFE, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_DEBUGGING_MODE     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_REPORTING_MODE     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_RUNNING_MODE       = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_MIN_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_MAX_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END },
            // 	RD03D_CMD_IDX_SET_MIN_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_MAX_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_DELAY_TIME     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MIN_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MAX_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MIN_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x02, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MAX_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x03, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_DELAY_TIME     = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x04, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SINGLE_TARGET_MODE = { RD03D_CMD_HEADER_BEGIN, 0x02, 0x00, 0x80, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_MULTI_TARGET_MODE  = { RD03D_CMD_HEADER_BEGIN, 0x02, 0x00, 0x90, 0x00, RD03D_CMD_HEADER_END },

            // return the length of the command
            static int rd03d_prepare_cmd(enum rd03d_protocol_cmd_idx cmd_idx, u8 *cmd_buffer, int value) {
                // Assume the header is already set
                int l = 4; // Skip header

                if (cmd_idx  == RD03D_CMD_IDX_OPEN_CMD_MODE) 
                {
                    cmd_buffer[l++] = 0x04; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0xFF; // Command word
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x01; // 
                    cmd_buffer[l++] = 0x00; //
                }
                else if (cmd_idx == RD03D_CMD_IDX_CLOSE_CMD_MODE) 
                {
                    cmd_buffer[l++] = 0x02; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0xFE; // Command word
                    cmd_buffer[l++] = 0x00; //
                } 
                else if (cmd_idx >= RD03D_CMD_IDX_DEBUGGING_MODE && cmd_idx <= RD03D_CMD_IDX_RUNNING_MODE) 
                {
                    cmd_buffer[l++] = 0x08; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x12; // Command word
                    cmd_buffer[l++] = 0x00; //
                    // 6 bytes of data
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x00; //

                    if (cmd_idx == RD03D_CMD_IDX_REPORTING_MODE) {
                        cmd_buffer[l++] = 0x04; // Reporting mode
                    } else if (cmd_idx == RD03D_CMD_IDX_RUNNING_MODE) {
                        cmd_buffer[l++] = 0x64; // Running mode
                    } else {
                        cmd_buffer[l++] = 0x00; // Debugging mode			
                    }

                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x00; //
                }
                else if (cmd_idx >= RD03D_CMD_IDX_SET_MIN_DISTANCE && cmd_idx <= RD03D_CMD_IDX_SET_DELAY_TIME) 
                {
                    cmd_buffer[l++] = 0x08; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x07; // Command word
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = cmd_idx - RD03D_CMD_IDX_SET_MIN_DISTANCE; // Command value
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = value & 0xFF; // Set value, 32-bit
                    cmd_buffer[l++] = (value >> 8) & 0xFF;
                    cmd_buffer[l++] = (value >> 16) & 0xFF;
                    cmd_buffer[l++] = (value >> 24) & 0xFF;
                }
                else if (cmd_idx >= RD03D_CMD_IDX_GET_MIN_DISTANCE && cmd_idx <= RD03D_CMD_IDX_GET_DELAY_TIME) 
                {
                    cmd_buffer[l++] = 0x04; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x08; // Command word
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = cmd_idx - RD03D_CMD_IDX_GET_MIN_DISTANCE; // Command value
                    cmd_buffer[l++] = 0x00; //
                }
                else if (cmd_idx >= RD03D_CMD_IDX_SINGLE_TARGET_MODE && cmd_idx <= RD03D_CMD_IDX_MULTI_TARGET_MODE) 
                {
                    cmd_buffer[l++] = 0x02; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = RD03D_CMD_IDX_SINGLE_TARGET_MODE ? 0x80 : 0x90; // Command word
                    cmd_buffer[l++] = 0x00; //
                }

                // Write the tail
                cmd_buffer[l++] = RD03D_CMD_TAIL[0];
                cmd_buffer[l++] = RD03D_CMD_TAIL[1];
                cmd_buffer[l++] = RD03D_CMD_TAIL[2];
                cmd_buffer[l++] = RD03D_CMD_TAIL[3];

                return l;
            }

            // ACK command protocol frame format
            // |--------------------------------------------------------------------------------
            // | Frame header  | Intra-frame data length  |  Intra-frame data  |  End of frame |
            // | FD FC FB FA   |       2bytes             |      See table 6   |   04 03 02 01 |
            // |--------------------------------------------------------------------------------

            // Table 6
            // ACK intra-frame data format
            // |------------------------------------------------------------------
            // |  Send Command Word | 0x0100 (2 bytes)  | Return value (N bytes)  |
            // |------------------------------------------------------------------

            // static const u8 RD03D_CMD_ACK_IDX_OPEN_CMD_MODE[]   = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, RD03D_CMD_HEADER_END };
            // static const u8 RD03D_CMD_ACK_IDX_CLOSE_CMD_MODE[]  = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0xFE, 0x01, 0x00, 0x00, RD03D_CMD_HEADER_END };

            // Protocol, ACKS we get for set and get commands, the ACK related to the get cmd contains a 4 byte variable at (rx_buffer[10] to rx_buffer[13])
            // static const u8 RD03D_CMD_ACK_IDX_SET_XXX[] = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x07, 0x01, 0x00, 0x00, RD03D_CMD_HEADER_END },
            // static const u8 RD03D_CMD_ACK_IDX_GET_XXX[] = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END },

            // clang-format on
            static inline bool data_frame_has_valid_len(int data_len) { return (data_len == 12 || data_len == 14 || data_len == 18); }
            static inline bool data_frame_has_valid_head(const u8 *data, int data_len) { return data[0] == RD03D_FRAME_HEAD[0] && data[1] == RD03D_FRAME_HEAD[1] && data[2] == RD03D_FRAME_HEAD[2] && data[3] == RD03D_FRAME_HEAD[3]; }

            static inline int data_frame_get_intra_frame_data_len(const u8 *data, int data_len)
            {
                const int len = data[4] | (data[5] << 8);
                return (len != 2 && len != 4 && len != 8) ? -1 : len;
            }

            static inline bool data_frame_has_valid_tail(const u8 *data, int data_len, int intra_frame_data_len)
            {
                if (data_len != (4 + 2 + intra_frame_data_len + 4))
                {
                    return false;
                }

                return data[4 + 2 + intra_frame_data_len + 0] == RD03D_FRAME_TAIL[0] && data[4 + 2 + intra_frame_data_len + 1] == RD03D_FRAME_TAIL[1] && data[4 + 2 + intra_frame_data_len + 2] == RD03D_FRAME_TAIL[2] &&
                       data[4 + 2 + intra_frame_data_len + 3] == RD03D_FRAME_TAIL[3];
            }

            static int verify_ack_for_open_cmd(const u8 *data, int data_len)
            {
                const int intra_frame_data_len = 8;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0xFF && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00 && data[10] == 0x01 && data[11] == 0x00 && data[12] == 0x40 && data[13] == 0x00)
                {
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static int verify_ack_for_close_cmd(const u8 *data, int data_len)
            {
                const int intra_frame_data_len = 4;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0xFE && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00)
                {
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static int verify_ack_for_set_cmd(const u8 *data, int data_len)
            {
                const int intra_frame_data_len = 4;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0x07 && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00)
                {
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static int verify_ack_for_get_cmd(const u8 *data, int data_len, int *value)
            {
                *value = 0;

                const int intra_frame_data_len = 8;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0x08 && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00)
                {
                    *value = sys_get_le32(data + 10);
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static void rd03d_uart_flush(device_t *dev)
            {
                while (dev->serial->read() >= 0)
                {
                    // Discard data
                }
            }

#    define RD03D_TX   2
#    define RD03D_RX   1
#    define RD03D_TXRX (RD03D_TX | RD03D_RX)

            static int rd03d_send_cmd(device_t *dev, enum rd03d_protocol_cmd_idx cmd_idx, u8 txrx, int value)
            {
                struct rd03d_data_t      *data = dev->data;
                const struct rd03d_cfg_t *cfg  = dev->config;
                int                       ret;

                if (data->operation_mode != RD03D_OPERATION_MODE_CMD)
                {
                    return -EPERM;
                }

                data->tx_data_len = rd03d_prepare_cmd(cmd_idx, data->tx_data, value);

                if (txrx & RD03D_TX)
                {
                    // Transmit the command
                    rd03d_tx_data(dev);
                }

                if (txrx & RD03D_RX)
                {
                    // Wait for data to be available
                    while (dev->serial->available() == 0)
                    {
                        ntimer::delay_us(100);
                    }

                    // Receive the response
                    rd03d_rx_frame(dev);
                }

                return ret;
            }

            static int rd03d_open_cmd_mode(const device_t *dev)
            {
                struct rd03d_data_t      *data = dev->data;
                const struct rd03d_cfg_t *cfg  = dev->config;
                int                       ret;

                /*
                Note in the documentation:
                   (1) Send "Open command mode" (because the chip may still output
                       data, the data received by the serial port will contain
                       waveform data)
                   (2) Empty serial port cache data (generally delay for around 100ms,
                       to ensure that serial port data is empty)
                   (3) Send the Open Command Mode, once again, and analyze returned
                       results.
                */

                // Open command mode
                ret = rd03d_send_cmd(dev, RD03D_CMD_IDX_OPEN_CMD_MODE, RD03D_TX, 1);
                if (ret < 0)
                {
                    return ret;
                }

                // Wait for 100ms to ensure that the serial port data is not
                // receiving anymore report data
                ntimer::delay(100);

                // Flush the serial port cache
                rd03d_uart_flush(dev);

                // Open command mode again
                ret = rd03d_send_cmd(dev, RD03D_CMD_IDX_OPEN_CMD_MODE, RD03D_TXRX, 1);
                if (ret < 0)
                {
                    return ret;
                }

                // Verify the command was acknowledged successfully
                ret = verify_ack_for_open_cmd(data->rx_data, data->rx_data_len);
                return ret;
            }

            static int rd03d_close_cmd_mode(const device_t *dev)
            {
                struct rd03d_data_t *data = dev->data;
                int                  ret;

                // Close command mode
                ret = rd03d_send_cmd(dev, RD03D_CMD_IDX_CLOSE_CMD_MODE, RD03D_TXRX, 1);
                if (ret < 0)
                {
                    return ret;
                }

                // Verify the command was acknowledged successfully
                ret = verify_ack_for_close_cmd(data->rx_data, data->rx_data_len);
                return ret;
            }

            static int rd03d_set_attribute(const device_t *dev, enum rd03d_protocol_cmd_idx cmd_idx, int value)
            {
                struct rd03d_data_t *data = dev->data;

                int ret;

                // This is always a mutable command, so we need to copy the command into
                // the transmit (tx) buffer, set the value, and then send the command.

                // Note: The user has to explicitly set the command mode to be able to
                //       set and get attributes/channel data.
                if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == 0)
                {
                    ret = rd03d_open_cmd_mode(dev);
                    if (ret < 0)
                    {
                        LOG_ERR("Error, open command mode failed");
                        return -EINVAL;
                    }
                    data->operation_mode |= RD03D_OPERATION_MODE_CMD;
                }

                // Set the attribute value in the command
                ret = rd03d_send_cmd(dev, cmd_idx, RD03D_TXRX, value);
                if (ret < 0)
                {
                    LOG_ERR("Error, set attribute command (%d) failed", cmd_idx);
                    return ret;
                }

                // Verify the command was acknowledged successfully
                ret = verify_ack_for_set_cmd(data->rx_data, data->rx_data_len);
                if (ret < 0)
                {
                    LOG_ERR("Error, set attribute command (%d) did not get valid ACK", cmd_idx);
                    return ret;
                }

                // Note: When setting RD03D_ATTR_OPERATION_MODE to any of the reporting
                //       modes, the sensor will close the command mode.
                //       This means that the command mode will be closed automatically
                //       and the sensor will enter the reporting mode.
                if ((cmd_idx >= RD03D_CMD_IDX_DEBUGGING_MODE && cmd_idx <= RD03D_CMD_IDX_RUNNING_MODE))
                {
                    if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == RD03D_OPERATION_MODE_CMD)
                    {
                        ret = rd03d_close_cmd_mode(dev);
                        if (ret < 0)
                        {
                            LOG_ERR("Error, close command mode failed");
                            return -EINVAL;
                        }
                        data->operation_mode &= ~RD03D_OPERATION_MODE_CMD;
                    }
                }

                return ret;
            }

            static int rd03d_attr_set(const device_t *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
            {
                int ret;

                if (!(chan >= (enum sensor_channel)SENSOR_CHAN_RD03D_CONFIG_DISTANCE && chan <= (enum sensor_channel)SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE))
                {
                    return -ENOTSUP;
                }
                if (!(attr >= (enum sensor_attribute)SENSOR_ATTR_RD03D_CONFIG_VALUE && attr <= (enum sensor_attribute)SENSOR_ATTR_RD03D_CONFIG_MAXIMUM))
                {
                    return -ENOTSUP;
                }

                switch ((enum sensor_channel_rd03d)chan)
                {
                    case SENSOR_CHAN_RD03D_CONFIG_DISTANCE:
                        switch ((enum sensor_attribute_rd03d)attr)
                        {
                            case SENSOR_ATTR_RD03D_CONFIG_MINIMUM: rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MIN_DISTANCE, val->val1); break;
                            case SENSOR_ATTR_RD03D_CONFIG_MAXIMUM: rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MAX_DISTANCE, val->val1); break;
                            default: break;
                        }
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_FRAMES:
                        switch ((enum sensor_attribute_rd03d)attr)
                        {
                            case SENSOR_ATTR_RD03D_CONFIG_MINIMUM: rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MIN_FRAMES, val->val1); break;
                            case SENSOR_ATTR_RD03D_CONFIG_MAXIMUM: rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MAX_FRAMES, val->val1); break;
                            default: break;
                        }
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_DELAY_TIME: rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_DELAY_TIME, val->val1); break;
                    case SENSOR_CHAN_RD03D_CONFIG_DETECTION_MODE:
                        switch (val->val1)
                        {
                            case RD03D_DETECTION_MODE_SINGLE_TARGET: rd03d_set_attribute(dev, RD03D_CMD_IDX_SINGLE_TARGET_MODE, val->val1); break;
                            case RD03D_DETECTION_MODE_MULTI_TARGET: rd03d_set_attribute(dev, RD03D_CMD_IDX_MULTI_TARGET_MODE, val->val1); break;
                            default: ret = -EINVAL; break;
                        }
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE:
                        switch (val->val1)
                        {
                            case RD03D_OPERATION_MODE_DEBUG: rd03d_set_attribute(dev, RD03D_CMD_IDX_DEBUGGING_MODE, val->val1); break;
                            case RD03D_OPERATION_MODE_REPORT: rd03d_set_attribute(dev, RD03D_CMD_IDX_REPORTING_MODE, val->val1); break;
                            case RD03D_OPERATION_MODE_RUN: rd03d_set_attribute(dev, RD03D_CMD_IDX_RUNNING_MODE, val->val1); break;
                            default: ret = -EINVAL; break;
                        }
                        break;
                    default: ret = -ENOTSUP; break;
                }

                return ret;
            }

            static inline int rd03d_get_attribute(const device_t *dev, enum rd03d_protocol_cmd_idx cmd_idx, int *value)
            {
                struct rd03d_data_t *data = dev->data;

                int ret;

                // This is always a mutable command, so we need to copy the command into
                // the transmit (tx) buffer, set the value, and then send the command.

                // Note: The user has to explicitly set the command mode to be able to
                //       set and get attributes/channel data.
                if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == 0)
                {
                    ret = rd03d_open_cmd_mode(dev);
                    if (ret < 0)
                    {
                        LOG_ERR("Error, open command mode failed");
                        return -EINVAL;
                    }
                    data->operation_mode |= RD03D_OPERATION_MODE_CMD;
                }

                // Get command
                ret = rd03d_send_cmd(dev, cmd_idx, RD03D_TXRX, 0);
                if (ret < 0)
                {
                    LOG_ERR("Error, get attribute command (%d) failed", (int)cmd_idx);
                    return ret;
                }

                // Read the value from the response
                ret = verify_ack_for_get_cmd(data->rx_data, data->rx_data_len, value);
                if (ret < 0)
                {
                    LOG_ERR("Error, get attribute command (%d) did not get valid ACK", cmd_idx);
                    return ret;
                }

                // Note: When setting RD03D_ATTR_OPERATION_MODE to any of the reporting
                //       modes, the sensor will close the command mode.
                //       This means that the command mode will be closed automatically
                //       and the sensor will enter the reporting mode.
                if ((cmd_idx >= RD03D_CMD_IDX_DEBUGGING_MODE && cmd_idx <= RD03D_CMD_IDX_RUNNING_MODE))
                {
                    if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == RD03D_OPERATION_MODE_CMD)
                    {
                        ret = rd03d_close_cmd_mode(dev);
                        if (ret < 0)
                        {
                            return -EINVAL;
                        }
                        data->operation_mode &= ~RD03D_OPERATION_MODE_CMD;
                    }
                }

                return ret;
            }

            static int rd03d_attr_get(const device_t *dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value *val)
            {
                struct rd03d_data_t *data = dev->data;
                int                  ret;

                enum sensor_channel_rd03d   rdchan = (enum sensor_channel_rd03d)chan;
                enum sensor_attribute_rd03d rdattr = (enum sensor_attribute_rd03d)attr;

                if (!(rdchan >= SENSOR_CHAN_RD03D_CONFIG_DISTANCE && rdchan <= SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE))
                {
                    return -ENOTSUP;
                }
                if (!(rdattr >= SENSOR_ATTR_RD03D_CONFIG_VALUE && rdattr <= SENSOR_ATTR_RD03D_CONFIG_MAXIMUM))
                {
                    return -ENOTSUP;
                }

                int ti = 0;

                switch (chan)
                {
                    case SENSOR_CHAN_PROX:
                        val->val1 = 0;
                        for (ti = 0; ti < RD03D_MAX_TARGETS; ti++)
                        {
                            if (data->targets[ti].x != 0 && data->targets[ti].y != 0)
                            {
                                val->val1 |= (1 << ti);
                            }
                        }
                        val->val2 = 0;
                        return 0;
                    case SENSOR_CHAN_DISTANCE:
                    {
                        int target = CLAMP(val->val1, 0, RD03D_MAX_TARGETS - 1);
                        val->val1  = data->targets[target].distance;
                        val->val2  = 0;
                        return 0;
                    }
                    default: return -ENOTSUP;
                }

                switch (rdchan)
                {
                    case SENSOR_CHAN_RD03D_POS:
                        ti        = CLAMP((attr - SENSOR_ATTR_RD03D_TARGET_0), 0, RD03D_MAX_TARGETS - 1);
                        val->val1 = data->targets[ti].x;
                        val->val2 = data->targets[ti].y;
                        break;
                    case SENSOR_CHAN_RD03D_SPEED:
                        ti        = CLAMP((attr - SENSOR_ATTR_RD03D_TARGET_0), 0, RD03D_MAX_TARGETS - 1);
                        val->val1 = data->targets[ti].speed;
                        val->val2 = 0;
                        break;
                    case SENSOR_CHAN_RD03D_DISTANCE:
                        ti        = CLAMP((attr - SENSOR_ATTR_RD03D_TARGET_0), 0, RD03D_MAX_TARGETS - 1);
                        val->val1 = data->targets[ti].distance;
                        val->val2 = 0;
                        break;

                    case SENSOR_CHAN_RD03D_CONFIG_DISTANCE:
                        switch (rdattr)
                        {
                            case SENSOR_ATTR_RD03D_CONFIG_MINIMUM: ret = rd03d_get_attribute(dev, RD03D_CMD_IDX_GET_MIN_DISTANCE, &val->val1); break;
                            case SENSOR_ATTR_RD03D_CONFIG_MAXIMUM: ret = rd03d_get_attribute(dev, RD03D_CMD_IDX_GET_MAX_DISTANCE, &val->val1); break;
                            default: ret = -ENOTSUP; break;
                        }
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_FRAMES:
                        switch (rdattr)
                        {
                            case SENSOR_ATTR_RD03D_CONFIG_MINIMUM: ret = rd03d_get_attribute(dev, RD03D_CMD_IDX_GET_MIN_FRAMES, &val->val1); break;
                            case SENSOR_ATTR_RD03D_CONFIG_MAXIMUM: ret = rd03d_get_attribute(dev, RD03D_CMD_IDX_GET_MAX_FRAMES, &val->val1); break;
                            default: ret = -ENOTSUP; break;
                        }
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_DELAY_TIME: ret = rd03d_get_attribute(dev, RD03D_CMD_IDX_GET_DELAY_TIME, &val->val1); break;
                    case SENSOR_CHAN_RD03D_CONFIG_DETECTION_MODE:
                        if (data->detection_mode == RD03D_DETECTION_MODE_SINGLE_TARGET)
                        {
                            val->val1 = RD03D_DETECTION_MODE_SINGLE_TARGET;
                        }
                        else
                        {
                            val->val1 = RD03D_DETECTION_MODE_MULTI_TARGET;
                        }
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE:
                        switch (data->operation_mode & ~RD03D_OPERATION_MODE_CMD)
                        {
                            case RD03D_OPERATION_MODE_DEBUG: val->val1 = RD03D_OPERATION_MODE_DEBUG; break;
                            case RD03D_OPERATION_MODE_REPORT: val->val1 = RD03D_OPERATION_MODE_REPORT; break;
                            case RD03D_OPERATION_MODE_RUN: val->val1 = RD03D_OPERATION_MODE_RUN; break;
                            default: ret = -ENOTSUP; break;
                        }
                        if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == RD03D_OPERATION_MODE_CMD)
                        {
                            val->val2 = RD03D_OPERATION_MODE_CMD;
                        }
                        break;

                    default: return -ENOTSUP;
                }

                return ret;
            }

            static int rd03d_channel_get(const device_t *dev, enum sensor_channel chan, struct sensor_value *val)
            {
                rd03d_data_t *data = dev->data;

                int ret;

                const int                 target = CLAMP(val->val1, 0, RD03D_MAX_TARGETS - 1);
                enum sensor_channel_rd03d rdchan = (enum sensor_channel_rd03d)chan;

                switch (chan)
                {
                    case SENSOR_CHAN_PROX:
                        val->val1 = 0;
                        for (int ti = 0; ti < RD03D_MAX_TARGETS; ti++)
                        {
                            if (data->targets[ti].x != 0 && data->targets[ti].y != 0)
                            {
                                val->val1 |= (1 << ti);
                            }
                        }
                        val->val2 = 0;
                        return 0;
                    case SENSOR_CHAN_DISTANCE:
                        val->val1 = data->targets[target].distance;
                        val->val2 = 0;
                        return 0;
                    default: return -ENOTSUP;
                }

                switch (rdchan)
                {
                    case SENSOR_CHAN_RD03D_POS:
                        val->val1 = data->targets[target].x;
                        val->val2 = data->targets[target].y;
                        break;
                    case SENSOR_CHAN_RD03D_SPEED: val->val1 = data->targets[target].speed; break;
                    case SENSOR_CHAN_RD03D_DISTANCE: val->val1 = data->targets[target].distance; break;

                    case SENSOR_CHAN_RD03D_CONFIG_DISTANCE:
                        // TODO: get attribute, min/max
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_FRAMES:
                        // TODO: get attribute, min/max
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_DELAY_TIME:
                        // TODO: get attribute
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_DETECTION_MODE:
                        if (data->detection_mode == RD03D_DETECTION_MODE_SINGLE_TARGET)
                        {
                            val->val1 = RD03D_DETECTION_MODE_SINGLE_TARGET;
                        }
                        else
                        {
                            val->val1 = RD03D_DETECTION_MODE_MULTI_TARGET;
                        }
                        val->val2 = 0;
                        break;
                    case SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE:
                        switch (data->operation_mode & ~RD03D_OPERATION_MODE_CMD)
                        {
                            case RD03D_OPERATION_MODE_DEBUG: val->val1 = RD03D_OPERATION_MODE_DEBUG; break;
                            case RD03D_OPERATION_MODE_REPORT: val->val1 = RD03D_OPERATION_MODE_REPORT; break;
                            case RD03D_OPERATION_MODE_RUN: val->val1 = RD03D_OPERATION_MODE_RUN; break;
                        }
                        if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == RD03D_OPERATION_MODE_CMD)
                        {
                            val->val2 = RD03D_OPERATION_MODE_CMD;
                        }
                        break;
                    default: LOG_ERR("Unsupported channel %d", chan); return -ENOTSUP;
                }

                return ret;
            }

            static int rd03d_sample_fetch(const device_t *dev, enum sensor_channel chan)
            {
                // When in 'reporting' mode, the sensor will send 'reports' continuously
                // and data will become available in the RX buffer.
                rd03d_data_t      *data = dev->data;
                const rd03d_cfg_t *cfg  = dev->config;

                int ret;

                // We decode the rx buffer into data->targets
                if (data->operation_mode == RD03D_OPERATION_MODE_REPORT)
                {
                    if (rd03d_rx_frame(dev))
                    {
                        u8 const *rx = data->rx_data;

                        // TODO actually the verification has already been done when receiving, we
                        //      should not really have to verify again!
#    ifdef RD03D_DEBUG
                        const int report_len = 8 * RD03D_MAX_TARGETS;
                        if (data->rx_data_len != sizeof(RD03D_REPORT_HEAD) + report_len + sizeof(RD03D_REPORT_TAIL))
                        {
                            LOG_ERR("Invalid report frame, data frame length mismatch");
                            return -EINVAL;
                        }

                        if (rx[0] != RD03D_REPORT_HEAD[0] || rx[1] != RD03D_REPORT_HEAD[1] || rx[2] != RD03D_REPORT_HEAD[2] || rx[3] != RD03D_REPORT_HEAD[3])
                        {
                            LOG_ERR("Invalid report frame, head mismatch");
                            return -EINVAL;
                        }

                        if (rx[28] != RD03D_REPORT_TAIL[0] || rx[29] != RD03D_REPORT_TAIL[1])
                        {
                            LOG_ERR("Invalid report frame, tail mismatch");
                            return -EINVAL;
                        }
#    endif
                        // Decode the response
                        // RD03D_REPORT_HEAD
                        //   Target 1 { x, y, speed, distance }
                        //   Target 2 { x, y, speed, distance }
                        //   Target 3 { x, y, speed, distance }
                        // RD03D_REPORT_TAIL

                        // For decoding targets, assume each target occupies 8 bytes and parsing them
                        // sequentially. Here we first skip the header and move through the rx buffer.
                        int ti = 0;
                        for (int i = 4; i < (data->rx_bytes - 2) && ti < RD03D_MAX_TARGETS; i += 8)
                        {
                            data->targets[ti].x        = (int16_t)(rx[i] | (rx[i + 1] << 8)) - 0x200;
                            data->targets[ti].y        = (int16_t)(rx[i + 2] | (rx[i + 3] << 8)) - 0x8000;
                            data->targets[ti].speed    = (int16_t)(rx[i + 4] | (rx[i + 5] << 8)) - 0x10;
                            data->targets[ti].distance = (uint16_t)(rx[i + 6] | (rx[i + 7] << 8));
                        }
                    }
                }

                return -ENOTSUP;
            }

            static void rd03d_tx_data(const device_t *dev)
            {
                rd03d_data_t* data = dev->data;
                data->rx_bytes = 0;

                while (true)
                {
                    data->tx_bytes += dev->serial->write(&data->tx_data[data->tx_bytes], data->tx_data_len - data->tx_bytes);
                    if (data->tx_bytes == data->tx_data_len)
                    {
                        data->tx_bytes = 0;
                        break;
                    }
                }
            }

            static bool rd03d_rx_frame(const device_t *dev)
            {
                rd03d_data_t* data = dev->data;
                data->rx_bytes = 0;

                while (dev->serial->available())
                {
                    u8  *rxb    = &data->rx_data[data->rx_frame_start];
                    const int byreq  = RD03D_RX_BUF_MAX_LEN - (data->rx_frame_start + data->rx_bytes); /* Avoid buffer overrun */
                    const int byread = dev->serial->readBytes(&rxb[data->rx_bytes], byreq);
                    data->rx_bytes += byread;

                    /* The minimum data frame length is 14 bytes, and the maximum
                       data frame length is a report which is 30 bytes.
                       Our receive buffer has a size of 64 bytes, so we should be
                       able to receive a full data frame within the buffer, if
                       not then something is incorrect regarding the protocol.

                       The command + ACK protocol should not pose any out-of-sync
                       issues, as the ACK always follows the CMD.

                       The report stream is a bit more tricky, as the sensor
                       continuously sends frame data, and we need to be able to
                       detect the start of a new report frame.
                       We might start receiving a report frame in the middle
                       of a report frame, which we should ignore and continue
                       to receive until we find the start of a new report frame.
                    */

                determine_rx_data_len:
                    if (data->rx_data_len == 0 && (data->rx_bytes >= (4 + 2 + 4 + 4)))
                    {
                        const u8 h1 = rxb[0];
                        const u8 h2 = rxb[1];
                        const u8 h3 = rxb[2];
                        const u8 h4 = rxb[3];
                        if (h1 == 0xFD && h2 == 0xFC && h3 == 0xFB && h4 == 0xFA)
                        {
                            data->rx_data_len = 4 + 2 + rxb[4] + 4;
                        }
                        else if (h1 == 0xAA && h2 == 0xFF && h3 == 0x03 && h4 == 0x00)
                        {
                            data->rx_data_len = 30;
                        }
                        else
                        {
                            // Scan rx-buffer until 'FD FC FB FA' or 'AA FF 03 00'
                            int i;
                            for (i = 1; i <= data->rx_bytes - 4; i++)
                            {
                                const u8 h1 = rxb[i + 0];
                                const u8 h2 = rxb[i + 1];
                                const u8 h3 = rxb[i + 2];
                                const u8 h4 = rxb[i + 3];
                                if (h1 == 0xFD && h2 == 0xFC && h3 == 0xFB && h4 == 0xFA)
                                {
                                    data->rx_bytes -= i;
                                    data->rx_frame_start = i;
                                    rxb                  = &data->rx_data[i];
                                    goto determine_rx_data_len;
                                    break;
                                }
                                else if (h1 == 0xAA && h2 == 0xFF && h3 == 0x03 && h4 == 0x00)
                                {
                                    data->rx_bytes -= i;
                                    data->rx_frame_start = i;
                                    rxb                  = &data->rx_data[i];
                                    goto determine_rx_data_len;
                                    break;
                                }
                            }

                            // Scanning the rx buffer did not yield a valid frame start.
                            // So we can reset the rx buffer and continue reading data
                            // from the uart fifo.
                            data->rx_frame_start = 0;

                            // The last 4 bytes did not match any of the headers, so copy
                            // the last 3 bytes to the start of the buffer, and continue
                            // scanning from there.
                            data->rx_bytes = 3;
                            rxb[0]         = rxb[data->rx_bytes - 3];
                            rxb[1]         = rxb[data->rx_bytes - 2];
                            rxb[2]         = rxb[data->rx_bytes - 1];
                        }
                    }

                    /* when we have identified the frame data length, we know we have a */
                    /* data frame when we have read enough to fullfill the length*/
                    if (data->rx_bytes >= data->rx_data_len)
                    {
                        data->rx_bytes = 0;
                        return true;
                    }
                }
                return false;
            }

            static int rd03d_init(const device_t *dev)
            {
                rd03d_data_t      *data = dev->data;
                const rd03d_cfg_t *cfg  = dev->config;

                int ret = 0;

                data->tx_bytes    = 0;
                data->tx_data_len = 0;
                memset(data->tx_data, 0, RD03D_TX_BUF_MAX_LEN);
                data->tx_data[0] = RD03D_CMD_HEAD[0];
                data->tx_data[1] = RD03D_CMD_HEAD[1];
                data->tx_data[2] = RD03D_CMD_HEAD[2];
                data->tx_data[3] = RD03D_CMD_HEAD[3];

                data->rx_bytes       = 0;
                data->rx_frame_start = 0;
                data->rx_data_len    = 0;
                memset(data->rx_data, 0, RD03D_RX_BUF_MAX_LEN);

                /* Default operation mode when turning on the device */
                data->operation_mode = RD03D_OPERATION_MODE_REPORT;
                data->detection_mode = RD03D_DETECTION_MODE_MULTI_TARGET;

                for (int i = 0; i < RD03D_MAX_TARGETS; i++)
                {
                    data->targets[i].x        = 0;
                    data->targets[i].y        = 0;
                    data->targets[i].distance = 0;
                    data->targets[i].speed    = 0;
                }

                rd03d_uart_flush(dev);

                // Set configured attributes
                if (cfg->min_distance != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MIN_DISTANCE, cfg->min_distance);
                }
                if (cfg->max_distance != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MAX_DISTANCE, cfg->max_distance);
                }
                if (cfg->min_frames != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MIN_FRAMES, cfg->min_frames);
                }
                if (cfg->max_frames != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MAX_FRAMES, cfg->max_frames);
                }
                if (cfg->delay_time != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_DELAY_TIME, cfg->delay_time);
                }

                /* Activate report mode */
                ret = rd03d_set_attribute(dev, SENSOR_CHAN_RD03D_CONFIG_OPERATION_MODE, SENSOR_ATTR_RD03D_CONFIG_VALUE);

                return ret;
            }

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#endif  // 0

#ifdef TARGET_ESP32

namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            class module_t
            {
            public:
                module_t();
                void begin(u8 rxPin, u8 txPin, nbaud::Enum baud);
                bool update();

                target_t m_target[3];
                bool     m_detected[3];
                u8       m_buffer[30];
                u32      m_index;

                bool parseData(const u8 *buffer, u32 len);
            };

            enum EState
            {
                WAIT_AA,
                WAIT_FF,
                WAIT_03,
                WAIT_00,
                RECEIVE_FRAME
            };

            module_t::module_t() {}

            void module_t::begin(u8 rxPin, u8 txPin, nbaud::Enum baud) { nserial1::begin(baud, nconfig::MODE_8N1, rxPin, txPin); }

            // Parser state-machine for UART data
            bool module_t::update()
            {
                bool   data_updated = false;
                EState state        = WAIT_AA;

                m_index = 0;

                while (nserial1::available())
                {
                    byte byteIn;
                    nserial1::read_bytes(&byteIn, 1);

                    switch (state)
                    {
                        case WAIT_AA:
                            if (byteIn == 0xAA)
                                state = WAIT_FF;
                            break;

                        case WAIT_FF:
                            if (byteIn == 0xFF)
                                state = WAIT_03;
                            else
                                state = WAIT_AA;
                            break;

                        case WAIT_03:
                            if (byteIn == 0x03)
                                state = WAIT_00;
                            else
                                state = WAIT_AA;
                            break;

                        case WAIT_00:
                            if (byteIn == 0x00)
                            {
                                m_index = 0;
                                state   = RECEIVE_FRAME;
                            }
                            else
                                state = WAIT_AA;
                            break;

                        case RECEIVE_FRAME:
                            m_buffer[m_index++] = byteIn;
                            if (m_index >= 26)
                            {  // 24 bytes data + 2 tail bytes
                                if (m_buffer[24] == 0x55 && m_buffer[25] == 0xCC)
                                {
                                    data_updated = parseData(m_buffer, 24);
                                }
                                state   = WAIT_AA;
                                m_index = 0;
                            }
                            break;
                    }
                }
                return data_updated;
            }

            bool module_t::parseData(const u8 *buf, u32 len)
            {
                if (len != 24)
                    return false;

                bool any_target_detected = false;
                for (s16 i = 0; i < 3; i++, buf += 8)
                {
                    const int16_t  raw_x = (int16_t)buf[0] | ((int16_t)buf[1] << 8);  // x coordinate
                    const int16_t  raw_y = (int16_t)buf[2] | ((int16_t)buf[3] << 8);  // y coordinate
                    const int16_t  raw_v = (int16_t)buf[4] | ((int16_t)buf[5] << 8);  // v speed
                    const uint16_t raw_s = (int16_t)buf[6] | ((int16_t)buf[7] << 8);  // s distance

                    m_detected[i] = !(raw_x == 0 && raw_y == 0 && raw_v == 0 && raw_s == 0);
                    if (m_detected[i])
                    {
                        target_t &t = m_target[i];
                        t.x         = ((raw_x & 0x8000) ? 1 : -1) * (raw_x & 0x7FFF);
                        t.y         = ((raw_y & 0x8000) ? 1 : -1) * (raw_y & 0x7FFF);
                        t.v         = ((raw_v & 0x8000) ? 1 : -1) * (raw_v & 0x7FFF);
                        t.s         = sqrt(t.x * t.x + t.y * t.y);

                        any_target_detected = true;
                    }
                    else
                    {
                        target_t &t = m_target[i];
                        t.x         = 0;
                        t.y         = 0;
                        t.v         = 0;
                        t.s         = 0;
                    }
                }

                return any_target_detected;
            }

            module_t gModule;
            void     begin(u8 rxPin, u8 txPin) { gModule.begin(rxPin, txPin, nbaud::Rate256000); }
            bool     update() { return gModule.update(); }
            bool     getTarget(s8 i, target_t &t)
            {
                if (i < 0 || i >= 3)
                    return false;
                t = gModule.m_target[i];
                return gModule.m_detected[i];
            }

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#else
namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            void     begin(u8 rxPin, u8 txPin) {}
            bool     update() { return false; }
            target_t getTarget()
            {
                target_t t;
                t.s = 0;
                t.v = 0;
                t.x = 0;
                t.y = 0;
                return t;
            }

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore

#endif
