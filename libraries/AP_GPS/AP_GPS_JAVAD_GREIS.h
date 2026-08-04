/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  Javad GREIS GPS driver
 */
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_JAVAD_GREIS_ENABLED

class AP_GPS_JAVAD_GREIS : public AP_GPS_Backend
{
public:
    AP_GPS_JAVAD_GREIS(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read() override;

    AP_GPS_FixType highest_supported_status(void) override { return AP_GPS_FixType::RTK_FIXED; }

    bool is_configured(void) const override;

    void broadcast_configuration_failure_reason(void) const override;

    const char *name() const override { return "JAVAD_GREIS"; }

private:
    enum class Parse_State : uint8_t {
        ID1,
        ID2,
        LEN1,
        LEN2,
        LEN3,
        BODY,
    };

    enum class Message : uint8_t {
        Unknown,
        GT,
        ST,
        RT,
        ET,
        PG,
        VG,
        PV,
        SG,
        DP,
        PS,
        AR,
        RE,
        ER,
    };

    enum class Config_State : uint8_t {
        Disabled,
        DisableOutput,
        EnableStream,
        Complete,
    };

    struct Message_Info {
        Message id;
        uint16_t minimum_payload_length;
        uint8_t checksum_length;
    };

    struct Time_Data {
        bool valid;
        uint16_t week;
        uint32_t tow_ms;
    };

    struct Epoch_Data {
        bool valid;
        bool published;
        uint32_t rt_key_ms;
        uint32_t st_key_ms;
        uint32_t et_key_ms;

        bool have_st;
        bool have_rt;
        bool have_et;
        bool have_gt;
        bool have_position;
        bool have_velocity;
        bool have_pv;
        bool have_pv_geodetic;
        bool have_sg;
        bool have_dp;
        bool have_ps;

        uint16_t week;
        uint32_t tow_ms;

        double latitude_rad;
        double longitude_rad;
        double ellipsoid_height_m;
        float position_sigma_m;
        Vector3f velocity;
        float velocity_sigma_m;

        Vector3d pv_ecef_position;
        Vector3f pv_ecef_velocity;
        double pv_latitude_rad;
        double pv_longitude_rad;
        double pv_ellipsoid_height_m;
        Vector3f pv_velocity_ned;
        float pv_position_sigma_m;
        float pv_velocity_sigma_m;

        float horizontal_accuracy_m;
        float vertical_accuracy_m;
        float speed_accuracy_m_s;
        uint16_t hdop;
        uint16_t vdop;
        uint8_t num_sats;

        uint8_t st_sol_type;
        uint8_t position_sol_type;
        uint8_t velocity_sol_type;
        uint8_t pv_sol_type;
        uint8_t sg_sol_type;
        uint8_t dp_sol_type;
        uint8_t ps_sol_type;
    };

    struct Rotation_Data {
        bool valid;
        uint32_t time_ms;
        float pitch_rad;
        float roll_rad;
        float heading_rad;
        float pitch_rms_rad;
        float roll_rms_rad;
        float heading_rms_rad;
        uint8_t pitch_sol_type;
        uint8_t roll_sol_type;
        uint8_t heading_sol_type;
        uint8_t flags;
    };

    bool parse(uint8_t byte);
    bool dispatch_message();
    void reset_parser();
    void restart_parser_with(uint8_t byte);

    static bool is_message_id_char(uint8_t byte);
    static int8_t ascii_hex_value(uint8_t byte);
    static uint8_t checksum_rotate(uint8_t value);
    void checksum_start();
    void checksum_add(uint8_t byte);
    bool validate_checksum() const;

    Message_Info message_info(const char id[2]) const;

    bool decode_gt();
    bool decode_rt();
    bool decode_st();
    bool decode_et();
    bool decode_pg();
    bool decode_vg();
    bool decode_pv();
    bool decode_sg();
    bool decode_dp();
    bool decode_ps();
    bool decode_ar();
    void decode_reply(bool error_reply);

    void reset_epoch();
    bool epoch_accepts_data() const;
    void mark_seen(uint32_t bit);
    bool required_messages_seen() const;
    bool epoch_complete() const;
    bool publish_epoch();

    void update_config();
    bool send_command(const char *command);

    uint16_t read_u2(uint16_t offset) const;
    uint32_t read_u4(uint16_t offset) const;
    float read_float(uint16_t offset) const;
    double read_double(uint16_t offset) const;
    static AP_GPS_FixType sol_type_to_status(uint8_t sol_type);
    static uint16_t dop_to_scaled_uint16(float dop);
    static bool ecef_velocity_to_ned(const Vector3f &ecef_velocity,
                                     double latitude_rad,
                                     double longitude_rad,
                                     Vector3f &ned_velocity);

    static constexpr uint16_t GREIS_MAX_PAYLOAD = 0x0FFFU;
    static constexpr uint16_t PAYLOAD_BUFFER_LEN = 128U;

    static constexpr uint16_t GT_MIN_PAYLOAD = 8U;
    static constexpr uint16_t ST_MIN_PAYLOAD = 6U;
    static constexpr uint16_t RT_MIN_PAYLOAD = 5U;
    static constexpr uint16_t ET_MIN_PAYLOAD = 5U;
    static constexpr uint16_t PG_MIN_PAYLOAD = 30U;
    static constexpr uint16_t VG_MIN_PAYLOAD = 18U;
    static constexpr uint16_t PV_MIN_PAYLOAD = 46U;
    static constexpr uint16_t SG_MIN_PAYLOAD = 18U;
    static constexpr uint16_t DP_MIN_PAYLOAD = 18U;
    static constexpr uint16_t PS_MIN_PAYLOAD = 9U;
    static constexpr uint16_t AR_MIN_PAYLOAD = 33U;

    static constexpr uint32_t CONFIG_COMMAND_INTERVAL_MS = 3000U;
    static constexpr uint8_t PS_KNOWN_EXTRA_SYSTEMS = 5U;

    enum Seen_Message : uint32_t {
        SEEN_GT = (1U << 0),
        SEEN_RT = (1U << 1),
        SEEN_ST = (1U << 2),
        SEEN_ET = (1U << 3),
        SEEN_PG = (1U << 4),
        SEEN_VG = (1U << 5),
        SEEN_PV = (1U << 6),
        SEEN_PS = (1U << 7),
        SEEN_SG = (1U << 8),
        SEEN_DP = (1U << 9),
        SEEN_AR = (1U << 10),
    };

    Parse_State _parse_state;
    Message _message;
    Config_State _config_state;

    char _id[2];
    char _length_ascii[3];
    uint16_t _payload_length;
    uint16_t _payload_read;
    uint16_t _payload_stored;
    uint8_t _payload[PAYLOAD_BUFFER_LEN];
    uint8_t _checksum_res;
    uint8_t _checksum_bytes[2];
    uint8_t _checksum_length;
    uint16_t _minimum_payload_length;
    uint16_t _last_message_length;

    uint32_t _seen_messages;
    uint32_t _next_config_ms;
    uint32_t _reply_error_count;

    Time_Data _pending_gt;
    Epoch_Data _epoch;
    Rotation_Data _last_ar;
};

#endif  // AP_GPS_JAVAD_GREIS_ENABLED
