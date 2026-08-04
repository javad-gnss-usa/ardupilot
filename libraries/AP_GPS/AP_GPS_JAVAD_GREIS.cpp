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

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_GPS.h"
#include "AP_GPS_JAVAD_GREIS.h"

#if AP_GPS_JAVAD_GREIS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <math.h>
#include <string.h>

#define JAVAD_GREIS_DEBUGGING 0

#if JAVAD_GREIS_DEBUGGING
 #define Debug(fmt, args ...) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JAVAD GREIS: " fmt, ## args)
#else
 #define Debug(fmt, args ...)
#endif

AP_GPS_JAVAD_GREIS::AP_GPS_JAVAD_GREIS(AP_GPS &_gps,
                                       AP_GPS::Params &_params,
                                       AP_GPS::GPS_State &_state,
                                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _params, _state, _port)
{
    reset_parser();
    reset_epoch();

    _seen_messages = 0;
    _reply_error_count = 0;
    _pending_gt.valid = false;
    _last_ar.valid = false;

    if (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        _config_state = Config_State::Disabled;
    } else {
        _config_state = Config_State::DisableOutput;
    }
    _next_config_ms = AP_HAL::millis() + 200U;

    // GREIS PG altitude is ellipsoidal. Do not claim an AMSL
    // height or undulation until geoid support is added.
    state.have_undulation = false;
    state.undulation = 0.0f;
}

bool AP_GPS_JAVAD_GREIS::read()
{
    update_config();

    bool ret = false;
    if (port == nullptr) {
        return false;
    }

    const uint32_t available_bytes = port->available();
    for (uint32_t i = 0; i < available_bytes; i++) {
        const uint8_t temp = port->read();
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&temp, 1);
#endif
        ret |= parse(temp);
    }

    update_config();

    return ret;
}

bool AP_GPS_JAVAD_GREIS::is_configured(void) const
{
    return (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) ||
           (_config_state == Config_State::Complete);
}

void AP_GPS_JAVAD_GREIS::broadcast_configuration_failure_reason(void) const
{
    if (gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE &&
        _config_state != Config_State::Complete) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %u: JAVAD GREIS not configured state=%u seen=0x%08x errors=%u",
                      (unsigned int)(state.instance + 1),
                      (unsigned int)_config_state,
                      (unsigned int)_seen_messages,
                      (unsigned int)_reply_error_count);
    }
}

void AP_GPS_JAVAD_GREIS::reset_parser()
{
    _parse_state = Parse_State::ID1;
    _message = Message::Unknown;
    _id[0] = 0;
    _id[1] = 0;
    _length_ascii[0] = 0;
    _length_ascii[1] = 0;
    _length_ascii[2] = 0;
    _payload_length = 0;
    _payload_read = 0;
    _payload_stored = 0;
    _checksum_res = 0;
    _checksum_bytes[0] = 0;
    _checksum_bytes[1] = 0;
    _checksum_length = 0;
    _minimum_payload_length = 0;
    _last_message_length = 0;
}

void AP_GPS_JAVAD_GREIS::restart_parser_with(uint8_t byte)
{
    reset_parser();
    if (byte == '\r' || byte == '\n') {
        return;
    }
    if (is_message_id_char(byte)) {
        _id[0] = (char)byte;
        _parse_state = Parse_State::ID2;
    }
}

bool AP_GPS_JAVAD_GREIS::parse(uint8_t byte)
{
    switch (_parse_state) {
    case Parse_State::ID1:
        if (byte == '\r' || byte == '\n') {
            return false;
        }
        if (is_message_id_char(byte)) {
            _id[0] = (char)byte;
            _parse_state = Parse_State::ID2;
        }
        return false;

    case Parse_State::ID2:
        if (is_message_id_char(byte)) {
            _id[1] = (char)byte;
            _parse_state = Parse_State::LEN1;
        } else {
            restart_parser_with(byte);
        }
        return false;

    case Parse_State::LEN1: {
        const int8_t value = ascii_hex_value(byte);
        if (value < 0) {
            restart_parser_with(byte);
            return false;
        }
        _length_ascii[0] = (char)byte;
        _payload_length = (uint16_t)value << 8U;
        _parse_state = Parse_State::LEN2;
        return false;
    }

    case Parse_State::LEN2: {
        const int8_t value = ascii_hex_value(byte);
        if (value < 0) {
            restart_parser_with(byte);
            return false;
        }
        _length_ascii[1] = (char)byte;
        _payload_length |= (uint16_t)value << 4U;
        _parse_state = Parse_State::LEN3;
        return false;
    }

    case Parse_State::LEN3: {
        const int8_t value = ascii_hex_value(byte);
        if (value < 0) {
            restart_parser_with(byte);
            return false;
        }
        _length_ascii[2] = (char)byte;
        _payload_length |= (uint16_t)value;
        if (_payload_length > GREIS_MAX_PAYLOAD) {
            restart_parser_with(byte);
            return false;
        }

        const Message_Info info = message_info(_id);
        _message = info.id;
        _minimum_payload_length = info.minimum_payload_length;
        _checksum_length = info.checksum_length;
        _payload_read = 0;
        _payload_stored = 0;
        _checksum_bytes[0] = 0;
        _checksum_bytes[1] = 0;
        checksum_start();

        if (_payload_length == 0) {
            bool ret = false;
            if (_message != Message::Unknown) {
                ret = dispatch_message();
            }
            reset_parser();
            return ret;
        }

        _parse_state = Parse_State::BODY;
        return false;
    }

    case Parse_State::BODY: {
        if (_message != Message::Unknown) {
            if (_payload_stored < PAYLOAD_BUFFER_LEN) {
                _payload[_payload_stored++] = byte;
            }

            const uint16_t covered_length = (_payload_length >= _checksum_length) ?
                                            (_payload_length - _checksum_length) : 0U;
            if (_checksum_length != 0) {
                if (_payload_read < covered_length) {
                    checksum_add(byte);
                } else {
                    const uint16_t checksum_offset = _payload_read - covered_length;
                    if (checksum_offset < sizeof(_checksum_bytes)) {
                        _checksum_bytes[checksum_offset] = byte;
                    }
                }
            }
        }

        _payload_read++;
        if (_payload_read >= _payload_length) {
            bool ret = false;
            if (_message != Message::Unknown) {
                ret = dispatch_message();
            }
            reset_parser();
            return ret;
        }
        return false;
    }
    }

    reset_parser();
    return false;
}

bool AP_GPS_JAVAD_GREIS::is_message_id_char(uint8_t byte)
{
    return byte >= 0x21U && byte <= 0x7eU;
}

int8_t AP_GPS_JAVAD_GREIS::ascii_hex_value(uint8_t byte)
{
    if (byte >= '0' && byte <= '9') {
        return byte - '0';
    }
    if (byte >= 'A' && byte <= 'F') {
        return byte - 'A' + 10;
    }
    return -1;
}

uint8_t AP_GPS_JAVAD_GREIS::checksum_rotate(uint8_t value)
{
    return (uint8_t)((value << 2U) | (value >> 6U));
}

void AP_GPS_JAVAD_GREIS::checksum_start()
{
    _checksum_res = 0;
    if (_message == Message::Unknown || _checksum_length == 0) {
        return;
    }

    checksum_add((uint8_t)_id[0]);
    checksum_add((uint8_t)_id[1]);
    checksum_add((uint8_t)_length_ascii[0]);
    checksum_add((uint8_t)_length_ascii[1]);
    checksum_add((uint8_t)_length_ascii[2]);
}

void AP_GPS_JAVAD_GREIS::checksum_add(uint8_t byte)
{
    _checksum_res = checksum_rotate(_checksum_res) ^ byte;
}

bool AP_GPS_JAVAD_GREIS::validate_checksum() const
{
    if (_checksum_length == 0) {
        return true;
    }
    if (_payload_length < _checksum_length) {
        return false;
    }

    const uint8_t checksum = checksum_rotate(_checksum_res);
    if (_checksum_length == 1) {
        return checksum == _checksum_bytes[0];
    }
    if (_checksum_length == 2) {
        const int8_t high = ascii_hex_value(_checksum_bytes[0]);
        const int8_t low = ascii_hex_value(_checksum_bytes[1]);
        if (high < 0 || low < 0) {
            return false;
        }
        return checksum == (uint8_t)((high << 4U) | low);
    }

    return false;
}

AP_GPS_JAVAD_GREIS::Message_Info AP_GPS_JAVAD_GREIS::message_info(const char id[2]) const
{
    if (id[0] == 'G' && id[1] == 'T') {
        return { Message::GT, GT_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'S' && id[1] == 'T') {
        return { Message::ST, ST_MIN_PAYLOAD, 1U };
    }
    // /msg/jps/RT is transmitted on the wire with the GREIS [~~] ID.
    if ((id[0] == '~' && id[1] == '~') ||
        (id[0] == 'R' && id[1] == 'T')) {
        return { Message::RT, RT_MIN_PAYLOAD, 1U };
    }
    // /msg/jps/ET is transmitted on the wire with the GREIS [::] ID.
    if ((id[0] == ':' && id[1] == ':') ||
        (id[0] == 'E' && id[1] == 'T')) {
        return { Message::ET, ET_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'P' && id[1] == 'G') {
        return { Message::PG, PG_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'V' && id[1] == 'G') {
        return { Message::VG, VG_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'P' && id[1] == 'V') {
        return { Message::PV, PV_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'S' && id[1] == 'G') {
        return { Message::SG, SG_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'D' && id[1] == 'P') {
        return { Message::DP, DP_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'P' && id[1] == 'S') {
        return { Message::PS, PS_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'A' && id[1] == 'R') {
        return { Message::AR, AR_MIN_PAYLOAD, 1U };
    }
    if (id[0] == 'R' && id[1] == 'E') {
        return { Message::RE, 0U, 0U };
    }
    if (id[0] == 'E' && id[1] == 'R') {
        return { Message::ER, 0U, 0U };
    }
    return { Message::Unknown, 0U, 0U };
}

bool AP_GPS_JAVAD_GREIS::dispatch_message()
{
    _last_message_length = 5U + _payload_length;

    if (_message == Message::Unknown) {
        return false;
    }
    if (_payload_length < _minimum_payload_length) {
        Debug("short message %c%c len=%u min=%u",
              _id[0], _id[1], (unsigned)_payload_length, (unsigned)_minimum_payload_length);
        return false;
    }
    if (_payload_stored < MIN(_payload_length, PAYLOAD_BUFFER_LEN) &&
        _minimum_payload_length > _payload_stored) {
        return false;
    }
    if (!validate_checksum()) {
        Debug("checksum failed %c%c", _id[0], _id[1]);
        return false;
    }

    bool decoded = false;
    switch (_message) {
    case Message::GT:
        decoded = decode_gt();
        break;
    case Message::ST:
        decoded = decode_st();
        break;
    case Message::RT:
        decoded = decode_rt();
        break;
    case Message::ET:
        decoded = decode_et();
        break;
    case Message::PG:
        decoded = decode_pg();
        break;
    case Message::VG:
        decoded = decode_vg();
        break;
    case Message::PV:
        decoded = decode_pv();
        break;
    case Message::SG:
        decoded = decode_sg();
        break;
    case Message::DP:
        decoded = decode_dp();
        break;
    case Message::PS:
        decoded = decode_ps();
        break;
    case Message::AR:
        decoded = decode_ar();
        break;
    case Message::RE:
        decode_reply(false);
        break;
    case Message::ER:
        decode_reply(true);
        break;
    case Message::Unknown:
        break;
    }

    if (decoded) {
        return publish_epoch();
    }

    return false;
}

bool AP_GPS_JAVAD_GREIS::decode_gt()
{
    const uint32_t tow = read_u4(0);
    const uint32_t week = read_u2(4) + ((uint32_t)_payload[6] * 1024U);

    _pending_gt.valid = true;
    _pending_gt.week = (uint16_t)week;
    _pending_gt.tow_ms = tow;

    if (_epoch.valid && _epoch.have_rt && !_epoch.have_gt) {
        _epoch.week = _pending_gt.week;
        _epoch.tow_ms = _pending_gt.tow_ms;
        _epoch.have_gt = true;
        _pending_gt.valid = false;
    }

    mark_seen(SEEN_GT);
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_rt()
{
    const uint32_t time = read_u4(0);

    reset_epoch();
    _epoch.valid = true;
    _epoch.have_rt = true;
    _epoch.rt_key_ms = time;

    if (_pending_gt.valid) {
        _epoch.week = _pending_gt.week;
        _epoch.tow_ms = _pending_gt.tow_ms;
        _epoch.have_gt = true;
        _pending_gt.valid = false;
    }

    mark_seen(SEEN_RT);
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_st()
{
    const uint32_t time = read_u4(0);
    const uint8_t sol_type = _payload[4];

    mark_seen(SEEN_ST);
    if (!_epoch.valid || !_epoch.have_rt || _epoch.have_et || _epoch.published) {
        return true;
    }

    _epoch.st_key_ms = time;

    // RT.tod and ST.time are both decoded as receiver-time milliseconds
    // modulo one day. Reject mismatches to avoid cross-epoch data mixing.
    if (time != _epoch.rt_key_ms) {
        Debug("ST time mismatch rt=%u st=%u",
              (unsigned)_epoch.rt_key_ms,
              (unsigned)time);
        return true;
    }

    _epoch.have_st = true;
    _epoch.st_sol_type = sol_type;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_et()
{
    const uint32_t time = read_u4(0);
    mark_seen(SEEN_ET);
    if (!_epoch.valid || !_epoch.have_rt || _epoch.published) {
        return true;
    }

    _epoch.et_key_ms = time;
    _epoch.have_et = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_pg()
{
    const double latitude = read_double(0);
    const double longitude = read_double(8);
    const double height = read_double(16);
    const float position_sigma = read_float(24);
    const uint8_t sol_type = _payload[28];

    if (!isfinite(latitude) || !isfinite(longitude) || !isfinite(height) ||
        latitude < -90.0 * DEG_TO_RAD_DOUBLE ||
        latitude > 90.0 * DEG_TO_RAD_DOUBLE ||
        longitude < -180.0 * DEG_TO_RAD_DOUBLE ||
        longitude > 180.0 * DEG_TO_RAD_DOUBLE) {
        return false;
    }

    mark_seen(SEEN_PG);
    if (!epoch_accepts_data()) {
        return true;
    }

    _epoch.latitude_rad = latitude;
    _epoch.longitude_rad = longitude;
    _epoch.ellipsoid_height_m = height;
    _epoch.position_sigma_m = position_sigma;
    _epoch.position_sol_type = sol_type;
    _epoch.have_position = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_vg()
{
    const float north = read_float(0);
    const float east = read_float(4);
    const float up = read_float(8);
    const float velocity_sigma = read_float(12);
    const uint8_t sol_type = _payload[16];

    if (!isfinite(north) || !isfinite(east) || !isfinite(up)) {
        return false;
    }

    mark_seen(SEEN_VG);
    if (!epoch_accepts_data()) {
        return true;
    }

    _epoch.velocity.x = north;
    _epoch.velocity.y = east;
    _epoch.velocity.z = -up;
    _epoch.velocity_sigma_m = velocity_sigma;
    _epoch.velocity_sol_type = sol_type;
    _epoch.have_velocity = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_pv()
{
    const Vector3d ecef_position(read_double(0), read_double(8), read_double(16));
    const float position_sigma = read_float(24);
    const Vector3f ecef_velocity(read_float(28), read_float(32), read_float(36));
    const float velocity_sigma = read_float(40);
    const uint8_t sol_type = _payload[44];

    if (!isfinite(ecef_position.x) ||
        !isfinite(ecef_position.y) ||
        !isfinite(ecef_position.z) ||
        !isfinite(ecef_velocity.x) ||
        !isfinite(ecef_velocity.y) ||
        !isfinite(ecef_velocity.z)) {
        return false;
    }

    mark_seen(SEEN_PV);
    if (!epoch_accepts_data()) {
        return true;
    }

    _epoch.pv_ecef_position = ecef_position;
    _epoch.pv_ecef_velocity = ecef_velocity;
    _epoch.pv_position_sigma_m = position_sigma;
    _epoch.pv_velocity_sigma_m = velocity_sigma;
    _epoch.pv_sol_type = sol_type;
    _epoch.have_pv = true;
    _epoch.have_pv_geodetic = false;

    const double radius = sqrt((ecef_position.x * ecef_position.x) +
                               (ecef_position.y * ecef_position.y) +
                               (ecef_position.z * ecef_position.z));
    if (!isfinite(radius) ||
        radius < WGS84_A * 0.5 ||
        radius > WGS84_A * 2.0) {
        return true;
    }

    Vector3d llh;
    wgsecef2llh(ecef_position, llh);
    Vector3f ned_velocity;
    if (!isfinite(llh.x) ||
        !isfinite(llh.y) ||
        !isfinite(llh.z) ||
        llh.x < -90.0 * DEG_TO_RAD_DOUBLE ||
        llh.x > 90.0 * DEG_TO_RAD_DOUBLE ||
        llh.y < -180.0 * DEG_TO_RAD_DOUBLE ||
        llh.y > 180.0 * DEG_TO_RAD_DOUBLE ||
        !ecef_velocity_to_ned(ecef_velocity, llh.x, llh.y, ned_velocity)) {
        return true;
    }

    _epoch.pv_latitude_rad = llh.x;
    _epoch.pv_longitude_rad = llh.y;
    _epoch.pv_ellipsoid_height_m = llh.z;
    _epoch.pv_velocity_ned = ned_velocity;
    _epoch.have_pv_geodetic = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_sg()
{
    const float hpos = read_float(0);
    const float vpos = read_float(4);
    const float hvel = read_float(8);
    const uint8_t sol_type = _payload[16];

    mark_seen(SEEN_SG);
    if (!epoch_accepts_data()) {
        return true;
    }
    if (!isfinite(hpos) || !isfinite(vpos) || !isfinite(hvel) ||
        hpos < 0.0f || vpos < 0.0f || hvel < 0.0f) {
        return true;
    }

    _epoch.horizontal_accuracy_m = hpos;
    _epoch.vertical_accuracy_m = vpos;
    _epoch.speed_accuracy_m_s = hvel;
    _epoch.sg_sol_type = sol_type;
    _epoch.have_sg = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_dp()
{
    const float hdop = read_float(0);
    const float vdop = read_float(4);
    const uint8_t sol_type = _payload[12];

    mark_seen(SEEN_DP);
    if (!epoch_accepts_data()) {
        return true;
    }

    const uint16_t scaled_hdop = dop_to_scaled_uint16(hdop);
    const uint16_t scaled_vdop = dop_to_scaled_uint16(vdop);
    if (scaled_hdop == GPS_UNKNOWN_DOP || scaled_vdop == GPS_UNKNOWN_DOP) {
        return true;
    }

    _epoch.hdop = scaled_hdop;
    _epoch.vdop = scaled_vdop;
    _epoch.dp_sol_type = sol_type;
    _epoch.have_dp = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_ps()
{
    const uint8_t sol_type = _payload[0];
    uint16_t num_sats = (uint16_t)_payload[5] + (uint16_t)_payload[6];

    const uint16_t stat_count = MIN((uint16_t)PS_KNOWN_EXTRA_SYSTEMS, (uint16_t)((_payload_length - PS_MIN_PAYLOAD) / 3U));
    for (uint16_t i = 0; i < stat_count; i++) {
        const uint16_t used_offset = 8U + (i * 3U) + 2U;
        if (used_offset < _payload_stored) {
            num_sats += _payload[used_offset];
        }
    }

    mark_seen(SEEN_PS);
    if (!epoch_accepts_data()) {
        return true;
    }

    _epoch.num_sats = (uint8_t)MIN(num_sats, (uint16_t)UINT8_MAX);
    _epoch.ps_sol_type = sol_type;
    _epoch.have_ps = true;
    return true;
}

bool AP_GPS_JAVAD_GREIS::decode_ar()
{
    _last_ar.valid = false;

    const uint32_t time = read_u4(0);
    const float pitch = read_float(4);
    const float roll = read_float(8);
    const float heading = read_float(12);
    const float pitch_rms = read_float(16);
    const float roll_rms = read_float(20);
    const float heading_rms = read_float(24);

    if (!isfinite(pitch) ||
        !isfinite(roll) ||
        !isfinite(heading) ||
        !isfinite(pitch_rms) ||
        !isfinite(roll_rms) ||
        !isfinite(heading_rms)) {
        return false;
    }

    _last_ar.valid = true;
    _last_ar.time_ms = time;
    _last_ar.pitch_rad = pitch;
    _last_ar.roll_rad = roll;
    _last_ar.heading_rad = heading;
    _last_ar.pitch_rms_rad = pitch_rms;
    _last_ar.roll_rms_rad = roll_rms;
    _last_ar.heading_rms_rad = heading_rms;
    _last_ar.pitch_sol_type = _payload[28];
    _last_ar.roll_sol_type = _payload[29];
    _last_ar.heading_sol_type = _payload[30];
    _last_ar.flags = _payload[31];

    // AR is intentionally not published into GPS_State in this first implementation.
    Debug("AR time=%u heading=%f heading_rms=%f flags=0x%02x",
          (unsigned)time,
          (double)heading,
          (double)heading_rms,
          (unsigned)_last_ar.flags);
    mark_seen(SEEN_AR);
    return true;
}

void AP_GPS_JAVAD_GREIS::decode_reply(bool error_reply)
{
    if (!error_reply) {
        return;
    }

    _reply_error_count++;
}

void AP_GPS_JAVAD_GREIS::reset_epoch()
{
    _epoch.valid = false;
    _epoch.published = false;
    _epoch.rt_key_ms = 0;
    _epoch.st_key_ms = 0;
    _epoch.et_key_ms = 0;
    _epoch.have_st = false;
    _epoch.have_rt = false;
    _epoch.have_et = false;
    _epoch.have_gt = false;
    _epoch.have_position = false;
    _epoch.have_velocity = false;
    _epoch.have_pv = false;
    _epoch.have_pv_geodetic = false;
    _epoch.have_sg = false;
    _epoch.have_dp = false;
    _epoch.have_ps = false;
    _epoch.week = 0;
    _epoch.tow_ms = 0;
    _epoch.latitude_rad = 0.0;
    _epoch.longitude_rad = 0.0;
    _epoch.ellipsoid_height_m = 0.0;
    _epoch.position_sigma_m = -1.0f;
    _epoch.velocity = Vector3f();
    _epoch.velocity_sigma_m = -1.0f;
    _epoch.pv_ecef_position = Vector3d();
    _epoch.pv_ecef_velocity = Vector3f();
    _epoch.pv_latitude_rad = 0.0;
    _epoch.pv_longitude_rad = 0.0;
    _epoch.pv_ellipsoid_height_m = 0.0;
    _epoch.pv_velocity_ned = Vector3f();
    _epoch.pv_position_sigma_m = -1.0f;
    _epoch.pv_velocity_sigma_m = -1.0f;
    _epoch.horizontal_accuracy_m = 0.0f;
    _epoch.vertical_accuracy_m = 0.0f;
    _epoch.speed_accuracy_m_s = 0.0f;
    _epoch.hdop = GPS_UNKNOWN_DOP;
    _epoch.vdop = GPS_UNKNOWN_DOP;
    _epoch.num_sats = 0;
    _epoch.st_sol_type = 0;
    _epoch.position_sol_type = 0;
    _epoch.velocity_sol_type = 0;
    _epoch.pv_sol_type = 0;
    _epoch.sg_sol_type = 0;
    _epoch.dp_sol_type = 0;
    _epoch.ps_sol_type = 0;
}

bool AP_GPS_JAVAD_GREIS::epoch_accepts_data() const
{
    return _epoch.valid && _epoch.have_rt && !_epoch.have_et && !_epoch.published;
}

void AP_GPS_JAVAD_GREIS::mark_seen(uint32_t bit)
{
    _seen_messages |= bit;
    if (required_messages_seen() &&
        gps._auto_config != AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        _config_state = Config_State::Complete;
    }
}

bool AP_GPS_JAVAD_GREIS::required_messages_seen() const
{
    constexpr uint32_t required = SEEN_GT | SEEN_RT | SEEN_ST | SEEN_ET | SEEN_PG | SEEN_VG;
    return (_seen_messages & required) == required;
}

bool AP_GPS_JAVAD_GREIS::epoch_complete() const
{
    const bool have_primary_nav = _epoch.have_position && _epoch.have_velocity;
    const bool have_pv_nav = _epoch.have_pv_geodetic;

    if (!_epoch.valid ||
        !_epoch.have_rt ||
        !_epoch.have_st ||
        !_epoch.have_et ||
        !_epoch.have_gt ||
        (!have_primary_nav && !have_pv_nav)) {
        return false;
    }

    return !_epoch.published;
}

bool AP_GPS_JAVAD_GREIS::publish_epoch()
{
    if (!epoch_complete()) {
        return false;
    }

    const bool use_primary_nav = _epoch.have_position && _epoch.have_velocity;
    const bool use_pv_nav = !use_primary_nav && _epoch.have_pv_geodetic;

    AP_GPS_FixType status = AP_GPS_FixType::NONE;
    bool have_status = false;
    const auto consider_status = [&status, &have_status](uint8_t sol_type) {
        const AP_GPS_FixType message_status = AP_GPS_JAVAD_GREIS::sol_type_to_status(sol_type);
        if (!have_status || (uint8_t)message_status < (uint8_t)status) {
            status = message_status;
            have_status = true;
        }
    };

    if (_epoch.have_st) {
        consider_status(_epoch.st_sol_type);
    }
    if (use_primary_nav) {
        consider_status(_epoch.position_sol_type);
        consider_status(_epoch.velocity_sol_type);
    } else if (use_pv_nav) {
        consider_status(_epoch.pv_sol_type);
    }
    if (_epoch.have_ps) {
        consider_status(_epoch.ps_sol_type);
    }
    if (_epoch.have_sg) {
        consider_status(_epoch.sg_sol_type);
    }
    if (_epoch.have_dp) {
        consider_status(_epoch.dp_sol_type);
    }

    state.status = status;
    state.time_week = _epoch.week;
    state.time_week_ms = _epoch.tow_ms;
    check_new_itow(_epoch.tow_ms, _last_message_length);
    state.last_gps_time_ms = AP_HAL::millis();

    const double latitude_rad = use_pv_nav ? _epoch.pv_latitude_rad : _epoch.latitude_rad;
    const double longitude_rad = use_pv_nav ? _epoch.pv_longitude_rad : _epoch.longitude_rad;
    const double ellipsoid_height_m = use_pv_nav ? _epoch.pv_ellipsoid_height_m : _epoch.ellipsoid_height_m;
    const float position_sigma_m = use_pv_nav ? _epoch.pv_position_sigma_m : _epoch.position_sigma_m;
    const float velocity_sigma_m = use_pv_nav ? _epoch.pv_velocity_sigma_m : _epoch.velocity_sigma_m;

    state.location.lat = (int32_t)(latitude_rad * RAD_TO_DEG_DOUBLE * (double)1e7);
    state.location.lng = (int32_t)(longitude_rad * RAD_TO_DEG_DOUBLE * (double)1e7);

    // GREIS PG/PV height is ellipsoidal. A validated geoid/AMSL
    // conversion must be added before this can be treated as AMSL.
    state.location.alt = (int32_t)(ellipsoid_height_m * 100.0);
    state.have_undulation = false;
    state.undulation = 0.0f;

    state.velocity = use_pv_nav ? _epoch.pv_velocity_ned : _epoch.velocity;
    state.have_vertical_velocity = true;
    velocity_to_speed_course(state);

    state.num_sats = _epoch.have_ps ? _epoch.num_sats : 0;
    state.rtk_num_sats = _epoch.num_sats;
    state.rtk_age_ms = 0;

    if (_epoch.have_dp) {
        state.hdop = _epoch.hdop;
        state.vdop = _epoch.vdop;
    } else {
        state.hdop = GPS_UNKNOWN_DOP;
        state.vdop = GPS_UNKNOWN_DOP;
    }

    state.have_horizontal_accuracy = false;
    state.have_vertical_accuracy = false;
    state.have_speed_accuracy = false;
    if (_epoch.have_sg) {
        state.horizontal_accuracy = _epoch.horizontal_accuracy_m;
        state.vertical_accuracy = _epoch.vertical_accuracy_m;
        state.speed_accuracy = _epoch.speed_accuracy_m_s;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.have_speed_accuracy = true;
    } else {
        if (isfinite(position_sigma_m) && position_sigma_m >= 0.0f) {
            state.horizontal_accuracy = position_sigma_m;
            state.have_horizontal_accuracy = true;
        }
        if (isfinite(velocity_sigma_m) && velocity_sigma_m >= 0.0f) {
            state.speed_accuracy = velocity_sigma_m;
            state.have_speed_accuracy = true;
        }
    }

    _epoch.published = true;
    return true;
}

void AP_GPS_JAVAD_GREIS::update_config()
{
    if (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE ||
        _config_state == Config_State::Disabled ||
        _config_state == Config_State::Complete ||
        port == nullptr) {
        return;
    }

    if (required_messages_seen()) {
        _config_state = Config_State::Complete;
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now < _next_config_ms) {
        return;
    }

    switch (_config_state) {
    case Config_State::DisableOutput:
        if (send_command("dm\n")) {
            _config_state = Config_State::EnableStream;
            _next_config_ms = now + 200U;
        }
        break;

    case Config_State::EnableStream:
        if (send_command("dm,;em,,jps/{RT,GT,ST,PG,VG,SG,DP,PS,ET}:0.1\n")) {
            _next_config_ms = now + CONFIG_COMMAND_INTERVAL_MS;
        }
        break;

    case Config_State::Disabled:
    case Config_State::Complete:
        break;
    }
}

bool AP_GPS_JAVAD_GREIS::send_command(const char *command)
{
    const size_t length = strlen(command);
    if (port == nullptr || port->txspace() < length) {
        return false;
    }

    port->write((const uint8_t *)command, length);
    return true;
}

uint16_t AP_GPS_JAVAD_GREIS::read_u2(uint16_t offset) const
{
    // Javad GREIS/JPS numeric payloads targeted by this backend are decoded
    // little-endian. Keep decoding byte-wise to avoid unaligned host casts.
    return (uint16_t)_payload[offset] |
           ((uint16_t)_payload[offset + 1U] << 8U);
}

uint32_t AP_GPS_JAVAD_GREIS::read_u4(uint16_t offset) const
{
    return (uint32_t)_payload[offset] |
           ((uint32_t)_payload[offset + 1U] << 8U) |
           ((uint32_t)_payload[offset + 2U] << 16U) |
           ((uint32_t)_payload[offset + 3U] << 24U);
}

float AP_GPS_JAVAD_GREIS::read_float(uint16_t offset) const
{
    const uint32_t bits = read_u4(offset);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

double AP_GPS_JAVAD_GREIS::read_double(uint16_t offset) const
{
    uint64_t bits = 0;
    for (uint8_t i = 0; i < 8U; i++) {
        bits = (bits << 8U) | _payload[offset + 7U - i];
    }

    double value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool AP_GPS_JAVAD_GREIS::ecef_velocity_to_ned(const Vector3f &ecef_velocity,
                                              double latitude_rad,
                                              double longitude_rad,
                                              Vector3f &ned_velocity)
{
    if (!isfinite(latitude_rad) ||
        !isfinite(longitude_rad) ||
        !isfinite(ecef_velocity.x) ||
        !isfinite(ecef_velocity.y) ||
        !isfinite(ecef_velocity.z)) {
        return false;
    }

    const double sin_lat = sin(latitude_rad);
    const double cos_lat = cos(latitude_rad);
    const double sin_lon = sin(longitude_rad);
    const double cos_lon = cos(longitude_rad);

    ned_velocity.x = (float)((-sin_lat * cos_lon * ecef_velocity.x) -
                             ( sin_lat * sin_lon * ecef_velocity.y) +
                             ( cos_lat * ecef_velocity.z));
    ned_velocity.y = (float)((-sin_lon * ecef_velocity.x) +
                             ( cos_lon * ecef_velocity.y));
    ned_velocity.z = (float)((-cos_lat * cos_lon * ecef_velocity.x) -
                             ( cos_lat * sin_lon * ecef_velocity.y) -
                             ( sin_lat * ecef_velocity.z));

    return isfinite(ned_velocity.x) &&
           isfinite(ned_velocity.y) &&
           isfinite(ned_velocity.z);
}

AP_GPS_FixType AP_GPS_JAVAD_GREIS::sol_type_to_status(uint8_t sol_type)
{
    switch (sol_type) {
    case 0:
        return AP_GPS_FixType::NONE;
    case 1:
        return AP_GPS_FixType::FIX_3D;
    case 2:
        return AP_GPS_FixType::DGPS;
    case 3:
        return AP_GPS_FixType::RTK_FLOAT;
    case 4:
        return AP_GPS_FixType::RTK_FIXED;
    case 5:
        return AP_GPS_FixType::STATIC;
    default:
        return AP_GPS_FixType::NONE;
    }
}

uint16_t AP_GPS_JAVAD_GREIS::dop_to_scaled_uint16(float dop)
{
    if (!isfinite(dop) || dop < 0.0f || dop >= ((float)GPS_UNKNOWN_DOP * 0.01f)) {
        return GPS_UNKNOWN_DOP;
    }

    return (uint16_t)(dop * 100.0f);
}

#endif  // AP_GPS_JAVAD_GREIS_ENABLED
