//
// Created by clemens on 10.01.23.
//

#include "nmea_gps_interface.h"

using namespace std::chrono;

void xbot::driver::gps::NmeaGpsInterface::reset_parser_state() {

}

size_t xbot::driver::gps::NmeaGpsInterface::parse_rx_buffer() {
    size_t size = rx_buffer_.size();
    if(size > 0) {
        for(size_t i = 0; i < size; i++) {
            try {
                parser.readByte(rx_buffer_[i]);
            } catch (nmea::NMEAParseError &e) {
                log(std::string("NMEA parse exception. message: ")+e.message + ", sentence: " + e.nmea.text, WARN);
            } catch (std::exception &e) {
                log(std::string("NMEA parse exception: ")+e.what(), WARN);
            }
        }
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + size);
    }
    return 1;
}

xbot::driver::gps::NmeaGpsInterface::NmeaGpsInterface(bool verboseLogging) : GpsInterface(), gps(parser) {
    //enable/disable NMEA parser logging
    parser.logWarn = true;
    parser.logInfo = verboseLogging;
    
    gps.onUpdate += [this]() {
        auto &fix = this->gps.fix;

        if(fix.type != 2 && fix.type != 3) {
            gps_state_valid_ = false;
            log(std::string("invalid gnssFix - dropping message. fix was: ") + std::to_string(fix.type), WARN_THROTTLE);
            return;
        }

        if(fix.quality == 0) {
            gps_state_valid_ = false;
            log("invalid lat, lon, height - dropping message", WARN);
            return;
        }

        switch (fix.type) {
            case 2:
                gps_state_.fix_type = GpsState::FixType::FIX_2D;
                break;
            case 3:
                gps_state_.fix_type = GpsState::FixType::FIX_3D;
                break;
            default:
                gps_state_.fix_type = GpsState::FixType::NO_FIX;
                break;
        }

        switch(fix.quality) {
            case 5:
                gps_state_.rtk_type = GpsState::RTK_FLOAT;
                break;
            case 4:
                gps_state_.rtk_type = GpsState::RTK_FIX;
                break;
            default:
                gps_state_.rtk_type = GpsState::RTK_NONE;
                break;
        }

        double e, n;
        std::string zone;
        RobotLocalization::NavsatConversions::LLtoUTM(fix.latitude, fix.longitude, n, e, zone);
        gps_state_.pos_lat = fix.latitude;
        gps_state_.pos_lon = fix.longitude;
        gps_state_.position_valid = true;
        gps_state_.pos_e = mode_ == GpsInterface::ABSOLUTE ? e - datum_e_ : e;
        gps_state_.pos_n = mode_ == GpsInterface::ABSOLUTE ? n - datum_n_ : n;
        gps_state_.pos_u = mode_ == GpsInterface::ABSOLUTE ? fix.altitude - datum_u_ : fix.altitude;

        gps_state_.position_accuracy = fix.horizontalAccuracy();

        // speed in m/s
        double speed = fix.speed / 3.6;
        double angle_rad = fix.travelAngle * M_PI / 180.0;
        gps_state_.vel_e = sin(angle_rad) * speed;
        gps_state_.vel_n = cos(angle_rad) * speed;
        gps_state_.vel_u = 0;

        gps_state_.motion_heading = angle_rad;
        gps_state_.motion_heading_valid = true;
        gps_state_.vehicle_heading_valid = false;

        gps_state_.position_time_ms = fix.positionTimestamp.getTimeMilliseconds();
        gps_state_.speed_time_ms = fix.speedTimestamp.getTimeMilliseconds();
        gps_state_.deviation_time_ms = fix.deviationTimestamp.getTimeMilliseconds();
        gps_state_.received_time = fix.positionTimestamp.getTime();

        gps_state_valid_ = true;

        if (state_callback)
            state_callback(gps_state_);
    };
}
