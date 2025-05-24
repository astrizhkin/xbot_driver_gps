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

xbot::driver::gps::NmeaGpsInterface::NmeaGpsInterface(bool verboseLogging, bool reportEveryUpdate) : GpsInterface(), gps(parser) {
    //enable/disable NMEA parser logging
    parser.logWarn = true;
    parser.logInfo = verboseLogging;
    uint64_t last_GSA_ms = 0;
    
    gps.onUpdate += [this, reportEveryUpdate, &last_GSA_ms](NMEASentence::MessageID messageId) {
        //we skip updates for messages not involed in gps status calucaltions
        if(messageId == NMEASentence::MessageID::VTG || messageId == NMEASentence::MessageID::GSV) {
            return;
        }
        auto &fix = this->gps.fix;

        uint64_t GGA_ms  = fix.GGA_epoch.getTimeMilliseconds();
        uint64_t RMC_ms  = fix.RMC_epoch.getTimeMilliseconds();
        uint64_t GST_ms  = fix.GST_epoch.getTimeMilliseconds();
        uint64_t GSA_ms  = fix.GSA_epoch.getTimeMilliseconds();
        uint64_t last_ms = fix.last_epoch.getTimeMilliseconds();

        if(messageId == NMEASentence::MessageID::GSA) {
            //we skip repating GSA messages as they are not involed in gps status calucaltions
            if(GSA_ms == last_GSA_ms) {
                return;
            }
            last_GSA_ms = GSA_ms;
        }

        //GSA/type
        if(fix.type != 2 && fix.type != 3) {
            gps_state_valid_ = false;
            log(std::string("invalid gnssFix - dropping message. fix was: ") + std::to_string(fix.type), WARN_THROTTLE);
            return;
        }

        //GGA/quality
        if(fix.quality == 0) {
            gps_state_valid_ = false;
            log("invalid lat, lon, height - dropping message", WARN);
            return;
        }

        //GSA/type
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

        //GGA/quality
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

        //GGA/latitude,longitude,altitude
        double e, n;
        std::string zone;
        RobotLocalization::NavsatConversions::LLtoUTM(fix.latitude, fix.longitude, n, e, zone);
        gps_state_.pos_lat = fix.latitude;
        gps_state_.pos_lon = fix.longitude;
        gps_state_.position_valid = GGA_ms==last_ms; //if latitude, longitude, altitude from last epoch
        gps_state_.pos_e = mode_ == GpsInterface::ABSOLUTE ? e - datum_e_ : e;
        gps_state_.pos_n = mode_ == GpsInterface::ABSOLUTE ? n - datum_n_ : n;
        gps_state_.pos_u = mode_ == GpsInterface::ABSOLUTE ? fix.altitude - datum_u_ : fix.altitude;

        //GST/deviation or GSA/dilution or GGA/quality
        gps_state_.position_accuracy_valid = GST_ms==last_ms && GSA_ms==last_ms && GGA_ms == last_ms;
        gps_state_.position_accuracy = fix.horizontalAccuracy();

        //RMC/speed, travelAngle 
        //speed in m/s
        double speed = fix.speed / 3.6;
        double angle_rad = fix.travelAngle * M_PI / 180.0;
        gps_state_.vel_e = sin(angle_rad) * speed;
        gps_state_.vel_n = cos(angle_rad) * speed;
        gps_state_.vel_u = 0;
        gps_state_.motion_heading = angle_rad;
        gps_state_.motion_heading_valid = RMC_ms == last_ms;//if the speed and travelAngle from last epoch
        gps_state_.vehicle_heading_valid = false;

        gps_state_.epoch_ms = last_ms;
        gps_state_.received_time = fix.last_epoch.getTime();

        gps_state_valid_ = gps_state_.position_valid && gps_state_.position_accuracy_valid && gps_state_.motion_heading_valid;

        log(std::string("gps update message ") + std::to_string(messageId), INFO);

        if(reportEveryUpdate || gps_state_valid_) {
            if (state_callback) {
                state_callback(gps_state_);
            }
        }
    };
}
