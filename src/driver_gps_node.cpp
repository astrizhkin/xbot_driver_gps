//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ros/ros.h"
#include "devices/serial_gps_device.h"
#include "devices/tcp_gps_device.h"
#include "interfaces/ublox_gps_interface.h"
#include "interfaces/nmea_gps_interface.h"
#include "xbot_driver_gps/SetDatumSrv.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "xbot_msgs/AbsolutePose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/UInt32.h"
#include "sensor_msgs/Imu.h"
#include "rtcm_msgs/Message.h"
#include "xbot_msgs/GNSSInfo.h"
#include <nmeaparse/nmea.h>
#include "GeographicLib/DMS.hpp"
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/Constants.hpp"
#include <boost/algorithm/string.hpp>
#include "nmea_msgs/Sentence.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <rosgraph_msgs/Log.h>

using namespace xbot::driver::gps;
using namespace nmea;

ros::Publisher pose_pub;
ros::Publisher xbot_pose_pub;
ros::Publisher latency_pub1;
ros::Publisher latency_pub2;
ros::Publisher latency_pub3;
ros::Publisher imu_pub;
ros::Publisher vrs_nmea_pub;

bool isUbxInterface = false;
GpsInterface *gpsInterface;

bool allow_verbose_logging = false;
xbot_msgs::AbsolutePose pose_result;

std_msgs::UInt32 latency_msg1, latency_msg2, latency_msg3;
sensor_msgs::Imu imu_msg;

double datum_lat, datum_long, datum_height;
bool has_datum; 

ros::Time last_vrs_feedback(0.0);
nmea_msgs::Sentence vrs_msg;

static std::map<std::string, ros::Time> g_rtcm_last_stamp;
ros::Time g_last_rtcm(0.0);

uint8_t radio_log_levels = 0;

ros::Publisher gnss_info_pub;

struct RtcM1005State {
    bool     valid       = false;
    uint32_t station_id  = 0;
    double   base_lat    = 0.0;
    double   base_lon    = 0.0;
    double   base_height = 0.0;
} g_rtcm1005;

// ── Bit-level RTCM 1005 parser ──────────────────────────────────────────────
static void parse_rtcm1005(const rtcm_msgs::Message& rtcm) {
    const std::vector<uint8_t>& data = rtcm.message;
    if (data.size() < 6)
        return;

    // RTCM3 frame: preamble(1) + length(2) + payload(N) + crc(3)
    // Payload starts at byte offset 3.
    const uint8_t* p = data.data() + 3;

    // Message type: first 12 bits of payload
    uint16_t msg_type = (static_cast<uint16_t>(p[0]) << 4) | ((p[1] >> 4) & 0x0Fu);
    if (msg_type != 1005)
        return;

    // Bit reader over the payload bytes, starting at bit 12 (after msg_type).
    size_t payload_sz = data.size() - 3;
    int bit_pos = 12;
    auto read_bits = [&](int nbits) -> uint32_t {
        uint32_t val = 0;
        for (int i = 0; i < nbits; i++) {
            int byte_idx = bit_pos / 8;
            int bit_idx  = 7 - (bit_pos % 8);
            if (byte_idx < static_cast<int>(payload_sz))
                val = (val << 1) | ((p[byte_idx] >> bit_idx) & 1u);
            bit_pos++;
        }
        return val;
    };

    // Reference station ID (20 bits)
    uint32_t station_id = read_bits(20);

    // L1 phase range (32 bits) – skip
    read_bits(32);
    // L2 phase range (32 bits) – skip
    read_bits(32);
    // DGNSS/RTK indicator (1 bit) – skip
    read_bits(1);
    // Survey mode (1 bit) – skip
    read_bits(1);
    // Age of survey (16 bits) – skip
    read_bits(16);
    // Number of survey-in epochs (16 bits) – skip
    read_bits(16);
    // Average N mm (16 bits) – skip
    read_bits(16);
    // Average E mm (16 bits) – skip
    read_bits(16);
    // Average D mm (16 bits) – skip
    read_bits(16);
    // Antenna manufacturer (128 bits) – skip
    read_bits(128);
    // Antenna type (128 bits) – skip
    read_bits(128);

    // ARP coordinates: ECEF X/Y/Z (32-bit signed, 1 mm resolution)
    int32_t arp_x_mm = static_cast<int32_t>(read_bits(32));
    int32_t arp_y_mm = static_cast<int32_t>(read_bits(32));
    int32_t arp_z_mm = static_cast<int32_t>(read_bits(32));

    double xp = arp_x_mm / 1000.0;  // mm → m
    double yp = arp_y_mm / 1000.0;
    double zp = arp_z_mm / 1000.0;

    // ECEF → WGS84 lat/lon/height
    GeographicLib::Geocentric earth(
        GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    double lat = 0, lon = 0, height = 0;
    earth.Reverse(xp, yp, zp, lat, lon, height);

    g_rtcm1005.valid     = true;
    g_rtcm1005.station_id = station_id;
    g_rtcm1005.base_lat   = lat;
    g_rtcm1005.base_lon   = lon;
    g_rtcm1005.base_height = height;
}

void generate_nmea(double lat_in, double lon_in) {
    // only send every 10 seconds, this will be more than needed
    if ((ros::Time::now() - last_vrs_feedback).toSec() < 10.0) {
        return;
    }
    last_vrs_feedback = ros::Time::now();
    NMEACommand cmd1;

    auto lat = GeographicLib::DMS::Encode(lat_in, GeographicLib::DMS::component::MINUTE, 4,
                                          GeographicLib::DMS::flag::LATITUDE, ';');
    auto lon = GeographicLib::DMS::Encode(lon_in, GeographicLib::DMS::component::MINUTE, 4,
                                          GeographicLib::DMS::flag::LONGITUDE, ';');

    // remove separator char
    boost::erase_all(lat, ";");
    boost::erase_all(lon, ";");

    auto lat_hemisphere = lat.substr(lat.length() - 1, 1);
    auto lon_hemisphere = lon.substr(lon.length() - 1, 1);


    std::stringstream message_ss;
    auto time_facet = new boost::posix_time::time_facet("%H%M%s");

    message_ss.imbue(std::locale(message_ss.getloc(), time_facet));
    message_ss << ros::Time::now().toBoost() << "," <<
               lat.substr(0, lat.length() - 1) << "," <<
               lat_hemisphere << "," <<
               lon.substr(0, lon.length() - 1) << "," <<
               lon_hemisphere << ",1,0,0,0,M,0,M,0000,";

    //build message
    cmd1.name = "GPGGA";
    cmd1.message = message_ss.str();

    vrs_msg.header.frame_id = "gps";
    vrs_msg.header.seq++;
    vrs_msg.header.stamp = ros::Time::now();
    vrs_msg.sentence = cmd1.toString();
    boost::erase_all(vrs_msg.sentence, "\r\n");
    vrs_nmea_pub.publish(vrs_msg);
}

void gps_log(std::string text, LogLevel level) {
    switch (level) {
        case VERBOSE:
            if (!allow_verbose_logging) {
                return;
            }
            ROS_INFO_STREAM("[driver_gps] " << text);
            break;
        case INFO:
            ROS_INFO_STREAM("[driver_gps] " << text);
            break;
        case INFO_THROTTLE:
            ROS_INFO_STREAM_THROTTLE(5,"[driver_gps] " << text);
            break;
        case WARN:
            ROS_WARN_STREAM("[driver_gps] " << text);
            break;
        case WARN_THROTTLE:
            ROS_WARN_STREAM_THROTTLE(5,"[driver_gps] " << text);
            break;
        default:
            ROS_ERROR_STREAM("[driver_gps] " << text);
            break;
    }
}

void rtcm_received(const rtcm_msgs::Message::ConstPtr &rtcm) {
    g_last_rtcm = rtcm->header.stamp;
    g_rtcm_last_stamp[rtcm->header.frame_id] = rtcm->header.stamp;
    parse_rtcm1005(*rtcm);
    gpsInterface->send_rtcm(rtcm->message.data(), rtcm->message.size());
}

void log_received(const rosgraph_msgs::Log::ConstPtr &msg) {
    if (radio_log_levels & msg->level) {
        ROS_DEBUG_STREAM("[driver_gps] Sending radio message [" << msg->msg << "]");
        const uint8_t *chars = (const uint8_t *)msg->msg.data();
        gpsInterface->send_rtcm(chars, msg->msg.length());
    }
}

void convert_gps_result(const GpsInterface::GpsState &state, xbot_msgs::AbsolutePose &result) {
    result.header.seq++;
    result.header.frame_id = "gps";
    result.header.stamp = ros::Time::now();

    result.source = xbot_msgs::AbsolutePose::SOURCE_GPS;
    result.flags = 0;
    result.epoch_ms  = state.epoch_ms;
    result.received_stamp = state.received_time;

    switch (state.rtk_type) {
        case GpsInterface::GpsState::RTK_FLOAT:
            result.flags = xbot_msgs::AbsolutePose::FLAG_GPS_RTK | xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT;
            break;
        case GpsInterface::GpsState::RTK_FIX:
            result.flags = xbot_msgs::AbsolutePose::FLAG_GPS_RTK | xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED;
            break;
        default:
            result.flags = 0;
    }

    if (state.fix_type == GpsInterface::GpsState::FixType::DR_ONLY ||
        state.fix_type == GpsInterface::GpsState::FixType::GNSS_DR_COMBINED) {
        result.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_DEAD_RECKONING;
    }

    //orientation
    result.orientation_valid = state.vehicle_heading_valid;
    result.orientation_accuracy = state.vehicle_heading_accuracy;

    //position accuracy
    result.position_accuracy = state.position_accuracy;
    result.position_accuracy_valid = state.position_accuracy_valid;

    //pose and heading
    result.pose_valid = state.position_valid;
    result.pose.pose.position.x = state.pos_e;
    result.pose.pose.position.y = state.pos_n;
    result.pose.pose.position.z = state.pos_u;

    double heading = state.vehicle_heading_valid ? state.vehicle_heading : state.motion_heading;
    double headingAcc = state.vehicle_heading_valid ? state.vehicle_heading_accuracy : state.motion_heading_accuracy;

    tf2::Quaternion q_mag;
    q_mag.setRPY(0.0, 0.0, heading);
    result.pose.pose.orientation = tf2::toMsg(q_mag);

    result.pose.covariance = {
            pow(state.position_accuracy, 2), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pow(state.position_accuracy, 2), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, pow(state.position_accuracy, 2), 0.0, 0.0,
            0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, pow(headingAcc, 2)
    };

    //motion vector
    result.motion_vector_valid = state.motion_heading_valid;
    result.motion_vector.x = state.vel_e;
    result.motion_vector.y = state.vel_n;
    result.motion_vector.z = state.vel_u;

    result.vehicle_heading = state.vehicle_heading;
    result.motion_heading = state.motion_heading;

}

void gps_state_received(const GpsInterface::GpsState &state) {
    // new state received, publish
    convert_gps_result(state, pose_result);
    xbot_pose_pub.publish(pose_result);
    pose_pub.publish(pose_result.pose);
    double rtcm_age = (ros::Time::now() - g_last_rtcm).toSec();
    ROS_INFO_THROTTLE(10,"[driver_gps] GNSS: fix %d, rtk %d, (GSV %d/%d, avgSNR %f) (PUBX %d/%d, avgSNR %f/%f), RTCM received age %fs, DGNSS age %fs",
        state.fix_type,state.rtk_type, 
        state.tracking_satelites, state.visible_satelites, state.average_snr,
        state.pubx_used_satelites, state.pubx_tracking_satelites, state.pubx_average_used_snr,state.pubx_average_tracking_snr,
        rtcm_age,state.diff_age);
    // send feedback to VRS
    generate_nmea(state.pos_lat, state.pos_lon);

    // --- Publish GNSSInfo ---
    static xbot_msgs::GNSSInfo gnss_info;
    gnss_info.header.seq++;
    gnss_info.header.frame_id = "gps";
    gnss_info.header.stamp = ros::Time::now();

    gnss_info.has_rtcm1005  = g_rtcm1005.valid;
    gnss_info.base_station_id = g_rtcm1005.station_id;
    gnss_info.base_lat_deg  = g_rtcm1005.base_lat;
    gnss_info.base_lon_deg  = g_rtcm1005.base_lon;
    gnss_info.base_height_m = g_rtcm1005.base_height;

    gnss_info.used_satellites     = state.pubx_used_satelites;
    gnss_info.tracking_satellites = state.pubx_tracking_satelites;
    gnss_info.avg_used_snr        = state.pubx_average_used_snr;
    gnss_info.avg_tracking_snr    = state.pubx_average_tracking_snr;

    gnss_info.rtcm_age_sec  = (ros::Time::now() - g_last_rtcm).toSec();
    gnss_info.dgnss_age_sec = state.diff_age;

    gnss_info_pub.publish(gnss_info);
}

void wheel_latency_received(uint32_t wheel_tick_stamp, uint32_t wheel_tick_stamp_ublox,
                       uint32_t wheel_tick_round_trip_stamp) {
    latency_msg1.data = wheel_tick_stamp;
    latency_msg2.data = wheel_tick_stamp_ublox;
    latency_msg3.data = wheel_tick_round_trip_stamp;
    latency_pub1.publish(latency_msg1);
    latency_pub2.publish(latency_msg2);
    latency_pub3.publish(latency_msg3);
}

void imu_received(const GpsInterface::ImuState &state) {
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "gps";
    imu_msg.header.seq++;
    imu_msg.angular_velocity.x = state.gx;
    imu_msg.angular_velocity.y = state.gy;
    imu_msg.angular_velocity.z = state.gz;
    imu_msg.linear_acceleration.x = state.ax;
    imu_msg.linear_acceleration.y = state.ay;
    imu_msg.linear_acceleration.z = state.az;
    imu_pub.publish(imu_msg);
}

uint8_t
read_config_levels(std::string& levels_string) {
    uint8_t levels = 0;

    std::stringstream ss(levels_string);

    while( ss.good() ) {
        std::string level;
        getline( ss, level, ',' );
        if (level == "DEBUG") {
            levels |= rosgraph_msgs::Log::DEBUG;
        } else if (level == "INFO") {
            levels |= rosgraph_msgs::Log::INFO;
        } else if (level == "WARN") {
            levels |= rosgraph_msgs::Log::WARN;
        } else if (level == "ERROR") {
            levels |= rosgraph_msgs::Log::ERROR;
        } else if (level == "FATAL") {
            levels |= rosgraph_msgs::Log::FATAL;
        } else {
            throw std::runtime_error(std::string("Unknown log level ") + level);
        }
    }

    return levels;
}

bool setDatum(xbot_driver_gps::SetDatumSrvRequest &req, xbot_driver_gps::SetDatumSrvResponse &res) {
    if(req.revert_default) {
        if(has_datum) {
            ROS_INFO_STREAM("[driver_gps] Revert default datum");
            gpsInterface->set_datum(datum_lat, datum_long, datum_height);        
        } else {
            ROS_WARN_STREAM("[driver_gps] No default datum to revert. Set to NAN");
            gpsInterface->set_datum(NAN, NAN, NAN);
            return false;
        }
    } else {
        ROS_INFO_STREAM("[driver_gps] Set datum "<<req.longitute<<"E, "<<req.latitude<<"N, "<<req.height);
        gpsInterface->set_datum(req.latitude, req.longitute, req.height);
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_driver_gps");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    allow_verbose_logging = paramNh.param("verbose", false);
    if (allow_verbose_logging) {
        ROS_WARN("[driver_gps] GPS node has verbose logging enabled");
    }

    std::string radio_log_levels_string = paramNh.param("radio_log_levels", std::string(""));
    radio_log_levels = read_config_levels(radio_log_levels_string);
    if (!radio_log_levels) {
        ROS_WARN("[driver_gps] Radio logging is is disabled");
    }

    isUbxInterface = paramNh.param("ubx_mode", true);
    if(isUbxInterface) {
        ROS_INFO_STREAM("[driver_gps] Using UBX mode for GPS");
        gpsInterface = new UbxGpsInterface();
    } else {
        ROS_INFO_STREAM("[driver_gps] Using NMEA mode for GPS");
        gpsInterface = new NmeaGpsInterface(allow_verbose_logging, true);
    }

    gpsInterface->set_log_function(gps_log);

    std::string device_type = paramNh.param("device_type", std::string("serial"));
    if (device_type == "serial") {
        SerialGpsDevice *device = new SerialGpsDevice();
        device->set_baudrate(paramNh.param("baudrate", 38400));
        device->set_serial_port(paramNh.param("serial_port", std::string("/dev/ttyACM0")));
        gpsInterface->set_device(device);
    } else if (device_type == "tcp") {
        TcpGpsDevice *device = new TcpGpsDevice();
        device->set_host(paramNh.param("tcp_host", std::string("")));
        device->set_port(paramNh.param("tcp_port", std::string("")));
        gpsInterface->set_device(device);
    } else if (device_type == "file") {
        ROS_INFO_STREAM("[driver_gps] Reading GPS data from file!");
        gpsInterface->set_file_name(paramNh.param("filename", std::string("/dev/null")));
    } else {
        ROS_ERROR_STREAM("[driver_gps] Invalid device type");
        return 2;
    }

    std::string mode = paramNh.param("mode", std::string("absolute"));
    if (mode == "absolute") {
        ROS_INFO_STREAM("[driver_gps] Using absolute mode for GPS");
        gpsInterface->set_mode(xbot::driver::gps::GpsInterface::ABSOLUTE);
        has_datum = true;
        has_datum &= paramNh.getParam("datum_lat", datum_lat);
        has_datum &= paramNh.getParam("datum_long", datum_long);
        has_datum &= paramNh.getParam("datum_height", datum_height);
        if (!has_datum) {
            ROS_ERROR_STREAM(
                    "[driver_gps] You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
            return 2;
        }
        gpsInterface->set_datum(datum_lat, datum_long, datum_height);
    } else if (mode == "relative") {
        ROS_INFO_STREAM("[driver_gps] Using relative mode for GPS");
        gpsInterface->set_mode(xbot::driver::gps::GpsInterface::RELATIVE);
    }


    ros::Subscriber rtcm_sub = n.subscribe("rtcm", 0, rtcm_received,
                                           ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber rosout_sub = n.subscribe("radio_log_in", 0, log_received,
                                           ros::TransportHints().tcpNoDelay(true));


    vrs_nmea_pub = n.advertise<nmea_msgs::Sentence>("/nmea", 10);
    pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovariance>("pose", 10);
    xbot_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose", 10);
    imu_pub = paramNh.advertise<sensor_msgs::Imu>("imu", 10);
    gnss_info_pub = paramNh.advertise<xbot_msgs::GNSSInfo>("gnss_info", 10);

    ros::ServiceServer set_datum_srv = paramNh.advertiseService("set_datum", setDatum);

    gpsInterface->set_state_callback(gps_state_received);

    if (paramNh.param("publish_latency", true) && isUbxInterface) {
        latency_pub1 = paramNh.advertise<std_msgs::UInt32>("wheel_tick_stamp_esc", 100);
        latency_pub2 = paramNh.advertise<std_msgs::UInt32>("wheel_tick_ublox_rx", 100);
        latency_pub3 = paramNh.advertise<std_msgs::UInt32>("wheel_tick_round_trip_host", 100);
        dynamic_cast<UbxGpsInterface*>(gpsInterface)->set_wheel_latency_callback(wheel_latency_received);
    }
    gpsInterface->set_imu_callback(imu_received);

    if (!gpsInterface->start()) {
        return 1;
    }


    while (ros::ok()) {
        ros::spin();
    }

    gpsInterface->stop();
    delete gpsInterface;
    return 0;
}
