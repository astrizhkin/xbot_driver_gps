#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <rtcm_msgs/Message.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>

#include <serial/serial.h>

#include "e3_lib/e3parser.h"
#include "radio/rtcmparser.h"

// ── Constants ──────────────────────────────────────────────────────────────
static constexpr size_t  READ_CHUNK      = 512;   ///< bytes per serial::read call
static constexpr int     READ_TIMEOUT_MS = 100;   ///< serial read timeout (ms)
static constexpr int     RECONNECT_DELAY_S = 1;   ///< pause between reconnect attempts

// RSSI query command: C0 C1 C2 C3 + start_addr(0x00) + read_length(0x02)
// reads two registers: 0x00 = ambient noise RSSI, 0x01 = last-packet RSSI
static constexpr uint8_t RSSI_CMD[] = { 0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02 };

// Response preamble from the radio for register-read replies
static constexpr uint8_t RSSI_RESPONSE_PREAMBLE = 0xC1;

// dBm conversion per spec: dBm = -(256 - RSSI)
static inline int rssi_to_dbm(uint8_t rssi) { return -(256 - static_cast<int>(rssi)); }

static double g_rssi_period;
static ros::Time         g_rssi_sent_at;          // written/read on main thread only
static uint32_t g_tx_idle_delay_ms;               // ms to wait for parser idle before TX

// ── Globals (node-scoped) ──────────────────────────────────────────────────
static ros::Publisher          g_rtcm_pub;
static ros::Publisher          g_e3_payload_pub;
static ros::Publisher          g_rssi_pub;

static serial::Serial          g_serial;
static std::atomic<bool>       g_stopped { false };
static std::atomic<bool>       g_inject_rssi { false };

// TX queue
static std::vector<uint8_t>    g_tx_buf;
static std::mutex              g_tx_mutex;
static std::condition_variable g_tx_cv;

// E3 sender ID (derived from ROBOT_ID env var)
static uint16_t g_e3_sender_id { 0 };

// ── Forward declarations ───────────────────────────────────────────────────
void rx_thread_fn(const std::string& port, uint32_t baudrate);
void tx_thread_fn();
void on_serial_write(const std_msgs::UInt8MultiArray::ConstPtr& msg);
void on_tx_e3_payload(const std_msgs::UInt8MultiArray::ConstPtr& msg);
void on_packet(uint8_t preamble, const uint8_t* frame, size_t length, uint16_t msg_type);
void scheduleRSSI();

RTCMParser parser;


// ── TX helper — enqueue bytes and wake tx_thread ───────────────────────────
static void enqueue_tx(const uint8_t* data, size_t len) {
  {
    std::lock_guard<std::mutex> lk(g_tx_mutex);
    g_tx_buf.insert(g_tx_buf.end(), data, data + len);
    if (g_tx_buf.size() > 1000) {
      ROS_WARN_THROTTLE(5, "[radio] TX buffer growing large: %zu bytes", g_tx_buf.size());
    }
  }
  g_tx_cv.notify_one();
}

// ── E3 frame builder — wraps payload with preamble/length/sender/CRC ──────
static void enqueue_e3_frame(const uint8_t* payload, size_t payload_len) {
  std::vector<uint8_t> kv_bytes(payload, payload + payload_len);

  std::vector<uint8_t> raw;
  raw.push_back(0xE3);
  uint16_t total_len = static_cast<uint16_t>(kv_bytes.size());
  raw.push_back((total_len >> 8) & 0xFF);
  raw.push_back(total_len & 0xFF);
  raw.push_back((g_e3_sender_id >> 8) & 0xFF);
  raw.push_back(g_e3_sender_id & 0xFF);
  raw.insert(raw.end(), kv_bytes.begin(), kv_bytes.end());

  uint32_t crc = e3::crc24q(raw.data(), raw.size());
  raw.push_back((crc >> 16) & 0xFF);
  raw.push_back((crc >> 8) & 0xFF);
  raw.push_back(crc & 0xFF);

  enqueue_tx(raw.data(), raw.size());
}

// ── Packet callback (called from rx_thread) ────────────────────────────────
void on_packet(uint8_t preamble, const uint8_t* frame, size_t length, uint16_t msg_type) {
  if (preamble == RSSI_RESPONSE_PREAMBLE) {
    parser.await_e22_rssi(false);
    if(length!=5) {
      ROS_WARN("[radio] Got RSSI packet len=%zu, expected 5", length);
      return;
    }
    const uint8_t addr     = frame[1];   // expect 0x00
    const uint8_t data_len = frame[2];   // expect 0x02

    if (addr != 0x00 || data_len !=0x02) {
      ROS_WARN("[radio] Unexpected RSSI response addr=0x%02X read_len=%u", addr, data_len);
      return;
    }

    const int ambient_dbm     = rssi_to_dbm(frame[3]);  // reg 0x00
    const int last_packet_dbm = rssi_to_dbm(frame[4]);  // reg 0x01
    //absolute minimum for signal -131dBm (E22-Txxx22S)
    //strength = last_packet_dbm + 131;

    ROS_INFO("[radio] RSSI ambient: %d dBm, last packet: %d dBm, RTCM3/CMD invalid: %d, valid: %d", ambient_dbm, last_packet_dbm,parser.invalid_count(),parser.valid_count());

    if (g_rssi_pub.getNumSubscribers() > 0) {
      std_msgs::Float32 msg;
      msg.data = static_cast<float>(last_packet_dbm);
      g_rssi_pub.publish(msg);
    }
    return;
  }
  if (preamble == 0xD3) {
    // Standard RTCM3 — publish as rtcm_msgs/Message
    rtcm_msgs::Message msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::to_string(msg_type);
    msg.message.assign(frame, frame + length);
    g_rtcm_pub.publish(msg);
    scheduleRSSI();
    return;
  }
  if (preamble == 0xE3) {
    // Debug: hex dump full E3 frame
    {
      std::string hex;
      for (size_t i = 0; i < length; i++) {
          char hb[4];
          snprintf(hb, sizeof(hb), "%02X ", frame[i]);
          hex += hb;
      }
      ROS_INFO("[radio] E3 RX: %s (%zu bytes)", hex.c_str(), length);
    }

    // Extract sender_id for logging, publish only payload bytes
    if (length >= 8) {
      uint16_t sender_id = (uint16_t(frame[3]) << 8) | frame[4];
      uint16_t total_len = (uint16_t(frame[1]) << 8) | frame[2];
      ROS_INFO("[radio] E3 payload sender=0x%04X kv_len=%u", sender_id, total_len);

      std_msgs::UInt8MultiArray msg;
      msg.data.assign(frame + 5, frame + 5 + total_len);
      g_e3_payload_pub.publish(msg);
    }
    scheduleRSSI();
    return;
  }
}

void scheduleRSSI() {
  if((ros::Time::now() - g_rssi_sent_at).toSec()<g_rssi_period) {
    return;
  }
  if(parser.is_await_e22_rssi()){
    ROS_WARN("[radio] RSSI response timeout (sent %.2f s ago)",
             (ros::Time::now() - g_rssi_sent_at).toSec());
  }
  
  g_inject_rssi.store(true);
  g_tx_cv.notify_one();
}

// ── RX thread ──────────────────────────────────────────────────────────────
void rx_thread_fn(const std::string& port, uint32_t baudrate) {

  std::vector<uint8_t> buf;
  buf.reserve(READ_CHUNK);

  while (!g_stopped) {
    // ── (Re)connect ─────────────────────────────────────────────────────
    if (!g_serial.isOpen()) {
      ROS_INFO("[radio] Opening serial port %s @ %u baud", port.c_str(), baudrate);
      try {
        g_serial.setPort(port);
        g_serial.setBaudrate(baudrate);
        auto to = serial::Timeout::simpleTimeout(READ_TIMEOUT_MS);
        g_serial.setTimeout(to);
        g_serial.open();
        ROS_INFO("[radio] Serial port opened successfully");

        // Unblock TX thread so it can start writing if queued data exists
        g_tx_cv.notify_all();
      }
      catch (const std::exception& e) {
        ROS_WARN("[radio] Failed to open serial port: %s — retrying in %ds", e.what(), RECONNECT_DELAY_S);
        ros::Duration(RECONNECT_DELAY_S).sleep();
        continue;
      }
    }

    // ── Read ────────────────────────────────────────────────────────────
    try {
      size_t n = g_serial.read(buf, READ_CHUNK);
      if (n > 0) {
        parser.feed(buf.data(), n);

        buf.clear();
      }
      // n == 0 is a normal read timeout — loop and try again
    } catch (const std::exception& e) {
      ROS_WARN("[radio] Serial read error: %s — closing port, will reconnect", e.what());
      try { g_serial.close(); } catch (...) {}
      parser.await_e22_rssi(false);   // cancel any pending RSSI wait on disconnect
      buf.clear();
    }
  }

  // Shutdown: close the port cleanly
  if (g_serial.isOpen()) {
    try { g_serial.close(); } catch (...) {}
  }
}

// ── TX thread ──────────────────────────────────────────────────────────────
void tx_thread_fn() {
  while (!g_stopped) {
    std::unique_lock<std::mutex> lk(g_tx_mutex);

    // Block until data is queued or we are asked to stop
    g_tx_cv.wait_for(lk, std::chrono::seconds(1),
                     [] { return !g_tx_buf.empty() || g_stopped.load() || g_inject_rssi.load(); });


    bool inject_rssi = g_inject_rssi.load();
    if (inject_rssi){
      g_inject_rssi.store(false);
      g_tx_buf.insert(g_tx_buf.begin(), RSSI_CMD, RSSI_CMD + sizeof(RSSI_CMD));
    }

    if (g_tx_buf.empty()) continue;

    if (!g_serial.isOpen()) {
      ROS_WARN_THROTTLE(5, "[radio] TX: serial port not open, dropping %zu bytes", g_tx_buf.size());
      g_tx_buf.clear();
      continue;
    }

    // Snapshot and release the lock before the (potentially blocking) write
    std::vector<uint8_t> to_write;
    to_write.swap(g_tx_buf);
    lk.unlock();

    // Wait for parser to be idle max 2000ms before transmitting
    uint32_t wait_time = 2000;
    ros::Time wait_start = ros::Time::now();
    while (!g_stopped && !parser.is_idle_at_least(g_tx_idle_delay_ms) && wait_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      wait_time-=5;
    }

    if(!parser.is_idle_at_least(g_tx_idle_delay_ms)){
      ROS_WARN("[radio] Parser states RX channel is still busy after 2000ms");
    }

    //set await rssi right before serial write
    if(inject_rssi) {
      g_rssi_sent_at = ros::Time::now();
      parser.await_e22_rssi(true);
    }

    // Debug: hex dump E3 frame
    {
        std::string hex;
        for (uint8_t b : to_write) {
            char hb[4];
            snprintf(hb, sizeof(hb), "%02X ", b);
            hex += hb;
        }
        ROS_INFO("[radio] wait %dms, TX: %s (%zu bytes)", (int)(1000*(ros::Time::now() - wait_start).toSec()), hex.c_str(), to_write.size());
    }

    try {
      size_t written = g_serial.write(to_write);
      if (written != to_write.size()) {
        ROS_WARN("[radio] TX: partial write - %zu of %zu bytes sent", written, to_write.size());
        // Re-queue the unsent tail
        std::lock_guard<std::mutex> lk2(g_tx_mutex);
        g_tx_buf.insert(g_tx_buf.begin(),
                        to_write.begin() + written,
                        to_write.end());
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("[radio] TX write error: %s", e.what());
    }
  }
}

// ── ROS subscriber callback — enqueue raw bytes for TX ─────────────────────
void on_serial_write(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) return;
  enqueue_tx(msg->data.data(), msg->data.size());
}

// ── ROS subscriber callback — build E3 frame from payload and enqueue ──────
void on_tx_e3_payload(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) return;
  enqueue_e3_frame(msg->data.data(), msg->data.size());
}

// ── main ───────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "rtcm_serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // ── Parameters ──────────────────────────────────────────────────────────
  const std::string port     = pnh.param("serial_port", std::string(""));
  const uint32_t    baudrate = pnh.param("baudrate", 115200);
  g_rssi_period = pnh.param("rssi_poll_period", 5.0);   // seconds
  
  //safe margin for 128 bytes packet = 75ms
  //safe margin for 240 bytes packet = 125ms
  g_tx_idle_delay_ms = pnh.param("tx_idle_delay_ms", 75); // milliseconds

  parser.init({0xD3, 0xE3}, on_packet);

  if (port.empty() || baudrate == 0) {
    ROS_FATAL("[radio] serial_port and baudrate must be set");
    return 1;
  }

  // Derive E3 sender ID from robot_id param (first 4 hex chars of /etc/machine-id)
  {
    std::string robot_id = pnh.param("robot_id", std::string());
    if (robot_id.size() >= 4) {
      uint16_t sid;
      if (sscanf(robot_id.c_str(), "%04hx", &sid) == 1) {
        g_e3_sender_id = sid;
      } else {
        g_e3_sender_id = 0x0001;
        ROS_WARN("[radio] Invalid robot_id '%s', defaulting E3 sender_id=0x0001", robot_id.c_str());
      }
    } else {
      g_e3_sender_id = 0x0001;
      ROS_WARN("[radio] robot_id not set, defaulting E3 sender_id=0x0001");
    }
  }

  // ── Publishers / subscribers ─────────────────────────────────────────────
  g_rtcm_pub       = pnh.advertise<rtcm_msgs::Message>("rtcm", 10);
  g_e3_payload_pub = pnh.advertise<std_msgs::UInt8MultiArray>("rx_e3_payload", 10);
  g_rssi_pub       = pnh.advertise<std_msgs::Float32>("rssi", 10);

  ros::Subscriber write_sub = pnh.subscribe<std_msgs::UInt8MultiArray>(
      "radio_write", 10, on_serial_write,
      ros::TransportHints().tcpNoDelay(true));

  ros::Subscriber e3_payload_sub = pnh.subscribe<std_msgs::UInt8MultiArray>(
      "tx_e3_payload", 10, on_tx_e3_payload,
      ros::TransportHints().tcpNoDelay(true));

  // ── Start background threads ─────────────────────────────────────────────
  std::thread rx_thread(rx_thread_fn, port, baudrate);
  std::thread tx_thread(tx_thread_fn);

  ROS_INFO("[radio] Started: port=%s baudrate=%u e3_sender=0x%04X rssi_poll=%.1fs tx_delay=%u",
           port.c_str(), baudrate, g_e3_sender_id, g_rssi_period, g_tx_idle_delay_ms);

  ros::spin();   // blocks here; handles write_sub callbacks on the main thread

  // ── Shutdown ─────────────────────────────────────────────────────────────
  g_stopped = true;
  g_tx_cv.notify_all();   // unblock tx_thread if waiting

  rx_thread.join();
  tx_thread.join();

  ROS_INFO("[radio] Stopped");
  return 0;
}
