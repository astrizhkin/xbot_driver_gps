#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <rtcm_msgs/Message.h>
#include <std_msgs/UInt8MultiArray.h>

#include <serial/serial.h>

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

// ── RSSI polling state ─────────────────────────────────────────────────────
//
// 0xC1 is always registered in the parser to keep the parser simple.
// g_await_rssi is the gate: on_packet() only acts on a 0xC1 frame when true.
// Set on the main thread (timer), cleared on rx_thread (on_packet or error).
// std::atomic gives the necessary cross-thread visibility without a mutex.
//
static ros::Time         g_rssi_sent_at;          // written/read on main thread only

// ── Globals (node-scoped) ──────────────────────────────────────────────────
static ros::Publisher          g_rtcm_pub;
static ros::Publisher          g_cmd_pub;

static serial::Serial          g_serial;
static std::atomic<bool>       g_stopped { false };

// TX queue
static std::vector<uint8_t>    g_tx_buf;
static std::mutex              g_tx_mutex;
static std::condition_variable g_tx_cv;

// ── Forward declarations ───────────────────────────────────────────────────
void rx_thread_fn(const std::string& port, uint32_t baudrate);
void tx_thread_fn();
void on_serial_write(const std_msgs::UInt8MultiArray::ConstPtr& msg);
void on_packet(uint8_t preamble, const uint8_t* frame, size_t length,uint16_t msg_type);

RTCMParser parser({0xD3, 0xE3}, on_packet);


// ── TX helper — enqueue bytes and wake tx_thread ───────────────────────────
static void enqueue_tx(const uint8_t* data, size_t len) {
  {
    std::lock_guard<std::mutex> lk(g_tx_mutex);
    g_tx_buf.insert(g_tx_buf.end(), data, data + len);
    if (g_tx_buf.size() > 5000) {
      ROS_WARN_THROTTLE(5, "[radio] TX buffer growing large: %zu bytes", g_tx_buf.size());
    }
  }
  g_tx_cv.notify_one();
}

// ── Packet callback (called from rx_thread) ────────────────────────────────
//

// ROS1 Publisher::publish() is thread-safe, so we publish directly here
// without bouncing through the main thread.
void on_packet(uint8_t preamble, const uint8_t* frame, size_t length,uint16_t msg_type) {
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

    ROS_INFO("[radio] RSSI - ambient: %d dBm,  last packet: %d dBm", ambient_dbm, last_packet_dbm);
    return;
  }
  if (preamble == 0xD3) {
    // Standard RTCM3 — publish as rtcm_msgs/Message
    rtcm_msgs::Message msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::to_string(msg_type);
    msg.message.assign(frame, frame + length);
    g_rtcm_pub.publish(msg);
    //ROS_INFO("[radio] Published RTCM3 msg type=%u len=%zu", msg_type, length);
    return;
  }
  if(preamble == 0xE3){
    std_msgs::UInt8MultiArray msg;
    msg.data.assign(frame, frame + length);
    ROS_INFO("[radio] Radio CMD preamble=0x%02X type=%u len=%zu", preamble, msg_type, length);
    g_cmd_pub.publish(msg);
    return;
  }
}

// ── RSSI poll timer (fires on main thread via ros::spin) ───────────────────
void on_rssi_timer(const ros::TimerEvent&) {
  if (!g_serial.isOpen()) {
    ROS_WARN("[radio] RSSI poll skipped - port not open");
    return;
  }

  // Warn if the previous request never got a response
  if (parser.is_await_e22_rssi()) {
    ROS_WARN("[radio] RSSI response timeout (sent %.2f s ago)",
             (ros::Time::now() - g_rssi_sent_at).toSec());
    parser.await_e22_rssi(false);
  }

  parser.await_e22_rssi(true);
  g_rssi_sent_at = ros::Time::now();
  enqueue_tx(RSSI_CMD, sizeof(RSSI_CMD));
  //ROS_INFO("[radio] RSSI query sent");
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
                     [] { return !g_tx_buf.empty() || g_stopped.load(); });

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

// ── ROS subscriber callback — enqueue bytes for TX ────────────────────────
void on_serial_write(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) return;
  enqueue_tx(msg->data.data(), msg->data.size());
}

// ── main ───────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "rtcm_serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // ── Parameters ──────────────────────────────────────────────────────────
  const std::string port     = pnh.param("serial_port", std::string(""));
  const uint32_t    baudrate = static_cast<uint32_t>(pnh.param("baudrate", 57600));
  const double      rssi_period = pnh.param("rssi_poll_period", 5.0);   // seconds

  if (port.empty() || baudrate == 0) {
    ROS_FATAL("[radio] serial_port and baudrate must be set");
    return 1;
  }

  // ── Publishers / subscribers ─────────────────────────────────────────────
  g_rtcm_pub = pnh.advertise<rtcm_msgs::Message>("rtcm", 10);
  g_cmd_pub  = pnh.advertise<std_msgs::UInt8MultiArray>("radio_cmd", 10);

  ros::Subscriber write_sub = pnh.subscribe<std_msgs::UInt8MultiArray>(
      "radio_write", 10, on_serial_write,
      ros::TransportHints().tcpNoDelay(true));

  // Repeating timer fires on the main thread — no extra locking needed
  // for g_rssi_sent_at which is only touched in timer callbacks
  ros::Timer rssi_timer = nh.createTimer(ros::Duration(rssi_period), on_rssi_timer);

  // ── Start background threads ─────────────────────────────────────────────
  std::thread rx_thread(rx_thread_fn, port, baudrate);
  std::thread tx_thread(tx_thread_fn);

  ROS_INFO("[radio] Started: port=%s baudrate=%u rssi_poll=%.1fs",
           port.c_str(), baudrate, rssi_period);

  ros::spin();   // blocks here; handles write_sub callbacks on the main thread

  // ── Shutdown ─────────────────────────────────────────────────────────────
  g_stopped = true;
  g_tx_cv.notify_all();   // unblock tx_thread if waiting

  rx_thread.join();
  tx_thread.join();

  ROS_INFO("[radio] Stopped");
  return 0;
}