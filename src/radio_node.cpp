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

// ── Packet callback (called from rx_thread) ────────────────────────────────
//

// ROS1 Publisher::publish() is thread-safe, so we publish directly here
// without bouncing through the main thread.
void on_packet(uint8_t preamble, const uint8_t* frame, size_t length,uint16_t msg_type) {
  if (preamble == 0xD3) {
    // Standard RTCM3 — publish as rtcm_msgs/Message
    rtcm_msgs::Message msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::to_string(msg_type);
    msg.message.assign(frame, frame + length);
    g_rtcm_pub.publish(msg);
    //ROS_INFO("[radio] Published RTCM3 msg type=%u len=%zu", msg_type, length);
  }
  else {
    // COMMAND packet (0xE3 or other registered preamble)
    std_msgs::UInt8MultiArray msg;
    msg.data.assign(frame, frame + length);
    ROS_INFO("[radio] Radio CMD preamble=0x%02X type=%u len=%zu", preamble, msg_type, length);
    g_cmd_pub.publish(msg);
  }
}

// ── RX thread ──────────────────────────────────────────────────────────────
void rx_thread_fn(const std::string& port, uint32_t baudrate) {
  RTCMParser parser({0xD3, 0xE3}, on_packet);

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
        ROS_WARN("[radio] TX: partial write — %zu of %zu bytes sent", written, to_write.size());
        // Re-queue the unsent tail
        std::unique_lock<std::mutex> lk2(g_tx_mutex);
        g_tx_buf.insert(g_tx_buf.begin(),
                        to_write.begin() + written,
                        to_write.end());
      }
    } catch (const std::exception& e){
      ROS_ERROR("[radio] TX write error: %s", e.what());
    }
  }
}

// ── ROS subscriber callback — enqueue bytes for TX ────────────────────────
void on_serial_write(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) return;

  {
    std::unique_lock<std::mutex> lk(g_tx_mutex);
    g_tx_buf.insert(g_tx_buf.end(), msg->data.begin(), msg->data.end());
    if (g_tx_buf.size() > 5000) {
      ROS_WARN_THROTTLE(5, "[radio] TX buffer growing large: %zu bytes", g_tx_buf.size());
    }
  }
  g_tx_cv.notify_one();
}

// ── main ───────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "rtcm_serial_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // ── Parameters ──────────────────────────────────────────────────────────
  const std::string port     = pnh.param("serial_port", std::string(""));
  const uint32_t    baudrate = static_cast<uint32_t>(pnh.param("baudrate", 57600));

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

  // ── Start background threads ─────────────────────────────────────────────
  std::thread rx_thread(rx_thread_fn, port, baudrate);
  std::thread tx_thread(tx_thread_fn);

  ROS_INFO("[radio] rtcm_serial_node started, port=%s, baudrate=%u", port.c_str(), baudrate);

  ros::spin();   // blocks here; handles write_sub callbacks on the main thread

  // ── Shutdown ─────────────────────────────────────────────────────────────
  g_stopped = true;
  g_tx_cv.notify_all();   // unblock tx_thread if waiting

  rx_thread.join();
  tx_thread.join();

  ROS_INFO("[radio] rtcm_serial_node stopped");
  return 0;
}