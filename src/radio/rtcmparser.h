#pragma once

#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <string>
#include <ros/ros.h>

/**
 * Streaming parser for RTCM3-style framed packets.
 *
 * Frame layout (identical for every registered preamble):
 *   preamble(1) | reserved+length_hi(1) | length_lo(1) | payload(N) | CRC-24Q(3)
 *
 * Multiple preamble values can be registered, each with its own callback.
 * The parser is preamble-agnostic: whichever registered preamble byte it sees
 * first wins and drives the rest of that frame.
 *
 * Callback signature:
 *   void(uint8_t preamble, const uint8_t* frame, size_t length, uint16_t msg_type)
 *
 * 'frame' points to the complete raw frame (preamble … CRC inclusive).
 * The pointer is only valid for the duration of the callback.
 */
class RTCMParser
{
public:
  using PacketCallback = std::function<void(uint8_t        preamble,
                                            const uint8_t* frame,
                                            size_t         length,
                                            uint16_t       msg_type)>;


  /** Feed a buffer of bytes into the parser. */
  void feed(const uint8_t* data, size_t length);

  /** Feed a single byte. */
  void feed(uint8_t byte);

  void await_e22_rssi(bool await_rssi) { await_e22_rssi_.exchange(await_rssi); };
  bool is_await_e22_rssi() { return await_e22_rssi_.load(); };
  uint32_t valid_count()   const { return valid_count_;   }
  uint32_t invalid_count() const { return invalid_count_; }
  bool is_idle()           const { return state_ == State::WAIT_PREAMBLE; }
  uint32_t in_idle_time() const;

  /**
   * Init the parser with a set of recognised preambles and a single
   * shared callback.  Any byte NOT in the preamble list is silently discarded
   * while the parser is waiting for a frame start.
   *
   * Example:
   *   init({0xD3, 0xE3}, my_callback);
   */
  void init(std::initializer_list<uint8_t> preambles, PacketCallback callback);

private:
  static constexpr uint8_t RSSI_RESPONSE_PREAMBLE = 0xC1;
  static constexpr uint8_t RSSI_REQUEST_PREAMBLE = 0xC0;
  
  std::atomic<bool> await_e22_rssi_ { false };
  
  struct Timing {
    double length_ms;
    uint8_t active_premable;
  };

  std::list<Timing> timing_;
  std::map<uint8_t,Timing> timing_stat_;

  enum class State : uint8_t {
    WAIT_PREAMBLE,
    LENGTH_H,
    LENGTH_L,
    PAYLOAD,
    CRC_0,
    CRC_1,
    CRC_2,
    E22_ADDR,
    E22_LEN,
    E22_DATA,
    // E3 (0xE3) specific states — has SENDER_ID field after length
    E3_SENDER_H,
    E3_SENDER_L,
    E3_PAYLOAD,
    E22_RSSI_MAGIC1,// rssi request magic1
    E22_RSSI_MAGIC2,// rssi request magic2
    E22_RSSI_MAGIC3,// rssi request magic3
    E22_RSSI1,      // rssi request 1
    E22_RSSI2,      // rssi request 2
  };

  void process_byte(uint8_t byte);

  inline void update_crc(uint8_t byte)
  {
    calc_crc_ = ((calc_crc_ << 8) & 0x00FFFFFFu)
                ^ CRC_LOOKUP[((calc_crc_ >> 16) ^ byte) & 0xFFu];
  }

  void build_stat(){
    if(timing_.size() > 100) {
      while(timing_.size()>50) {
        Timing &front = timing_.front();
        auto it = timing_stat_.find(front.active_premable);
        if (it != timing_stat_.end()) {
          it->second.length_ms += front.length_ms;
        }else{
          timing_stat_[front.active_premable] = front;
        }
        timing_.pop_front();
      }
      print_stat();
    }
  }

  void print_stat() {
    std::string stat;
    for (auto& [prem, t] : timing_stat_) {
      char buf[32];
      snprintf(buf, sizeof(buf), "0x%02X=%.1fms ", prem, t.length_ms);
      stat += buf;
    }
    ROS_INFO("[RTCMParser] avg: %s", stat.c_str());

    std::string seq;
    for (auto& t : timing_) {
      char buf[16];
      snprintf(buf, sizeof(buf), "0x%02X=%.1fms ", t.active_premable, t.length_ms);
      seq += buf;
    }
    ROS_INFO("[RTCMParser] seq: %s", seq.c_str());
  }

  void record_idle_time(double ms) {
    Timing t { .length_ms = ms, .active_premable = 0 };
    timing_.push_back(t);
    build_stat();
  }

  void record_parse_time(double ms) {
    Timing t { .length_ms = ms, .active_premable = active_preamble_ };
    timing_.push_back(t);
    build_stat();
  }

  void set_parser_state(State new_state) {
    if(state_ == State::WAIT_PREAMBLE && new_state != State::WAIT_PREAMBLE){
      ros::Time now = ros::Time::now();
      record_idle_time((now - switch_state_time_).toSec()*1000.0);
      switch_state_time_ = now;
    }
    if(state_ != State::WAIT_PREAMBLE && new_state == State::WAIT_PREAMBLE) {
      ros::Time now = ros::Time::now();
      record_parse_time((now - switch_state_time_).toSec()*1000.0);
      switch_state_time_ = now;
    }
    state_ = new_state;
  }

  /** Return true if 'byte' is a registered preamble. */
  bool is_preamble(uint8_t byte) const;

  // ── Per-frame state ────────────────────────────────────────────────────────
  State    state_          { State::WAIT_PREAMBLE };
  uint8_t  active_preamble_{ 0 };   ///< preamble of the frame being parsed
  uint16_t msg_length_     { 0 };
  uint16_t msg_type_       { 0 };
  uint16_t payload_count_  { 0 };
  uint32_t calc_crc_       { 0 };
  uint32_t recv_crc_       { 0 };
  uint32_t valid_count_    { 0 };
  uint32_t invalid_count_  { 0 };
  uint16_t e3_sender_id_   { 0 };

  // Max frame: preamble(1) + header(2) + sender_id(2, E3 only) + payload(1023) + CRC(3) = 1031 bytes
  static constexpr size_t MAX_FRAME_SIZE = 1031;
  uint8_t frame_buf_[MAX_FRAME_SIZE];
  size_t  frame_len_ { 0 };

  // Registered preambles (small fixed array — typical usage is 1-4 values)
  static constexpr size_t MAX_PREAMBLES = 8;
  uint8_t preambles_[MAX_PREAMBLES];
  size_t  preamble_count_ { 0 };

  PacketCallback callback_;
  ros::Time switch_state_time_;

  static const uint32_t CRC_LOOKUP[256];
};