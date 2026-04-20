#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>
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

  /**
   * Construct the parser with a set of recognised preambles and a single
   * shared callback.  Any byte NOT in the preamble list is silently discarded
   * while the parser is waiting for a frame start.
   *
   * Example:
   *   RTCMParser parser({0xD3, 0xE3}, my_callback);
   */
  RTCMParser(std::initializer_list<uint8_t> preambles, PacketCallback callback);

  /** Feed a buffer of bytes into the parser. */
  void feed(const uint8_t* data, size_t length);

  /** Feed a single byte. */
  void feed(uint8_t byte);

  void await_e22_rssi(bool await_rssi) { await_e22_rssi_.exchange(await_rssi); };
  bool is_await_e22_rssi() { await_e22_rssi_.load() };
  uint32_t valid_count()   const { return valid_count_;   }
  uint32_t invalid_count() const { return invalid_count_; }

private:
  static constexpr uint8_t RSSI_RESPONSE_PREAMBLE = 0xC1;
  std::atomic<bool> await_e22_rssi_ { false };

  enum class State : uint8_t
  {
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
  };

  void process_byte(uint8_t byte);

  inline void update_crc(uint8_t byte)
  {
    calc_crc_ = ((calc_crc_ << 8) & 0x00FFFFFFu)
                ^ CRC_LOOKUP[((calc_crc_ >> 16) ^ byte) & 0xFFu];
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

  // Max frame: preamble(1) + header(2) + payload(1023) + CRC(3) = 1029 bytes
  static constexpr size_t MAX_FRAME_SIZE = 1029;
  uint8_t frame_buf_[MAX_FRAME_SIZE];
  size_t  frame_len_ { 0 };

  // Registered preambles (small fixed array — typical usage is 1-4 values)
  static constexpr size_t MAX_PREAMBLES = 8;
  uint8_t preambles_[MAX_PREAMBLES];
  size_t  preamble_count_ { 0 };

  PacketCallback callback_;

  static const uint32_t CRC_LOOKUP[256];
};