/**
  ******************************************************************************
  * @file    usb_protocol.h
  * @brief   USB CDC LED Frame Streaming Protocol Header
  *          Defines the framing protocol, parser state machine, and API
  *          for receiving LED frames from Raspberry Pi host over USB CDC.
  ******************************************************************************
  */

#ifndef __USB_PROTOCOL_H
#define __USB_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* LED Configuration */
#define NUM_STRIPS      8
#define LEDS_PER_STRIP  120
#define BYTES_PER_LED   3
#define FRAME_BYTES     (NUM_STRIPS * LEDS_PER_STRIP * BYTES_PER_LED)  // 2880 bytes

/* Protocol Constants */
#define FRAME_SOF1      0xAA
#define FRAME_SOF2      0x55
#define FRAME_VERSION   0x01
#define FRAME_TYPE_FULL 0x01

/* Frame Header Structure - must be packed for exact byte layout */
#pragma pack(push, 1)
typedef struct {
    uint8_t  sof1;       // Start of frame byte 1: 0xAA
    uint8_t  sof2;       // Start of frame byte 2: 0x55
    uint8_t  version;    // Protocol version: 0x01
    uint8_t  type;       // Frame type: 0x01 = full LED frame
    uint16_t length;     // Payload length in bytes (little endian), e.g., 2880
    uint16_t frame_id;   // Frame counter, increments each frame, wraps at 65535
    uint8_t  checksum;   // 8-bit sum of payload bytes mod 256
} frame_hdr_t;
#pragma pack(pop)

#define FRAME_HDR_SIZE sizeof(frame_hdr_t)  // Should be 9 bytes

/**
 * @brief Initialize the USB protocol parser and buffers
 * @note  Call this once during system initialization
 */
void protocol_init(void);

/**
 * @brief Feed incoming USB CDC bytes to the protocol parser
 * @param data Pointer to received data buffer
 * @param len  Number of bytes received
 * @note  This function implements the streaming state machine that
 *        reconstructs frames from arbitrarily chunked USB packets.
 *        Safe to call from USB interrupt context (non-blocking).
 */
void protocol_feed_bytes(uint8_t *data, uint32_t len);

/**
 * @brief Get pointer to the active LED frame buffer
 * @return Pointer to current active frame (read-only)
 * @note   The active frame is the most recently completed, validated frame.
 *         This pointer is safe to read from the main loop.
 */
const uint8_t* protocol_get_active_frame(void);

/**
 * @brief Check if a new frame is ready for display
 * @return 1 if new frame available, 0 otherwise
 * @note   This flag is atomically cleared when you acknowledge the frame.
 */
uint8_t protocol_new_frame_ready(void);

/**
 * @brief Acknowledge that the new frame has been processed
 * @note  Call this after you've sent the frame to the LEDs.
 *        This clears the new_frame_ready flag atomically.
 */
void protocol_ack_frame(void);

/**
 * @brief Get the frame ID of the last successfully received frame
 * @return Last frame ID (wraps at 65535)
 */
uint16_t protocol_get_last_frame_id(void);

/**
 * @brief Get protocol statistics (optional, for debugging)
 */
typedef struct {
    uint32_t frames_received;      // Total valid frames received
    uint32_t frames_dropped;       // Frames with checksum errors
    uint32_t sync_errors;          // SOF sync errors
    uint32_t length_errors;        // Invalid length field errors
} protocol_stats_t;

void protocol_get_stats(protocol_stats_t *stats);
void protocol_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_PROTOCOL_H */
