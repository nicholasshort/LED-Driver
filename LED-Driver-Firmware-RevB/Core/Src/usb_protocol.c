/**
  ******************************************************************************
  * @file    usb_protocol.c
  * @brief   USB CDC LED Frame Streaming Protocol Implementation
  *          Implements streaming parser with state machine, double buffering,
  *          and frame validation.
  ******************************************************************************
  */

#include "usb_protocol.h"
#include <string.h>
#include "stm32f1xx_hal.h"  // For __disable_irq() / __enable_irq()

/* Parser State Machine States */
typedef enum {
    RX_WAIT_SOF1,      // Waiting for first start-of-frame byte (0xAA)
    RX_WAIT_SOF2,      // Waiting for second start-of-frame byte (0x55)
    RX_READ_HEADER,    // Reading remaining header bytes
    RX_READ_PAYLOAD    // Reading payload data into pending frame buffer
} rx_state_t;

/* Double Buffering: Two frame buffers that we swap between */
static uint8_t frame_a[FRAME_BYTES];
static uint8_t frame_b[FRAME_BYTES];

/* Buffer pointers - these get swapped atomically */
static volatile uint8_t *active_frame  = frame_a;   // Frame currently being displayed
static volatile uint8_t *pending_frame = frame_b;   // Frame currently being written

/* Frame synchronization flags */
static volatile uint8_t  new_frame_ready = 0;       // Set when new frame is validated
static volatile uint16_t last_frame_id   = 0;       // Last successfully received frame ID

/* Parser state */
static rx_state_t  rx_state      = RX_WAIT_SOF1;
static frame_hdr_t rx_header;                       // Header being assembled
static uint16_t    rx_bytes_read = 0;               // Bytes read for current state
static uint8_t     rx_hdr_buf[FRAME_HDR_SIZE];     // Temporary buffer for header bytes

/* Statistics */
static protocol_stats_t stats = {0};

/* Forward declarations */
static void on_full_frame_received(const frame_hdr_t *hdr);
static uint8_t validate_and_swap_frame(const frame_hdr_t *hdr);

/**
 * @brief Initialize the protocol parser
 */
void protocol_init(void)
{
    rx_state = RX_WAIT_SOF1;
    rx_bytes_read = 0;
    new_frame_ready = 0;
    last_frame_id = 0;

    // Clear frame buffers
    memset(frame_a, 0, FRAME_BYTES);
    memset(frame_b, 0, FRAME_BYTES);

    // Reset statistics
    memset(&stats, 0, sizeof(stats));
}

/**
 * @brief Streaming parser - feeds bytes through state machine
 * @param data Incoming byte buffer
 * @param len  Number of bytes
 *
 * This function can handle arbitrary chunking of data across USB packets.
 * It maintains state between calls and reconstructs complete frames.
 */
void protocol_feed_bytes(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        switch (rx_state)
        {
            case RX_WAIT_SOF1:
                if (byte == FRAME_SOF1)
                {
                    rx_hdr_buf[0] = byte;
                    rx_bytes_read = 1;
                    rx_state = RX_WAIT_SOF2;
                }
                else
                {
                    // Stay in this state, waiting for SOF1
                    stats.sync_errors++;
                }
                break;

            case RX_WAIT_SOF2:
                if (byte == FRAME_SOF2)
                {
                    rx_hdr_buf[1] = byte;
                    rx_bytes_read = 2;
                    rx_state = RX_READ_HEADER;
                }
                else
                {
                    // Invalid SOF2, restart
                    rx_state = RX_WAIT_SOF1;
                    stats.sync_errors++;
                }
                break;

            case RX_READ_HEADER:
                // Continue filling header buffer
                rx_hdr_buf[rx_bytes_read++] = byte;

                if (rx_bytes_read >= FRAME_HDR_SIZE)
                {
                    // Full header received, parse it
                    memcpy(&rx_header, rx_hdr_buf, FRAME_HDR_SIZE);

                    // Validate header fields
                    if (rx_header.version != FRAME_VERSION ||
                        rx_header.type != FRAME_TYPE_FULL ||
                        rx_header.length > FRAME_BYTES)
                    {
                        // Invalid header, restart
                        rx_state = RX_WAIT_SOF1;
                        stats.length_errors++;
                        break;
                    }

                    // Header is valid, prepare to receive payload
                    rx_bytes_read = 0;
                    rx_state = RX_READ_PAYLOAD;
                }
                break;

            case RX_READ_PAYLOAD:
                // Write directly into pending frame buffer (avoid extra copy)
                ((uint8_t*)pending_frame)[rx_bytes_read++] = byte;

                if (rx_bytes_read >= rx_header.length)
                {
                    // Full payload received, validate and process
                    on_full_frame_received(&rx_header);

                    // Reset to wait for next frame
                    rx_state = RX_WAIT_SOF1;
                    rx_bytes_read = 0;
                }
                break;
        }
    }
}

/**
 * @brief Called when a complete frame has been received
 * @param hdr Pointer to the received frame header
 *
 * This function validates the checksum and atomically swaps the
 * active and pending frame buffers if validation passes.
 */
static void on_full_frame_received(const frame_hdr_t *hdr)
{
    // Validate checksum
    if (validate_and_swap_frame(hdr))
    {
        stats.frames_received++;
    }
    else
    {
        stats.frames_dropped++;
    }
}

/**
 * @brief Validate checksum and swap frame buffers if valid
 * @param hdr Frame header containing expected checksum
 * @return 1 if valid and swapped, 0 if checksum failed
 */
static uint8_t validate_and_swap_frame(const frame_hdr_t *hdr)
{
    // Calculate checksum: 8-bit sum of all payload bytes mod 256
    uint8_t calculated_checksum = 0;
    for (uint16_t i = 0; i < hdr->length; i++)
    {
        calculated_checksum += ((uint8_t*)pending_frame)[i];
    }

    // Verify checksum matches
    if (calculated_checksum != hdr->checksum)
    {
        // Checksum mismatch, reject frame
        return 0;
    }

    // Checksum valid! Atomically swap the frame buffers
    // Disable interrupts briefly to ensure atomic pointer swap
    __disable_irq();

    uint8_t *tmp   = (uint8_t *)active_frame;
    active_frame   = pending_frame;
    pending_frame  = tmp;
    last_frame_id  = hdr->frame_id;
    new_frame_ready = 1;

    __enable_irq();

    return 1;
}

/**
 * @brief Get pointer to the current active frame
 * @return Pointer to active frame buffer (safe to read from main loop)
 */
const uint8_t* protocol_get_active_frame(void)
{
    return (const uint8_t*)active_frame;
}

/**
 * @brief Check if a new frame is ready
 * @return 1 if new frame available, 0 otherwise
 */
uint8_t protocol_new_frame_ready(void)
{
    return new_frame_ready;
}

/**
 * @brief Acknowledge frame processing (clears new_frame_ready flag)
 */
void protocol_ack_frame(void)
{
    __disable_irq();
    new_frame_ready = 0;
    __enable_irq();
}

/**
 * @brief Get last received frame ID
 * @return Frame ID of last successfully received frame
 */
uint16_t protocol_get_last_frame_id(void)
{
    return last_frame_id;
}

/**
 * @brief Get protocol statistics
 * @param stats Pointer to statistics structure to fill
 */
void protocol_get_stats(protocol_stats_t *stats_out)
{
    __disable_irq();
    memcpy(stats_out, (void*)&stats, sizeof(protocol_stats_t));
    __enable_irq();
}

/**
 * @brief Reset protocol statistics
 */
void protocol_reset_stats(void)
{
    __disable_irq();
    memset(&stats, 0, sizeof(protocol_stats_t));
    __enable_irq();
}
