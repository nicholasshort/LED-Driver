#!/usr/bin/env python3
"""
USB CDC LED Frame Streaming Host Script for Raspberry Pi

This script sends RGB LED frame data to an STM32F1 over USB CDC
at approximately 30 frames per second using the defined protocol.

Hardware:
- Raspberry Pi (or any Linux host)
- STM32F1 with USB CDC device (/dev/ttyACM0)
- 8 LED strips, 120 LEDs each, 3 bytes per LED (RGB)

Protocol:
- Frame header (9 bytes):
  - SOF1: 0xAA
  - SOF2: 0x55
  - Version: 0x01
  - Type: 0x01 (full frame)
  - Length: 2880 (little endian uint16)
  - Frame ID: increments each frame (little endian uint16)
  - Checksum: 8-bit sum of payload bytes mod 256
- Payload: 2880 bytes of RGB data
  - Layout: strip0[LED0..119 as R,G,B], strip1[...], ... strip7[...]

Dependencies:
- pyserial: pip install pyserial

Usage:
- python3 host_send_led_frames.py [device]
- Default device: /dev/ttyACM0
"""

import serial
import struct
import time
import math
import signal
import sys
from typing import List, Tuple

# LED Configuration (must match STM32 firmware)
NUM_STRIPS = 8
LEDS_PER_STRIP = 120
BYTES_PER_LED = 3
FRAME_BYTES = NUM_STRIPS * LEDS_PER_STRIP * BYTES_PER_LED  # 2880 bytes

# Protocol Constants
FRAME_SOF1 = 0xAA
FRAME_SOF2 = 0x55
FRAME_VERSION = 0x01
FRAME_TYPE_FULL = 0x01

# Frame rate
TARGET_FPS = 45
FRAME_PERIOD = 1.0 / TARGET_FPS

# Global flag for clean shutdown
running = True


def signal_handler(sig, frame):
    """Handle Ctrl+C for clean shutdown"""
    global running
    print("\n\nShutdown requested... Turning off all LEDs")
    running = False


def calculate_checksum(payload: bytearray) -> int:
    """
    Calculate 8-bit checksum: sum of all bytes mod 256

    Args:
        payload: Frame payload bytes

    Returns:
        8-bit checksum value (0-255)
    """
    return sum(payload) & 0xFF


def build_frame_header(frame_id: int, payload_length: int, checksum: int) -> bytes:
    """
    Build the 9-byte frame header using the protocol specification

    Args:
        frame_id: Frame counter (0-65535, wraps around)
        payload_length: Number of bytes in payload (should be 2880)
        checksum: 8-bit checksum of payload

    Returns:
        9-byte header as bytes object

    Header format (little endian for multi-byte values):
    - sof1 (uint8): 0xAA
    - sof2 (uint8): 0x55
    - version (uint8): 0x01
    - type (uint8): 0x01
    - length (uint16): payload length in bytes
    - frame_id (uint16): frame counter
    - checksum (uint8): payload checksum
    """
    # Use struct.pack with '<' for little endian byte order
    # Format string: < = little endian
    #                B = unsigned char (1 byte)
    #                H = unsigned short (2 bytes)
    header = struct.pack(
        '<BBBBHHB',  # Little endian: 4 bytes, 2 uint16, 1 byte
        FRAME_SOF1,
        FRAME_SOF2,
        FRAME_VERSION,
        FRAME_TYPE_FULL,
        payload_length,
        frame_id,
        checksum
    )

    return header


def create_solid_color(r: int, g: int, b: int) -> bytearray:
    """
    Create a frame with all LEDs set to the same color

    Args:
        r, g, b: RGB color values (0-255)

    Returns:
        Frame payload (2880 bytes)
    """
    payload = bytearray(FRAME_BYTES)

    for i in range(0, FRAME_BYTES, BYTES_PER_LED):
        payload[i + 0] = r
        payload[i + 1] = g
        payload[i + 2] = b

    return payload


def create_rainbow_frame(frame_num: int) -> bytearray:
    """
    Create a moving rainbow pattern across all strips

    Args:
        frame_num: Current frame number (for animation)

    Returns:
        Frame payload (2880 bytes)
    """
    payload = bytearray(FRAME_BYTES)

    # Rainbow parameters
    speed = 0.05  # Animation speed
    offset = frame_num * speed

    for strip in range(NUM_STRIPS):
        for led in range(LEDS_PER_STRIP):
            # Calculate position in frame buffer
            index = (strip * LEDS_PER_STRIP + led) * BYTES_PER_LED

            # Create rainbow based on position
            hue = (led / LEDS_PER_STRIP + offset) % 1.0
            r, g, b = hsv_to_rgb(hue, 1.0, 0.5)  # 50% brightness

            payload[index + 0] = r
            payload[index + 1] = g
            payload[index + 2] = b

    return payload


def create_chaser_frame(frame_num: int) -> bytearray:
    """
    Create a chasing dot pattern on each strip

    Args:
        frame_num: Current frame number (for animation)

    Returns:
        Frame payload (2880 bytes)
    """
    payload = bytearray(FRAME_BYTES)  # All zeros (off) by default

    # Each strip gets a different color and phase offset
    strip_colors = [
        (255, 0, 0),    # Red
        (255, 127, 0),  # Orange
        (255, 255, 0),  # Yellow
        (0, 255, 0),    # Green
        (0, 255, 255),  # Cyan
        (0, 0, 255),    # Blue
        (127, 0, 255),  # Purple
        (255, 0, 255),  # Magenta
    ]

    for strip in range(NUM_STRIPS):
        # Moving position with phase offset per strip
        position = (frame_num * 2 + strip * 15) % LEDS_PER_STRIP

        # Light up 5 LEDs with trailing effect
        for offset in range(5):
            led = (position - offset) % LEDS_PER_STRIP
            index = (strip * LEDS_PER_STRIP + led) * BYTES_PER_LED

            # Fade trail
            brightness = (5 - offset) / 5.0
            r, g, b = strip_colors[strip]

            payload[index + 0] = int(r * brightness)
            payload[index + 1] = int(g * brightness)
            payload[index + 2] = int(b * brightness)

    return payload


def hsv_to_rgb(h: float, s: float, v: float) -> Tuple[int, int, int]:
    """
    Convert HSV color to RGB

    Args:
        h: Hue (0.0 - 1.0)
        s: Saturation (0.0 - 1.0)
        v: Value/Brightness (0.0 - 1.0)

    Returns:
        Tuple of (r, g, b) values (0-255)
    """
    if s == 0.0:
        r = g = b = int(v * 255)
        return (r, g, b)

    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i = i % 6

    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q

    return (int(r * 255), int(g * 255), int(b * 255))


def send_frame(ser: serial.Serial, frame_id: int, payload: bytearray) -> None:
    """
    Send a complete LED frame over USB CDC

    Args:
        ser: Serial port object
        frame_id: Frame counter (0-65535)
        payload: Frame payload (2880 bytes)
    """
    # Calculate checksum
    checksum = calculate_checksum(payload)

    # Build header
    header = build_frame_header(frame_id, len(payload), checksum)

    # Send header + payload
    frame = header + payload
    ser.write(frame)


def main():
    """Main function"""
    global running

    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Get device from command line or use default
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

    print("=" * 60)
    print("LED Frame Streaming Host - Raspberry Pi to STM32F1")
    print("=" * 60)
    print(f"Device: {device}")
    print(f"Frame size: {FRAME_BYTES} bytes")
    print(f"Target FPS: {TARGET_FPS}")
    print(f"Configuration: {NUM_STRIPS} strips × {LEDS_PER_STRIP} LEDs")
    print("=" * 60)
    print("\nPress Ctrl+C to stop\n")

    # Open serial port
    # Note: Baud rate is ignored for USB CDC, but required by pyserial
    try:
        ser = serial.Serial(
            port=device,
            baudrate=115200,  # Ignored for USB CDC
            timeout=1,
            write_timeout=1
        )
        print(f"✓ Opened {device}")
    except serial.SerialException as e:
        print(f"✗ Failed to open {device}: {e}")
        sys.exit(1)

    # Give the STM32 a moment to initialize
    time.sleep(0.5)

    # Animation selection (change this to switch patterns)
    # Options: 'rainbow', 'chaser', 'solid'
    animation_mode = 'rainbow'

    frame_id = 0
    frame_count = 0
    start_time = time.time()

    try:
        while running:
            frame_start = time.time()

            # Generate frame payload based on animation mode
            if animation_mode == 'rainbow':
                payload = create_rainbow_frame(frame_id)
            elif animation_mode == 'chaser':
                payload = create_chaser_frame(frame_id)
            elif animation_mode == 'solid':
                # Breathing blue effect
                brightness = (math.sin(frame_id * 0.1) + 1.0) / 2.0
                payload = create_solid_color(
                    0,
                    0,
                    int(255 * brightness)
                )
            else:
                payload = create_solid_color(0, 0, 0)

            # Send frame
            send_frame(ser, frame_id, payload)

            # Update counters
            frame_id = (frame_id + 1) & 0xFFFF  # Wrap at 65535
            frame_count += 1

            # Calculate and display stats every 100 frames
            if frame_count % 100 == 0:
                elapsed = time.time() - start_time
                actual_fps = frame_count / elapsed
                print(f"Frames sent: {frame_count:6d} | "
                      f"Actual FPS: {actual_fps:5.1f} | "
                      f"Frame ID: {frame_id:5d}")

            # Sleep to maintain target frame rate
            frame_elapsed = time.time() - frame_start
            sleep_time = FRAME_PERIOD - frame_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected")

    finally:
        # Send all-black frame before exiting
        print("Turning off all LEDs...")
        black_frame = create_solid_color(0, 0, 0)
        send_frame(ser, frame_id, black_frame)
        time.sleep(0.1)

        # Close serial port
        ser.close()
        print(f"✓ Closed {device}")

        # Final stats
        total_time = time.time() - start_time
        avg_fps = frame_count / total_time if total_time > 0 else 0
        print(f"\nTotal frames sent: {frame_count}")
        print(f"Average FPS: {avg_fps:.1f}")
        print("\nDone!")


if __name__ == '__main__':
    main()
