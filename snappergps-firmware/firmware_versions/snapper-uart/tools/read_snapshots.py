#!/usr/bin/env python3
"""
Read and decode the binary frames sent by the snapper-uart firmware
(firmware_versions/snapper-uart) over its software UART (115200 8N1), and
assemble them into a snapshot-gnss-algorithms-compatible dataset.

Connect a USB-to-UART adapter's RX pin to the device's GPIO1 pin and its
GND to the device's GND (there is no TX/RX pair on the device side, it is
transmit-only), then run e.g.:

    python3 read_snapshots.py /dev/cu.usbserial-XXXX

On macOS, use the /dev/cu.* device, not /dev/tty.* : this is a one-way
link with no RTS/CTS/DCD lines connected, and the tty.* node's blocking
behaviour is tied to modem control signals that will never appear here.

Start this script *before* powering on or resetting the device: the info
frame is only sent once, right at boot, with no way to request it again.

Each captured snapshot is saved as "<out-dir>/YYYYMMDD_hhmmss.bin", the
exact filename convention snapshot-gnss-algorithms' main.py expects (it
parses the capture timestamp straight out of the filename). At startup,
any existing .bin/meta.json/.rnx files in out-dir are removed first, so
each run starts a fresh dataset. On Ctrl+C, a fresh merged broadcast
ephemeris file for the current UTC day is fetched (see
fetch_ephemeris.py) and a meta.json is written listing every captured
snapshot with its timestamp, matching dataset A's schema (main.py itself
does not read meta.json - it is for your own bookkeeping).

See the "Software UART Firmware" section of the top-level README.md for
the full wire format this mirrors.
"""

import argparse
import datetime
import json
import struct
import sys
import time
from pathlib import Path

import serial

from fetch_ephemeris import fetch_and_reorder_ephemeris, EphemerisFetchError

# firmware_versions/snapper-uart/tools/ -> snapshot_gnss/ -> sibling repo's data/Z
DEFAULT_DATASET_DIR = (
    Path(__file__).resolve().parents[4]
    / "snapshot-gnss-algorithms" / "data" / "Z"
)

SYNC_WORD = bytes([0xAA, 0x55, 0xAA, 0x55])

FRAME_INFO = 0x01
FRAME_SNAPSHOT = 0x02

CRC_POLY = 0x1021

# Struct formats for everything after the 4-byte sync word. All fields are
# little-endian, the Cortex-M0+'s native byte order.

INFO_FORMAT = "<BQ3s32sIIIIH"
INFO_LENGTH = struct.calcsize(INFO_FORMAT)

SNAPSHOT_HEADER_FORMAT = "<BIHhHH"
SNAPSHOT_HEADER_LENGTH = struct.calcsize(SNAPSHOT_HEADER_FORMAT)

# RMU_RSTCAUSE bits (efm32hg_rmu.h), for decoding uartInfoFrame_t.resetCause

RESET_CAUSE_BITS = [
    (0, "PORST (power-on)"),
    (1, "BODUNREGRST (brown-out, unregulated domain)"),
    (2, "BODREGRST (brown-out, regulated domain)"),
    (3, "EXTRST (external reset pin)"),
    (4, "WDOGRST (watchdog)"),
    (5, "LOCKUPRST (CPU lockup)"),
    (6, "SYSREQRST (software reset)"),
    (7, "EM4RST (EM4 entry)"),
    (8, "EM4WURST (EM4 wake-up)"),
    (9, "BODAVDD0"),
    (10, "BODAVDD1"),
]


def decode_reset_cause(reset_cause):
    names = [name for bit, name in RESET_CAUSE_BITS if reset_cause & (1 << bit)]
    return ", ".join(names) if names else f"unknown (0x{reset_cause:08X})"


def _crc_step(crc, bit):
    """Mirrors updateCRC() in firmware_versions/snapper-uart/src/main.c."""

    xor = (crc >> 15) & 1
    crc = (crc << 1) & 0xFFFF
    if bit:
        crc += 1
    if xor:
        crc ^= CRC_POLY
    return crc


def crc_update(crc, data):
    """Mirrors crcUpdateBytes() in main.c: feed data bit by bit, MSB first."""

    for byte in data:
        for mask in (0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01):
            crc = _crc_step(crc, byte & mask)
    return crc


def crc_finalize(crc):
    """Mirrors crcFinalize() in main.c: flush with 16 zero bits."""

    for _ in range(16):
        crc = _crc_step(crc, 0)
    return crc


class FrameError(Exception):
    pass


def read_exact(ser, length):
    buf = bytearray()
    while len(buf) < length:
        chunk = ser.read(length - len(buf))
        if not chunk:
            raise FrameError(f"timed out waiting for {length - len(buf)} more byte(s)")
        buf += chunk
    return bytes(buf)


def find_sync_word(ser):
    window = bytearray()
    while True:
        byte = ser.read(1)
        if not byte:
            raise FrameError("timed out waiting for sync word")
        window += byte
        if len(window) > len(SYNC_WORD):
            del window[0]
        if bytes(window) == SYNC_WORD:
            return


def read_info_frame(ser, frame_type):
    body = bytes([frame_type]) + read_exact(ser, INFO_LENGTH - 1)

    (_, device_id, fw_version, fw_description, measurement_interval,
     start_time, end_time, reset_cause, crc_received) = struct.unpack(INFO_FORMAT, body)

    crc_calculated = crc_finalize(crc_update(0, body[:-2]))  # everything but the trailing crc field

    return {
        "type": "info",
        "deviceID": device_id,
        "firmwareVersion": ".".join(str(b) for b in fw_version),
        "firmwareDescription": fw_description.split(b"\x00", 1)[0].decode("ascii", "replace"),
        "measurementInterval": measurement_interval,
        "startTime": start_time,
        "endTime": end_time,
        "resetCause": reset_cause,
        "crcOk": crc_calculated == crc_received,
    }


def read_snapshot_frame(ser, frame_type):
    header_bytes = bytes([frame_type]) + read_exact(ser, SNAPSHOT_HEADER_LENGTH - 1)

    (_, time_, ticks, temperature,
     battery_voltage, snapshot_length) = struct.unpack(SNAPSHOT_HEADER_FORMAT, header_bytes)

    payload = read_exact(ser, snapshot_length)
    crc_received = struct.unpack("<H", read_exact(ser, 2))[0]

    crc_calculated = crc_finalize(crc_update(crc_update(0, header_bytes), payload))

    return {
        "type": "snapshot",
        "time": time_,
        "ticks": ticks,
        "temperatureC": temperature / 10.0,
        "batteryVoltageV": battery_voltage / 100.0,
        "length": snapshot_length,
        "payload": payload,
        "crcOk": crc_calculated == crc_received,
    }


def read_frame(ser):
    find_sync_word(ser)

    frame_type = read_exact(ser, 1)[0]

    if frame_type == FRAME_INFO:
        return read_info_frame(ser, frame_type)

    if frame_type == FRAME_SNAPSHOT:
        return read_snapshot_frame(ser, frame_type)

    raise FrameError(f"unknown frame type 0x{frame_type:02X}")


def format_timestamp(unix_time):
    return datetime.datetime.fromtimestamp(unix_time, datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")


def dataset_filename(unix_time):
    """The "YYYYMMDD_hhmmss.bin" convention snapshot-gnss-algorithms' main.py
    expects; it parses the capture time straight out of this exact filename
    format, not from any file content."""

    return datetime.datetime.fromtimestamp(unix_time, datetime.timezone.utc).strftime("%Y%m%d_%H%M%S") + ".bin"


def handle_frame(frame, out_dir):
    if frame["type"] == "info":
        print(
            f"[info] device 0x{frame['deviceID']:016X}  "
            f"firmware {frame['firmwareDescription']} v{frame['firmwareVersion']}  "
            f"interval={frame['measurementInterval']}s  "
            f"start={format_timestamp(frame['startTime'])}  "
            f"end={format_timestamp(frame['endTime']) if frame['endTime'] != 0x7FFFFFFF else 'never'}  "
            f"resetCause={decode_reset_cause(frame['resetCause'])}"
        )

    elif frame["type"] == "snapshot":
        print(
            f"[snapshot] {format_timestamp(frame['time'])} (+{frame['ticks']}/1024 s)  "
            f"{frame['temperatureC']:.1f} C  "
            f"{frame['batteryVoltageV']:.2f} V  "
            f"{frame['length']} bytes"
        )

        filename = out_dir / dataset_filename(frame["time"])
        filename.write_bytes(frame["payload"])


def listen(port, baud, timeout, out_dir):
    with serial.Serial(port, baud, timeout=timeout) as ser:
        print(f"Listening on {port} at {baud} baud...")

        while True:
            try:
                frame = read_frame(ser)
            except FrameError as error:
                print(f"[warning] {error}, resyncing...", file=sys.stderr)
                continue

            if not frame["crcOk"]:
                print("[warning] CRC mismatch, discarding frame", file=sys.stderr)
                continue

            handle_frame(frame, out_dir)


def cleanup_dataset_dir(out_dir):
    removed = []
    for pattern in ("*.bin", "meta.json", "*.rnx"):
        for path in out_dir.glob(pattern):
            path.unlink()
            removed.append(path.name)
    if removed:
        print(f"[info] removed {len(removed)} old file(s) from {out_dir}", file=sys.stderr)


def write_meta_json(out_dir, latitude, longitude, intermediate_frequency):
    """Write a meta.json matching dataset A's schema. Note that main.py
    itself never reads this file - it derives everything (ground truth,
    per-file timestamps) from its own hardcoded dictionaries plus the
    filenames themselves. This is purely for your own bookkeeping."""

    bin_files = sorted(out_dir.glob("*.bin"))

    files = []
    timestamps = []
    for path in bin_files:
        capture_time = datetime.datetime.strptime(
            path.stem, "%Y%m%d_%H%M%S"
        ).replace(tzinfo=datetime.timezone.utc)
        files.append(path.name)
        timestamps.append(capture_time.strftime("%Y-%m-%dT%H:%M:%S.000Z"))

    meta = {
        "latitude": latitude,
        "longitude": longitude,
        "intermediate_frequency": intermediate_frequency,
        "file": files,
        "timestamp": timestamps,
    }

    meta_path = out_dir / "meta.json"
    meta_path.write_text(json.dumps(meta, indent=4))
    print(f"[info] wrote {meta_path} ({len(files)} snapshot(s))", file=sys.stderr)


def finalize_dataset(out_dir, latitude, longitude, intermediate_frequency):
    print("[info] fetching current ephemeris...", file=sys.stderr)
    try:
        destination = fetch_and_reorder_ephemeris(out_dir)
        print(f"[info] wrote {destination}", file=sys.stderr)
    except (EphemerisFetchError, OSError) as error:
        print(f"[warning] failed to fetch ephemeris: {error}", file=sys.stderr)

    write_meta_json(out_dir, latitude, longitude, intermediate_frequency)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="Serial port, e.g. /dev/cu.usbserial-XXXX or COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for each expected byte")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_DATASET_DIR,
                         help=f"Dataset directory to populate (default: {DEFAULT_DATASET_DIR})")
    parser.add_argument("--latitude", type=float, default=52.27614766579258,
                         help="Ground-truth latitude to record in meta.json")
    parser.add_argument("--longitude", type=float, default=10.541453358315112,
                         help="Ground-truth longitude to record in meta.json")
    parser.add_argument("--intermediate-frequency", type=float, default=4092000.0,
                         help="Receiver intermediate frequency [Hz] to record in meta.json")
    parser.add_argument("--reconnect-delay", type=float, default=2.0,
                         help="Seconds to wait before reopening the port after a serial error")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    cleanup_dataset_dir(args.out_dir)

    try:
        # A bit-banged, TX-only link on an unpowered/replugged/contested
        # USB-serial adapter can drop out at any time; reopen the port
        # instead of crashing.
        while True:
            try:
                listen(args.port, args.baud, args.timeout, args.out_dir)
            except serial.SerialException as error:
                print(f"[warning] serial error: {error}", file=sys.stderr)
                print(f"[warning] reconnecting in {args.reconnect_delay:.0f}s...", file=sys.stderr)
                time.sleep(args.reconnect_delay)
    except KeyboardInterrupt:
        print("\n[info] stopped", file=sys.stderr)
        finalize_dataset(args.out_dir, args.latitude, args.longitude, args.intermediate_frequency)


if __name__ == "__main__":
    main()
