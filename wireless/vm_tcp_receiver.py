#!/usr/bin/env python3
"""TCP receiver for the nRF9151 cellular uplink.

Parses the SnapperGPS frame format sent by cellular_uplink.c (mirrors
firmware_versions/snapper-uart/tools/read_snapshots.py's wire-format
parsing/CRC bit for bit) and organizes captures into per-session folders:

    <out-dir>/experiment_<YYYYMMDD_HHMMSS>/<YYYYMMDD_HHMMSS>.bin

- The session folder name is the wall-clock arrival time of that session's
  first frame.
- Each snapshot's .bin filename is the device's own capture timestamp, in
  the same "YYYYMMDD_hhmmss.bin" convention snapshot-gnss-algorithms' main.py
  expects (see read_snapshots.py).

Session boundaries: there is no explicit START/STOP marker on the wire
today - SnapperGPS's INFO frame fires once at its own boot, not per
Button-1 press, and each frame arrives as its own separate TCP connection
(cellular_uplink.c does one connect/send/close per frame, no persistent
session-level connection). Sessions are inferred from a wall-clock gap
between frame arrivals instead: if more than --session-gap seconds pass
with nothing received, the next frame starts a new experiment_* folder.
This is a heuristic, not a guarantee - a single very slow retry could
split what was really one session, and the INFO frame at each device boot
will typically produce its own small (often empty) session folder before
the first real Button-1 press.

Unlike the previous version of this script, received frames are NOT
echoed back - cellular_uplink.c never reads a response (connect/send/close
only), so nothing on the device side depends on it. slm_tcp_test.py did
rely on the echo for its own check; point that at a plain nc/socat echo
listener instead if you still need it, not this script.

Each session folder also gets a meta.json (latitude/longitude/
intermediate_frequency left at their tool-wide default values - these
aren't derived from anything the device sends, only file/timestamp reflect
the session's actual snapshots) and a copy of the current day's merged
broadcast ephemeris (BRDC00IGS_R_<YYYYDDD>0000_01D_MN.rnx) - see
"Ephemeris fetching" below - matching the layout snapshot-gnss-algorithms'
main.py expects (see data/L for a reference example).

Usage (run this on the VM):
    python3 vm_tcp_receiver.py [port] [--out-dir DIR] [--session-gap SECONDS]
        [--latitude DEG] [--longitude DEG] [--intermediate-frequency HZ]

No dependencies beyond the Python standard library.
"""
import argparse
import datetime
import gzip
import json
import re
import shutil
import socket
import struct
import sys
import urllib.error
import urllib.request
from pathlib import Path

# Ground-truth defaults recorded in meta.json - not derived from the device,
# just carried through unchanged. Matches read_snapshots.py's own defaults.
DEFAULT_LATITUDE = 52.27614766579258
DEFAULT_LONGITUDE = 10.541453358315112
DEFAULT_INTERMEDIATE_FREQUENCY = 4092000.0

SYNC_WORD = bytes([0xAA, 0x55, 0xAA, 0x55])

FRAME_INFO = 0x01
FRAME_SNAPSHOT = 0x02

CRC_POLY = 0x1021

# Struct formats for everything after the 4-byte sync word - mirrors
# read_snapshots.py exactly. All fields little-endian.
INFO_FORMAT = "<BQ3s32sIIIIH"
INFO_LENGTH = struct.calcsize(INFO_FORMAT)

SNAPSHOT_HEADER_FORMAT = "<BIHhHH"
SNAPSHOT_HEADER_LENGTH = struct.calcsize(SNAPSHOT_HEADER_FORMAT)


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
    """Mirrors crcUpdateBytes(): feed data bit by bit, MSB first."""
    for byte in data:
        for mask in (0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01):
            crc = _crc_step(crc, byte & mask)
    return crc


def crc_finalize(crc):
    """Mirrors crcFinalize(): flush with 16 zero bits."""
    for _ in range(16):
        crc = _crc_step(crc, 0)
    return crc


class FrameError(Exception):
    pass


def parse_frame(data):
    """Parses exactly one frame starting at data[0:4] == SYNC_WORD.

    Returns (frame_dict, bytes_consumed). Raises FrameError if data is too
    short to contain a complete frame yet, or the frame type is unknown -
    callers should treat "too short" the same as any other FrameError
    here, since a whole TCP connection's bytes are parsed at once (there's
    no byte-by-byte streaming state machine needed for this transport).
    """
    if len(data) < 5 or data[:4] != SYNC_WORD:
        raise FrameError("missing/misaligned sync word")

    frame_type = data[4]

    if frame_type == FRAME_INFO:
        if len(data) < 4 + INFO_LENGTH:
            raise FrameError("info frame truncated")
        body = data[4:4 + INFO_LENGTH]
        (_, device_id, fw_version, fw_description, measurement_interval,
         start_time, end_time, reset_cause, crc_received) = struct.unpack(INFO_FORMAT, body)
        crc_calculated = crc_finalize(crc_update(0, body[:-2]))
        frame = {
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
        return frame, 4 + INFO_LENGTH

    if frame_type == FRAME_SNAPSHOT:
        if len(data) < 4 + SNAPSHOT_HEADER_LENGTH:
            raise FrameError("snapshot header truncated")
        header_bytes = data[4:4 + SNAPSHOT_HEADER_LENGTH]
        (_, time_, ticks, temperature, battery_voltage,
         snapshot_length) = struct.unpack(SNAPSHOT_HEADER_FORMAT, header_bytes)
        total_len = 4 + SNAPSHOT_HEADER_LENGTH + snapshot_length + 2
        if len(data) < total_len:
            raise FrameError("snapshot payload truncated")
        payload = bytes(data[4 + SNAPSHOT_HEADER_LENGTH: total_len - 2])
        crc_received = struct.unpack("<H", data[total_len - 2: total_len])[0]
        crc_calculated = crc_finalize(crc_update(crc_update(0, header_bytes), payload))
        frame = {
            "type": "snapshot",
            "time": time_,
            "ticks": ticks,
            "temperatureC": temperature / 10.0,
            "batteryVoltageV": battery_voltage / 100.0,
            "length": snapshot_length,
            "payload": payload,
            "crcOk": crc_calculated == crc_received,
        }
        return frame, total_len

    raise FrameError(f"unknown frame type 0x{frame_type:02X}")


def format_timestamp(unix_time):
    return datetime.datetime.fromtimestamp(unix_time, datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")


def dataset_filename(unix_time):
    """The "YYYYMMDD_hhmmss.bin" convention snapshot-gnss-algorithms' main.py
    expects; it parses the capture time straight out of this exact filename
    format, not from any file content. Matches read_snapshots.py exactly."""
    return datetime.datetime.fromtimestamp(unix_time, datetime.timezone.utc).strftime("%Y%m%d_%H%M%S") + ".bin"


def write_meta_json(session_dir, latitude, longitude, intermediate_frequency):
    """Write/overwrite session_dir/meta.json listing every .bin currently in
    that folder, matching dataset A's schema. Called after every snapshot is
    written (not just once at the end) so a killed/crashed process still
    leaves a consistent meta.json behind - this receiver is a long-running
    server, not a one-shot script with a clean Ctrl+C exit to hook into like
    read_snapshots.py has. Note main.py itself doesn't read meta.json - it's
    for your own bookkeeping, same as in read_snapshots.py."""

    bin_files = sorted(session_dir.glob("*.bin"))

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

    (session_dir / "meta.json").write_text(json.dumps(meta, indent=4))


# --- Ephemeris fetching -----------------------------------------------------
#
# Ported from firmware_versions/snapper-uart/tools/fetch_ephemeris.py - keep
# the two in sync by hand if either changes; not shared as an import so this
# script stays a single self-contained file to `scp` (matching its existing
# "no dependencies beyond the standard library" design).
#
# Source is BKG (igs.bkg.bund.de), not CDDIS: CDDIS requires a NASA
# Earthdata login, which this unattended VM service has no way to prompt
# for or safely store credentials for. BKG mirrors the identical merged
# multi-GNSS BRDC product with anonymous access - same content, just
# reachable without a login. The only difference from a raw BKG download is
# that snapshot-gnss-algorithms' preprocess_rinex() needs GPS(G), then
# Galileo(E), then BeiDou(C) records in that order, while BKG's file has
# them sorted alphabetically by constellation letter - reorder_rinex_text()
# below fixes that, then the result is saved under the BRDC00IGS_R_* name
# main.py's glob expects (matching what CDDIS's own filename would be).

BKG_BASE_URL = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC"

# RINEX 3 nav message record-start lines: one uppercase constellation
# letter, a 2-digit satellite number, then a space.
RECORD_START = re.compile(r"^[A-Z]\d{2} ")

REQUIRED_GNSS_ORDER = ["G", "E", "C"]


class EphemerisFetchError(Exception):
    pass


def _bkg_url(for_date):
    year = for_date.year
    day_of_year = for_date.timetuple().tm_yday
    return (
        f"{BKG_BASE_URL}/{year:04d}/{day_of_year:03d}/"
        f"BRDC00WRD_R_{year:04d}{day_of_year:03d}0000_01D_MN.rnx.gz"
    )


def _ephemeris_filename(for_date):
    """The BRDC00IGS_R_<YYYYDDD>0000_01D_MN.rnx name main.py's glob expects
    - matches the CDDIS naming convention even though the bytes come from
    BKG's mirror instead (see module note above)."""
    year = for_date.year
    day_of_year = for_date.timetuple().tm_yday
    return f"BRDC00IGS_R_{year:04d}{day_of_year:03d}0000_01D_MN.rnx"


def _split_into_records(body_lines):
    starts = [i for i, line in enumerate(body_lines) if RECORD_START.match(line)]
    records = []
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(body_lines)
        record_lines = body_lines[start:end]
        records.append((record_lines[0][0], record_lines))
    return records


def _reorder_rinex_text(text):
    lines = text.splitlines(keepends=True)

    header_end = next((i for i, line in enumerate(lines) if "END OF HEADER" in line), None)
    if header_end is None:
        raise EphemerisFetchError("Downloaded file has no RINEX header (END OF HEADER not found)")

    header = lines[:header_end + 1]
    body = lines[header_end + 1:]

    records = _split_into_records(body)

    records_by_gnss = {}
    order_seen = []
    for gnss, record_lines in records:
        records_by_gnss.setdefault(gnss, []).append(record_lines)
        if gnss not in order_seen:
            order_seen.append(gnss)

    missing = [g for g in REQUIRED_GNSS_ORDER if g not in records_by_gnss]
    if missing:
        raise EphemerisFetchError(f"Downloaded RINEX file has no records for required GNSS {missing}")

    reordered = list(header)
    for gnss in REQUIRED_GNSS_ORDER:
        for record_lines in records_by_gnss[gnss]:
            reordered.extend(record_lines)
    for gnss in order_seen:
        if gnss in REQUIRED_GNSS_ORDER:
            continue
        for record_lines in records_by_gnss[gnss]:
            reordered.extend(record_lines)

    return "".join(reordered)


def fetch_and_reorder_ephemeris(destination_dir, for_date, timeout=30):
    """Download, decompress, and reorder the BRDC file for `for_date` (a
    datetime.date, UTC) into destination_dir, named as main.py expects.
    Returns the written Path. Raises EphemerisFetchError or a urllib error
    on failure (e.g. today's file not published yet)."""

    with urllib.request.urlopen(_bkg_url(for_date), timeout=timeout) as response:
        compressed = response.read()

    text = gzip.decompress(compressed).decode("ascii", "replace")
    reordered_text = _reorder_rinex_text(text)

    destination_dir.mkdir(parents=True, exist_ok=True)
    destination = destination_dir / _ephemeris_filename(for_date)
    destination.write_text(reordered_text)

    return destination


def get_or_fetch_daily_ephemeris(cache_dir):
    """Returns a Path to today's (UTC) ephemeris file, downloading into
    cache_dir if not already cached there. Falls back to yesterday's date
    if today's isn't published yet (common near UTC midnight - the source
    typically lags real time by a few hours). Returns None (logging a
    warning, not raising) if both attempts fail - a missing ephemeris
    shouldn't stop data collection, it can be backfilled later."""

    for days_back in (0, 1):
        for_date = datetime.datetime.now(datetime.timezone.utc).date() - datetime.timedelta(days=days_back)
        cached = cache_dir / _ephemeris_filename(for_date)

        if cached.exists():
            return cached

        try:
            return fetch_and_reorder_ephemeris(cache_dir, for_date)
        except (EphemerisFetchError, urllib.error.URLError, OSError) as error:
            print(f"[warning] ephemeris fetch failed for {for_date.isoformat()}: {error!r}",
                  file=sys.stderr)

    print("[warning] no ephemeris available (today's and yesterday's both failed) - "
          "session folders will be missing their .rnx file", file=sys.stderr)
    return None


class SessionWriter:
    """Groups incoming frames into experiment_<arrival-timestamp> folders
    under root_dir, starting a new one whenever more than gap_seconds
    elapses between consecutive frame *arrivals* (wall-clock, not the
    device's own capture timestamp - a long cellular retry delay means
    "nothing arrived for a while" is the more honest real-world signal
    than the device's own clock, which free-runs independently)."""

    def __init__(self, root_dir, gap_seconds, latitude, longitude, intermediate_frequency):
        self.root_dir = root_dir
        self.gap_seconds = gap_seconds
        self.latitude = latitude
        self.longitude = longitude
        self.intermediate_frequency = intermediate_frequency
        self.ephemeris_cache_dir = root_dir / ".ephemeris_cache"
        self.session_dir = None
        self.last_arrival = None
        self.snapshot_count = 0

    def _start_new_session(self, now):
        name = "experiment_" + now.strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.root_dir / name
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.snapshot_count = 0
        print(f"[session] new session: {self.session_dir}", flush=True)

        ephemeris = get_or_fetch_daily_ephemeris(self.ephemeris_cache_dir)
        if ephemeris is not None:
            shutil.copy(ephemeris, self.session_dir / ephemeris.name)
            print(f"[session] copied {ephemeris.name} into {self.session_dir.name}", flush=True)

    def handle_frame(self, frame):
        now = datetime.datetime.now(datetime.timezone.utc)

        if (self.session_dir is None or self.last_arrival is None
                or (now - self.last_arrival).total_seconds() > self.gap_seconds):
            self._start_new_session(now)

        self.last_arrival = now

        if frame["type"] == "info":
            print(
                f"[info] device 0x{frame['deviceID']:016X}  "
                f"firmware {frame['firmwareDescription']} v{frame['firmwareVersion']}  "
                f"interval={frame['measurementInterval']}s  "
                f"crcOk={frame['crcOk']}",
                flush=True,
            )
            return

        filename = self.session_dir / dataset_filename(frame["time"])
        filename.write_bytes(frame["payload"])
        self.snapshot_count += 1
        print(
            f"[snapshot] {format_timestamp(frame['time'])} (+{frame['ticks']}/1024 s)  "
            f"{frame['temperatureC']:.1f} C  {frame['batteryVoltageV']:.2f} V  "
            f"{frame['length']} bytes -> {filename.relative_to(self.root_dir)}  "
            f"(session total: {self.snapshot_count})",
            flush=True,
        )

        write_meta_json(self.session_dir, self.latitude, self.longitude, self.intermediate_frequency)


def handle_connection(conn, addr, writer):
    ts = datetime.datetime.now().isoformat()
    print(f"[{ts}] connection from {addr}", flush=True)

    data = bytearray()
    try:
        while True:
            chunk = conn.recv(8192)
            if not chunk:
                break
            data += chunk
    except OSError as error:
        print(f"[{datetime.datetime.now().isoformat()}] connection error: {error!r}", flush=True)

    offset = 0
    while offset < len(data):
        idx = data.find(SYNC_WORD, offset)
        if idx == -1:
            if len(data) - offset > 0:
                print(f"[warning] {len(data) - offset} trailing byte(s) with no sync word, "
                      f"discarding", file=sys.stderr)
            break
        if idx != offset:
            print(f"[warning] {idx - offset} byte(s) before sync word, skipping", file=sys.stderr)

        try:
            frame, consumed = parse_frame(bytes(data[idx:]))
        except FrameError as error:
            print(f"[warning] {error}, resyncing", file=sys.stderr)
            offset = idx + 1
            continue

        if not frame["crcOk"]:
            print("[warning] CRC mismatch, discarding frame", file=sys.stderr)
            offset = idx + consumed
            continue

        writer.handle_frame(frame)
        offset = idx + consumed

    print(f"[{datetime.datetime.now().isoformat()}] connection closed ({len(data)} bytes total)",
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", type=int, nargs="?", default=5000)
    parser.add_argument("--out-dir", type=Path, default=Path.home() / "snapperGPS",
                         help="Root directory for experiment_* session folders "
                              "(default: ~/snapperGPS)")
    parser.add_argument("--session-gap", type=float, default=90.0,
                         help="Seconds of silence before the next frame starts a new "
                              "experiment_* session folder (default: 90s)")
    parser.add_argument("--latitude", type=float, default=DEFAULT_LATITUDE,
                         help="Ground-truth latitude to record in each session's meta.json")
    parser.add_argument("--longitude", type=float, default=DEFAULT_LONGITUDE,
                         help="Ground-truth longitude to record in each session's meta.json")
    parser.add_argument("--intermediate-frequency", type=float, default=DEFAULT_INTERMEDIATE_FREQUENCY,
                         help="Receiver intermediate frequency [Hz] to record in each "
                              "session's meta.json")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    writer = SessionWriter(args.out_dir, args.session_gap, args.latitude, args.longitude,
                            args.intermediate_frequency)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"Listening on 0.0.0.0:{args.port}, writing sessions under {args.out_dir} ...",
          flush=True)

    while True:
        conn, addr = srv.accept()
        with conn:
            handle_connection(conn, addr, writer)


if __name__ == "__main__":
    main()
