"""
Fetch the current day's merged multi-GNSS broadcast ephemeris (RINEX 3 NAV,
the "BRDC" product) and reorder it for snapshot-gnss-algorithms.

snapshot-gnss-algorithms' rinex_preprocessor.preprocess_rinex() scans a RINEX
3 nav file forward-only, looking first for a contiguous run of GPS (G)
records, then continuing forward for a contiguous run of Galileo (E)
records, then BeiDou (C) - it raises if they are not present in that
relative order. CDDIS (the source the algorithms repo points to) requires a
NASA Earthdata login. BKG (igs.bkg.bund.de) mirrors the same merged product
with anonymous access and low latency, but its records are sorted
alphabetically by constellation letter (C, E, G, J, R, S, ...) rather than
G-E-C - which breaks preprocess_rinex's forward scan as-is. This module
downloads from BKG and rewrites the file with G, then E, then C, then
everything else appended (inert for preprocess_rinex, which stops reading
once it has consumed the C block, but keeps the file a faithful, complete
copy of the original beyond just those three systems).
"""

import gzip
import re
import urllib.request
from datetime import date, timezone, datetime

BKG_BASE_URL = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC"

# RINEX 3 nav message record-start lines: one uppercase constellation
# letter, a 2-digit satellite number, then a space.
RECORD_START = re.compile(r"^[A-Z]\d{2} ")

# The order preprocess_rinex requires; anything else found in the file is
# appended afterwards, unchanged.
REQUIRED_GNSS_ORDER = ["G", "E", "C"]


class EphemerisFetchError(Exception):
    pass


def bkg_url(for_date):
    """URL of the BKG merged BRDC file covering the UTC day `for_date`."""

    year = for_date.year
    day_of_year = for_date.timetuple().tm_yday
    return (
        f"{BKG_BASE_URL}/{year:04d}/{day_of_year:03d}/"
        f"BRDC00WRD_R_{year:04d}{day_of_year:03d}0000_01D_MN.rnx.gz"
    )


def target_filename(for_date):
    """Filename main.py's glob (BRDC00IGS_R_*_01D_MN.rnx) expects."""

    year = for_date.year
    day_of_year = for_date.timetuple().tm_yday
    return f"BRDC00IGS_R_{year:04d}{day_of_year:03d}0000_01D_MN.rnx"


def _split_into_records(body_lines):
    """Split nav message body lines into (gnss_letter, lines) records.

    Record boundaries are found by locating every record-start line, rather
    than assuming a fixed line count per record: RINEX 3 nav records are 8
    lines for GPS/Galileo/BeiDou/QZSS/NavIC but fewer for GLONASS/SBAS, and
    the exact count is not worth relying on.
    """

    starts = [i for i, line in enumerate(body_lines) if RECORD_START.match(line)]

    records = []
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(body_lines)
        record_lines = body_lines[start:end]
        records.append((record_lines[0][0], record_lines))

    return records


def reorder_rinex_text(text):
    """Reorder a RINEX 3 mixed nav file's body into G, E, C, then the rest."""

    lines = text.splitlines(keepends=True)

    header_end = next(
        (i for i, line in enumerate(lines) if "END OF HEADER" in line), None
    )
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
        raise EphemerisFetchError(
            f"Downloaded RINEX file has no records for required GNSS {missing}"
        )

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


def fetch_and_reorder_ephemeris(destination_dir, for_date=None, timeout=30):
    """Download, decompress, and reorder the BRDC file for `for_date` (a
    datetime.date, UTC; defaults to today) into destination_dir, named as
    main.py expects. Returns the written Path. Raises EphemerisFetchError
    or a urllib error on failure (e.g. today's file not published yet)."""

    if for_date is None:
        for_date = datetime.now(timezone.utc).date()

    url = bkg_url(for_date)

    with urllib.request.urlopen(url, timeout=timeout) as response:
        compressed = response.read()

    text = gzip.decompress(compressed).decode("ascii", "replace")

    reordered_text = reorder_rinex_text(text)

    destination_dir.mkdir(parents=True, exist_ok=True)
    destination = destination_dir / target_filename(for_date)
    destination.write_text(reordered_text)

    return destination
