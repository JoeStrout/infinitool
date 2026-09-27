#!/usr/bin/env python3
"""
One-stop management shell for an InfiniTime watch, over bleak.

Files, clock and firmware, in one connection. Runs anywhere bleak does: CoreBluetooth
on macOS, BlueZ on Linux, WinRT on Windows.

    $ ./infinitool.py
    Found InfiniTime (C3764D0D-...)
    InfiniTime 1.16.0  |  BLE FS v4  |  battery 87%
    infinitool> ls /images
    infinitool> cp !fuji.bin /images/fuji.bin
    infinitool> time set
    infinitool> notify "Test" "Hello from infinitool"
    infinitool> flash !../../build/output/pinetime-mcuboot-app-dfu-1.16.0.zip
    infinitool> exit

Any command can be run non-interactively with -c, which is repeatable:

    ./infinitool.py -c "cp !fuji.bin /images/fuji.bin" -c "ls /images"

Plain paths refer to the remote (device) filesystem. A local path is prefixed with
`!`, as in ftp/sftp where `!` escapes to the local machine:

    cp !./fuji.bin /images/fuji.bin      upload
    cp /fonts/teko.bin !teko.bin         download
    cp !big.bin /images/                 trailing slash keeps the local basename
    cp -u !watchfiles.zip /              unpack a zip into the matching directories

Services used, all served by the running firmware (no bootloader mode needed):

    adaf0100/adaf0200  InfiniTime file transfer   doc/BLEFS.md, FSService.cpp
    00001530-...       Nordic legacy DFU          DfuService.cpp (via dfu_bleak.py)
    0x1805 / 0x2a2b    Current Time Service       CurrentTimeService.cpp
    0x180a             Device Information         DeviceInformationService.cpp
    0x180f / 0x2a19    Battery level              BatteryInformationService.cpp
    0x1811 / 0x2a46    Alert Notification         AlertNotificationService.cpp
    0x1802 / 0x2a06    Immediate Alert            ImmediateAlertService.cpp
    0x180d / 0x2a37    Heart rate                 HeartRateService.cpp
    00030000-...       Motion (steps, accel)      MotionService.cpp
    00000000-...       Music                      MusicService.cpp
    00050001-...       Simple weather             SimpleWeatherService.cpp

The `flash` command needs dfu_bleak.py and unpacker.py, which sit beside this file and
are imported on demand; every other command depends on nothing outside this file.

Firmware quirks this works around -- all verified in the source, and all of them produce
misleading errors in other clients:

  - Success status is 0x01, not 0 (FSService.cpp:102, :246). Negative values are LittleFS
    error codes; see LFS_ERRORS below.
  - The WRITE header sets resp.status ONLY on success (FSService.cpp:172-176), and
    WRITE_DATA sets it ONLY on failure (FSService.cpp:185-198). WriteResponse has no
    default initialisers (FSService.h:120), so in each case the other path returns
    uninitialised stack memory. We therefore never trust status on data writes: uploads
    are tracked by our own byte count and then VERIFIED by asking for the file's size
    (a zero-length READ, which replies once with totallen) and comparing it.
  - Read and write chunks are not clamped to the connection MTU ("TODO add mtu somehow",
    FSService.cpp:111), so the client must size them or replies get truncated.
  - A directory listing ends with a terminator entry whose path_length is 0
    (FSService.cpp:292-296); it is not a file.
  - A listing sends every entry as its own notification, all from inside the GATT
    callback (FSService.cpp:257-290). The NimBLE host task is blocked meanwhile, so its
    buffers are not freed, and past roughly 27 entries the rest -- terminator included --
    are silently dropped. We stop once all `totalentries` have arrived, give up after a
    short gap, and let `ls` show what did arrive. rm -R deletes what arrived and lists
    again, until the directory is small enough to list whole.
  - Everything here is gated behind Settings -> "Firmware & files" on the watch. When it
    is Disabled, every request is refused and the version characteristic reads 0, not 4.
  - The Alert Notification write has a 3-byte header, but only byte 0 (the category) is
    ever read (AlertNotificationService.cpp:64); the "number of new alerts" byte the SIG
    spec defines is discarded. Of the text after it, at most 99 bytes survive: the copy
    length is min(packetLen + 1, 103) - 3 - 1 (AlertNotificationService.cpp:56-63).
  - Motion, heart rate and battery are the only sensor values BLE exposes, and all three
    are READ|NOTIFY (MotionService.cpp:37, HeartRateService.cpp:25). There is no write
    path to the step count -- it lives in MotionController, and nothing in the firmware
    lets a central set it -- so `info` reports these and no command sets them.
  - Music and navigation characteristics are flagged READ|WRITE, but OnCommand handles
    only the write op (MusicService.cpp:128), so reads come back empty rather than
    echoing what was sent. `music` therefore prints what it sent, not what is stored.
  - MusicService registers the track-length UUID twice (MusicService.cpp:84-91), so the
    watch really does advertise two characteristics with the same UUID and bleak will
    not resolve it by UUID at all; see Device._music_characteristic.
  - Reading the Current Time characteristic returns 10 bytes, but the firmware fills in
    only the first 8 (CurrentTimeService.cpp:57-67): dayofweek and reason are left as
    whatever was on the stack. We parse the date and time and ignore those two fields.
"""

import argparse
import asyncio
import datetime
import glob
import inspect
import json
import os
import readline
import shlex
import struct
import sys
import textwrap
import zipfile

try:
    from bleak import BleakClient, BleakScanner
    # BleakGATTProtocolError arrived in bleak 3.0, which is the floor this pins us to.
    from bleak.exc import BleakError, BleakGATTProtocolError, BleakGATTProtocolErrorCode
except ImportError:
    sys.exit("bleak 3.0 or newer is not installed. Run: pip install 'bleak>=3.0'")

# `flash` reuses dfu_bleak.py rather than reimplementing legacy DFU. That module and the
# unpacker.py it needs live beside this file, but are imported lazily, inside cmd_flash,
# so that every other command works from this file alone.
DFU_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))


UUID_VERSION = "adaf0100-4669-6c65-5472-616e73666572"
UUID_TRANSFER = "adaf0200-4669-6c65-5472-616e73666572"

# Standard SIG characteristics, served by DeviceInformationService, CurrentTimeService
# and BatteryInformationService respectively.
UUID_CURRENT_TIME = "00002a2b-0000-1000-8000-00805f9b34fb"
UUID_LOCAL_TIME = "00002a0f-0000-1000-8000-00805f9b34fb"
UUID_BATTERY_LEVEL = "00002a19-0000-1000-8000-00805f9b34fb"
UUID_MANUFACTURER = "00002a29-0000-1000-8000-00805f9b34fb"
UUID_MODEL_NUMBER = "00002a24-0000-1000-8000-00805f9b34fb"
UUID_SERIAL_NUMBER = "00002a25-0000-1000-8000-00805f9b34fb"
UUID_FW_REVISION = "00002a26-0000-1000-8000-00805f9b34fb"
UUID_HW_REVISION = "00002a27-0000-1000-8000-00805f9b34fb"
UUID_SW_REVISION = "00002a28-0000-1000-8000-00805f9b34fb"
UUID_NEW_ALERT = "00002a46-0000-1000-8000-00805f9b34fb"
UUID_ALERT_LEVEL = "00002a06-0000-1000-8000-00805f9b34fb"
UUID_HEART_RATE = "00002a37-0000-1000-8000-00805f9b34fb"


def infinitime_uuid(service, characteristic):
    """One of InfiniTime's own 128-bit UUIDs, 0000ssss-cccc pattern.

    Every custom service in the firmware is built from the same vendor base by
    substituting two bytes; see the CharUuid helper at the top of MusicService.cpp,
    MotionService.cpp and friends.
    """
    return f"{service:04x}{characteristic:04x}-78fc-48fe-8e23-433b3a1942d0"


UUID_STEP_COUNT = infinitime_uuid(0x0003, 0x0001)
UUID_MOTION_VALUES = infinitime_uuid(0x0003, 0x0002)
UUID_WEATHER_DATA = infinitime_uuid(0x0005, 0x0001)

# AlertNotificationService.h:38-49. Only Call is treated specially by the firmware --
# it raises the incoming-call screen; everything else becomes a plain SimpleAlert.
ANS_CATEGORIES = {
    "simple": 0x00,
    "email": 0x01,
    "news": 0x02,
    "call": 0x03,
    "missed-call": 0x04,
    "sms": 0x05,
    "voicemail": 0x06,
    "schedule": 0x07,
    "high-priority": 0x08,
    "im": 0x09,
}

# See the header note: the firmware copies at most this much of the text that follows
# the 3-byte header, title and NUL separator included.
ANS_MAX_TEXT = 99

# ImmediateAlertService::Levels, ImmediateAlertService.h. The watch turns each of these
# into a notification reading "Alert : None" / "Mild" / "High".
ALERT_LEVELS = {"none": 0x00, "mild": 0x01, "high": 0x02}

# MusicService.cpp:38-49, with the encoding each characteristic's write path expects
# (MusicService.cpp:127-180). Note the numbers are big-endian there, unlike every other
# multi-byte field in this firmware.
MUSIC_FIELDS = {
    "status": (infinitime_uuid(0x0000, 0x0002), "flag"),
    "artist": (infinitime_uuid(0x0000, 0x0003), "str"),
    "track": (infinitime_uuid(0x0000, 0x0004), "str"),
    "album": (infinitime_uuid(0x0000, 0x0005), "str"),
    "position": (infinitime_uuid(0x0000, 0x0006), "u32"),
    "length": (infinitime_uuid(0x0000, 0x0007), "u32"),
    "number": (infinitime_uuid(0x0000, 0x0008), "u32"),
    "total": (infinitime_uuid(0x0000, 0x0009), "u32"),
    "speed": (infinitime_uuid(0x0000, 0x000A), "speed"),
    "repeat": (infinitime_uuid(0x0000, 0x000B), "flag"),
    "shuffle": (infinitime_uuid(0x0000, 0x000C), "flag"),
}

# MusicService.cpp:51. Longer strings are accepted but the tail is replaced with "...".
MUSIC_MAX_STRING = 40

# SimpleWeatherService::Icons, SimpleWeatherService.h:55-65.
WEATHER_ICONS = {
    "sun": 0,
    "clouds-sun": 1,
    "clouds": 2,
    "broken-clouds": 3,
    "heavy-shower": 4,
    "rain": 5,
    "thunderstorm": 6,
    "snow": 7,
    "smog": 8,
}

WEATHER_MAX_FORECAST_DAYS = 5      # SimpleWeatherService::MaxNbForecastDays
WEATHER_LOCATION_SIZE = 32         # SimpleWeatherService::Location, minus its terminator

# 1024 raw units = 1g: the BMA421 driver rescales to "binary milli-g" before the values
# reach MotionController (Bma421.cpp:118-123).
ACCEL_UNITS_PER_G = 1024

# CtsCurrentTimeData, CurrentTimeService.h:36-47. Ten bytes, little endian.
CTS_FORMAT = "<HBBBBBBBB"
# CtsLocalTimeData, CurrentTimeService.h:49-52. Both fields count quarter hours:
# DateTimeController.h:132 multiplies their sum by 15 * 60.
CTS_LOCAL_FORMAT = "<bb"
QUARTER_HOUR = 15 * 60

# Legacy DFU defaults, matching dfu_bleak.py's own argparse defaults.
DFU_CHUNK_SIZE = 20
DFU_PRN_INTERVAL = 10
DFU_TIMEOUT = 60.0

CMD_READ = 0x10
CMD_READ_DATA = 0x11
CMD_READ_PACING = 0x12
CMD_WRITE = 0x20
CMD_WRITE_PACING = 0x21
CMD_WRITE_DATA = 0x22
CMD_DELETE = 0x30
CMD_DELETE_STATUS = 0x31
CMD_MKDIR = 0x40
CMD_MKDIR_STATUS = 0x41
CMD_LISTDIR = 0x50
CMD_LISTDIR_ENTRY = 0x51

STATUS_OK = 0x01

# Claimed size of the write probe that `df` uses to read free space. Must exceed the
# 4 MB flash so the firmware's min() falls on the real figure, and must stay under
# INT32_MAX: the firmware stores it in an int (FSService.h:81).
FREESPACE_PROBE_SIZE = 0x7FFFFFFF

READ_RESPONSE_HEADER = 16       # command, status, pad, chunkoff, totallen, chunklen
WRITE_RESPONSE_HEADER = 20      # command, status, pad, offset, modTime, freespace
WRITE_DATA_HEADER = 12          # command, status, pad, offset, dataSize
LISTDIR_RESPONSE_HEADER = 28

# Listing entries come about 100 ms apart (the vTaskDelay(100) in FSService.cpp:288), so
# a gap this long after the first one means the rest were dropped, not delayed.
LISTDIR_GAP_TIMEOUT = 2.0

LFS_ERRORS = {
    0: "OK",
    -5: "IO — device operation failed",
    -84: "CORRUPT — filesystem is corrupted",
    -2: "NOENT — no such file or directory",
    -17: "EXIST — entry already exists",
    -20: "NOTDIR — entry is not a directory",
    -21: "ISDIR — entry is a directory",
    -39: "NOTEMPTY — directory is not empty",
    -9: "BADF — bad file number",
    -27: "FBIG — file is too large",
    -22: "INVAL — invalid parameter",
    -28: "NOSPC — no space left on device",
    -12: "NOMEM — no more memory available",
    -61: "NOATTR — no data/attr available",
    -36: "NAMETOOLONG — file name too long",
}

# Three distinct GATT codes that all mean the same thing here: the watch will not serve
# this characteristic to a central it has no valid bond with. InfiniTime requires pairing
# for the FS, DFU and CTS characteristics, so a mistyped pairing code -- or a half-finished
# bond still cached by the OS -- surfaces as one of these on the first read, not at connect
# time, because CoreBluetooth reports a connection long before any encryption is agreed.
PAIRING_ERROR_CODES = frozenset({
    BleakGATTProtocolErrorCode.INSUFFICIENT_AUTHENTICATION,
    BleakGATTProtocolErrorCode.INSUFFICIENT_AUTHORIZATION,
    BleakGATTProtocolErrorCode.INSUFFICIENT_ENCRYPTION,
})

# How to drop the stale bond, per platform. The pairing lives on the machine, not (only)
# on the watch, so re-running this tool alone will keep failing the same way.
FORGET_DEVICE = {
    "darwin": "System Settings -> Bluetooth, click the (i) beside InfiniTime, "
              "then 'Forget This Device'",
    "linux": "bluetoothctl remove ADDRESS",
    "win32": "Settings -> Bluetooth & devices -> the watch -> Remove device",
}

HELP_PREAMBLE = """\
Paths refer to the remote (device) filesystem unless prefixed with ! (local), as in ftp/sftp.
Run 'help COMMAND' for more about one command.
"""


class FsError(Exception):
    pass


class Listing(list):
    """A directory's entries, plus how many the watch dropped (0 when complete)."""

    def __init__(self, entries=(), missing=0):
        super().__init__(entries)
        self.missing = missing


def describe_ble_error(exc):
    """Explain a bleak exception in terms of what to do about it.

    Only the pairing failures get a real explanation; anything else falls back to bleak's
    own text, which is usually specific enough.
    """
    # BleakGATTProtocolError stringifies as its whole args tuple, code included, which
    # reads badly in a message. The last arg is the human-readable half.
    detail = exc.args[-1] if exc.args and isinstance(exc.args[-1], str) else str(exc)

    if isinstance(exc, BleakGATTProtocolError) and exc.code in PAIRING_ERROR_CODES:
        forget = FORGET_DEVICE.get(
            sys.platform, "remove the watch from this machine's Bluetooth settings"
        )
        return (
            f"the watch refused the request -- {detail}.\n"
            "  The Bluetooth pairing is missing or was not accepted. Usually the pairing code\n"
            "  was mistyped, and this machine now holds a bond the watch will not honour.\n"
            "  To fix it:\n"
            "    1. forget the watch on this machine:\n"
            f"{textwrap.fill(forget, width=88, initial_indent=' ' * 7, subsequent_indent=' ' * 7)}\n"
            "    2. run infinitool again, and enter the code the watch displays"
        )
    return f"Bluetooth error -- {detail}"


def command_help(handler):
    """Split a cmd_* docstring into (usage, summary, details).

    Every command handler documents itself in the same shape: the first line is the
    usage, the second is the one-line summary `help` lists, and anything after that is
    the detail `help COMMAND` adds. cleandoc, not dedent: the first line of a docstring
    carries no indentation, which would defeat dedent's common-prefix calculation.
    """
    lines = inspect.cleandoc(handler.__doc__ or "").split("\n")
    usage = lines[0]
    summary = lines[1] if len(lines) > 1 else ""
    return usage, summary, "\n".join(lines[2:]).strip("\n")


def describe_status(status):
    if status == STATUS_OK:
        return "OK"
    return LFS_ERRORS.get(status, f"unrecognised status {status}")


def is_local(path):
    return path.startswith("!")


def local_path(path):
    # A bare "!" means the current local directory, which makes `ls !` and
    # `cp /fonts/teko.bin !` do the obvious thing.
    return os.path.expanduser(path[1:]) or "."


def parse_options(args, known, command):
    """Parse a flat list of `--key value` pairs into a dict, rejecting anything else.

    Small enough not to want argparse, which would want to exit the process on a bad
    option rather than hand back an error the shell can print and carry on from.
    """
    options = {}
    rest = list(args)
    while rest:
        key = rest.pop(0)
        if key not in known:
            raise FsError(f"unknown option {key!r} for {command}; one of: "
                          f"{', '.join(sorted(known))}")
        if not rest:
            raise FsError(f"{key} needs a value")
        options[key] = rest.pop(0)
    return options


def parse_temperature(text):
    """A temperature in hundredths of a degree Celsius, which is the wire format.

    Bare numbers are Celsius; a C or F suffix says which, so `70F` works.
    """
    value = text.strip()
    scale = "C"
    if value and value[-1].upper() in ("C", "F"):
        value, scale = value[:-1], value[-1].upper()
    try:
        degrees = float(value)
    except ValueError:
        raise FsError(f"bad temperature {text!r}; expected a number, optionally with "
                      f"a C or F suffix")
    if scale == "F":
        degrees = (degrees - 32) * 5 / 9
    hundredths = round(degrees * 100)
    # int16 on the wire (SimpleWeatherService.cpp:76), so the range is +/-327.67 C.
    if not -32768 <= hundredths <= 32767:
        raise FsError(f"temperature {text!r} is outside the range the watch can store")
    return hundredths


def parse_weather_icon(name):
    icon = WEATHER_ICONS.get(name.strip().lower())
    if icon is None:
        raise FsError(f"unknown weather icon {name!r}; one of: {', '.join(WEATHER_ICONS)}")
    return icon


def parse_minutes(text):
    """An HH:MM local time as minutes into the day, which is what the wire format wants."""
    try:
        hours, minutes = text.split(":")
        total = int(hours) * 60 + int(minutes)
    except ValueError:
        raise FsError(f"bad time {text!r}; expected HH:MM")
    if not 0 <= total < 1440:
        raise FsError(f"time {text!r} is not within a single day")
    return total


def parse_music_value(field, text):
    """Turn one `music FIELD VALUE` argument into the type that field's writer wants."""
    kind = MUSIC_FIELDS[field][1]
    if kind == "str":
        return text
    if kind == "flag":
        lowered = text.strip().lower()
        if lowered in ("play", "playing", "on", "yes", "true", "1"):
            return True
        if lowered in ("pause", "paused", "stop", "off", "no", "false", "0"):
            return False
        raise FsError(f"bad value {text!r} for {field}; expected play/pause (or on/off)")
    if kind == "u32":
        try:
            value = int(text)
        except ValueError:
            raise FsError(f"bad value {text!r} for {field}; expected a whole number")
        if not 0 <= value <= 0xFFFFFFFF:
            raise FsError(f"{field} must be between 0 and {0xFFFFFFFF}")
        return value
    try:
        speed = float(text)
    except ValueError:
        raise FsError(f"bad value {text!r} for speed; expected a number such as 1.5")
    if not 0 <= speed * 100 <= 0xFFFFFFFF:
        raise FsError("speed is out of range")
    return speed


def human(size):
    for unit in ("B", "KiB", "MiB"):
        if size < 1024 or unit == "MiB":
            return f"{size:.0f} {unit}" if unit == "B" else f"{size / 1:.0f} {unit}"
        size /= 1024
    return f"{size} B"


class BleFs:
    def __init__(self, client, timeout, verbose):
        self.client = client
        self.timeout = timeout
        self.verbose = verbose
        self.notifications = asyncio.Queue()
        mtu = getattr(client, "mtu_size", 23) or 23
        self.mtu = mtu
        # The firmware does not clamp to the MTU, so we must (FSService.cpp:111).
        self.read_chunk = max(16, mtu - 3 - READ_RESPONSE_HEADER)
        self.write_chunk = max(16, mtu - 3 - WRITE_DATA_HEADER)

    def log(self, message):
        if self.verbose:
            print(f"  [fs] {message}", file=sys.stderr)

    def _on_notify(self, _characteristic, data):
        self.notifications.put_nowait(bytes(data))

    async def start(self):
        await self.client.start_notify(UUID_TRANSFER, self._on_notify)

    async def _response(self, timeout=None):
        timeout = self.timeout if timeout is None else timeout
        try:
            return await asyncio.wait_for(self.notifications.get(), timeout)
        except asyncio.TimeoutError:
            raise FsError(
                f"no response within {timeout}s — check Settings -> "
                "'Firmware & files' is Enabled on the watch"
            )

    async def _send(self, payload):
        # Every reply to the previous request has been consumed by now, so anything still
        # queued is a straggler (say, from a listing we gave up on) and would be misread.
        while not self.notifications.empty():
            self.log(f"discarding stale reply {self.notifications.get_nowait().hex()}")
        await self.client.write_gatt_char(UUID_TRANSFER, payload, response=True)

    async def version(self):
        raw = await self.client.read_gatt_char(UUID_VERSION)
        return struct.unpack("<H", raw[:2])[0]

    # -- listing -----------------------------------------------------------------

    async def listdir(self, path, allow_partial=False):
        """Entries of a watch directory, without . and ..

        Large directories lose entries in the firmware (see the notes at the top of this
        file). With allow_partial, that returns what arrived, with the number lost in
        .missing; otherwise it raises, for callers that would act wrongly on an
        incomplete list.
        """
        encoded = path.encode()
        await self._send(struct.pack("<BBH", CMD_LISTDIR, 0, len(encoded)) + encoded)

        entries = Listing()
        seen = set()
        total = None

        def incomplete():
            entries.missing = total - len(seen)
            if not allow_partial:
                raise FsError(
                    f"{path}: the watch dropped {entries.missing} of {total} listing "
                    "entries (a firmware limit on large directories)"
                )
            return entries

        while True:
            try:
                data = await self._response(None if total is None else LISTDIR_GAP_TIMEOUT)
            except FsError:
                if total is None:
                    raise
                return incomplete()
            if len(data) < LISTDIR_RESPONSE_HEADER:
                raise FsError(f"short listdir response: {data.hex()}")
            (command, status, path_len, entry, total,
             flags, _modtime, size) = struct.unpack_from("<BbHIIIQI", data, 0)
            if command != CMD_LISTDIR_ENTRY:
                raise FsError(f"unexpected reply 0x{command:02x} to listdir")
            if status != STATUS_OK:
                raise FsError(f"{path}: {describe_status(status)}")
            if total == 0:
                return entries
            name = data[LISTDIR_RESPONSE_HEADER:LISTDIR_RESPONSE_HEADER + path_len].decode(
                errors="replace"
            )
            # Terminator entry, not a file (FSService.cpp:292-296).
            if not path_len or entry >= total:
                return entries if len(seen) >= total else incomplete()
            if entry not in seen:
                seen.add(entry)
                if name not in (".", ".."):
                    entries.append({"name": name, "size": size, "is_dir": bool(flags & 1)})
            # Don't wait on the terminator: it is the likeliest entry to be dropped.
            if len(seen) >= total:
                return entries

    async def size_of(self, path):
        """Size of a file on the watch, or None if absent. Used to verify uploads.

        A READ asking for zero bytes gets exactly one reply, carrying the size in
        totallen, and the firmware opens and closes the file within that one request
        (FSService.cpp:90-123). Unlike re-listing the parent, that works however full the
        directory is. Files only: on a directory the firmware reads from a file it failed
        to open.
        """
        encoded = path.encode()
        await self._send(struct.pack("<BBHII", CMD_READ, 0, len(encoded), 0, 0) + encoded)
        data = await self._response()
        if len(data) < READ_RESPONSE_HEADER:
            raise FsError(f"short read response: {data.hex()}")
        command, status, _pad, _offset, totallen, _chunklen = struct.unpack_from(
            "<BbHIII", data, 0
        )
        if command != CMD_READ_DATA:
            raise FsError(f"unexpected reply 0x{command:02x} to read")
        if status == -2:
            return None
        if status != STATUS_OK:
            raise FsError(f"{path}: {describe_status(status)}")
        return totallen

    # -- reading -----------------------------------------------------------------

    async def read_file(self, path, on_progress=None):
        encoded = path.encode()
        await self._send(
            struct.pack("<BBHII", CMD_READ, 0, len(encoded), 0, self.read_chunk) + encoded
        )

        content = bytearray()
        while True:
            data = await self._response()
            if len(data) < READ_RESPONSE_HEADER:
                raise FsError(f"short read response: {data.hex()}")
            command, status, _pad, offset, totallen, chunklen = struct.unpack_from(
                "<BbHIII", data, 0
            )
            if command != CMD_READ_DATA:
                raise FsError(f"unexpected reply 0x{command:02x} to read")
            if status != STATUS_OK:
                raise FsError(f"{path}: {describe_status(status)}")

            chunk = data[READ_RESPONSE_HEADER:READ_RESPONSE_HEADER + chunklen]
            if len(chunk) != chunklen:
                raise FsError(
                    f"truncated chunk: header claimed {chunklen} bytes, got {len(chunk)}"
                )
            content.extend(chunk)
            if on_progress:
                on_progress(len(content), totallen)

            if len(content) >= totallen or chunklen == 0:
                break

            await self._send(
                struct.pack(
                    "<BBHII", CMD_READ_PACING, STATUS_OK, 0, len(content),
                    min(self.read_chunk, totallen - len(content)),
                )
            )
        return bytes(content)

    # -- writing -----------------------------------------------------------------

    async def write_file(self, path, content, on_progress=None):
        encoded = path.encode()
        header = struct.pack(
            "<BBHIQI", CMD_WRITE, 0, len(encoded), 0, 0, len(content)
        ) + encoded
        await self._send(header)

        data = await self._response()
        if len(data) < WRITE_RESPONSE_HEADER:
            raise FsError(f"short write response: {data.hex()}")
        command, status, _pad, _offset, _modtime, _free = struct.unpack_from(
            "<BbHIQI", data, 0
        )
        if command != CMD_WRITE_PACING:
            raise FsError(f"unexpected reply 0x{command:02x} to write")
        # This one IS meaningful: the header handler assigns status only on success,
        # so anything other than 0x01 means FileOpen failed (and the value is garbage).
        if status != STATUS_OK:
            raise FsError(
                f"could not create {path}: {describe_status(status)}. "
                "Does the parent directory exist? (mkdir it first)"
            )

        sent = 0
        while sent < len(content):
            chunk = content[sent:sent + self.write_chunk]
            await self._send(
                struct.pack("<BBHII", CMD_WRITE_DATA, STATUS_OK, 0, sent, len(chunk))
                + chunk
            )
            # The reply's status is uninitialised on success (FSService.cpp:196-198),
            # so it is deliberately not checked. We just need the round trip for pacing.
            await self._response()
            sent += len(chunk)
            if on_progress:
                on_progress(sent, len(content))

        # Because we cannot trust the status bytes, confirm the result independently.
        actual = await self.size_of(path)
        if actual is None:
            raise FsError(f"upload finished but {path} is not on the watch")
        if actual != len(content):
            raise FsError(f"size mismatch: watch has {actual} bytes, sent {len(content)}")
        return actual

    async def delete(self, path):
        encoded = path.encode()
        await self._send(struct.pack("<BBH", CMD_DELETE, 0, len(encoded)) + encoded)
        data = await self._response()
        command, status = struct.unpack_from("<Bb", data, 0)
        if command != CMD_DELETE_STATUS:
            raise FsError(f"unexpected reply 0x{command:02x} to delete")
        if status != STATUS_OK:
            raise FsError(f"{path}: {describe_status(status)}")

    async def mkdir(self, path):
        encoded = path.encode()
        await self._send(
            struct.pack("<BBHIQ", CMD_MKDIR, 0, len(encoded), 0, 0) + encoded
        )
        data = await self._response()
        command, status = struct.unpack_from("<Bb", data, 0)
        if command != CMD_MKDIR_STATUS:
            raise FsError(f"unexpected reply 0x{command:02x} to mkdir")
        if status != STATUS_OK:
            raise FsError(f"{path}: {describe_status(status)}")

    async def freespace(self):
        """Free bytes, learned from a write probe to a scratch path.

        The freespace field is not simply the free space: FSService.cpp:179 returns
        min(free space, totalSize - offset), i.e. how much of the write just announced
        the watch can still take. A zero-length probe therefore always answers 0, so the
        probe has to claim a transfer larger than the flash to see the real figure.
        The file is opened and closed but never written to, then deleted below.
        """
        encoded = b"/.blefs_probe"
        await self._send(
            struct.pack("<BBHIQI", CMD_WRITE, 0, len(encoded), 0, 0, FREESPACE_PROBE_SIZE)
            + encoded
        )
        data = await self._response()
        _cmd, _status, _pad, _off, _mt, free = struct.unpack_from("<BbHIQI", data, 0)
        try:
            await self.delete("/.blefs_probe")
        except FsError:
            pass
        return free


class Device:
    """Identity, battery and clock: everything served outside the filesystem service."""

    def __init__(self, client):
        self.client = client

    async def _read_str(self, uuid):
        """A Device Information string, or None if this build does not expose it."""
        try:
            return (await self.client.read_gatt_char(uuid)).decode(errors="replace").strip("\x00")
        except Exception:
            return None

    async def battery(self):
        try:
            return (await self.client.read_gatt_char(UUID_BATTERY_LEVEL))[0]
        except Exception:
            return None

    async def firmware_version(self):
        return await self._read_str(UUID_FW_REVISION)

    # -- read-only sensors -------------------------------------------------------
    #
    # All three are READ|NOTIFY with no write path anywhere in the firmware, so they
    # can be reported but not set. Each returns None if the characteristic is missing
    # (an older build) or the read is refused, which keeps `info` printing the rest.

    async def steps(self):
        """Steps counted today, as uint32 (MotionService.cpp:62-69)."""
        try:
            raw = await self.client.read_gatt_char(UUID_STEP_COUNT)
            return struct.unpack("<I", raw[:4])[0]
        except Exception:
            return None

    async def motion(self):
        """Latest accelerometer sample as (x, y, z) in binary milli-g."""
        try:
            raw = await self.client.read_gatt_char(UUID_MOTION_VALUES)
            return struct.unpack("<hhh", raw[:6])
        except Exception:
            return None

    async def heart_rate(self):
        """Last heart rate in bpm, or None. Zero means the sensor is not running.

        The characteristic is the standard two-byte HRM measurement, but the firmware
        always sends flags = 0 and a uint8 value (HeartRateService.cpp:51).
        """
        try:
            raw = await self.client.read_gatt_char(UUID_HEART_RATE)
            return raw[1] if len(raw) >= 2 else None
        except Exception:
            return None

    # -- notifications and alerts ------------------------------------------------

    async def send_notification(self, category, title, body):
        """Push a notification. Returns the text bytes that actually fit.

        Layout is category, count, separator, then "title\\0body" -- but the firmware
        reads only the category out of that header (see the notes at the top of this
        file), so the other two bytes are sent as zero.
        """
        text = title.encode() + b"\x00" + body.encode() if body else title.encode()
        text = text[:ANS_MAX_TEXT]
        await self.client.write_gatt_char(
            UUID_NEW_ALERT, bytes([category, 0, 0]) + text, response=True
        )
        return text

    async def send_alert(self, level):
        """Immediate Alert Service, one byte. Write-without-response is all it accepts."""
        await self.client.write_gatt_char(UUID_ALERT_LEVEL, bytes([level]), response=False)

    # -- music -------------------------------------------------------------------

    def _music_characteristic(self, uuid):
        """The lowest-handle characteristic with this UUID.

        Everywhere else a UUID identifies one characteristic, but MusicService registers
        the track-length UUID twice -- characteristicDefinition[6] and [7] are both
        msTotalLengthCharUuid (MusicService.cpp:84-91), evidently a copy-paste for what
        should have been a distinct field. bleak refuses to resolve a duplicated UUID
        and asks for a handle instead, so `music length` has to pick one. Both entries
        land in the same firmware variable, so the first will do.
        """
        matches = [characteristic
                   for characteristic in self.client.services.characteristics.values()
                   if characteristic.uuid == uuid]
        if not matches:
            raise FsError(f"this firmware does not serve characteristic {uuid}")
        return min(matches, key=lambda characteristic: characteristic.handle)

    async def set_music(self, field, value):
        """Write one Music Service characteristic. Returns what was sent, for display."""
        uuid, kind = MUSIC_FIELDS[field]
        if kind == "str":
            payload = value.encode()[:MUSIC_MAX_STRING]
            shown = payload.decode(errors="replace")
        elif kind == "flag":
            payload = bytes([1 if value else 0])
            words = ("play", "pause") if field == "status" else ("on", "off")
            shown = words[0] if value else words[1]
        elif kind == "u32":
            payload = struct.pack(">I", value)     # big-endian, MusicService.cpp:166
            shown = str(value)
        else:                                       # speed, a float sent as hundredths
            payload = struct.pack(">I", round(value * 100))
            shown = f"{value:g}x"
        await self.client.write_gatt_char(self._music_characteristic(uuid), payload,
                                          response=True)
        return shown

    # -- weather -----------------------------------------------------------------

    async def send_weather_current(self, temperature, minimum, maximum, icon,
                                   location, sunrise, sunset):
        """Message type 0, version 1 (SimpleWeatherService.cpp:39-84).

        Temperatures are centi-degrees Celsius; sunrise and sunset are minutes into the
        local day, or -1 for unknown. The firmware drops the whole pair if it is not
        internally consistent, so it validates rather than silently ignoring bad input.
        """
        payload = struct.pack(
            "<BBQhhh32sBhh",
            0,                                      # CurrentWeather
            1,                                      # version, the one with sun times
            int(datetime.datetime.now().timestamp()),
            temperature, minimum, maximum,
            location.encode()[:WEATHER_LOCATION_SIZE],
            icon,
            sunrise, sunset,
        )
        await self.client.write_gatt_char(UUID_WEATHER_DATA, payload, response=True)

    async def send_weather_forecast(self, days):
        """Message type 1, version 0. days is a list of (min, max, icon), at most five."""
        payload = struct.pack(
            "<BBQB", 1, 0, int(datetime.datetime.now().timestamp()), len(days)
        )
        for minimum, maximum, icon in days:
            payload += struct.pack("<hhB", minimum, maximum, icon)
        await self.client.write_gatt_char(UUID_WEATHER_DATA, payload, response=True)

    async def identity(self):
        return {
            "manufacturer": await self._read_str(UUID_MANUFACTURER),
            "model": await self._read_str(UUID_MODEL_NUMBER),
            "serial": await self._read_str(UUID_SERIAL_NUMBER),
            "firmware": await self._read_str(UUID_FW_REVISION),
            "hardware": await self._read_str(UUID_HW_REVISION),
            "software": await self._read_str(UUID_SW_REVISION),
        }

    async def get_time(self):
        """The watch's local time. Second resolution, so drift is only good to +/-1s."""
        raw = await self.client.read_gatt_char(UUID_CURRENT_TIME)
        if len(raw) < struct.calcsize(CTS_FORMAT):
            raise FsError(f"short current-time response: {raw.hex()}")
        # dayofweek and reason are deliberately discarded: the firmware never assigns
        # them on the read path, so they are uninitialised stack bytes.
        year, month, day, hour, minute, second, _dow, _frac, _reason = struct.unpack(
            CTS_FORMAT, raw[:struct.calcsize(CTS_FORMAT)]
        )
        try:
            return datetime.datetime(year, month, day, hour, minute, second)
        except ValueError as exc:
            raise FsError(f"watch reported an impossible time ({year}-{month}-{day} "
                          f"{hour}:{minute}:{second}): {exc}")

    async def get_utc_offset(self):
        """(timezone, dst) as timedeltas, or (None, None) if the read fails."""
        try:
            raw = await self.client.read_gatt_char(UUID_LOCAL_TIME)
            tz, dst = struct.unpack(CTS_LOCAL_FORMAT, raw[:2])
        except Exception:
            return None, None
        return (datetime.timedelta(seconds=tz * QUARTER_HOUR),
                datetime.timedelta(seconds=dst * QUARTER_HOUR))

    async def set_time(self, when=None):
        """Set the clock from this machine. Writes the zone first, then the time."""
        when = when or datetime.datetime.now()

        # The whole UTC offset goes in the timezone field and dst is left at 0. Two
        # reasons: a naive datetime's astimezone() yields a fixed-offset tzinfo whose
        # dst() is always None, so the DST hour cannot be recovered here anyway; and
        # nothing in the firmware reads the two fields apart -- DateTimeController.h:94
        # and :132 only ever use tzOffset + dstOffset.
        total = when.astimezone().utcoffset() or datetime.timedelta(0)
        quarters = round(total.total_seconds() / QUARTER_HOUR)
        try:
            await self.client.write_gatt_char(
                UUID_LOCAL_TIME,
                struct.pack(CTS_LOCAL_FORMAT, quarters, 0),
                response=True,
            )
        except Exception as exc:
            print(f"  note: could not set the time zone ({exc}); setting the clock anyway")

        # isoweekday() is 1=Monday..7=Sunday, which is what the CTS field wants.
        await self.client.write_gatt_char(
            UUID_CURRENT_TIME,
            struct.pack(CTS_FORMAT, when.year, when.month, when.day, when.hour,
                        when.minute, when.second, when.isoweekday(), 0, 0),
            response=True,
        )
        return when


def describe_drift(watch_time, local_time=None):
    """Human-readable clock difference, e.g. '3s fast' or 'in sync'."""
    local_time = local_time or datetime.datetime.now()
    drift = (watch_time - local_time).total_seconds()
    if abs(drift) < 1.5:
        return "in sync with this machine (to the second)"
    direction = "fast" if drift > 0 else "slow"
    drift = abs(drift)
    if drift < 90:
        return f"{drift:.0f}s {direction}"
    if drift < 5400:
        return f"{drift / 60:.1f} min {direction}"
    return f"{drift / 3600:.1f} h {direction}"


def progress(done, total):
    if not total:
        return
    filled = int(30 * done / total)
    print(f"\r  [{'#' * filled}{'.' * (30 - filled)}] {done}/{total} B", end="", flush=True)


class Shell:
    def __init__(self, fs, device, assume_yes=False):
        self.fs = fs
        self.device = device
        self.assume_yes = assume_yes
        # Insertion order is the order `help` lists them in.
        self.handlers = {
            "ls": self.cmd_ls, "cp": self.cmd_cp, "rm": self.cmd_rm,
            "mkdir": self.cmd_mkdir, "df": self.cmd_df,
            "time": self.cmd_time, "info": self.cmd_info,
            "notify": self.cmd_notify, "alert": self.cmd_alert,
            "music": self.cmd_music, "weather": self.cmd_weather,
            "flash": self.cmd_flash,
            "help": self.cmd_help, "exit": self.cmd_exit,
        }
        # Dispatchable, but not listed separately by `help`.
        self.aliases = {"date": "time", "quit": "exit"}

    async def run_line(self, line):
        try:
            parts = shlex.split(line)
        except ValueError as exc:
            print(f"parse error: {exc}")
            return True
        if not parts:
            return True

        command, args = parts[0], parts[1:]
        handler = self.handlers.get(self.aliases.get(command, command))
        if handler is None:
            print(f"unknown command {command!r}; try 'help'")
            return True
        try:
            # `exit` returns False, and so does a successful flash: it reboots the watch,
            # which ends the session either way.
            if await handler(args) is False:
                return False
        except FsError as exc:
            print(f"error: {exc}")
        except BleakError as exc:
            # Losing the connection mid-session is not recoverable, but a single refused
            # request is: report it and stay at the prompt.
            print(f"error: {describe_ble_error(exc)}")
        except OSError as exc:
            print(f"local error: {exc}")
        return True

    def confirm(self, prompt):
        if self.assume_yes:
            return True
        try:
            return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")
        except EOFError:
            print()
            return False

    async def cmd_help(self, args):
        """help [COMMAND]
        list the commands, or explain one of them

        With no argument, one line per command. With a command name, that command's
        usage in full:

          help cp
          help flash
        """
        if not args:
            print(HELP_PREAMBLE)
            for name, handler in self.handlers.items():
                usage, summary, _ = command_help(handler)
                print(f"  {usage:<30}{summary}")
            return

        name = self.aliases.get(args[0], args[0])
        handler = self.handlers.get(name)
        if handler is None:
            raise FsError(f"no such command {args[0]!r}; 'help' lists them all")
        usage, summary, details = command_help(handler)
        print(f"{usage}\n  {summary}")
        if details:
            print(f"\n{details}")

    async def cmd_exit(self, _args):
        """exit
        quit (Ctrl-D also works)

        'quit' is an alias. A flash also ends the session, since the watch reboots.
        """
        return False

    async def cmd_ls(self, args):
        """ls [-r] [PATH]
        list a directory (default /)

        Sizes are in bytes; directories show a dash and a trailing /. With -r, descend
        into subdirectories, indenting each level. The firmware drops entries from big
        directories (past roughly 27); ls then warns and shows the ones that arrived. A ! path lists this machine instead,
        in the same format, and naming a local file just shows its size.

          ls /images                        on the watch
          ls -r /                           the whole device filesystem
          ls !                              local working directory
          ls !../build/src/resources        any local path
        """
        recursive = "-r" in args
        paths = [a for a in args if not a.startswith("-")]
        path = paths[0] if paths else "/"
        if is_local(path):
            self._ls_local(local_path(path), recursive, 0)
        else:
            await self._ls(path, recursive, 0)

    @staticmethod
    def _print_entries(entries, depth):
        for item in sorted(entries, key=lambda e: (not e["is_dir"], e["name"])):
            size = "        -" if item["is_dir"] else f"{item['size']:>9}"
            print(f"{size}  {'  ' * (depth + 1)}{item['name']}{'/' if item['is_dir'] else ''}")
            yield item

    async def _ls(self, path, recursive, depth):
        entries = await self.fs.listdir(path, allow_partial=True)
        if depth == 0:
            print(path)
        if entries.missing:
            print(
                f"warning: the watch dropped {entries.missing} entries of {path} "
                "(a firmware limit on large directories); showing the rest",
                file=sys.stderr,
            )
        for item in self._print_entries(entries, depth):
            if recursive and item["is_dir"]:
                await self._ls(f"{path.rstrip('/')}/{item['name']}", recursive, depth + 1)

    def _ls_local(self, path, recursive, depth):
        """Same output as _ls, but for the machine running this script."""
        if os.path.isfile(path):
            print(f"{os.path.getsize(path):>9}  {path}")
            return

        entries = []
        with os.scandir(path) as scan:
            for item in scan:
                try:
                    is_dir = item.is_dir()
                    size = 0 if is_dir else item.stat().st_size
                except OSError:
                    # Broken symlink, or something we cannot stat. Still worth listing.
                    is_dir, size = False, 0
                entries.append({"name": item.name, "size": size, "is_dir": is_dir})

        if depth == 0:
            print(os.path.abspath(path))
        for item in self._print_entries(entries, depth):
            if recursive and item["is_dir"]:
                self._ls_local(os.path.join(path, item["name"]), recursive, depth + 1)

    async def cmd_cp(self, args):
        """cp [-c|-u] SRC DST
        copy a file; exactly one side must be local (!)

        The ! side says which direction this goes: watch-to-watch and local-to-local are
        both refused. Local upload sources may contain shell-style wildcards (*, ?, []);
        multiple matches require DST to end in /. A DST ending in / (or, downloading, an
        existing local directory) keeps the source basename. Uploads are verified
        afterwards by reading back the file's size from the watch, because the firmware's
        own write status cannot be trusted (see the notes at the top of this file).

          cp !fuji.bin /images/fuji.bin     upload
          cp /fonts/teko.bin !teko.bin      download
          cp !fuji.bin /images/             keep the local basename
          cp !faces/*.bin /canvas/willie/   upload all matching local files

        Uploading a .zip can mean either of two things, so it asks which unless told:
        -c copies the archive itself to the watch, and -u unpacks it, writing the files
        inside to the watch and creating the directories they need on the way.

        An InfiniTime resources package (infinitime-resources-x.y.z.zip) is flat and
        carries a resources.json saying where each file belongs, so -u follows that
        manifest -- teko.bin to /fonts/teko.bin, fuji.bin to /images/fuji.bin -- and
        offers to delete the obsolete files it names. Any other zip has no manifest, and
        there each member keeps the path it has inside the archive instead. DST is the
        root either lands under, so / gives the paths as written.

        Non-interactively (the tool's own -c), there is nobody to ask, so a zip upload
        wants -c or -u spelled out; with -y and neither of them it copies as-is, which
        is what cp has always done.

          cp -u !infinitime-resources-1.16.0.zip /   install the stock resources
          cp -c !watchfiles.zip /watch.zip           store the archive itself
        """
        mode = None
        positional = []
        for arg in args:
            if arg == "-c":
                mode = "copy"
            elif arg == "-u":
                mode = "unpack"
            elif len(arg) > 1 and arg.startswith("-"):
                raise FsError(f"unknown option {arg}  (cp takes -c or -u)")
            else:
                positional.append(arg)

        if len(positional) != 2:
            raise FsError("usage: cp [-c|-u] SRC DST  (exactly one side prefixed with !)")
        src, dst = positional
        if is_local(src) == is_local(dst):
            raise FsError(
                "exactly one of SRC and DST must be local (!). "
                "Copying watch-to-watch or local-to-local is not supported."
            )
        if mode and not is_local(src):
            raise FsError("-c and -u only apply to uploading a local .zip")

        if is_local(src):  # upload
            pattern = local_path(src)
            has_magic = glob.has_magic(pattern)
            if has_magic:
                sources = [path for path in glob.glob(pattern) if os.path.isfile(path)]
                if not sources:
                    raise FsError(f"no local files match {pattern!r}")
                if len(sources) > 1 and not dst.endswith("/"):
                    raise FsError("multiple source files require DST ending in /")
                if len(sources) > 1:
                    for source in sorted(sources):
                        target = dst + os.path.basename(source)
                        with open(source, "rb") as handle:
                            content = handle.read()
                        print(f"{source} -> {target} ({len(content)} B)")
                        written = await self.fs.write_file(target, content, progress)
                        print(f"\n  verified {written} B on the watch")
                    return
                source = sources[0]
            else:
                source = pattern

            if zipfile.is_zipfile(source):
                if mode is None:
                    mode = self.ask_zip_action()
                    if mode is None:
                        print("Cancelled.")
                        return
                if mode == "unpack":
                    await self._unpack_zip(source, dst)
                    return
            elif mode == "unpack":
                raise FsError(f"{source} is not a zip archive, so there is nothing to unpack")

            target = dst
            if target.endswith("/"):
                target += os.path.basename(source)
            with open(source, "rb") as handle:
                content = handle.read()
            print(f"{source} -> {target} ({len(content)} B)")
            written = await self.fs.write_file(target, content, progress)
            print(f"\n  verified {written} B on the watch")
        else:  # download
            source = src
            target = local_path(dst)
            if target.endswith("/") or os.path.isdir(target):
                target = os.path.join(target, os.path.basename(source))
            print(f"{source} -> {target}")
            content = await self.fs.read_file(source, progress)
            with open(target, "wb") as handle:
                handle.write(content)
            print(f"\n  wrote {len(content)} B locally")

    def ask_zip_action(self):
        """Ask whether a zip upload means the archive or its contents.

        Returns "copy", "unpack", or None to abort. Anything but c/C/u/U aborts, on the
        principle that the two outcomes are too different to guess at from a typo.
        Under -y there is nobody to ask, so it takes the reading `cp` has always had.
        """
        if self.assume_yes:
            print("Zip archive: copying it to the device as-is (-y; pass -u to unpack).")
            return "copy"
        print("Zip archive: do you want to")
        print("   [C]opy the zip file to the device as-is, or")
        print("   [U]npack the archive to multiple files on device?")
        try:
            answer = input("==> ").strip()
        except EOFError:
            print()
            return None
        return {"c": "copy", "u": "unpack"}.get(answer.lower())

    async def _unpack_zip(self, source, dst):
        """Write the contents of the zip at `source` onto the watch, under `dst`.

        Where each file lands is decided one of two ways. An InfiniTime resources
        package (infinitime-resources-x.y.z.zip) is flat and carries a resources.json
        manifest naming the watch path for every file, so that manifest wins; see
        doc/ExternalResources.md in the firmware tree. Any other zip has no manifest,
        and there each member simply keeps the path it has inside the archive.

        Either way `dst` is the root it all hangs off, so `/` gives the paths as written.
        Missing directories are created as we go, one level at a time, because that is
        all mkdir does. Nothing here is atomic: a failure part way through leaves the
        files already written in place.
        """
        base = dst.rstrip("/")
        with zipfile.ZipFile(source) as archive:
            members = [item for item in archive.infolist()
                       if not item.is_dir()
                       and not item.filename.startswith("__MACOSX/")
                       and not os.path.basename(item.filename).startswith("._")]
            if not members:
                raise FsError(f"{source} contains no files to unpack")

            manifest = self._read_manifest(archive)
            if manifest is None:
                targets = [(item, self._watch_path(base, item.filename)) for item in members]
                obsolete = []
                print(f"{source} -> {dst}  ({len(targets)} files)")
            else:
                by_name = {item.filename: item for item in members}
                targets, obsolete = [], manifest["obsolete"]
                for filename, path in manifest["resources"]:
                    item = by_name.get(filename)
                    if item is None:
                        raise FsError(
                            f"{source}: resources.json lists {filename}, "
                            "which is not in the archive"
                        )
                    targets.append((item, self._watch_path(base, path)))
                print(f"{source} -> {dst}  ({len(targets)} files, per resources.json)")

            made = set()
            for item, target in targets:
                await self._ensure_dirs(os.path.dirname(target), made)
                content = archive.read(item)
                print(f"  {target} ({len(content)} B)")
                written = await self.fs.write_file(target, content, progress)
                print(f"\n    verified {written} B on the watch")
        print(f"unpacked {len(targets)} files")

        if obsolete:
            # The manifest only says these are no longer needed, so this is the watch
            # owner's call, not ours: offer it, and leave them alone if declined.
            print(f"\nresources.json lists {len(obsolete)} obsolete file(s):")
            for path, since in obsolete:
                print(f"  {path}{f'  (since {since})' if since else ''}")
            if self.confirm("Delete them from the watch?"):
                for path, _since in obsolete:
                    try:
                        await self.fs.delete(self._watch_path(base, path))
                        print(f"  deleted {path}")
                    except FsError as exc:
                        # Not being there is the expected case, not a failure.
                        if "NOENT" not in str(exc):
                            raise
                        print(f"  {path} was not there")

    @staticmethod
    def _read_manifest(archive):
        """Parse an InfiniTime resources.json, or return None if this is a plain zip.

        Returns {"resources": [(filename, path), ...], "obsolete": [(path, since), ...]}.
        A resources.json that does not parse is worth complaining about rather than
        silently falling back to the by-path rule, which would put the whole flat
        archive in one directory.
        """
        names = [name for name in archive.namelist()
                 if os.path.basename(name) == "resources.json" and "/" not in name.strip("/")]
        if not names:
            return None
        try:
            data = json.loads(archive.read(names[0]))
            resources = [(entry["filename"], entry["path"]) for entry in data["resources"]]
        except (ValueError, KeyError, TypeError) as exc:
            raise FsError(f"resources.json in this archive is not usable: {exc}")
        # generate-package.py writes {} here when the firmware build had no obsolete
        # list, so this is a list only some of the time.
        entries = data.get("obsolete_files") or []
        obsolete = [(entry["path"], entry.get("since")) for entry in entries
                    if isinstance(entry, dict) and entry.get("path")]
        return {"resources": resources, "obsolete": obsolete}

    @staticmethod
    def _watch_path(base, raw):
        """Join a path out of an archive onto `base`, refusing anything that climbs out.

        Manifest paths are absolute (/fonts/teko.bin) and member paths are relative;
        both are just anchored at `base` here. Neither is trustworthy in principle, and
        this writes straight into a filesystem, so `..` is refused outright.
        """
        parts = [part for part in raw.replace("\\", "/").split("/") if part not in ("", ".")]
        if ".." in parts:
            raise FsError(f"refusing to unpack member with unsafe path: {raw}")
        if not parts:
            raise FsError(f"refusing to unpack member with empty path: {raw!r}")
        return f"{base}/{'/'.join(parts)}"

    async def _ensure_dirs(self, directory, made):
        """mkdir every level of `directory` that is not there yet, remembering which."""
        parts = [part for part in directory.split("/") if part]
        path = ""
        for part in parts:
            path = f"{path}/{part}"
            if path in made:
                continue
            try:
                await self.fs.mkdir(path)
                print(f"  created {path}/")
            except FsError as exc:
                # Already there is the common case, and the only one we can carry on from;
                # a real failure will surface on the write that follows.
                if "EXIST" not in str(exc):
                    raise
            made.add(path)

    async def cmd_rm(self, args):
        """rm [-R] PATH
        delete a file on the watch, or a whole directory with -R

        Plain rm deletes one thing and does not ask. The firmware deletes through
        lfs_remove, so an empty directory can be removed this way too; a directory with
        anything in it fails with NOTEMPTY.

        -R (or -r) empties a directory first and then removes it, walking it depth-first
        because lfs_remove only ever takes one entry at a time. That walk is also what
        makes it worth confirming: the count is shown before anything is deleted, and -y
        answers yes. The firmware drops entries from big directories, so the walk may
        not see everything at first; rm then deletes what it saw and walks again, as
        many passes as it takes, until the directory lists completely. `rm -R /` is allowed, and empties the watch without removing the
        root itself, so think twice. Local paths are refused outright -- use your own
        shell for those.

          rm /images/fuji.bin               one file
          rm -R /cptest                     the directory and everything under it
        """
        recursive = False
        positional = []
        for arg in args:
            if arg in ("-R", "-r"):
                recursive = True
            elif len(arg) > 1 and arg.startswith("-"):
                raise FsError(f"unknown option {arg}  (rm takes -R)")
            else:
                positional.append(arg)

        if len(positional) != 1:
            raise FsError("usage: rm [-R] PATH")
        path = positional[0]
        if is_local(path):
            raise FsError("rm only deletes on the watch; use your own shell for local files")

        if not recursive:
            await self.fs.delete(path)
            print(f"deleted {path}")
            return

        # -R on a file is not an error anywhere else, and it should not be here either.
        is_root = not path.strip("/")
        if not is_root and not await self._is_dir(path):
            await self.fs.delete(path)
            print(f"deleted {path}")
            return

        path = "/" if is_root else path.rstrip("/")
        files, dirs, missing = await self._walk(path)
        if not files and not dirs and not missing and is_root:
            print("/ is already empty")
            return

        count = f"{len(files)} file(s) and {len(dirs)} directory(ies)"
        if missing:
            count += f", plus {missing} more entries the watch did not list"
        print(f"{path} holds {count}." if files or dirs or missing else f"{path} is empty.")
        if not self.confirm(f"Delete {'everything under ' if is_root else ''}{path}?"):
            print("Cancelled.")
            return

        while True:
            # Depth-first: _walk already ordered the directories deepest-first, and every
            # file goes before any directory, so nothing is ever removed while non-empty.
            for target in files + dirs:
                await self.fs.delete(target)
                print(f"  deleted {target}")
            if not missing:
                break
            # Each pass shrinks the directories, so their listings eventually fit.
            print(f"  listing again for the {missing} entries the watch dropped")
            files, dirs, missing = await self._walk(path)
            if missing and not files and not dirs:
                raise FsError(f"{path}: the watch keeps dropping entries; giving up")
        if is_root:
            print("emptied /")  # lfs has no way to remove the root itself
        else:
            await self.fs.delete(path)
            print(f"deleted {path}")

    async def _is_dir(self, path):
        """Whether `path` is a directory, by trying to list it. NOENT if it is absent.

        Listing the path itself, rather than finding it in its parent's listing, works
        even when the parent is too big to list completely: lfs_dir_open refuses a file
        with NOTDIR before any entries are sent.
        """
        try:
            await self.fs.listdir(path, allow_partial=True)
        except FsError as exc:
            if describe_status(-20) in str(exc):
                return False
            raise
        return True

    async def _walk(self, path):
        """Everything under `path`: (files, directories, missing), directories
        deepest-first, with missing the number of entries the watch dropped.

        The order is the point -- it is what the caller deletes in, and lfs_remove
        refuses a directory that still has anything in it. For the same reason a
        directory whose listing came back incomplete is left out of `directories`: it
        still holds entries nobody has seen, so it waits for a later walk.
        """
        files, dirs = [], []
        missing = 0

        async def visit(directory):
            """Walk one directory; True if it and everything under it listed fully."""
            nonlocal missing
            listing = await self.fs.listdir(directory, allow_partial=True)
            missing += listing.missing
            complete = not listing.missing
            for item in listing:
                child = f"{directory.rstrip('/')}/{item['name']}"
                if not item["is_dir"]:
                    files.append(child)
                elif await visit(child):
                    dirs.append(child)  # after its own children, so children go first
                else:
                    complete = False
            return complete

        await visit(path)
        return files, dirs, missing

    async def cmd_mkdir(self, args):
        """mkdir PATH
        create a directory on the watch

        One level at a time: the parent has to exist already, and an existing path
        fails with EXIST.
        """
        if len(args) != 1:
            raise FsError("usage: mkdir PATH")
        if is_local(args[0]):
            raise FsError("mkdir only creates directories on the watch")
        await self.fs.mkdir(args[0])
        print(f"created {args[0]}")

    async def cmd_df(self, _args):
        """df
        show free space on the watch

        Free space is not something the protocol reports directly: this asks by starting
        an oversized write to a scratch path, reading the free figure out of the reply,
        then deleting the file (see FileSystem.freespace). Nothing is ever written to it.
        """
        free = await self.fs.freespace()
        print(f"{free} bytes free ({free / 1024:.1f} KiB)")

    async def cmd_time(self, args):
        """time [set]
        show the watch clock and its drift, or set it from this machine

        'date' is an alias. Plain 'time' also reports the watch's UTC offset. 'time set'
        writes the time zone first, then the clock, always from this machine's current
        time -- there is no way to pass a time in -- and reads it back afterwards, since
        the write is unacknowledged above the GATT layer.

        The whole UTC offset goes into the time zone field with DST left at zero. The
        firmware only ever uses the sum of the two, so this reads back correctly, but a
        watch set this way reports no separate DST hour.
        """
        if args and args[0] == "set":
            if len(args) > 1:
                raise FsError("usage: time set  (the clock is always taken from this machine)")
            when = await self.device.set_time()
            print(f"watch clock set to {when:%Y-%m-%d %H:%M:%S}")
            # Read it back: the write is unacknowledged beyond the GATT layer.
            print(f"watch now reports {await self.device.get_time():%Y-%m-%d %H:%M:%S}")
            return
        if args:
            raise FsError("usage: time [set]")

        watch_time = await self.device.get_time()
        tz, dst = await self.device.get_utc_offset()
        print(f"{watch_time:%Y-%m-%d %H:%M:%S}  ({describe_drift(watch_time)})")
        if tz is not None:
            total = tz + dst
            sign = "-" if total < datetime.timedelta(0) else "+"
            hours, remainder = divmod(abs(total).seconds, 3600)
            print(f"UTC{sign}{hours:02d}:{remainder // 60:02d}"
                  f"{f' (includes {dst.seconds // 3600}h DST)' if dst else ''}")

    async def cmd_info(self, _args):
        """info
        firmware version, battery, clock, filesystem and connection

        A summary of everything readable in one go: the Device Information strings,
        battery percentage, the clock with its drift from this machine, the BLE
        filesystem version with free space, and the negotiated MTU with the read and
        write chunk sizes derived from it.
        """
        ident = await self.device.identity()
        battery = await self.device.battery()

        print(f"{ident['software'] or 'firmware'} {ident['firmware'] or '?'} "
              f"on {ident['model'] or 'unknown model'} rev {ident['hardware'] or '?'}"
              f"  ({ident['manufacturer'] or 'unknown manufacturer'})")
        if battery is not None:
            print(f"battery      {battery}%")
        try:
            watch_time = await self.device.get_time()
            print(f"clock        {watch_time:%Y-%m-%d %H:%M:%S}  ({describe_drift(watch_time)})")
        except FsError as exc:
            print(f"clock        unavailable: {exc}")
        steps = await self.device.steps()
        if steps is not None:
            print(f"steps        {steps} today")
        motion = await self.device.motion()
        if motion is not None:
            x, y, z = motion
            magnitude = (x * x + y * y + z * z) ** 0.5 / ACCEL_UNITS_PER_G
            print(f"motion       x {x:>6}  y {y:>6}  z {z:>6}  "
                  f"(binary milli-g; |a| = {magnitude:.2f} g)")
        heart_rate = await self.device.heart_rate()
        if heart_rate is not None:
            # The characteristic reads back 0 whenever the sensor is idle, which is most
            # of the time: InfiniTime only runs it in the Heart Rate app, or continuously
            # if that has been switched on in Settings.
            print(f"heart rate   {heart_rate} bpm" if heart_rate
                  else "heart rate   not measuring (start the Heart Rate app on the watch)")
        print(f"filesystem   BLE FS v{await self.fs.version()}, "
              f"{await self.fs.freespace()} bytes free")
        print(f"connection   MTU {self.fs.mtu}: {self.fs.read_chunk} B reads, "
              f"{self.fs.write_chunk} B writes")

    async def cmd_notify(self, args):
        """notify [-c CAT] TITLE [BODY]
        send a notification to the watch

        The watch shows TITLE in bold with BODY beneath it, and keeps it in the
        notification list. Together they may total 99 bytes; anything past that is
        dropped by the firmware, so this truncates and says so.

        CAT defaults to 'simple'. Only 'call' behaves differently -- it raises the
        incoming-call screen with its accept and reject buttons instead of a plain
        notification. Every other category is stored as a simple alert, so the choice is
        cosmetic, but the full set the protocol defines is accepted:

          simple  email  news  call  missed-call  sms  voicemail  schedule
          high-priority  im

          notify "Test" "Hello from infinitool"
          notify -c sms "Alice" "on my way"
          notify -c call "Bob Smith"
        """
        category_name = "simple"
        positional = []
        rest = list(args)
        while rest:
            arg = rest.pop(0)
            if arg in ("-c", "--category"):
                if not rest:
                    raise FsError("-c needs a category name")
                category_name = rest.pop(0)
            elif len(arg) > 1 and arg.startswith("-"):
                raise FsError(f"unknown option {arg}  (notify takes -c CATEGORY)")
            else:
                positional.append(arg)

        if not 1 <= len(positional) <= 2:
            raise FsError("usage: notify [-c CATEGORY] TITLE [BODY]")
        category = ANS_CATEGORIES.get(category_name.lower())
        if category is None:
            raise FsError(f"unknown category {category_name!r}; one of: "
                          f"{', '.join(ANS_CATEGORIES)}")

        title = positional[0]
        body = positional[1] if len(positional) > 1 else ""
        wanted = len(title.encode()) + (1 + len(body.encode()) if body else 0)
        sent = await self.device.send_notification(category, title, body)
        print(f"sent {category_name} notification ({len(sent)} bytes of text)")
        if wanted > len(sent):
            print(f"  note: truncated to {ANS_MAX_TEXT} bytes, which is all the "
                  f"firmware copies")

    async def cmd_alert(self, args):
        """alert [none|mild|high]
        buzz the watch with an Immediate Alert (default high)

        The shortest way to make the watch react. The firmware turns the level into a
        notification reading 'Alert : High' and vibrates; 'none' pushes 'Alert : None'
        and is only useful for testing that path. The write is unacknowledged -- the
        characteristic is write-without-response -- so nothing is read back.
        """
        if len(args) > 1:
            raise FsError("usage: alert [none|mild|high]")
        name = (args[0] if args else "high").lower()
        level = ALERT_LEVELS.get(name)
        if level is None:
            raise FsError(f"unknown level {name!r}; one of: {', '.join(ALERT_LEVELS)}")
        await self.device.send_alert(level)
        print(f"sent {name} alert")

    async def cmd_music(self, args):
        """music FIELD VALUE ...
        set what the watch's Music app displays

        Populates the Music screen as a phone would. Nothing here plays anything: the
        watch is the remote, and these are the fields it shows. Each pair is written to
        its own characteristic, in the order given.

          track artist album      text, truncated by the firmware at 40 bytes
          status                  play or pause
          position length         seconds, as whole numbers
          number total            track number, and how many are in the queue
          speed                   playback rate, e.g. 1 or 1.5
          repeat shuffle          on or off

        The watch cannot be read back -- these characteristics return nothing on a read
        (see the notes at the top of this file) -- so what is printed is what was sent.

          music track "Blue Monday" artist "New Order" status play
          music length 450 position 12
          music status pause
        """
        if not args or len(args) % 2:
            raise FsError("usage: music FIELD VALUE [FIELD VALUE ...]  "
                          f"(fields: {', '.join(MUSIC_FIELDS)})")

        pairs = []
        for field, value in zip(args[::2], args[1::2]):
            field = field.lower()
            if field not in MUSIC_FIELDS:
                raise FsError(f"unknown field {field!r}; one of: {', '.join(MUSIC_FIELDS)}")
            pairs.append((field, parse_music_value(field, value)))

        for field, value in pairs:
            shown = await self.device.set_music(field, value)
            print(f"  {field:<9} {shown}")
            if MUSIC_FIELDS[field][1] == "str" and len(value.encode()) > MUSIC_MAX_STRING:
                print(f"    note: truncated to {MUSIC_MAX_STRING} bytes, the firmware's limit")

    async def cmd_weather(self, args):
        """weather TEMP [OPTIONS]
        send weather (or, with 'forecast', a five-day outlook) to the watch

            weather TEMP [OPTIONS]
            weather forecast MIN/MAX/ICON [...]

        Temperatures are Celsius unless suffixed with F (as in 70F), and go over the
        air as hundredths of a degree. ICON is one of:

          sun  clouds-sun  clouds  broken-clouds  heavy-shower  rain  thunderstorm
          snow  smog

        The first form sends current conditions, timestamped now. Options:

          --min TEMP --max TEMP     the day's range (default: TEMP itself)
          --icon NAME               default sun
          --location NAME           up to 32 bytes, default 'Testville'
          --sunrise HH:MM           local time, both must be given, sunrise first
          --sunset HH:MM            the firmware rejects the pair if they disagree

        The second form sends a forecast instead, one MIN/MAX/ICON per day, up to five
        days starting tomorrow.

          weather 21.5
          weather 70F --min 55F --max 74F --icon rain --location Portland
          weather 18 --sunrise 06:12 --sunset 20:44
          weather forecast 12/19/rain 14/22/clouds-sun 15/24/sun
        """
        if not args:
            raise FsError("usage: weather TEMP [options]  |  weather forecast DAY ...")

        if args[0] == "forecast":
            days = args[1:]
            if not days:
                raise FsError("usage: weather forecast MIN/MAX/ICON [...]")
            if len(days) > WEATHER_MAX_FORECAST_DAYS:
                raise FsError(f"the watch stores at most {WEATHER_MAX_FORECAST_DAYS} "
                              f"forecast days; {len(days)} given")
            parsed = []
            for day in days:
                fields = day.split("/")
                if len(fields) != 3:
                    raise FsError(f"bad forecast day {day!r}; expected MIN/MAX/ICON")
                parsed.append((parse_temperature(fields[0]), parse_temperature(fields[1]),
                               parse_weather_icon(fields[2])))
            await self.device.send_weather_forecast(parsed)
            print(f"sent a {len(parsed)}-day forecast")
            return

        temperature = parse_temperature(args[0])
        options = parse_options(
            args[1:], {"--min", "--max", "--icon", "--location", "--sunrise", "--sunset"},
            "weather",
        )
        minimum = parse_temperature(options["--min"]) if "--min" in options else temperature
        maximum = parse_temperature(options["--max"]) if "--max" in options else temperature
        if minimum > maximum:
            raise FsError("--min is above --max")
        icon = parse_weather_icon(options.get("--icon", "sun"))
        location = options.get("--location", "Testville")

        # The firmware validates the pair together and throws both away if either is
        # missing or they are out of order (SimpleWeatherService.cpp:60-72), so catch
        # that here rather than leaving the watch showing nothing and no reason why.
        if ("--sunrise" in options) != ("--sunset" in options):
            raise FsError("--sunrise and --sunset must be given together; the firmware "
                          "discards one without the other")
        sunrise = parse_minutes(options["--sunrise"]) if "--sunrise" in options else -1
        sunset = parse_minutes(options["--sunset"]) if "--sunset" in options else -1
        if sunrise >= 0 and sunrise >= sunset:
            raise FsError("--sunrise must be earlier in the day than --sunset")

        await self.device.send_weather_current(temperature, minimum, maximum, icon,
                                               location, sunrise, sunset)
        detail = f" ({minimum / 100:.1f} to {maximum / 100:.1f})" if minimum != maximum else ""
        print(f"sent {temperature / 100:.1f} C{detail} for {location}")

    async def cmd_flash(self, args):
        """flash FILE
        reflash the firmware from a DFU zip, after confirming

        FILE is a *-dfu-*.zip package, always local, so the ! prefix is optional here.
        The transfer is Nordic legacy DFU over the connection this session already
        holds; the running firmware serves it, so the watch does not need to be put into
        bootloader mode first. It reboots when the flash finishes, which ends the
        session. Under -c, the confirmation prompt needs -y.

        The new image is NOT yet validated after the reboot: on the watch, go to quick
        settings -> cog -> Firmware and validate it, or the next reset rolls back.

          flash !pinetime-mcuboot-app-dfu-1.16.0.zip
        """
        if len(args) != 1:
            raise FsError("usage: flash FILE  (a *-dfu-*.zip package)")
        # The zip can only ever be local, so the ! prefix is optional here.
        path = local_path(args[0]) if is_local(args[0]) else os.path.expanduser(args[0])
        if not os.path.isfile(path):
            raise FsError(f"no such file: {path}")

        # See DFU_TOOLS_DIR: the DFU implementation is pulled in only here, and only
        # when a flash is actually requested.
        if DFU_TOOLS_DIR not in sys.path:
            sys.path.insert(0, DFU_TOOLS_DIR)
        try:
            from dfu_bleak import DfuError, LegacyDfu
            from unpacker import Unpacker
        except ImportError as exc:
            raise FsError(
                f"could not load the DFU tools from {DFU_TOOLS_DIR}: {exc}. "
                "flash needs dfu_bleak.py and unpacker.py from that directory."
            )

        unpacker = Unpacker()
        try:
            binfile, datfile = unpacker.unpack_zipfile(path)
            with open(binfile, "rb") as handle:
                firmware = handle.read()
            with open(datfile, "rb") as handle:
                init_packet = handle.read()
        except Exception as exc:
            unpacker.delete()
            raise FsError(f"could not read DFU package {path}: {exc}")

        current = await self.device.firmware_version()
        print(f"About to reflash the watch from {os.path.basename(path)}")
        print(f"  firmware image  {len(firmware)} bytes")
        print(f"  currently running  {current or 'unknown version'}")
        print("  the watch will reboot, and this session will end")
        if not self.confirm("Proceed?"):
            unpacker.delete()
            print("Cancelled.")
            return

        try:
            dfu = LegacyDfu(self.fs.client, DFU_CHUNK_SIZE, DFU_PRN_INTERVAL,
                            DFU_TIMEOUT, self.fs.verbose)
            await dfu.run(firmware, init_packet)
        except DfuError as exc:
            raise FsError(f"flash failed: {exc}")
        finally:
            unpacker.delete()

        print()
        print("Done. The watch is rebooting into the new firmware.")
        print("IMPORTANT: the image is not validated yet. On the watch, swipe right ->")
        print("cog -> Firmware -> validate, or the next reset will roll it back.")
        return False  # the connection is gone; end the session


async def find_device(name, address, scan_time):
    if address:
        device = await BleakScanner.find_device_by_address(address, timeout=scan_time)
        if device is None:
            raise FsError(f"no device with address/UUID {address}")
        return device
    print(f"Scanning for {name!r}...", file=sys.stderr)
    device = await BleakScanner.find_device_by_filter(
        lambda d, _adv: bool(d.name) and name.lower() in d.name.lower(),
        timeout=scan_time,
    )
    if device is None:
        raise FsError(
            f"no device named {name!r}. Make sure the watch is awake, in range, and not "
            "connected to a phone or browser (BLE allows one central at a time)."
        )
    return device


async def main_async(args):
    device = await find_device(args.name, args.address, args.scan_time)
    print(f"Found {device.name} ({device.address})")

    async with BleakClient(device) as client:
        if client.services.get_characteristic(UUID_TRANSFER) is None:
            raise FsError("no BLE FS transfer characteristic — is this an InfiniTime watch?")

        fs = BleFs(client, args.timeout, args.verbose)
        await fs.start()

        device = Device(client)

        version = await fs.version()
        if version == 0:
            raise FsError(
                "BLE FS version reads 0, which means file access is denied. "
                "On the watch: Settings -> 'Firmware & files' -> Enabled."
            )

        # One banner line, in the spirit of a login MOTD: what am I talking to?
        ident = await device.identity()
        battery = await device.battery()
        banner = [f"{ident['software'] or 'InfiniTime'} "
                  f"{ident['firmware'] or '(unknown version)'}", f"BLE FS v{version}"]
        if battery is not None:
            banner.append(f"battery {battery}%")
        print("  |  ".join(banner))

        shell = Shell(fs, device, assume_yes=args.yes)

        if args.command:
            for line in args.command:
                print(f"infinitool> {line}")
                if not await shell.run_line(line):
                    break
            return

        print("Type 'help' for commands, 'exit' to quit.")
        loop = asyncio.get_running_loop()
        while True:
            try:
                line = await loop.run_in_executor(None, input, "infinitool> ")
            except EOFError:
                print()
                break
            if not await shell.run_line(line):
                break
        print("Bye.")


def main():
    parser = argparse.ArgumentParser(
        description="Manage an InfiniTime watch over BLE: files, clock, sensors, "
                    "notifications and firmware.",
        epilog="Run './infinitool.py -c help' for the command list.",
    )
    parser.add_argument("-n", "--name", default="InfiniTime", help="advertised name to match")
    parser.add_argument("-a", "--address", default=None, help="specific address/UUID")
    parser.add_argument("--scan-time", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("-c", "--command", action="append",
                        help="run a command and exit; repeatable")
    parser.add_argument("-y", "--yes", action="store_true",
                        help="skip confirmation prompts (needed to 'flash' under -c)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except FsError as exc:
        sys.exit(f"Error: {exc}")
    except BleakError as exc:
        sys.exit(f"Error: {describe_ble_error(exc)}")
    except KeyboardInterrupt:
        sys.exit("\nInterrupted.")


if __name__ == "__main__":
    main()
