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
    infinitool> flash !../../build/output/pinetime-mcuboot-app-dfu-1.16.0.zip
    infinitool> exit

Any command can be run non-interactively with -c, which is repeatable:

    ./infinitool.py -c "cp !fuji.bin /images/fuji.bin" -c "ls /images"

Plain paths refer to the remote (device) filesystem. A local path is prefixed with
`!`, as in ftp/sftp where `!` escapes to the local machine:

    cp !./fuji.bin /images/fuji.bin      upload
    cp /fonts/teko.bin !teko.bin         download
    cp !big.bin /images/                 trailing slash keeps the local basename

Services used, all served by the running firmware (no bootloader mode needed):

    adaf0100/adaf0200  InfiniTime file transfer   doc/BLEFS.md, FSService.cpp
    00001530-...       Nordic legacy DFU          DfuService.cpp (via dfu_bleak.py)
    0x1805 / 0x2a2b    Current Time Service       CurrentTimeService.cpp
    0x180a             Device Information         DeviceInformationService.cpp
    0x180f / 0x2a19    Battery level              BatteryInformationService.cpp

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
    are tracked by our own byte count and then VERIFIED by re-listing the parent
    directory and comparing the file size.
  - Read and write chunks are not clamped to the connection MTU ("TODO add mtu somehow",
    FSService.cpp:111), so the client must size them or replies get truncated.
  - A directory listing ends with a terminator entry whose path_length is 0
    (FSService.cpp:292-296); it is not a file.
  - Everything here is gated behind Settings -> "Firmware & files" on the watch. When it
    is Disabled, every request is refused and the version characteristic reads 0, not 4.
  - Reading the Current Time characteristic returns 10 bytes, but the firmware fills in
    only the first 8 (CurrentTimeService.cpp:57-67): dayofweek and reason are left as
    whatever was on the stack. We parse the date and time and ignore those two fields.
"""

import argparse
import asyncio
import datetime
import inspect
import os
import shlex
import struct
import sys
import textwrap

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

    async def _response(self):
        try:
            return await asyncio.wait_for(self.notifications.get(), self.timeout)
        except asyncio.TimeoutError:
            raise FsError(
                f"no response within {self.timeout}s — check Settings -> "
                "'Firmware & files' is Enabled on the watch"
            )

    async def _send(self, payload):
        await self.client.write_gatt_char(UUID_TRANSFER, payload, response=True)

    async def version(self):
        raw = await self.client.read_gatt_char(UUID_VERSION)
        return struct.unpack("<H", raw[:2])[0]

    # -- listing -----------------------------------------------------------------

    async def listdir(self, path):
        encoded = path.encode()
        await self._send(struct.pack("<BBH", CMD_LISTDIR, 0, len(encoded)) + encoded)

        entries = []
        while True:
            data = await self._response()
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
            if path_len and name not in (".", ".."):
                entries.append({"name": name, "size": size, "is_dir": bool(flags & 1)})
            if entry >= total:
                return entries

    async def size_of(self, path):
        """Size of a file on the watch, or None if absent. Used to verify uploads."""
        parent, _, name = path.rpartition("/")
        for item in await self.listdir(parent or "/"):
            if item["name"] == name and not item["is_dir"]:
                return item["size"]
        return None

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
            raise FsError(f"upload finished but {path} is not in the directory listing")
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
            "time": self.cmd_time, "info": self.cmd_info, "flash": self.cmd_flash,
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
                print(f"  {usage:<22}{summary}")
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
        into subdirectories, indenting each level. A ! path lists this machine instead,
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
        entries = await self.fs.listdir(path)
        if depth == 0:
            print(path)
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
        """cp SRC DST
        copy a file; exactly one side must be local (!)

        The ! side says which direction this goes: watch-to-watch and local-to-local are
        both refused. A DST ending in / (or, downloading, an existing local directory)
        keeps the source basename. Uploads are verified afterwards by re-listing the
        parent directory and comparing the size, because the firmware's own write status
        cannot be trusted (see the notes at the top of this file).

          cp !fuji.bin /images/fuji.bin     upload
          cp /fonts/teko.bin !teko.bin      download
          cp !fuji.bin /images/             keep the local basename
        """
        if len(args) != 2:
            raise FsError("usage: cp SRC DST  (exactly one side prefixed with !)")
        src, dst = args
        if is_local(src) == is_local(dst):
            raise FsError(
                "exactly one of SRC and DST must be local (!). "
                "Copying watch-to-watch or local-to-local is not supported."
            )

        if is_local(src):  # upload
            source = local_path(src)
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

    async def cmd_rm(self, args):
        """rm PATH
        delete a file on the watch

        There is no recursive form and no confirmation. The firmware deletes through
        lfs_remove, so an empty directory can be removed this way too; a directory with
        anything in it fails with NOTEMPTY. Local files are refused outright — use your
        own shell for those.
        """
        if len(args) != 1:
            raise FsError("usage: rm PATH")
        if is_local(args[0]):
            raise FsError("rm only deletes on the watch; use your own shell for local files")
        await self.fs.delete(args[0])
        print(f"deleted {args[0]}")

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
        print(f"filesystem   BLE FS v{await self.fs.version()}, "
              f"{await self.fs.freespace()} bytes free")
        print(f"connection   MTU {self.fs.mtu}: {self.fs.read_chunk} B reads, "
              f"{self.fs.write_chunk} B writes")

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
        description="Manage an InfiniTime watch over BLE: files, clock and firmware.",
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
