# infinitool

A one-stop management shell for a [PineTime](https://pine64.org/devices/pinetime/)
smartwatch running [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime), over
Bluetooth LE.  It provides the following functionality:

- flash new firmware
- view/update files ("external resources")
- get/set time and date
- read the sensors (battery, step count, accelerometer, heart rate)
- send test notifications, alerts, music info and weather

In short: it manages files, clock, and firmware, in a single connection.

It runs anywhere Python and the [bleak](https://github.com/hbldh/bleak) module are available — communicating with the device via CoreBluetooth on macOS, BlueZ on Linux, and WinRT on Windows.

Everything it uses is served by the normal running firmware; the watch never has to be
put into bootloader mode.

## Prerequisites

You will need a standard Python (3.10+) installation, and [bleak](https://github.com/hbldh/bleak):

```sh
pip install bleak
```

The 3.10 floor is bleak's, not this tool's. Developed and tested against bleak 3.0.2 on
Python 3.12 (macOS/CoreBluetooth); Linux and Windows should work, as bleak covers BlueZ
and WinRT, but neither has been tried. Older bleak releases that don't expose
mtu_size fall back to 23-byte transfers, which works but is slow.

Also, in order to connect successfully to your watch, you will need to go (on the watch) to `Settings` → `Over the Air` and enable "Firmware & files".

## Sample usage

```
$ ./infinitool.py
Found InfiniTime (C3764D0D-...)
InfiniTime 1.16.0  |  BLE FS v4  |  battery 87%
infinitool> ls /images
infinitool> cp !fuji.bin /images/fuji.bin
infinitool> time set
infinitool> flash !pinetime-mcuboot-app-dfu-1.16.0.zip
infinitool> exit
```

Plain paths are on the watch; a local path is prefixed with `!`, as in ftp/sftp.

Uploading a `.zip` can mean two different things, so `cp` asks which you want unless
you say: `cp -c` copies the archive itself to the watch, and `cp -u` unpacks it,
writing the files inside to the watch and creating directories as needed. So the
stock external resources are installed with:

```
infinitool> cp -u !infinitime-resources-1.16.0.zip /
```

That package is flat and ships a `resources.json` saying where each file belongs, so
`-u` follows the manifest — `teko.bin` to `/fonts/teko.bin`, `fuji.bin` to
`/images/fuji.bin` — and offers to delete the obsolete files it lists. A zip with no
manifest falls back to giving each file the path it has inside the archive. Either
way the destination is the root it all lands under.

Any command can also be run non-interactively using `-c`:

```sh
./infinitool.py -c "cp !fuji.bin /images/fuji.bin" -c "ls /images"
```

Run `help` at the prompt for the command list — one line each — or `help COMMAND`
(e.g. `help cp`) for the detail on any one of them.

## Supported commands

### Files

| Command | What it does |
|---|---|
| `ls [-r] [PATH]` | List a directory, default `/`. `-r` descends into subdirectories. A `!` path lists this machine instead. |
| `cp [-c\|-u] SRC DST` | Copy a file; exactly one side must be local (`!`). A `DST` ending in `/` keeps the source basename. Uploading a zip: `-c` copies the archive, `-u` unpacks it onto the watch. Uploads are verified afterwards by reading back the file's size. |
| `rm [-R] PATH` | Delete a file on the watch. `-R` empties a directory first and then removes it, showing the count and asking before it starts. |
| `mkdir PATH` | Create a directory on the watch. |
| `df` | Show free space. |

### Clock, identity and sensors

| Command | What it does |
|---|---|
| `time [set]` | Show the watch clock, its drift from this machine, and its UTC offset; `time set` writes the time zone and clock from this machine. `date` is an alias. |
| `info` | Everything readable in one go: firmware and model, battery, clock and drift, step count, the latest accelerometer sample, heart rate, filesystem version and free space, and the negotiated MTU. |

The sensor values in `info` are read-only — InfiniTime exposes step count,
accelerometer and heart rate as read/notify characteristics with no write path, so
there is no way to set the step count over BLE. Heart rate reads back as "not
measuring" unless the Heart Rate app is running on the watch (or continuous
measurement is enabled in its settings).

### Sending things to the watch

| Command | What it does |
|---|---|
| `notify [-c CAT] TITLE [BODY]` | Send a notification. `CAT` defaults to `simple`; `call` raises the incoming-call screen instead. Title and body together may total 99 bytes. |
| `alert [none\|mild\|high]` | Buzz the watch with an Immediate Alert (default `high`) — the quickest way to make it react. |
| `music FIELD VALUE ...` | Populate the Music app: `track`, `artist`, `album` (text, 40 bytes each), `status` (play/pause), `position`, `length`, `number`, `total`, `speed`, `repeat`, `shuffle`. |
| `weather TEMP [OPTIONS]` | Send current conditions for watchfaces that show weather. `weather forecast MIN/MAX/ICON ...` sends up to five days instead. |

```
infinitool> notify -c sms "Alice" "on my way"
infinitool> alert mild
infinitool> music track "Blue Monday" artist "New Order" status play length 450
infinitool> weather 70F --min 55F --max 74F --icon rain --location Portland
infinitool> weather forecast 12/19/rain 14/22/clouds-sun 15/24/sun
```

Weather temperatures are Celsius unless suffixed with `F`. The icon names are `sun`,
`clouds-sun`, `clouds`, `broken-clouds`, `heavy-shower`, `rain`, `thunderstorm`,
`snow` and `smog`; `--sunrise HH:MM` and `--sunset HH:MM` must be given as a pair, as
the firmware discards one without the other.

### Firmware and the shell itself

| Command | What it does |
|---|---|
| `flash FILE` | Reflash from a `*-dfu-*.zip` package over Nordic legacy DFU, after confirming. The watch reboots, which ends the session. |
| `help [COMMAND]` | List the commands, or explain one in full. |
| `exit` | Quit; `quit` and Ctrl-D also work. |

## Files

| | |
|---|---|
| `infinitool.py` | the tool: BLE filesystem, current time, device info, sensors, notifications, music, weather, `flash` |
| `dfu_bleak.py`  | Nordic legacy DFU over bleak; also usable standalone |
| `unpacker.py`   | unpacks a `*-dfu-*.zip` package (from ota-dfu-python) |

## Licence

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for the DFU code's provenance.
