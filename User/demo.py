# -*- coding: utf-8 -*-

import ctypes
import os
import sys
import subprocess
import threading
import queue
import time
import math
import random
import re
import json

try:
    import serial as _pyserial
    from serial.tools import list_ports as _serial_list_ports
except ImportError:
    _pyserial = None
    _serial_list_ports = None

# C 侧源码当前是 CP936/GBK 编码，子进程输出按同编码解码。
C_SUBPROCESS_ENCODING = "cp936"

USER_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(USER_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from map_eligibility import (
    ACTIVE_MAX_BOMBS,
    ACTIVE_MAX_BOXES,
    count_map_entities as shared_count_map_entities,
    map_scope,
)

MAP_DIR = os.path.join(ROOT_DIR, "map")
_IMPORT_MAP_DIR = MAP_DIR
BUILD_SCRIPT = os.path.join(USER_DIR, "build_fast.ps1")
MAX_BOXES = 10
MAX_TARGETS = 10
SCAN_WAYPOINT_BOX_TAG_BASE = -16
SCAN_WAYPOINT_TARGET_TAG_BASE = -32
RESIDUAL_SIMULATION_MAP_KEY = "__residual_simulation__"
RESIDUAL_OVERLAP_BOX = "*"
MCU_SERIAL_BAUDRATE = 115200


def build_mcu_map_frame(map_rows):
    """Serialize the current 16x12 map for the RT1064 UART map parser.

    Spaces at the end of a row are significant map cells, so only line
    terminators are removed before validation.
    """
    rows = [str(row).rstrip("\r\n") for row in map_rows]
    if len(rows) != 16 or any(len(row) != 12 for row in rows):
        raise ValueError("MCU map frame requires exactly 16 rows of 12 columns")
    if any(any(ord(char) > 127 for char in row) for row in rows):
        raise ValueError("MCU map frame only accepts ASCII map cells")
    payload = "mapstar\r\n"
    payload += "".join(f"{row}\r\n" for row in rows)
    payload += "mapend\r\n"
    return payload.encode("ascii")


class SerialBridge:
    """Small optional UART bridge used by the GUI without blocking its loop."""

    def __init__(self, baudrate=MCU_SERIAL_BAUDRATE):
        self.baudrate = int(baudrate)
        self._serial = None
        self._reader_thread = None
        self._stop_event = threading.Event()
        self._lines = queue.Queue()
        self._write_lock = threading.Lock()

    @staticmethod
    def available_ports():
        if _serial_list_ports is None:
            return []
        try:
            return [info.device for info in _serial_list_ports.comports()]
        except Exception:
            return []

    @property
    def connected(self):
        handle = self._serial
        return handle is not None and bool(getattr(handle, "is_open", True))

    def connect(self, port):
        if _pyserial is None:
            raise RuntimeError("未安装 pyserial，请执行: python -m pip install pyserial")
        port = str(port or "").strip()
        if not port:
            raise RuntimeError("没有选择串口")
        self.close()
        handle = _pyserial.Serial(port=port, baudrate=self.baudrate, timeout=0.1)
        self._serial = handle
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._read_loop, args=(handle,), daemon=True
        )
        self._reader_thread.start()

    def _read_loop(self, handle):
        while not self._stop_event.is_set():
            try:
                raw = handle.readline()
            except Exception as exc:
                self._lines.put(f"[串口读取错误] {exc}")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            self._lines.put(line)

    def write(self, data):
        if not self.connected:
            raise RuntimeError("串口未连接")
        payload = data.encode("ascii") if isinstance(data, str) else bytes(data)
        with self._write_lock:
            self._serial.write(payload)
            self._serial.flush()

    def drain_lines(self):
        lines = []
        while True:
            try:
                lines.append(self._lines.get_nowait())
            except queue.Empty:
                return lines

    def close(self):
        self._stop_event.set()
        handle = self._serial
        self._serial = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass
        thread = self._reader_thread
        self._reader_thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.5)


def relaunch_detached_from_ide_if_needed():
    """Keep GUI runs attached when launched from an IDE."""
    if "--attached" in sys.argv:
        sys.argv.remove("--attached")
    if "--detached" in sys.argv:
        sys.argv.remove("--detached")
    return False


def natural_map_sort_key(name):
    """Sort names like map2 before map10 while keeping non-number names stable."""
    stem = os.path.splitext(str(name))[0]
    parts = re.split(r"(\d+)", stem)
    return tuple((1, int(part)) if part.isdigit() else (0, part.lower()) for part in parts)


def is_custom_map_file(filename):
    """Return whether a directory entry uses the supported map-file format."""
    return os.path.splitext(str(filename))[1].lower() == ".txt"


def has_map_filename_prefix(filename):
    """Keep existing map* names; other valid map files get a numbered name."""
    return os.path.splitext(str(filename))[0].lower().startswith("map")


def _next_numbered_map_filename(map_folder, occupied_filenames, next_number):
    """Find the next free mapN.txt name without overwriting an existing entry."""
    while True:
        filename = f"map{next_number}.txt"
        next_number += 1
        filename_key = filename.casefold()
        if filename_key in occupied_filenames:
            continue
        if os.path.lexists(os.path.join(map_folder, filename)):
            occupied_filenames.add(filename_key)
            continue
        return filename, next_number


def rename_non_map_file(map_folder, filename, occupied_filenames, next_number):
    """Rename a valid non-map file to a free mapN.txt filename.

    The caller has already parsed the source successfully, so unrelated text
    files are never renamed merely because they happen to live in map/.
    """
    source_path = os.path.join(map_folder, filename)
    while True:
        new_filename, next_number = _next_numbered_map_filename(
            map_folder, occupied_filenames, next_number
        )
        try:
            os.rename(source_path, os.path.join(map_folder, new_filename))
        except FileExistsError:
            # A file can appear after the free-name check; leave it untouched
            # and reserve that number before trying the next one.
            occupied_filenames.add(new_filename.casefold())
            continue

        occupied_filenames.discard(filename.casefold())
        occupied_filenames.add(new_filename.casefold())
        return new_filename, next_number


def project_map_directory():
    """Resolve map/ from the active project root instead of an import-time path."""
    if os.path.normcase(os.path.abspath(os.fspath(MAP_DIR))) != os.path.normcase(
        os.path.abspath(_IMPORT_MAP_DIR)
    ):
        return os.fspath(MAP_DIR)
    return os.path.join(os.fspath(ROOT_DIR), "map")


def count_map_entities(map_data):
    """Return the physical box and bomb counts represented by map rows."""
    return shared_count_map_entities(map_data)


def map_catalog_label(key, source):
    key = str(key)
    if source == "builtin":
        return f"地图 {key}"
    if key.lower().startswith("map_"):
        return key[4:].replace("_", " ")
    if re.fullmatch(r"map\d+", key, flags=re.IGNORECASE):
        return f"M{key[3:]}"
    if key.lower().startswith("map") and len(key) > 3:
        return key[3:]
    return key


def build_map_catalog(builtin_maps, custom_maps):
    """Merge map sources into box-count groups sorted by bomb count."""
    entries_by_key = {}

    def add_entries(map_items, source):
        for raw_key, map_data in map_items.items():
            key = str(raw_key)
            scope = map_scope(map_data)
            box_count = scope["box_count"]
            bomb_count = scope["bomb_count"]
            label = map_catalog_label(key, source)
            entries_by_key[key] = {
                "key": key,
                "label": label,
                "map_data": map_data,
                "source": source,
                "box_count": box_count,
                "bomb_count": bomb_count,
                "eligible": scope["eligible"],
                "eligibility_reason": scope["reason"],
            }

    add_entries(builtin_maps or {}, "builtin")
    add_entries(custom_maps or {}, "custom")
    entries = list(entries_by_key.values())
    entries.sort(
        key=lambda entry: (
            entry["box_count"],
            entry["bomb_count"],
            natural_map_sort_key(entry["key"]),
        )
    )

    grouped = {}
    for entry in entries:
        grouped.setdefault(entry["box_count"], []).append(entry)
    return grouped


def fit_button_text(font, text, max_width):
    """Keep compact panel labels inside their button without changing layout."""
    text = str(text)
    max_width = max(1, int(max_width))
    if font.size(text)[0] <= max_width:
        return text

    ellipsis = "..."
    if font.size(ellipsis)[0] > max_width:
        return ellipsis[:1]
    for end in range(len(text) - 1, 0, -1):
        candidate = text[:end].rstrip() + ellipsis
        if font.size(candidate)[0] <= max_width:
            return candidate
    return ellipsis



def warm_c_solver_process():
    """Run a tiny hidden C process once so first real solve does not pay binary cold-start cost."""
    main_exe = os.path.join(USER_DIR, "main.exe")
    if not os.path.exists(main_exe):
        return

    try:
        subprocess.run(
            [main_exe],
            input="WARMUP\nEXIT\n",
            text=True,
            encoding=C_SUBPROCESS_ENCODING,
            errors="replace",
            capture_output=True,
            timeout=5,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            check=False,
        )
    except Exception:
        pass


def ensure_c_solver_built():
    """启动时固定跑一次增量构建，确保 C 求解器 DLL 和 Driver 源码同 步。"""
    log = getattr(sys.stdout, "terminal", sys.stdout)

    if not os.path.exists(BUILD_SCRIPT):
        print(f"[自动编译] 找不到构建脚本: {BUILD_SCRIPT}", file=log)
        return False

    print("[自动编译] 每次启动前执行增量构建：User/build_fast.ps1", file=log)

    try:
        result = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                BUILD_SCRIPT,
            ],
            cwd=USER_DIR,
            text=True,
            encoding=C_SUBPROCESS_ENCODING,
            errors="replace",
            capture_output=True,
            check=False,
        )
    except Exception as e:
        print(f"[自动编译] 启动编译失败: {e}", file=log)
        return False

    if result.stdout:
        print(result.stdout.rstrip(), file=log)
    if result.stderr:
        print(result.stderr.rstrip(), file=log)

    if result.returncode != 0:
        print(f"[自动编译] 编译失败，退出码: {result.returncode}", file=log)
        print("[自动编译] 如果 DLL 被占用，请关闭其它 demo.py/python 进程后重试。", file=log)
        return False

    warm_c_solver_process()
    print("[自动编译] 编译完成，继续启动 demo。", file=log)
    return True


def copy_text_to_clipboard(text):
    if os.name != "nt":
        return False

    try:
        result = subprocess.run(
            ["cmd", "/c", "clip"],
            input=text,
            text=True,
            capture_output=True,
            timeout=2,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            check=False,
        )
        return result.returncode == 0
    except Exception:
        return False


# Try import pygame
try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False
    print("警告：未安装pygame，GUI已禁用")
    print("安装方法：pip install pygame")

# Try import psutil for CPU frequency detection
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    print("警告：未安装psutil，将使用默认CPU频率")
    print("安装方法：pip install psutil")

# ============================================================================
# CPU频率配置
# ============================================================================
# 目标单片机频率 (MHz)
TARGET_MCU_FREQ_MHZ = 600  

# 默认主机CPU频率 (MHz) - 仅在无法读取实际频率时使用
DEFAULT_HOST_CPU_FREQ_MHZ = 3000
_HOST_CPU_FREQ_CACHE_MHZ = None

def _valid_cpu_freq_mhz(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if 100.0 <= value <= 10000.0:
        return value
    return None

def _cpu_freq_candidates_from_psutil():
    if not HAS_PSUTIL:
        return []

    try:
        cpu_freq = psutil.cpu_freq()
    except Exception as e:
        print(f"警告：psutil无法读取CPU频率: {e}")
        return []

    if not cpu_freq:
        return []

    candidates = []
    for value in (cpu_freq.current, cpu_freq.max):
        freq = _valid_cpu_freq_mhz(value)
        if freq is not None:
            candidates.append(freq)
    return candidates

def _cpu_freq_candidates_from_windows_registry():
    if os.name != "nt":
        return []

    candidates = []
    try:
        import winreg

        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor") as root_key:
            index = 0
            while True:
                try:
                    cpu_key_name = winreg.EnumKey(root_key, index)
                except OSError:
                    break

                index += 1
                try:
                    with winreg.OpenKey(root_key, cpu_key_name) as cpu_key:
                        for value_name in ("~MHz", "MHz"):
                            try:
                                freq = _valid_cpu_freq_mhz(winreg.QueryValueEx(cpu_key, value_name)[0])
                            except OSError:
                                continue
                            if freq is not None:
                                candidates.append(freq)
                                break
                except OSError:
                    continue
    except Exception as e:
        print(f"警告：注册表无法读取CPU频率: {e}")

    return candidates

def _cpu_freq_candidates_from_windows_cim():
    if os.name != "nt":
        return []

    command = (
        "$vals=@();"
        "try{"
        "$perf=Get-CimInstance Win32_PerfFormattedData_Counters_ProcessorInformation -ErrorAction Stop | "
        "Where-Object {$_.Name -eq '_Total'};"
        "foreach($p in $perf){$vals += [double]$p.ProcessorFrequency}"
        "}catch{};"
        "try{"
        "Get-CimInstance Win32_Processor -ErrorAction Stop | ForEach-Object {"
        "$vals += [double]$_.CurrentClockSpeed;"
        "$vals += [double]$_.MaxClockSpeed"
        "}"
        "}catch{};"
        "$vals | Where-Object {$_ -gt 0} | ForEach-Object {[math]::Round($_, 0)}"
    )

    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", command],
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=3,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            check=False,
        )
    except Exception as e:
        print(f"警告：PowerShell无法读取CPU频率: {e}")
        return []

    candidates = []
    for match in re.findall(r"\d+(?:\.\d+)?", result.stdout or ""):
        freq = _valid_cpu_freq_mhz(match)
        if freq is not None:
            candidates.append(freq)
    return candidates

def get_host_cpu_freq():
    """
    获取主机CPU频率 (MHz)
    Windows 上 psutil 经常返回 0 或低频，使用多来源读取后缓存结果。
    """
    global _HOST_CPU_FREQ_CACHE_MHZ

    if _HOST_CPU_FREQ_CACHE_MHZ is not None:
        return _HOST_CPU_FREQ_CACHE_MHZ

    candidates = []
    candidates.extend(_cpu_freq_candidates_from_psutil())
    candidates.extend(_cpu_freq_candidates_from_windows_registry())
    if candidates:
        _HOST_CPU_FREQ_CACHE_MHZ = max(candidates)
        return _HOST_CPU_FREQ_CACHE_MHZ

    candidates.extend(_cpu_freq_candidates_from_windows_cim())

    if candidates:
        _HOST_CPU_FREQ_CACHE_MHZ = max(candidates)
        return _HOST_CPU_FREQ_CACHE_MHZ

    print(f"警告：无法读取CPU频率，将使用默认值 {DEFAULT_HOST_CPU_FREQ_MHZ}MHz")
    _HOST_CPU_FREQ_CACHE_MHZ = DEFAULT_HOST_CPU_FREQ_MHZ
    return _HOST_CPU_FREQ_CACHE_MHZ

def get_simulated_time(actual_time):
    """
    将实际运行时间转换为模拟单片机上的运行时间
    
    例如：如果主机是3000MHz，单片机是600MHz
    那么主机上运行0.1秒 = 单片机上运行0.5秒
    """
    host_freq = get_host_cpu_freq()
    # 时间比例 = 主机频率 / 单片机频率
    time_ratio = host_freq / TARGET_MCU_FREQ_MHZ
    return actual_time * time_ratio

def get_simulated_time_with_host_freq(actual_time):
    host_freq = get_host_cpu_freq()
    return actual_time * (host_freq / TARGET_MCU_FREQ_MHZ), host_freq

# 地图配置
MAP_ROWS, MAP_COLS, BLOCK_SIZE = 16, 12, 40
SCREEN_WIDTH = MAP_COLS * BLOCK_SIZE + 300
SCREEN_HEIGHT = MAP_ROWS * BLOCK_SIZE

SOURCE_MAP_TRANSLATION = str.maketrans({
    "-": " ",
    "*": "B",
})
RESIDUAL_EDITOR_TRANSLATION = str.maketrans({
    "-": " ",
})


def _rotate_source_map_ccw(rows):
    """Rotate a 12x16 source map into the project's 16x12 map layout."""
    width = max(len(row) for row in rows)
    padded_rows = [row.ljust(width) for row in rows]
    return ["".join(row) for row in list(zip(*padded_rows))[::-1]]


def normalize_external_map_rows(rows, map_name="<map>", preserve_residual_overlap=False):
    rows = [str(row).rstrip("\r\n") for row in rows]
    rows = [row for row in rows if row]
    if not rows:
        raise ValueError(f"{map_name}: empty map")

    max_width = max(len(row) for row in rows)
    min_width = min(len(row) for row in rows)
    if min_width != max_width:
        rows = [row.ljust(max_width) for row in rows]

    if len(rows) == MAP_ROWS and max_width == MAP_COLS:
        normalized = [row.translate(RESIDUAL_EDITOR_TRANSLATION) for row in rows]
    elif len(rows) == MAP_COLS and max_width == MAP_ROWS:
        translation = (
            RESIDUAL_EDITOR_TRANSLATION
            if preserve_residual_overlap
            else SOURCE_MAP_TRANSLATION
        )
        source_rows = [row.translate(translation) for row in rows]
        normalized = _rotate_source_map_ccw(source_rows)
    else:
        raise ValueError(
            f"{map_name}: unsupported map size {len(rows)}x{max_width}, "
            f"expected {MAP_ROWS}x{MAP_COLS} or source {MAP_COLS}x{MAP_ROWS}"
        )

    normalized = [row[:MAP_COLS].ljust(MAP_COLS) for row in normalized[:MAP_ROWS]]
    allowed = set("# $.@B*0123456789abcdefghijklmnopqrstuvwxyz")
    for y, row in enumerate(normalized):
        for x, char in enumerate(row):
            if char not in allowed:
                raise ValueError(f"{map_name}: unsupported char {char!r} at ({x},{y})")

    if not any("@" in row for row in normalized):
        preferred = (6, 1)
        px, py = preferred
        if 0 <= py < MAP_ROWS and 0 <= px < MAP_COLS and normalized[py][px] == " ":
            row = normalized[py]
            normalized[py] = row[:px] + "@" + row[px + 1:]
        else:
            placed = False
            for y in range(1, MAP_ROWS - 1):
                for x in range(1, MAP_COLS - 1):
                    if normalized[y][x] == " ":
                        row = normalized[y]
                        normalized[y] = row[:x] + "@" + row[x + 1:]
                        placed = True
                        break
                if placed:
                    break
            if not placed:
                raise ValueError(f"{map_name}: no empty cell available for player")

    return normalized


def parse_custom_map_text(text, map_name="<map>"):
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        data = text.splitlines()

    if not isinstance(data, list) or not all(isinstance(row, str) for row in data):
        raise ValueError(f"{map_name}: map file must be a row list or plain text rows")
    return normalize_external_map_rows(data, map_name)


# 颜色配置
COLOR_BG = (0, 0, 139)
COLOR_WALL = (160, 160, 160)
COLOR_BOX = (255, 255, 0)
COLOR_SIMULATED_ERROR_BOX = (150, 150, 70)
COLOR_TARGET = (255, 0, 255)
COLOR_PLAYER = (0, 255, 255)
COLOR_BOMB = (255, 69, 0)
COLOR_GRID = (0, 0, 0)

# Test maps - 按需加载优化

class MapManager:
    """地图管理器 - 实现按需加载和LRU缓存"""
    def __init__(self, cache_size=3):
        self.cache = {}
        self.cache_order = []
        self.cache_size = cache_size
        self._builtin_maps = None
    def _get_builtin_maps(self):
        """延迟加载内置地图数据"""
        if self._builtin_maps is not None:
            return self._builtin_maps
        
        self._builtin_maps = {
            "1": [
# "############",
# "#   #   @  #",
# "###        #",
# "#       #  #",
# "#     $  # #",
# "#### B   # #",
# "#   #     ##",
# "#    #     #",
# "#     #    #",
# "#  B   # $ #",
# "#   $ .  #B#",
# "#        .##",
# "#  ##      #",
# "###.#     ##",
# "#   #      #",
# "############"
          "############",
    "#    @  #  #",
    "# $ B #.  ##",
    "#  $ ####  #",
    "#       ## #",
    "####  ###  #",
    "#.#       ##",
    "# #    ### #",
    "# #        #",
    "# #    #####",
    "# # B    # #",
    "# #$ #   # #",
    "# #B## # # #",
    "#          #",
    "#         .#",
    "############"
    ],
    "2": [
        # "############",
        # "#. #       #",
        # "#  #     $ #",
        # "#  #  #   ##",
        # "#  # #     #",
        # "#       ####",
        # "#  #     $ #",
        # "#  # #     #",
        # "#    .#    #",
        # "##         #",
        # "#  #   #   #",
        # "#  # #     #",
        # "#        # #",
        # "#   $      #",
        # "#     #@  .#",
        # "############"
        "############",
                    "#      @   #",
                    "#   #  .   #",
                    "#  ##   $  #",
                    "#  $ #     #",
                    "#          #",
                    "#   #  #$  #",
                    "#  ..#..   #",
                    "#  $  ##   #",
                    "#       $  #",
                    "#     $    #",
                    "#  $##  .  #",
                    "# . #.  $  #",
                    "# #     ## #",
                    "#          #",
                    "############"
    ],
    "3": [
        "############",
        "#  #       #",
        "## #       #",
        "# .$       #",
        "#  #     # #",
        "#  # #   # #",
        "#  # #   # #",
        "#    #   # #",
        "#    #     #",
        "# ####$    #",
        "# #..   @  #",
        "# # ##     #",
        "# # ##$#   #",
        "#  #   #   #",
        "#          #",
        "############"
    ],
    "4": [
        "############",
        "#          #",
        "#  @       #",
        "#  ####$####",
        "#  #      .#",
        "#  #  $    #",
        "#  #       #",
        "#  #  .    #",
        "#  #########",
        "#  #       #",
        "#  #  $    #",
        "#         .#",
        "#  ####### #",
        "#          #",
        "#          #",
        "############"
    ],
    "5": [
        "############",
        "#          #",
        "## $# ##   #",
        "#      # $ #",
        "#   #  #   #",
        "#  #.  #   #",
        "# #  # #   #",
        "#   #  #   #",
        "#  #    #  #",
        "# $  #     #",
        "#   #   ####",
        "###  #  #. #",
        "# .   #    #",
        "#@##       #",
        "#          #",
        "############"
    ],
    "6": [
        "############",
        "#          #",
        "#  # ###   #",
        "#          #",
        "#  # ###   #",
        "# .#       #",
        "# # # # #  #",
        "#  $ $ $ @##",
        "# # # # #  #",
        "#          #",
        "#  ###     #",
        "#          #",
        "#  ### #####",
        "# .#      .#",
        "#       ####",
        "############"
    ],
    "7": [
        "############",
        "#          #",
        "# # ###### #",
        "# # #.   # #",
        "# # $ ## $ #",
        "# # # #.   #",
        "# # #$#  # #",
        "# #   .  # #",
        "# # @    # #",
        "# ######## #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "############"
    ],
    "8": [
        "############",
        "#   .      #",
        "#   $      #",
        "#######    #",
        "#   @      #",
        "#   $      #",
        "#######    #",
        "#   .      #",
        "#   $      #",
        "#######    #",
        "#   .      #",
        "#          #",
        "## # ### ###",
        "#  ##  ##  #",
        "## #  ##  ##",
        "############"
    ],
    "9": [
        "############",
        "#          #",
        "# ###### # #",
        "# #    # # #",
        "# # $$ # # #",
        "# #  . # # #",
        "# #  #   # #",
        "# #### ### #",
        "#    @     #",
        "#### # #####",
        "#    $     #",
        "#  . # .   #",
        "#    #     #",
        "#          #",
        "#          #",
        "############"
    ],
    "10": [
        "############",
        "##### #    #",
        "# @     # .#",
        "#    ##### #",
        "##$###   # #",
        "## ##    # #",
        "##.# # ##  #",
        "######.# # #",
        "#       $ $#",
        "## ##      #",
        "#   ## ##  #",
        "#          #",
        "#     #### #",
        "#  ##      #",
        "#      #####",
        "############"
    ],
    "11": [
        "############",
        "#.  #      #",
        "# # # #### #",
        "# #   #  # #",
        "# #####B # #",
        "#     #  # #",
        "#####$#  # #",
        "# #     $# #",
        "# # #B######",
        "#   #      #",
        "# #$#### #B#",
        "# #    # # #",
        "# ###  # #.#",
        "#   #. #   #",
        "#@  #  #   #",
        "############"
    ],
    "12": [
        "############",
        "#    @     #",
        "#          #",
        "#          #",
        "#     $    #",
        "######B$ ###",
        "##    $  # #",
        "# ##       #",
        "#  #       #",
        "#          #",
        "# ##   ##  #",
        "# ### ###  #",
        "# #######  #",
        "# #     #  #",
        "# #. . .#  #",
        "############"
        
#        "############",
# "#       #  #",
# "#         ##",
# "# ###  .   #",
# "#     $.   #",
# "#      .$###",
# "# $  # .   #",
# "#      .   #",
# "#   $ #. $ #",
# "# $    .   #",
# "# $##  .   #",
# "#   # #  # #",
# "#  $       #",
# "#          #",
# "#    @  # ##",
# "############"
    ],
    "13": [
        "############",
        "#     #    #",
        "# @   #  . #",
        "#  ####  . #",
        "## #  B  . #",
        "#  # ####  #",
        "#  #    #  #",
        "## #### # ##",
        "#  $  $ # ##",
        "#  #  $ #  #",
        "#  #    #  #",
        "#  ######  #",
        "#          #",
        "#          #",
        "#          #",
        "############"
    ],
    "14": [
        "############",
        "# @        #",
        "#   ####   #",
        "## ##  ##  #",
        "#  #    # ##",
        "#   B.  #  #",
        "## #### .  #",
        "#  #    .  #",
        "#  # ##### #",
        "#  $    #  #",
        "#  $ #  #  #",
        "#  $ #  #  #",
        "#    #     #",
        "#   ###    #",
        "#          #",
        "############"
    ],
    "15": [
        "############",
        "#       # .#",
        "#       #  #",
        "#  @    # .#",
        "#       #  #",
        "#       # .#",
        "#       ####",
        "#          #",
        "# $       ##",
        "#          #",
        "###### ##  #",
        "######$### #",
        "###### ##  #",
        "#  B $  #  #",
        "#       #B #",
        "############"
    ],
    "16": [
        "############",
        "#       # .#",
        "#       #  #",
        "#  @    # .#",
        "#       #  #",
        "#       # .#",
        "#       ####",
        "#          #",
        "# $  #### ##",
        "#       #  #",
        "#   ##  #  #",
        "######$### #",
        "###### ##  #",
        "#  B $  #  #",
        "#       #B #",
        "############"
    ],
    "17": [
        "############",
        "#          #",
        "# @ B $ B  #",
        "### ### ####",
        "# . # . # .#",
        "#   #   #  #",
        "### ### ####",
        "#   $ B $  #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "############"
        # "############",
        # "#-------#--#",
        # "#---------##",
        # "#-##-------#",
        # "#-----$----#",
        # "#-------$###",
        # "#-$--#-.---#",
        # "#------.---#",
        # "#-----#.---#",
        # "#------.---#",
        # "#-$##--.---#",
        # "#---#-#----#",
        # "#--$-------#",
        # "#----------#",
        # "##----@-#-##",
        # "############"
    ],
    "18": [
        "############",
        "#@         #",
        "#  B    B  #",
        "#  ####### #",
        "#  #.#.### #",
        "#  # ###.# #",
        "#  ### ### #",
        "#   $ $ $  #",
        "##  B      #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "############"
    ],
    "19": [
        "############",
        "#@         #",
        "######## # #",
        "#. . . # B #",
        "###### #   #",
        "#    # # B #",
        "# $$ # #   #",
        "# $  # # B #",
        "#####  #####",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "#          #",
        "############"
    ],
    "20": [
                "############",
        "#          #",
        "#   #   #  #",
        "# $ #      #",
        "#   ####B# #",
        "#   #      #",
        "##### $  $ #",
        "#          #",
        "# #######  #",
        "#   B  ##  #",
        "# ##########",
        "# #   #    #",
        "# #   #B#  #",
        "# ##### # .#",
        "#@      #..#",
        "############"
        #    "############",
        #     "#          #",
        #     "#  @       #",
        #     "#  ####$####",
        #     "#  #      .#",
        #     "#  #  $    #",
        #     "#  #       #",
        #     "#  #  . $  #",
        #     "#  #####$###",
        #     "#  #       #",
        #     "#  #  $    #",
        #     "#         .#",
        #     "#  ####### #",
        #     "#     .    #",
        #     "#  .       #",
        #     "############" 

    ],
    "21": [
        "############",
        "#         @#",
        "# $  $     #",
        "# $        #",
        "########## #",
        "#    #   # #",
        "#        B #",
        "########## #",
        "#     ## # #",
        "#        B #",
        "########## #",
        "#    ### # #",
        "#        B #",
        "#          #",
        "#.   .   . #",
        "############"
    ],
    "22": [
        "############",
        "#  #  @    #",
        "##  .# B $ #",
        "#  #### $  #",
        "# ##       #",
        "#  ###  ####",
        "##       #.#",
        "# ###    # #",
        "#        # #",
        "#####    # #",
        "# #    B # #",
        "# #   # $# #",
        "# # # ##B# #",
        "#          #",
        "#.         #",
        "############"
    ],
    "23": [
        "############",
        "#    @     #",
        "## ####  . #",
        "#  # B#    #",
        "# ##B ######",
        "# # $ # ## #",
        "# ## $ ##B #",
        "# # $ #    #",
        "#  #  #    #",
        "#  #  #    #",
        "#  ## #    #",
        "# .        #",
        "#    #######",
        "#       .  #",
        "#        # #",
        "############"
    ],
    "24": [
        "############",
        "#      ## ##",
        "# @    # # #",
        "#      ## ##",
        "#  # $ # B #",
        "# ###B$# # #",
        "#  # $ # # #",
        "#  #    #  #",
        "# #####  # #",
        "#          #",
        "#          #",
        "#   #B#    #",
        "# #######  #",
        "# #     #  #",
        "# #. . .#  #",
        "############"
    ],
    "25": [
        "############",
        "#@ #       #",
        "#  # $ $ $ #",
        "#  ### # ###",
        "# B  # #   #",
        "#### # # B #",
        "#    # ### #",
        "# ####  B  #",
        "#    #######",
        "#### # ... #",
        "#    #     #",
        "#    #######",
        "#          #",
        "#          #",
        "#          #",
        "############"
    ],
    "26": [
        "############",
        "#          #",
        "#   ###### #",
        "#   #  @ # #",
        "#   # #### #",
        "#   #    # #",
        "#   # B  # #",
        "#   # $  # #",
        "#   ###### #",
        "#      $ B #",
        "############",
        "#          #",
        "#  $  B    #",
        "########## #",
        "#. . .     #",
        "############"
    ],
    "27": [
        "############",
        "#@        .#",
        "#  ### #####",
        "# $ #      #",
        "# # # #### #",
        "# # B #  . #",
        "# # # #    #",
        "# # # #$   #",
        "#   ###    #",
        "### ###    #",
        "#     #  . #",
        "# #####    #",
        "# B      B #",
        "# ######$###",
        "#          #",
        "############"
    ],
    "28": [
        "############",
        "#          #",
        "#  B   B B #",
        "#          #",
        "#          #",
        "#    @   $ #",
        "#        $ #",
        "#        $ #",
        "# ######## #",
        "# ######## #",
        "# ######## #",
        "# ######## #",
        "# ######## #",
        "# #      # #",
        "# #. . . # #",
        "############"
    ],
    "29": [
        "############",
        "#          #",
        "# #### ###.#",
        "# #  # #   #",
        "# #  # #   #",
        "# ##$# # ###",
        "#       .  #",
        "# #  #$#BBB#",
        "# #  # #   #",
        "#          #",
        "# ##.# #$# #",
        "# #  # #   #",
        "# #  # #   #",
        "# #### ### #",
        "#     @    #",
        "############"
    ],
    "30": [
        "############",
        "#.  #      #",
        "# # # #### #",
        "# #   #  # #",
        "# #####B # #",
        "#     #  # #",
        "#####$# $# #",
        "# #      # #",
        "# # #B######",
        "#   #      #",
        "# #$#### #B#",
        "# #    # # #",
        "# ###  # #.#",
        "#   #. #   #",
        "#@  #  #   #",
        "############"
    ],
    "31": [
"############",
"#   #   @  #",
"###        #",
"#       #  #",
"#     $  # #",
"#### B   # #",
"#   #     ##",
"#    #     #",
"#     #    #",
"#  B   # $ #",
"#   $ .  #B#",
"#        .##",
"#  ##      #",
"###.#     ##",
"#   #      #",
"############"
    ],
    "32": [
        "############",
        "#          #",
        "# ######## #",
        "#.#      . #",
        "# # # # ## #",
        "# # # # #  #",
        "# # #$# #B##",
        "# # # #    #",
        "#   # # #  #",
        "##### B #  #",
        "#     # #  #",
        "# ##B # #$ #",
        "#     # #  #",
        "# #$###.#  #",
        "#@#     #  #",
        "############"
    ],
    # Imported source maps without an explicit player start.
    # The source maps do not include a player; '@' is placed at {6, 1}.
    "33": [
        "############",
        "#     @    #",
        "#          #",
        "# #.#B#### #",
        "# # $    # #",
        "# #      #B#",
        "#####    # #",
        "####     # #",
        "####     ###",
        "####    .###",
        "###### $####",
        "#### B     #",
        "#######    #",
        "## $       #",
        "##.##      #",
        "############"
    ],
    "34": [
        "############",
        "# .#  @    #",
        "#  $       #",
        "#  ####    #",
        "# #      B #",
        "#    #  ####",
        "##       ###",
        "#  #       #",
        "#  #    .# #",
        "##### $  # #",
        "#####    ###",
        "#######B ###",
        "# #$# ##   #",
        "# .     B  #",
        "#          #",
        "############"
    ],
    "35": [
        "############",
        "#     @    #",
        "#        $ #",
        "# #  #B###.#",
        "# #  #   # #",
        "# #B    ####",
        "# #    #####",
        "# #.   #####",
        "###    #####",
        "###   $  ###",
        "###   #    #",
        "#     # ## #",
        "#  B ####. #",
        "#    ## $ ##",
        "#       ####",
        "############"
    ],
    "36": [
        "############",
        "#     @    #",
        "# $        #",
        "#.# #B#  # #",
        "# #   #  # #",
        "###     B# #",
        "#####    # #",
        "##      .# #",
        "## ##    ###",
        "##   $   ###",
        "#   ##   ###",
        "# ##       #",
        "# .###  B  #",
        "###$###    #",
        "####       #",
        "############"
    ],
    "37": [
        "############",
        "#     @    #",
        "# $        #",
        "#.# #B## # #",
        "###   #  # #",
        "###     B# #",
        "#####    # #",
        "##      .# #",
        "### #    # #",
        "###  $  ####",
        "#    #  ####",
        "# ## #     #",
        "# .#### B  #",
        "###$       #",
        "####       #",
        "############"
    ],
    "38": [
        "############",
        "#     @ #  #",
        "#    B# $  #",
        "#  $ ## #. #",
        "#.    #### #",
        "#  #   ##  #",
        "#      #  ##",
        "# ##   #####",
        "# ##    # ##",
        "#    B######",
        "# #     ####",
        "# #      ###",
        "# #  ###   #",
        "#  B   $   #",
        "#       .  #",
        "############"
    ],
    "39": [
        "############",
        "# .#  @    #",
        "#  $ #     #",
        "#  # ##    #",
        "# ##     B #",
        "#   ##  ####",
        "##       ###",
        "#  #     ###",
        "#       .# #",
        "##### $  ###",
        "#  ##    ###",
        "# #####B ###",
        "# #$####   #",
        "# .     B  #",
        "#          #",
        "############"
    ],
    "40": [
        "############",
        "#     @    #",
        "#  B     . #",
        "### ## #$# #",
        "### B# #####",
        "###    #####",
        "###  $  ####",
        "###.    #  #",
        "###     #  #",
        "####      ##",
        "####   ##  #",
        "# B     #  #",
        "#    ####  #",
        "#       $  #",
        "#       #. #",
        "############"
    ]

        }
        return self._builtin_maps

# 创建全局 MAPS 变量
_map_manager = MapManager()
MAPS = _map_manager._get_builtin_maps()


class Direction(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_int8), ("dy", ctypes.c_int8)]


class Position(ctypes.Structure):
    _fields_ = [("x", ctypes.c_uint8), ("y", ctypes.c_uint8)]


class SokobanRecoveryResult(ctypes.Structure):
    _fields_ = [
        ("status", ctypes.c_int),
        ("next_status", ctypes.c_int),
        ("path", ctypes.POINTER(Direction)),
        ("path_len", ctypes.c_uint16),
        ("observation_pos", Position),
        ("observation_kind", ctypes.c_uint8),
        ("entity_pos", Position),
        ("view_direction", Direction),
    ]


SOKOBAN_RECOVERY_DIRECT = 0
SOKOBAN_RECOVERY_IDENTIFIED = 1
SOKOBAN_RECOVERY_ENTITY_NONE = 0
SOKOBAN_RECOVERY_ENTITY_TARGET = 1
SOKOBAN_RECOVERY_ENTITY_BOX = 2
RECOVERY_STATUS_NAMES = {
    0: "ERROR",
    1: "PATH_READY",
    2: "NEED_OBSERVATION",
    3: "NEED_ID",
    4: "RETRY_OBSERVATION",
    5: "COMPLETE",
    6: "PARTIAL_RETURNED",
}

DIRECTION_MAP = {
    'L': (-1, 0), 'R': (1, 0), 'U': (0, -1), 'D': (0, 1)
}

def load_c_library():
    """加载C求解器库"""
    lib_names = ["sokoban_solver.dll", "sokoban_solver.so", "sokoban_solver.dylib"]

    for lib_name in lib_names:
        lib_path = os.path.join(os.path.dirname(__file__), lib_name)
        if not os.path.exists(lib_path):
            continue

        try:
            lib = ctypes.CDLL(lib_path)
            print(f"已加载C库: {lib_name}")

            # 设置函数签名
            lib.solver_create.restype = ctypes.c_void_p
            lib.solver_destroy.argtypes = [ctypes.c_void_p]
            lib.solver_load_map_from_string.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
            lib.solver_load_map_from_string.restype = ctypes.c_bool
            lib.solver_solve.argtypes = [ctypes.c_void_p]
            lib.solver_solve.restype = ctypes.c_bool
            try:
                lib.solver_solve_robust.argtypes = [ctypes.c_void_p]
                lib.solver_solve_robust.restype = ctypes.c_bool
            except AttributeError:
                pass
            lib.solver_get_solution.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint16)]
            lib.solver_get_solution.restype = ctypes.POINTER(Direction)
            lib.solver_set_strict_target_mode.argtypes = [ctypes.c_void_p, ctypes.c_bool]
            lib.solver_generate_scan_path.argtypes = [ctypes.c_void_p]
            lib.solver_generate_scan_path.restype = ctypes.c_bool
            lib._recovery_api_available = False
            try:
                lib.sokoban_recovery_create.restype = ctypes.c_void_p
                lib.sokoban_recovery_reset.argtypes = [ctypes.c_void_p]
                lib.sokoban_recovery_begin.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
                lib.sokoban_recovery_begin.restype = ctypes.c_bool
                lib.sokoban_recovery_get_result.argtypes = [ctypes.c_void_p]
                lib.sokoban_recovery_get_result.restype = SokobanRecoveryResult
                lib.sokoban_recovery_submit_observation.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p]
                lib.sokoban_recovery_submit_observation.restype = SokobanRecoveryResult
                lib.sokoban_recovery_submit_id.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
                lib.sokoban_recovery_submit_id.restype = SokobanRecoveryResult
                lib._recovery_api_available = True
            except AttributeError:
                print("当前 C 库未导出残局复查接口，演示端只输出残局地图。")
            return lib
        except Exception as e:
            print(f"加载 {lib_name} 失败: {e}")

    print("\n未找到C库！请先编译")
    return None

def solve_with_c(lib, map_data, strict_mode=False):
    """使用C库求解"""
    if not lib:
        return None

    solver = lib.solver_create()
    if not solver:
        print("错误：创建求解器失败")
        return None

    lib.solver_set_strict_target_mode(solver, strict_mode)
    map_string = "|".join(map_data).encode('utf-8')

    if not lib.solver_load_map_from_string(solver, map_string):
        print("错误：加载地图失败")
        lib.solver_destroy(solver)
        return None

    print(f"\n使用C算法求解 (严格模式: {strict_mode})...")
    solve_func = getattr(lib, "solver_solve_robust", lib.solver_solve)
    if not solve_func(solver):
        print("未找到解决方案")
        lib.solver_destroy(solver)
        return None

    length = ctypes.c_uint16()
    path_ptr = lib.solver_get_solution(solver, ctypes.byref(length))
    if not path_ptr:
        lib.solver_destroy(solver)
        return None

    path = [(path_ptr[i].dx, path_ptr[i].dy) for i in range(length.value)]
    lib.solver_destroy(solver)
    return path

class GameDemo:
    def __init__(self, lib):
        if not HAS_PYGAME:
            raise ImportError("pygame required for GUI")

        pygame.init()
        # 【修改 1：完美匹配 120 帧的连发间隔】
        # 200ms 防误触，100ms 连发（1秒10步，手感极其舒适且可控）
        pygame.key.set_repeat(200, 100)
        self.move_duration = 0.1
        self.lib = lib
        self.map_panel_width = 220
        self.screen_w = MAP_COLS * BLOCK_SIZE + self.map_panel_width
        self.screen_h = MAP_ROWS * BLOCK_SIZE + 120 # 增加了底部UI空间
        self.screen = pygame.display.set_mode((self.screen_w, self.screen_h))
        pygame.display.set_caption("推箱子求解器 - 指定目标模式")

        self.font = pygame.font.SysFont("microsoftyahei,simhei,arial", 16)
        self.small_font = pygame.font.SysFont("microsoftyahei,simhei,arial", 13)
        self.clock = pygame.time.Clock()

        # 游戏状态
        self.current_map = "1"
        self.strict_mode = False
        self.auto_path = []
        self.is_playing = False
        self.is_scanning = False
        self.scan_pause_start = 0
        self.last_move_time = 0
        
        # --- 核心新增：总步数统计追踪 ---
        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.solve_planned_steps = 0
        self.scan_planned_turns = 0
        self.solve_planned_turns = 0

        # 动画与特效
        # 【修改 2：完美避免跳帧的动画时长】
        # 80ms 的动画时长 < 100ms 的连发间隔，保证每一帧动画都能丝滑播完
        self.move_duration = 0.08
        self.manual_first_repeat_delay = 0.22
        self.manual_move_interval = 0.14
        self.next_manual_move_time = 0
        self.manual_active_key = None
        self.set_player_mode = False
        self.visual_offsets = {}
        self.star_particles = []
        self.button_hover_states = {}
        self.premium_bg = self.create_premium_background()

        # 地图管理
        self.custom_maps = {}
        self.show_custom_maps = False
        self.map_catalog = {}
        self.selected_box_count = None
        self.load_custom_maps()
        self.map_buttons = []
        self.box_count_buttons = []
        self.control_buttons = []
        self.map_scroll_offset = 0
        self.map_list_scroll_offset = 0
        self.map_list_view_rect = pygame.Rect(0, 0, 0, 0)
        self.map_list_scrollbar_rect = pygame.Rect(0, 0, 0, 0)
        self.map_list_scrollbar_thumb_rect = pygame.Rect(0, 0, 0, 0)
        self.map_list_content_height = 0
        self.map_list_max_scroll = 0
        self.map_list_dragging_scrollbar = False
        self.map_list_drag_offset = 0
        self.auto_solve_all = False
        self.auto_solve_queue = []
        self.auto_solve_scope_exclusions = []
        self.auto_solve_index = 0
        self.auto_solve_base_delay = 0.1
        self.auto_solve_base_move_duration = self.move_duration
        self.auto_solve_speed_multiplier = 2.0
        self.auto_solve_results = []
        self.auto_solve_started_at = None
        self.auto_solve_started_text = ""
        self.auto_solve_current_started_at = None
        self.auto_solve_current_started_text = ""
        self.auto_solve_current_recorded = False
        self.auto_solve_scan_mode = False
        self.scan_auto_default_ids = False
        self.scan_auto_default_id = "0"
        self.right_panel_view_rect = pygame.Rect(0, 0, 0, 0)
        self.right_panel_scrollbar_rect = pygame.Rect(0, 0, 0, 0)
        self.right_panel_scrollbar_thumb_rect = pygame.Rect(0, 0, 0, 0)
        self.right_panel_content_height = 0
        self.right_panel_max_scroll = 0
        self.right_panel_dragging_scrollbar = False
        self.right_panel_drag_offset = 0

        # ID分配模式
        self.id_assignment_mode = False
        self.waiting_for_target = False
        self.current_id = 0
        self.box_ids = {}
        self.target_ids = {}
        self.temp_box_pos = None

        # 扫描求解模式
        self.main_process = None
        self.main_output_queue = queue.Queue()
        self.serial_bridge = SerialBridge()
        self.serial_ports = []
        self.serial_selected_port = os.environ.get("SOKOBAN_SERIAL_PORT", "").strip()
        self.serial_pending_mode = None
        self.serial_waiting_map_ready = False
        self.serial_map_sent_at = None
        self.serial_busy = False
        self.serial_log_lines = []
        self.serial_status_text = "未连接"
        self.serial_mode = None
        self.serial_scan_path_len = 0
        self.serial_scan_path = ""
        self.serial_final_path_len = 0
        self.serial_final_path = ""
        self.serial_waypoints_len = 0
        self.serial_waypoints = []
        self.serial_expecting = None
        self.serial_playback_active = False
        self.serial_scan_playback_started = False
        self.serial_scan_pause_count = 0
        self.serial_scan_plan_time = None
        self.serial_scan_assign_time = None
        self.serial_scan_wait_time = None
        self.serial_recognition_compute_time = None
        self.serial_recognition_total_time = None
        self.refresh_serial_ports()
        self._reset_protocol_phase_mirror()
        self.scan_solve_mode = False
        self.waiting_for_id_input = False
        self.current_scan_pause_index = 0
        self.scan_path_string = ""
        self.id_buttons = []
        self.scanned_box_ids = {}
        self.scanned_target_ids = {}
        self.scan_target_positions = []
        self.scan_target_tags = []
        self.current_target_position = None
        self.scan_focus_duration = 0.28
        self.pending_box_merge = None
        self.pending_start_solve = False
        self.pending_finalize_scan_ids = False

        # 模拟错误按路径轮次排队；旧的 source -> destination 映射仍保留，
        # 供已有脚本/夹具兼容。simulated_error_boxes 只表示历史视觉标记，
        # 不再决定一个箱子是否还能再次注错。
        self.simulated_error_previews = {}
        self.simulated_error_boxes = set()
        self.simulation_error_events = []
        self.simulation_error_history = []
        self.simulation_error_route_round = 0
        self.simulation_error_push_count = 0
        self.simulation_error_selected_round = 0
        self.simulation_error_mode = False
        self.pending_simulated_error_source = None

        # 首轮路径结束后才把当前残局提交给 C 侧复查会话。
        self.solution_origin_mode = None
        self.solution_original_map_text = None
        self.primary_solution_received = False
        self.awaiting_residual_review = False
        self.residual_capture_started_at = None
        self.residual_capture_delay_seconds = 1.0
        self.residual_capture_kind = None
        self.residual_review_submitted = False
        self.last_residual_map = []
        self.last_recovery_result = None
        self.recovery_solver = None
        self.recovery_session = None
        self.recovery_observation_count = 0
        self.recovery_path_count = 0
        self.recovery_path_active = False
        self.recovery_next_status = None
        self.waiting_for_recovery_id_input = False
        self.recovery_observation_pos = None
        self.recovery_observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE
        self.recovery_entity_pos = None
        self.recovery_view_direction = None

        # 路径轨迹与特效
        self.path_trail = []
        self.show_trail = False
        self.trail_type = "box"
        self.bomb_target_wall = None
        self.explosion_effect = None
        self.explosion_duration = 0.8
        self.flame_particles = []

        # 求解时间记录
        self.last_solve_time = None
        self.last_solve_simulated_time = None
        self.scan_solve_compute_time = 0.0
        self.scan_solve_compute_started_at = None

        # 地图创建模式
        self.map_creation_choice_mode = False
        self.creation_choice_buttons = []
        self.map_creation_mode = False
        self.creation_grid = []
        self.creation_save_mode = "new"
        self.creation_source_map = None
        self.current_brush = '#'
        self.dragging = False
        self.selected_tile_coord = None
        self.selected_tile_coord_copied_until = 0.0
        self.last_tile_click_coord = None
        self.last_tile_click_time = 0.0

        # 独立的残局模拟编辑器状态，不复用普通地图创建网格。
        self.residual_simulation_mode = False
        self.residual_simulation_grid = []
        self.residual_simulation_brush = '#'
        self.residual_simulation_buttons = []
        self.residual_simulation_runtime_rows = None
        self.residual_simulation_snapshots = []
        self.residual_simulation_snapshot_cursor = 0
        self.s_key_down_time = 0.0

        self.load_map(self.current_map)

    def load_custom_maps(self):
        """加载 map/ 中全部有效的 .txt 地图，并规范化非 map 前缀文件名。"""
        self.custom_maps = {}
        map_folder = project_map_directory()
        if not os.path.isdir(map_folder):
            self._refresh_map_catalog()
            return

        try:
            directory_entries = list(os.scandir(map_folder))
        except OSError as e:
            print(f"读取地图目录 {map_folder} 失败: {e}")
            self._refresh_map_catalog()
            return

        filenames = sorted(
            (
                entry.name
                for entry in directory_entries
                if entry.is_file() and is_custom_map_file(entry.name)
            ),
            key=natural_map_sort_key,
        )
        occupied_filenames = {entry.name.casefold() for entry in directory_entries}
        next_number = 1

        for filename in filenames:
            filepath = os.path.join(map_folder, filename)
            try:
                with open(filepath, "r", encoding="utf-8-sig") as f:
                    map_data = parse_custom_map_text(f.read(), filename)
            except Exception as e:
                print(f"加载地图 {filepath} 失败: {e}")
                continue

            map_filename = filename
            map_name = os.path.splitext(map_filename)[0]
            if not has_map_filename_prefix(map_filename):
                try:
                    map_filename, next_number = rename_non_map_file(
                        map_folder,
                        map_filename,
                        occupied_filenames,
                        next_number,
                    )
                    new_map_name = os.path.splitext(map_filename)[0]
                    print(f"地图文件已重命名: {filename} -> {map_filename}")
                    if getattr(self, "current_map", None) == map_name:
                        self.current_map = new_map_name
                    if getattr(self, "creation_source_map", None) == map_name:
                        self.creation_source_map = new_map_name
                    map_name = new_map_name
                except OSError as e:
                    print(f"重命名地图 {filepath} 失败，按原名称加载: {e}")

            self.custom_maps[map_name] = map_data
            print(f"加载自定义地图: {map_name}")
        self._refresh_map_catalog()

    def _refresh_map_catalog(self):
        self.map_catalog = build_map_catalog(MAPS, getattr(self, "custom_maps", {}))
        available_counts = sorted(self.map_catalog)
        current_key = str(getattr(self, "current_map", ""))
        current_entry = next(
            (
                entry
                for entries in self.map_catalog.values()
                for entry in entries
                if entry["key"] == current_key
            ),
            None,
        )
        if current_entry is not None:
            self.selected_box_count = current_entry["box_count"]
        elif getattr(self, "selected_box_count", None) not in available_counts:
            self.selected_box_count = available_counts[0] if available_counts else None

    def _get_map_catalog(self):
        catalog = getattr(self, "map_catalog", None)
        if catalog is None:
            self._refresh_map_catalog()
            catalog = self.map_catalog
        return catalog

    def _map_catalog_entry(self, map_key):
        key = str(map_key)
        for entries in self._get_map_catalog().values():
            for entry in entries:
                if entry["key"] == key:
                    return entry
        return None

    def load_map(self, map_name):
        """加载地图"""
        if map_name == RESIDUAL_SIMULATION_MAP_KEY:
            raw = getattr(self, "residual_simulation_runtime_rows", None)
        else:
            self.residual_simulation_runtime_rows = None
            self.residual_simulation_snapshots = []
            self.residual_simulation_snapshot_cursor = 0
            raw = self.custom_maps.get(map_name) or MAPS.get(map_name)
        if not raw:
            print(f"找不到地图数据: {map_name}")
            return
        catalog_entry = self._map_catalog_entry(map_name)
        if catalog_entry is not None:
            if getattr(self, "selected_box_count", None) != catalog_entry["box_count"]:
                self.selected_box_count = catalog_entry["box_count"]
                if hasattr(self, "map_list_scroll_offset"):
                    self.map_list_scroll_offset = 0
        self._close_recovery_session()
        # 重置游戏状态
        self.grid = []
        self.boxes = set()
        self.bombs = set()
        self.targets = set()
        self.player = None
        self.box_ids = {}
        self.target_ids = {}
        self.scanned_box_ids = {}
        self.scanned_target_ids = {}
        self.current_id = 0
        self.waiting_for_target = False
        self.temp_box_pos = None
        self.path_trail = []
        self.show_trail = False
        self.trail_type = "box"
        self.bomb_target_wall = None
        self.explosion_effect = None
        self.flame_particles = []
        self.last_solve_time = None
        self.last_solve_simulated_time = None
        self.scan_solve_compute_time = 0.0
        self.scan_solve_compute_started_at = None
        self.visual_offsets.clear()
        self.last_move_time = 0
        self.next_manual_move_time = 0
        self.manual_active_key = None
        self.scan_pause_start = 0
        self.waiting_for_id_input = False
        self.current_scan_pause_index = 0
        self.scan_path_string = ""
        self.scan_target_positions = []
        self.scan_target_tags = []
        self.current_target_position = None
        self.pending_finalize_scan_ids = False
        self.right_panel_dragging_scrollbar = False
        self.right_panel_drag_offset = 0
        self.map_list_dragging_scrollbar = False
        self.map_list_drag_offset = 0
        self.pending_box_merge = None
        self.selected_tile_coord = None
        self.selected_tile_coord_copied_until = 0.0
        self.last_tile_click_coord = None
        self.last_tile_click_time = 0.0
        self.residual_simulation_mode = False
        self.residual_simulation_grid = []
        self.residual_simulation_brush = '#'
        self.residual_simulation_buttons = []
        self.simulated_error_previews = {}
        self.simulated_error_boxes = set()
        self.simulation_error_events = []
        self.simulation_error_history = []
        self.simulation_error_route_round = 0
        self.simulation_error_push_count = 0
        self.simulation_error_selected_round = 0
        self.simulation_error_mode = False
        self.pending_simulated_error_source = None
        self.solution_origin_mode = None
        self.solution_original_map_text = None
        self.primary_solution_received = False
        self.awaiting_residual_review = False
        self.residual_capture_started_at = None
        self.residual_capture_kind = None
        self.residual_review_submitted = False
        self.last_residual_map = []
        self.last_recovery_result = None
        self.recovery_observation_count = 0
        self.recovery_path_count = 0
        self.recovery_path_active = False
        self.recovery_next_status = None
        self.waiting_for_recovery_id_input = False
        self.recovery_observation_pos = None
        self.recovery_observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE
        self.recovery_entity_pos = None
        self.recovery_view_direction = None

        # 还原步数
        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.strict_mode = False
        self.solve_planned_steps = 0
        self.scan_planned_turns = 0
        self.solve_planned_turns = 0
        self.set_player_mode = False

        has_explicit_ids = False
        for y, row in enumerate(raw):
            row_data = []
            for x, char in enumerate(row):
                if char == '#':
                    row_data.append('#')
                elif char in '$0123456789':
                    self.boxes.add((x, y))
                    if char.isdigit():
                        self.box_ids[(x, y)] = int(char)
                        has_explicit_ids = True
                    row_data.append(' ')
                elif char == RESIDUAL_OVERLAP_BOX:
                    self.boxes.add((x, y))
                    self.targets.add((x, y))
                    row_data.append('.')
                elif char in '.abcdefghijklmnopqrstuvwxyz':
                    self.targets.add((x, y))
                    if char.isalpha() and char != '.':
                        self.target_ids[(x, y)] = ord(char) - ord('a')
                        has_explicit_ids = True
                    elif char.isdigit():
                        self.target_ids[(x, y)] = int(char)
                        has_explicit_ids = True
                    row_data.append(char)
                elif char == '@':
                    self.player = (x, y)
                    row_data.append(' ')
                elif char == 'B':
                    self.bombs.add((x, y))
                    row_data.append(' ')
                else:
                    row_data.append(' ')
            self.grid.append(row_data)

        if has_explicit_ids and not self.id_assignment_mode:
            self.strict_mode = True
            print(f"地图 {map_name} 包含显式ID - 自动启用严格模式")

    def count_turns_in_path(self, path):
        turns = 0
        prev_dir = None

        if isinstance(path, str):
            directions = (DIRECTION_MAP[char] for char in path if char in DIRECTION_MAP)
        else:
            directions = path

        for direction in directions:
            if direction == (0, 0):
                continue
            if prev_dir is not None and direction != prev_dir:
                turns += 1
            prev_dir = direction

        return turns

    def generate_map_with_ids(self):
        """生成仅供 Python 侧检查使用的带 ID 地图字符串。"""
        map_lines = []
        for y in range(len(self.grid)):
            line = ""
            for x in range(len(self.grid[0])):
                pos = (x, y)
                if self.grid[y][x] == '#':
                    line += '#'
                elif pos == self.player:
                    line += '@'
                elif pos in self.bombs:
                    line += 'B'
                elif self._box_overlaps_target(pos):
                    line += self._target_map_symbol(pos)
                elif pos in self.boxes:
                    line += self._box_map_symbol(pos)
                elif pos in self.targets:
                    line += self._target_map_symbol(pos)
                else:
                    line += ' '
            map_lines.append(line)
        return map_lines

    def generate_current_map(self):
        """生成当前游戏状态的地图字符串（无ID）"""
        map_lines = []
        for y in range(len(self.grid)):
            line = ""
            for x in range(len(self.grid[0])):
                pos = (x, y)
                if self.grid[y][x] == '#':
                    line += '#'
                elif pos == self.player:
                    line += '@'
                elif pos in self.bombs:
                    line += 'B'
                elif self._box_overlaps_target(pos):
                    line += '.'
                elif pos in self.boxes:
                    line += '$'
                elif pos in self.targets:
                    line += '.'
                else:
                    line += ' '
            map_lines.append(line)
        return map_lines

    def generate_wire_map(self):
        """Generate the ID-free map format accepted by the MCU/C solver."""
        return self.generate_current_map()

    def refresh_serial_ports(self):
        """Refresh the selectable UART ports while preserving the user's choice."""
        self.serial_ports = SerialBridge.available_ports()
        configured = os.environ.get("SOKOBAN_SERIAL_PORT", "").strip()
        if configured and configured in self.serial_ports:
            self.serial_selected_port = configured
        elif self.serial_selected_port not in self.serial_ports:
            self.serial_selected_port = self.serial_ports[0] if self.serial_ports else ""
        return list(self.serial_ports)

    def _record_serial_line(self, line):
        line = str(line)
        if not line:
            return
        print(f"[MCU] {line}")
        self.serial_log_lines.append(line)
        if len(self.serial_log_lines) > 6:
            del self.serial_log_lines[:-6]
        self.serial_status_text = line

    def cycle_serial_port(self):
        ports = self.refresh_serial_ports()
        if not ports:
            if _pyserial is None:
                self._record_serial_line("未安装 pyserial，请执行 python -m pip install pyserial")
            else:
                self._record_serial_line("未发现串口设备")
            return False
        try:
            index = ports.index(self.serial_selected_port)
        except ValueError:
            index = -1
        self.serial_selected_port = ports[(index + 1) % len(ports)]
        self.serial_status_text = f"已选择 {self.serial_selected_port}"
        print(f"[串口] 已选择 {self.serial_selected_port}")
        return True

    def toggle_serial_connection(self):
        if self.serial_bridge.connected:
            self.serial_bridge.close()
            self.serial_pending_mode = None
            self.serial_waiting_map_ready = False
            self.serial_busy = False
            self.serial_status_text = "已断开"
            print("[串口] 已断开")
            return True

        self.refresh_serial_ports()
        if not self.serial_selected_port:
            if _pyserial is None:
                self._record_serial_line("未安装 pyserial，请执行 python -m pip install pyserial")
            else:
                self._record_serial_line("未发现串口，请先连接 RT1064 或设置 SOKOBAN_SERIAL_PORT")
            return False
        try:
            self.serial_bridge.connect(self.serial_selected_port)
        except Exception as exc:
            self._record_serial_line(f"串口连接失败: {exc}")
            return False
        self.serial_status_text = f"已连接 {self.serial_selected_port}"
        print(f"[串口] 已连接 {self.serial_selected_port} @ {MCU_SERIAL_BAUDRATE}")
        return True

    def _send_pending_serial_command(self):
        mode = self.serial_pending_mode
        if not mode or not self.serial_bridge.connected:
            return False
        command = "SOLVE_DIRECT" if mode == "direct" else "SOLVE_IDENTIFIED"
        try:
            self.serial_bridge.write(command + "\r\n")
        except Exception as exc:
            self._record_serial_line(f"串口命令发送失败: {exc}")
            self.serial_busy = False
            self.serial_pending_mode = None
            self.serial_waiting_map_ready = False
            return False
        self.serial_waiting_map_ready = False
        self.serial_pending_mode = None
        self.serial_status_text = f"已发送 {command}"
        print(f"[串口] -> {command}")
        return True

    def start_serial_solver(self, mode):
        """Send the current map and start direct or interactive-ID recognition solve."""
        if mode not in ("direct", "identified"):
            return False
        if self.serial_busy or self.is_playing or self.scan_solve_mode or self.main_process:
            return False
        if not self.serial_bridge.connected and not self.toggle_serial_connection():
            return False

        try:
            frame = build_mcu_map_frame(self.generate_wire_map())
        except ValueError as exc:
            self._record_serial_line(f"地图发送失败: {exc}")
            return False

        try:
            self.serial_bridge.write(frame)
        except Exception as exc:
            self._record_serial_line(f"地图发送失败: {exc}")
            return False

        self.serial_mode = mode
        self.solution_origin_mode = mode
        self.serial_scan_path_len = 0
        self.serial_scan_path = ""
        self.serial_final_path_len = 0
        self.serial_final_path = ""
        self.serial_waypoints_len = 0
        self.serial_waypoints = []
        self.serial_expecting = None
        self.serial_playback_active = False
        self.serial_scan_playback_started = False
        self.serial_scan_pause_count = 0
        self.serial_scan_plan_time = None
        self.serial_scan_assign_time = None
        self.serial_scan_wait_time = None
        self.serial_recognition_compute_time = None
        self.serial_recognition_total_time = None
        self.current_scan_pause_index = 0
        self.strict_mode = (mode == "identified")
        self.last_solve_time = None
        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.solve_planned_steps = 0
        self.scan_planned_turns = 0
        self.solve_planned_turns = 0

        self.serial_pending_mode = mode
        self.serial_waiting_map_ready = True
        self.serial_map_sent_at = time.monotonic()
        self.serial_busy = True
        self.serial_status_text = "地图已发送，等待 MCU 就绪"
        self.serial_log_lines.append("[Demo] mapstar/mapend 已发送")
        print(f"[串口] 已发送当前地图，等待 Map Ready，再启动 {'识别' if mode == 'identified' else '直接'}求解")
        return True

    def _start_serial_scan_playback(self):
        """Play the MCU scan prefix before any recognition ID is submitted."""
        scan_tokens = [
            c for c in self.serial_scan_path
            if c in DIRECTION_MAP or c == "P"
        ]
        scan_moves = [c for c in scan_tokens if c in DIRECTION_MAP]
        pause_count = sum(1 for c in scan_tokens if c == "P")

        if not scan_tokens or pause_count != len(self.serial_waypoints):
            self.serial_busy = False
            self.serial_status_text = "扫描路径与路标不匹配"
            print(
                "[串口扫描] 拒绝播放: "
                f"pause={pause_count}, waypoint={len(self.serial_waypoints)}"
            )
            return False

        self.solution_origin_mode = "identified"
        self.strict_mode = True
        self.is_scanning = True
        self.is_playing = True
        self.serial_playback_active = True
        self.serial_scan_playback_started = True
        self.serial_scan_pause_count = 0
        self.current_scan_pause_index = 0
        self.scan_target_positions = [
            waypoint['target_pos'] for waypoint in self.serial_waypoints
        ]
        self.scan_target_tags = [
            waypoint.get('tag') for waypoint in self.serial_waypoints
        ]
        self.scan_planned_total = len(scan_moves)
        self.scan_executed_steps = 0
        self.scan_planned_turns = self.count_turns_in_path("".join(scan_moves))
        self.solve_planned_steps = 0
        self.solve_executed_steps = 0
        self.solve_planned_turns = 0
        self.auto_path = [
            DIRECTION_MAP[c] if c in DIRECTION_MAP else (0, 0)
            for c in scan_tokens
        ]
        self.show_trail = True
        self.last_move_time = time.time()
        self.serial_status_text = "扫描路径已接收，按路标选择 ID"
        self.update_path_trail()
        print(
            f"[串口扫描] 先播放识别路径: 移动={len(scan_moves)}, "
            f"暂停={pause_count}"
        )
        return True

    def _start_serial_playback(self):
        """Play the formal solution after direct solve or completed ID scanning."""
        final_moves = [c for c in self.serial_final_path if c in DIRECTION_MAP]
        self.solve_planned_steps = len(final_moves)
        self.solve_executed_steps = 0
        self.solve_planned_turns = self.count_turns_in_path("".join(final_moves))
        self.primary_solution_received = True

        time_str = (
            f"{self.last_solve_time * 1000:.0f}ms"
            if self.last_solve_time is not None else ""
        )
        print(
            f"[串口执行] 开始播放正式方案: "
            f"求解步数={len(final_moves)}, 正式求解耗时={time_str}"
        )

        if not final_moves:
            print("[串口执行] 未接收到有效求解路径")
            return False

        if self.serial_mode == "identified":
            if not self.serial_scan_playback_started:
                self.serial_busy = False
                self.serial_status_text = "固件未先发送识别路径"
                print("[串口执行] 拒绝正式路径: 尚未完成交互式扫描阶段")
                return False
            self.solution_origin_mode = "identified"
            self.strict_mode = True
            self.auto_assign_ids()
            self._resolve_identified_target_boxes()
        else:
            self.solution_origin_mode = "direct"
            self.strict_mode = False
            self._resolve_direct_target_boxes()

        self.serial_playback_active = False
        self.is_scanning = False
        self.play_solution("".join(final_moves))
        return True

    def _serial_scan_visible_entities(self):
        """Return unassigned entities visible from the current player cell."""
        px, py = self.player
        candidates = []
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            pos = (px + dx, py + dy)
            if pos in self.boxes:
                if self._box_id_at(pos) is None:
                    candidates.append(("box", pos, (dx, dy)))
            elif pos in self.targets and self._target_id_at(pos) is None:
                candidates.append(("target", pos, (dx, dy)))
        return candidates

    def _handle_serial_scan_pause(self):
        """Pause serial scan playback until the user chooses a numeric ID."""
        self.serial_scan_pause_count += 1
        current_idx = self.current_scan_pause_index
        player_pos = self.player

        target_pos = None
        waypoint_tag = 0

        # 优先使用从串口接收到的底层路标坐标
        if getattr(self, "serial_waypoints", None) and current_idx < len(self.serial_waypoints):
            wp = self.serial_waypoints[current_idx]
            target_pos = wp['target_pos']
            waypoint_tag = wp.get('tag', 0)
        else:
            candidates = self._serial_scan_visible_entities()
            if candidates:
                _, target_pos, _ = candidates[0]

        if target_pos is not None:
            look_dx = target_pos[0] - player_pos[0]
            look_dy = target_pos[1] - player_pos[1]
            look_dir_map = {(1, 0): "向右", (-1, 0): "向左", (0, 1): "向下", (0, -1): "向上"}
            dir_str = look_dir_map.get((look_dx, look_dy), f"({look_dx},{look_dy})")

            # 对应 Driver/rt1064_porting_example.c: waypoint_tag 解码实体类型
            is_box = (-32 < waypoint_tag <= -16)
            is_target = (-48 < waypoint_tag <= -32)
            if not is_box and not is_target:
                is_box = target_pos in self.boxes
                is_target = target_pos in self.targets

            entity_str = "箱子" if is_box and not is_target else ("目标点" if is_target and not is_box else "实体")
            print(
                f"[串口扫描] P#{self.serial_scan_pause_count} "
                f"小车={player_pos} 目标{entity_str}={target_pos} "
                f"转向={dir_str}({look_dx},{look_dy})，等待用户选择 ID"
            )
        else:
            print(
                f"[串口扫描] P#{self.serial_scan_pause_count} "
                f"player={self.player} 附近没有可补 ID 的实体"
            )

        self.strict_mode = True
        self.waiting_for_id_input = True
        self.current_target_position = target_pos
        self.scan_pause_start = 0
        self.is_playing = False
        self.serial_status_text = (
            f"扫描暂停 {current_idx + 1}/{len(self.serial_waypoints)}，请选择 ID"
        )
        if self.show_trail:
            self.update_path_trail()

    def poll_serial_bridge(self):
        """Drain MCU output and advance the map-ready -> solve command handshake and solution playback."""
        bridge = getattr(self, "serial_bridge", None)
        if bridge is None:
            return
        for line in bridge.drain_lines():
            self._record_serial_line(line)
            clean_line = line.strip()

            if self.serial_waiting_map_ready and "map ready" in clean_line.lower():
                self._send_pending_serial_command()
                continue

            if "scan ready, waiting for numeric ids" in clean_line.lower():
                self._start_serial_scan_playback()
                continue

            # 匹配扫描路径长度: "Scan path len:46"
            m_scan = re.search(r"Scan path len:\s*(\d+)", clean_line, re.IGNORECASE)
            if m_scan:
                self.serial_scan_path_len = int(m_scan.group(1))
                self.serial_scan_path = ""
                self.serial_expecting = "scan"
                continue

            # 匹配扫描路标长度: "Scan waypoints len:4"
            m_wp_len = re.search(r"Scan waypoints len:\s*(\d+)", clean_line, re.IGNORECASE)
            if m_wp_len:
                self.serial_waypoints_len = int(m_wp_len.group(1))
                self.serial_waypoints = []
                self.serial_expecting = "waypoints"
                continue

            # 匹配路标行: "WP:2,2,-16,2,1"
            m_wp = re.search(r"(?:WP:)\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)", clean_line)
            if m_wp:
                tx, ty, tag, px, py = map(int, m_wp.groups())
                self.serial_waypoints.append({
                    'target_pos': (tx, ty),
                    'tag': tag,
                    'player_pos': (px, py)
                })
                if self.serial_waypoints_len and len(self.serial_waypoints) >= self.serial_waypoints_len:
                    self.serial_expecting = None
                continue

            # 匹配求解路径长度: "Final path len:122" 或 "Path len:122"
            m_final = re.search(r"(?:Final\s+)?path len:\s*(\d+)", clean_line, re.IGNORECASE)
            if m_final:
                self.serial_final_path_len = int(m_final.group(1))
                self.serial_final_path = ""
                self.serial_expecting = "final"
                continue

            m_scan_plan_time = re.search(
                r"Scan plan time:\s*([0-9.]+)\s*ms",
                clean_line,
                re.IGNORECASE,
            )
            if m_scan_plan_time:
                self.serial_scan_plan_time = float(m_scan_plan_time.group(1)) / 1000.0
                continue

            m_scan_assign_time = re.search(
                r"Scan ID assign time:\s*([0-9.]+)\s*ms",
                clean_line,
                re.IGNORECASE,
            )
            if m_scan_assign_time:
                self.serial_scan_assign_time = float(m_scan_assign_time.group(1)) / 1000.0
                continue

            m_scan_wait_time = re.search(
                r"Scan ID wait time:\s*([0-9.]+)\s*ms",
                clean_line,
                re.IGNORECASE,
            )
            if m_scan_wait_time:
                self.serial_scan_wait_time = float(m_scan_wait_time.group(1)) / 1000.0
                continue

            m_recognition_compute = re.search(
                r"Recognition compute time\(no wait\):\s*([0-9.]+)\s*ms",
                clean_line,
                re.IGNORECASE,
            )
            if m_recognition_compute:
                self.serial_recognition_compute_time = (
                    float(m_recognition_compute.group(1)) / 1000.0
                )
                continue

            m_recognition_total = re.search(
                r"Recognition total time:\s*([0-9.]+)\s*ms",
                clean_line,
                re.IGNORECASE,
            )
            if m_recognition_total:
                self.serial_recognition_total_time = (
                    float(m_recognition_total.group(1)) / 1000.0
                )
                continue

            # "Solve time" now measures only solver_solve_robust().
            m_time = re.search(r"Solve time(?:\([^)]*\))?:\s*([0-9.]+)\s*ms", clean_line, re.IGNORECASE)
            if m_time:
                time_ms = float(m_time.group(1))
                self.last_solve_time = time_ms / 1000.0
                self.last_solve_simulated_time = time_ms / 1000.0
                self.serial_busy = False
                self.serial_expecting = None
                self.serial_status_text = f"正式求解完成 ({time_ms:.0f}ms)"
                self._start_serial_playback()
                continue

            # 收集扫描路径字符
            if self.serial_expecting == "scan":
                chars = "".join([c for c in clean_line.upper() if c in "UDLRP?"])
                if chars:
                    self.serial_scan_path += chars
                    if len(self.serial_scan_path) >= self.serial_scan_path_len:
                        self.serial_expecting = None
                continue

            # 收集最终求解路径字符
            if self.serial_expecting == "final":
                chars = "".join([c for c in clean_line.upper() if c in "UDLR"])
                if chars:
                    self.serial_final_path += chars
                    if len(self.serial_final_path) >= self.serial_final_path_len:
                        self.serial_expecting = None
                continue

            if self.serial_busy and (
                "solve failed" in clean_line.lower()
                or "scan failed" in clean_line.lower()
                or "map load failed" in clean_line.lower()
                or "solver create failed" in clean_line.lower()
                or "no solution" in clean_line.lower()
            ):
                self.serial_busy = False
                self.serial_expecting = None
                self.serial_status_text = "求解失败"

        if (
            self.serial_waiting_map_ready
            and self.serial_map_sent_at is not None
            and time.monotonic() - self.serial_map_sent_at >= 3.0
        ):
            self._record_serial_line("未等到 Map Ready，按超时继续发送求解命令")
            self._send_pending_serial_command()

    def generate_recovery_bootstrap_map(self):
        """Keep editor-only overlap truth while initializing a recovery session."""
        map_lines = []
        for y in range(len(self.grid)):
            line = ""
            for x in range(len(self.grid[0])):
                pos = (x, y)
                if self.grid[y][x] == '#':
                    line += '#'
                elif pos == self.player:
                    line += '@'
                elif pos in self.bombs:
                    line += 'B'
                elif self._box_overlaps_target(pos):
                    line += RESIDUAL_OVERLAP_BOX
                elif pos in self.boxes:
                    line += '$'
                elif pos in self.targets:
                    line += '.'
                else:
                    line += ' '
            map_lines.append(line)
        return map_lines

    @staticmethod
    def _valid_entity_id(value):
        return isinstance(value, int) and not isinstance(value, bool) and 0 <= value < 10

    def _entity_id_at(self, pos, manual_ids, scanned_ids):
        manual_id = manual_ids.get(pos)
        if self._valid_entity_id(manual_id):
            return manual_id
        scanned_id = scanned_ids.get(pos)
        if self._valid_entity_id(scanned_id):
            return scanned_id
        return None

    def _box_id_at(self, pos):
        return self._entity_id_at(pos, self.box_ids, self.scanned_box_ids)

    def _target_id_at(self, pos):
        return self._entity_id_at(pos, self.target_ids, self.scanned_target_ids)

    def _is_identified_solution(self):
        return self.solution_origin_mode == "identified"

    def _box_overlaps_target(self, pos):
        return pos in self.boxes and pos in self.targets

    def _target_overlap_marker(self, pos):
        return "x" if self._box_overlaps_target(pos) else None

    def _box_target_ids_mismatch(self, box_pos, target_pos):
        if not self._is_identified_solution():
            return False
        box_id = self._box_id_at(box_pos)
        target_id = self._target_id_at(target_pos)
        return box_id is not None and target_id is not None and box_id != target_id

    def _hide_box_under_mismatched_target(self, pos):
        return (
            self._box_overlaps_target(pos)
            and self._box_target_ids_mismatch(pos, pos)
        )

    def _box_map_symbol(self, pos):
        box_id = self._box_id_at(pos)
        return str(box_id) if box_id is not None else '$'

    def _target_map_symbol(self, pos):
        target_id = self._target_id_at(pos)
        return chr(ord('a') + target_id) if target_id is not None else '.'

    def _manual_id_commands(self):
        for entity_ids in (self.box_ids, self.target_ids):
            for (x, y), entity_id in sorted(
                entity_ids.items(), key=lambda item: (item[0][1], item[0][0])
            ):
                if self._valid_entity_id(entity_id):
                    yield f"SET_ID_AT:{x},{y},{entity_id}"

    def generate_residual_map(self):
        """生成供残局复查使用的 ID-free 当前地图快照。"""
        return self.generate_wire_map()

    def _prepare_residual_review(self, origin_mode, map_data):
        self._close_recovery_session()
        self.solution_origin_mode = origin_mode
        self.solution_original_map_text = "|".join(map_data)
        self.primary_solution_received = False
        self.awaiting_residual_review = False
        self.residual_capture_started_at = None
        self.residual_capture_kind = None
        self.residual_review_submitted = False
        self.last_residual_map = []
        self.last_recovery_result = None
        self.recovery_observation_count = 0
        self.recovery_path_count = 0
        self.recovery_observation_pos = None
        self.recovery_observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE
        self.recovery_entity_pos = None
        self.recovery_view_direction = None
        self.simulation_error_route_round = 0
        self.simulation_error_push_count = 0
        self.simulation_error_selected_round = 0

    def _close_recovery_session(self):
        recovery = getattr(self, "recovery_session", None)
        solver = getattr(self, "recovery_solver", None)
        if recovery and self.lib and hasattr(self.lib, "sokoban_recovery_reset"):
            try:
                self.lib.sokoban_recovery_reset(recovery)
            except Exception:
                pass
        if solver and self.lib:
            try:
                self.lib.solver_destroy(solver)
            except Exception:
                pass
        self.recovery_solver = None
        self.recovery_session = None
        self.recovery_path_active = False
        self.recovery_next_status = None
        self.waiting_for_recovery_id_input = False
        self.recovery_observation_pos = None
        self.recovery_observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE
        self.recovery_entity_pos = None
        self.recovery_view_direction = None

    def _abort_recovery_session(self, reason):
        """Close an unusable recovery session and retire impossible events."""
        self._finish_pending_simulation_error_events(reason)
        if getattr(self, "protocol_phase_mirror", "empty") in (
            "recovery_pending",
            "recovery",
        ):
            self._set_protocol_phase_mirror("recovery_error", reason)
        self._close_recovery_session()

    def _discard_residual_review(self):
        """Discard a review session that never reached a final solution path."""
        self._close_recovery_session()
        self.solution_origin_mode = None
        self.solution_original_map_text = None
        self.primary_solution_received = False
        self.awaiting_residual_review = False
        self.residual_capture_started_at = None
        self.residual_capture_kind = None
        self.residual_review_submitted = False
        self.recovery_observation_count = 0
        self.recovery_path_count = 0
        self.last_residual_map = []
        self.last_recovery_result = None
        self.simulation_error_mode = False
        self.pending_simulated_error_source = None
        self.simulation_error_events = []
        self.simulated_error_previews = {}
        self.simulation_error_route_round = 0
        self.simulation_error_push_count = 0

    def _queue_residual_review_after_primary_solution(self):
        if (
            not self.primary_solution_received
            or not self.solution_origin_mode
            or self.residual_review_submitted
        ):
            return False
        self.awaiting_residual_review = True
        self.residual_capture_started_at = None
        self.residual_capture_kind = "first"
        self._set_protocol_phase_mirror("recovery_pending", "primary path complete")
        return True

    def _queue_next_residual_capture(self):
        """为当前会话排队下一轮完整残局观测。

        复推路径只是一个进展段；C 侧可能在每段之后再次请求观测，且轮数
        不设固定上限。因此 ``recovery_observation_count`` 只用于界面计数，
        不能作为会话终止条件。

        目标格停车由 Driver 在返回路径中统一完成；演示层只在路径执行完后
        提交观测，不再复制一份“玩家不能停在目标格”的终止规则。
        """
        if not self.recovery_session:
            return False
        self.awaiting_residual_review = True
        self.residual_capture_started_at = None
        self.residual_capture_kind = "recheck"
        return True

    # 保留旧私有名称兼容已有调用方和测试夹具；它现在表示“下一轮”，不再
    # 表示固定的第二轮。
    def _queue_second_residual_capture(self):
        return self._queue_next_residual_capture()

    def _residual_review_ready_to_submit(self):
        if not self.awaiting_residual_review:
            return False
        if self.simulation_error_mode or self.pending_simulated_error_source is not None:
            return False
        if self._residual_snapshot_submission_available():
            # A queued mapN is deterministic input.  Do not let the one-second
            # live capture timer race past it; F6 explicitly submits the entry.
            return False
        if self.is_playing or getattr(self, "auto_path", None) or self.pending_box_merge:
            return False
        return time.time() - self.last_move_time >= self.move_duration

    def _advance_residual_capture(self):
        if not self._residual_review_ready_to_submit():
            return False
        if self.residual_capture_started_at is None:
            self.residual_capture_started_at = time.time()
            capture_number = self.recovery_observation_count + 1
            print(
                f"[残局复查] Python 模拟拍摄第 {capture_number} 张开始，"
                f"{self.residual_capture_delay_seconds:.1f} 秒后自动提交残局地图。"
            )
            return False
        if (
            time.time() - self.residual_capture_started_at
            < self.residual_capture_delay_seconds
        ):
            return False
        self.awaiting_residual_review = False
        self._submit_residual_map_for_review()
        return True

    def _submit_residual_map_for_review(self):
        first_observation = not self.residual_review_submitted
        if not self.solution_original_map_text:
            return False
        if not first_observation and not self.recovery_session:
            return False
        # A submitted full map starts a new Driver identification ledger.  Do
        # not retain any entity metadata from the previous observation round.
        self.recovery_observation_pos = None
        self.recovery_observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE
        self.recovery_entity_pos = None
        self.recovery_view_direction = None
        map_data = self.generate_residual_map()
        self.last_residual_map = list(map_data)
        if first_observation:
            self.residual_review_submitted = True
        capture_number = self.recovery_observation_count + 1
        print("\n" + "=" * 50)
        print(
            f"Python 模拟拍摄第 {capture_number} 张完成，"
            "输入给复查函数的当前残局地图："
        )
        for line in map_data:
            print(line)
        print("=" * 50)

        if not self.lib or not getattr(self.lib, "_recovery_api_available", False):
            print("[残局复查] 当前 C 库没有可调用的复查接口，已保留残局地图快照。")
            return False

        try:
            if first_observation:
                self.recovery_solver = self.lib.solver_create()
                self.recovery_session = self.lib.sokoban_recovery_create()
                if not self.recovery_solver or not self.recovery_session:
                    print("[残局复查] 无法创建 C 侧复查会话。")
                    self._abort_recovery_session("无法创建 C 侧复查会话")
                    return False
                if not self.lib.solver_load_map_from_string(
                    self.recovery_solver, self.solution_original_map_text.encode("utf-8")
                ):
                    print("[残局复查] 无法载入首轮正确地图，未提交残局。")
                    self._abort_recovery_session("无法载入首轮正确地图")
                    return False

                recovery_mode = (
                    SOKOBAN_RECOVERY_IDENTIFIED
                    if self._is_identified_solution()
                    else SOKOBAN_RECOVERY_DIRECT
                )
                self.lib.solver_set_strict_target_mode(
                    self.recovery_solver,
                    recovery_mode == SOKOBAN_RECOVERY_IDENTIFIED,
                )
                if not self.lib.sokoban_recovery_begin(
                    self.recovery_session, self.recovery_solver, recovery_mode
                ):
                    print("[残局复查] C 侧复查会话初始化失败。")
                    self._abort_recovery_session("C 侧复查会话初始化失败")
                    return False
                self._set_protocol_phase_mirror("recovery", "sokoban_recovery_begin")

            result = self.lib.sokoban_recovery_submit_observation(
                self.recovery_session,
                self.recovery_solver,
                "|".join(map_data).encode("utf-8"),
            )
            # This counter is only the number of photos submitted for UI state.
            # Driver's consecutive-same-observation streak deliberately does
            # not consume malformed or RETRY_OBSERVATION photos.
            self.recovery_observation_count += 1
            self.residual_capture_kind = None
            return self._handle_recovery_result(result)
        except Exception as e:
            print(f"[残局复查] 调用 C 侧复查接口失败: {e}")
            self._abort_recovery_session("调用 C 侧复查接口失败")
            return False

    def _copy_recovery_path(self, result):
        if result.path_len == 0:
            return []
        if not result.path:
            return None
        path = []
        for index in range(result.path_len):
            direction = result.path[index]
            move = (int(direction.dx), int(direction.dy))
            if move not in DIRECTION_MAP.values():
                return None
            path.append(move)
        return path

    def _start_recovery_path_playback(self, result):
        path = self._copy_recovery_path(result)
        if path is None:
            print("[残局复查] C 侧返回了无效路径。")
            self._abort_recovery_session("C 侧返回无效路径")
            return False
        # One observation can yield several PATH_READY segments separated by
        # ID submissions.  Number paths themselves so each segment gets a
        # distinct simulation-error round.
        route_round = max(0, int(getattr(self, "recovery_path_count", 0))) + 1
        self.recovery_path_count = route_round
        self._begin_simulation_error_route(route_round)
        if not path:
            # PATH_READY with an empty segment still closes a route.  Pending
            # events for this round must not leak into a later recovery path.
            self._simulation_error_round_finished()
            return self._advance_recovery_after_path(
                result.next_status,
                result.observation_pos,
                getattr(result, "observation_kind", SOKOBAN_RECOVERY_ENTITY_NONE),
                getattr(result, "entity_pos", None),
                getattr(result, "view_direction", None),
            )

        self.recovery_path_active = True
        self.recovery_next_status = int(result.next_status)
        self.recovery_observation_pos = self._recovery_observation_position(
            result.observation_pos
        )
        self.recovery_observation_kind = int(
            getattr(result, "observation_kind", SOKOBAN_RECOVERY_ENTITY_NONE)
        )
        self.recovery_entity_pos = self._recovery_observation_position(
            getattr(result, "entity_pos", None)
        )
        self.recovery_view_direction = self._recovery_direction(
            getattr(result, "view_direction", None)
        )
        self.auto_path = path
        self.is_playing = True
        self.is_scanning = False
        self.show_trail = True
        self.update_path_trail()
        return True

    def _recovery_observation_position(self, observation_pos):
        if observation_pos is None:
            return None
        if isinstance(observation_pos, tuple) and len(observation_pos) == 2:
            return (int(observation_pos[0]), int(observation_pos[1]))
        try:
            return (int(observation_pos.x), int(observation_pos.y))
        except (AttributeError, TypeError, ValueError):
            return None

    def _recovery_direction(self, view_direction):
        if view_direction is None:
            return None
        if isinstance(view_direction, tuple) and len(view_direction) == 2:
            try:
                return (int(view_direction[0]), int(view_direction[1]))
            except (TypeError, ValueError):
                return None
        try:
            return (int(view_direction.dx), int(view_direction.dy))
        except (AttributeError, TypeError, ValueError):
            return None

    def _validated_recovery_id_metadata(
        self, observation_pos, observation_kind, entity_pos, view_direction
    ):
        observation = self._recovery_observation_position(observation_pos)
        entity = self._recovery_observation_position(entity_pos)
        direction = self._recovery_direction(view_direction)
        try:
            kind = int(observation_kind)
        except (TypeError, ValueError):
            return None

        if (
            observation is None
            or entity is None
            or direction not in DIRECTION_MAP.values()
            or not (0 <= observation[0] < MAP_COLS and 0 <= observation[1] < MAP_ROWS)
            or not (0 <= entity[0] < MAP_COLS and 0 <= entity[1] < MAP_ROWS)
            or (observation[0] + direction[0], observation[1] + direction[1]) != entity
        ):
            return None
        if kind == SOKOBAN_RECOVERY_ENTITY_BOX:
            positions = self.boxes
        elif kind == SOKOBAN_RECOVERY_ENTITY_TARGET:
            positions = self.targets
        else:
            return None
        if entity not in positions:
            return None
        return observation, kind, entity, direction

    def _begin_recovery_id_input(
        self,
        observation_pos,
        observation_kind=None,
        entity_pos=None,
        view_direction=None,
    ):
        metadata = self._validated_recovery_id_metadata(
            observation_pos, observation_kind, entity_pos, view_direction
        )
        if metadata is None:
            print(
                "[残局复查] C 侧返回的识别元数据无效，"
                "可能是接口/DLL 未同步。"
            )
            self._abort_recovery_session("C 侧返回无效或不同步的复查识别元数据")
            return False
        (
            self.recovery_observation_pos,
            self.recovery_observation_kind,
            self.recovery_entity_pos,
            self.recovery_view_direction,
        ) = metadata
        self.waiting_for_recovery_id_input = True
        print(
            "[残局复查] 到达复查识别点 "
            f"{self.recovery_observation_pos}，观察实体 {self.recovery_entity_pos}，"
            f"方向 {self.recovery_view_direction}；请选择 ID、识别失败或跳过。"
        )
        return True

    def _finish_recovery_session(self, status):
        self._finish_pending_simulation_error_events(
            "恢复会话结束前没有产生事件对应的路径"
        )
        status_name = RECOVERY_STATUS_NAMES.get(status, str(status))
        print(f"[残局复查] 会话结束: {status_name}")
        if getattr(self, "protocol_phase_mirror", "empty") in (
            "recovery_pending",
            "recovery",
        ):
            self._set_protocol_phase_mirror("recovery_done", status_name)
        self._close_recovery_session()
        return True

    def _advance_recovery_after_path(
        self,
        next_status,
        observation_pos,
        observation_kind=None,
        entity_pos=None,
        view_direction=None,
    ):
        self.recovery_path_active = False
        self.recovery_next_status = None
        if next_status == 2:
            return self._queue_next_residual_capture()
        if next_status == 3:
            return self._begin_recovery_id_input(
                observation_pos,
                observation_kind,
                entity_pos,
                view_direction,
            )
        if next_status in (5, 6):
            return self._finish_recovery_session(next_status)
        print(
            "[残局复查] 路径执行后的下一阶段无效: "
            f"{RECOVERY_STATUS_NAMES.get(next_status, str(next_status))}"
        )
        self._abort_recovery_session("路径执行后的下一阶段无效")
        return False

    def _finish_recovery_path_playback(self):
        if not self.recovery_path_active:
            return False
        self._simulation_error_round_finished()
        return self._advance_recovery_after_path(
            self.recovery_next_status,
            self.recovery_observation_pos,
            self.recovery_observation_kind,
            self.recovery_entity_pos,
            self.recovery_view_direction,
        )

    def _handle_recovery_result(self, result):
        self.last_recovery_result = result
        status = int(result.status)
        next_status = int(result.next_status)
        print(
            f"[残局复查] 第 {self.recovery_observation_count} 张残局审核结果: "
            f"{RECOVERY_STATUS_NAMES.get(status, str(status))} -> "
            f"{RECOVERY_STATUS_NAMES.get(next_status, str(next_status))}, "
            f"路径长度: {result.path_len}"
        )
        if status == 1:
            return self._start_recovery_path_playback(result)
        if status == 2:
            return self._queue_next_residual_capture()
        if status == 3:
            return self._begin_recovery_id_input(
                result.observation_pos,
                getattr(result, "observation_kind", SOKOBAN_RECOVERY_ENTITY_NONE),
                getattr(result, "entity_pos", None),
                getattr(result, "view_direction", None),
            )
        if status in (5, 6):
            return self._finish_recovery_session(status)
        if status == 4:
            print("[残局复查] 当前残局观测无效，保留会话并重新申请观测。")
            return self._queue_next_residual_capture()
        else:
            print("[残局复查] C 侧复查会话返回错误。")
        self._abort_recovery_session("C 侧复查会话返回错误")
        return False

    def _record_recovery_id(self, value, observation_kind=None, entity_pos=None):
        if not self._valid_entity_id(value):
            return False
        if observation_kind is None:
            observation_kind = self.recovery_observation_kind
        if entity_pos is None:
            entity_pos = self.recovery_entity_pos
        entity_pos = self._recovery_observation_position(entity_pos)
        if observation_kind == SOKOBAN_RECOVERY_ENTITY_BOX:
            positions = self.boxes
            primary_ids = self.box_ids
            scanned_ids = self.scanned_box_ids
        elif observation_kind == SOKOBAN_RECOVERY_ENTITY_TARGET:
            positions = self.targets
            primary_ids = self.target_ids
            scanned_ids = self.scanned_target_ids
        else:
            return False
        if entity_pos not in positions:
            return False
        primary_ids[entity_pos] = value
        scanned_ids[entity_pos] = value
        return True

    def handle_recovery_id_input(self, id_input):
        if not self.waiting_for_recovery_id_input or not self.recovery_session:
            return False
        if id_input == "?":
            value = -2
        elif id_input == "no":
            value = -1
        elif isinstance(id_input, str) and id_input.isdigit():
            value = int(id_input)
        else:
            return False

        print(
            f"[残局复查ID] observation={self.recovery_observation_pos} "
            f"entity={self.recovery_entity_pos} "
            f"direction={self.recovery_view_direction} "
            f"input={id_input}"
        )
        metadata = self._validated_recovery_id_metadata(
            self.recovery_observation_pos,
            self.recovery_observation_kind,
            self.recovery_entity_pos,
            self.recovery_view_direction,
        )
        if metadata is None:
            print("[残局复查] 待提交 ID 的实体元数据已失效，终止本轮恢复。")
            self._abort_recovery_session("待提交 ID 的复查识别元数据失效")
            return False
        try:
            result = self.lib.sokoban_recovery_submit_id(
                self.recovery_session, self.recovery_solver, value
            )
        except Exception as e:
            print(f"[残局复查] 提交复查 ID 失败: {e}")
            self._abort_recovery_session("提交复查 ID 失败")
            return False
        if self._valid_entity_id(value) and not self._record_recovery_id(
            value,
            self.recovery_observation_kind,
            self.recovery_entity_pos,
        ):
            print("[残局复查] 无法把可信 ID 写入 Driver 指定的实体坐标。")
            self._abort_recovery_session("无法按精确坐标记录复查 ID")
            return False
        self.waiting_for_recovery_id_input = False
        return self._handle_recovery_result(result)

    def _simulation_error_can_be_armed(self):
        """Return whether the current UI state can safely edit an error event.

        A path is deliberately paused while the two board clicks are pending.
        This makes an event deterministic without changing the C recovery
        session or allowing a manual move to race the selection.
        """
        if (
            self.waiting_for_recovery_id_input
            or self.waiting_for_id_input
            or self.residual_simulation_mode
            or self.map_creation_mode
            or self.map_creation_choice_mode
            or self.auto_solve_all
            or self.scan_solve_mode
        ):
            return False
        if self.is_playing:
            return bool(self.auto_path) and not self.is_scanning
        if self.awaiting_residual_review:
            # The current route has ended.  A new event is meaningful only
            # after the next observation creates another recovery path.
            return bool(self.recovery_session) and self.recovery_observation_count > 0
        return not self.recovery_session

    def _simulation_error_target_round(self):
        if self.recovery_path_active or self.is_playing:
            return max(0, int(getattr(self, "simulation_error_route_round", 0)))
        if self.awaiting_residual_review and self.recovery_session:
            # The current route has already ended.  This event is armed for
            # the next PATH_READY segment, even if one observation or a later
            # ID submission yields more than one path.
            return max(0, int(getattr(self, "recovery_path_count", 0))) + 1
        if self.recovery_session:
            return max(0, int(getattr(self, "recovery_path_count", 0))) + 1
        return 0

    @staticmethod
    def _simulation_error_id(value):
        if isinstance(value, int) and not isinstance(value, bool) and 0 <= value < 10:
            return value
        return None

    def _simulation_error_event_matches(self, event, source, box_id, scanned_box_id):
        if event.get("status", "pending") != "pending":
            return False
        if int(event.get("round", 0)) != int(getattr(self, "simulation_error_route_round", 0)):
            return False
        push_index = event.get("push_index")
        if push_index is not None and int(push_index) != int(getattr(self, "simulation_error_push_count", 0)) + 1:
            return False
        event_source = tuple(event.get("tracked_source", event.get("source", ())))
        if event_source == tuple(source):
            return True
        event_id = self._simulation_error_id(event.get("box_id"))
        if event_id is None:
            event_id = self._simulation_error_id(event.get("scanned_box_id"))
        return event_id is not None and event_id in {
            self._simulation_error_id(box_id),
            self._simulation_error_id(scanned_box_id),
        }

    def _record_simulation_error_event(self, event, status, reason="", actual_source=None, actual_destination=None):
        event["status"] = status
        if actual_source is not None:
            event["actual_source"] = tuple(actual_source)
        if actual_destination is not None:
            event["actual_destination"] = tuple(actual_destination)
        if reason:
            event["reason"] = reason
        history = getattr(self, "simulation_error_history", None)
        if history is None:
            self.simulation_error_history = []
            history = self.simulation_error_history
        history.append(dict(event))

    def _simulation_error_round_finished(self):
        """Mark queued events that were not observed in the finished path."""
        route_round = int(getattr(self, "simulation_error_route_round", 0))
        for event in getattr(self, "simulation_error_events", ()):
            if event.get("status", "pending") == "pending" and int(event.get("round", 0)) == route_round:
                self._record_simulation_error_event(
                    event,
                    "untriggered",
                    reason="本轮路径结束时未推动绑定箱子",
                )
                source = tuple(event.get("source", ()))
                tracked_source = tuple(event.get("tracked_source", source))
                destination = tuple(event.get("destination", ()))
                for preview_source in {source, tracked_source}:
                    if self.simulated_error_previews.get(preview_source) == destination:
                        self.simulated_error_previews.pop(preview_source, None)
                print(
                    f"[模拟错误] 第 {route_round} 轮事件未触发："
                    f"source={event.get('source')} destination={event.get('destination')}"
                )

    def _finish_pending_simulation_error_events(self, reason):
        """Close events whose route can no longer be produced by this session."""
        for event in getattr(self, "simulation_error_events", ()):
            if event.get("status", "pending") != "pending":
                continue
            self._record_simulation_error_event(event, "untriggered", reason=reason)
            source = tuple(event.get("source", ()))
            tracked_source = tuple(event.get("tracked_source", source))
            destination = tuple(event.get("destination", ()))
            for preview_source in {source, tracked_source}:
                if self.simulated_error_previews.get(preview_source) == destination:
                    self.simulated_error_previews.pop(preview_source, None)
            print(
                "[模拟错误] 会话结束，事件未触发："
                f"round={event.get('round')} source={source} destination={destination}"
            )

    def _begin_simulation_error_route(self, route_round):
        self.simulation_error_route_round = max(0, int(route_round))
        self.simulation_error_push_count = 0
        self.simulation_error_selected_round = self.simulation_error_route_round

    def _note_simulation_box_push(self):
        self.simulation_error_push_count = int(getattr(self, "simulation_error_push_count", 0)) + 1

    def _migrate_pending_simulation_error_events(
        self, source, destination, box_id=None, scanned_box_id=None
    ):
        """Follow a physical box so future no-ID events keep their identity."""
        source = tuple(source)
        destination = tuple(destination)
        current_ids = {
            self._simulation_error_id(box_id),
            self._simulation_error_id(scanned_box_id),
        }
        for event in getattr(self, "simulation_error_events", ()):
            if event.get("status", "pending") != "pending":
                continue
            tracked_source = tuple(
                event.get("tracked_source", event.get("source", ()))
            )
            event_id = self._simulation_error_id(event.get("box_id"))
            if event_id is None:
                event_id = self._simulation_error_id(event.get("scanned_box_id"))
            if tracked_source != source and not (
                event_id is not None and event_id in current_ids
            ):
                continue

            event_destination = tuple(event.get("destination", ()))
            if self.simulated_error_previews.get(tracked_source) == event_destination:
                self.simulated_error_previews.pop(tracked_source, None)
            event["tracked_source"] = destination
            if destination not in self.simulated_error_previews:
                self.simulated_error_previews[destination] = event_destination

    def queue_simulated_error_event(self, round_index, source, destination, push_index=None):
        """Queue a deterministic box offset for a recovery path round.

        This is the script-friendly seam for map1/map2/map3 regression cases;
        GUI selection uses the same method.  A missing source is rejected now,
        while a source that is not pushed in the selected round is recorded as
        ``untriggered`` instead of being applied to another box.
        """
        try:
            round_index = int(round_index)
            source = (int(source[0]), int(source[1]))
            destination = (int(destination[0]), int(destination[1]))
        except (TypeError, ValueError, IndexError):
            return False
        if round_index < 0 or source not in self.boxes:
            return False
        if push_index is not None:
            try:
                push_index = int(push_index)
            except (TypeError, ValueError):
                return False
            if push_index <= 0:
                return False
        if not self._can_place_simulated_error(source, destination):
            return False
        events = getattr(self, "simulation_error_events", None)
        if events is None:
            events = []
            self.simulation_error_events = events
        event = {
            "round": round_index,
            "source": source,
            "tracked_source": source,
            "destination": destination,
            "box_id": self._simulation_error_id(self.box_ids.get(source)),
            "scanned_box_id": self._simulation_error_id(self.scanned_box_ids.get(source)),
            "push_index": push_index,
            "status": "pending",
        }
        events.append(event)
        # Keep the legacy preview visible.  The event list, not this mapping,
        # is authoritative when several rounds target the same box.
        self.simulated_error_previews[source] = destination
        self.simulation_error_selected_round = round_index
        return True

    def _take_simulation_error_for_push(self, source, box_id, scanned_box_id):
        events = getattr(self, "simulation_error_events", ())
        candidate = None
        has_queued_for_box = False
        for event in events:
            if event.get("status", "pending") == "pending":
                event_source = tuple(
                    event.get("tracked_source", event.get("source", ()))
                )
                event_id = self._simulation_error_id(event.get("box_id"))
                if event_id is None:
                    event_id = self._simulation_error_id(event.get("scanned_box_id"))
                current_ids = {
                    self._simulation_error_id(box_id),
                    self._simulation_error_id(scanned_box_id),
                }
                if event_source == tuple(source) or (event_id is not None and event_id in current_ids):
                    has_queued_for_box = True
            if self._simulation_error_event_matches(event, source, box_id, scanned_box_id):
                # Prefer an exact source match over an ID-only match.
                if tuple(
                    event.get("tracked_source", event.get("source", ()))
                ) == tuple(source):
                    candidate = event
                    break
                if candidate is None:
                    candidate = event

        if candidate is not None:
            destination = tuple(candidate.get("destination", ()))
            preview_owner = tuple(
                candidate.get("tracked_source", candidate.get("source", ()))
            )
            if not self._can_place_simulated_error(
                source, destination, preview_owner=preview_owner
            ):
                self._record_simulation_error_event(
                    candidate,
                    "untriggered",
                    reason="错误落点在执行时已被占用",
                    actual_source=source,
                    actual_destination=destination,
                )
                print(
                    f"[模拟错误] 第 {getattr(self, 'simulation_error_route_round', 0)} 轮落点不可用，"
                    f"保持正确推进: {source}->{destination}"
                )
                for preview_source in {tuple(source), preview_owner}:
                    if self.simulated_error_previews.get(preview_source) == destination:
                        self.simulated_error_previews.pop(preview_source, None)
                return None
            self._record_simulation_error_event(
                candidate,
                "triggered",
                actual_source=source,
                actual_destination=destination,
            )
            for preview_source in {tuple(source), preview_owner}:
                if self.simulated_error_previews.get(preview_source) == destination:
                    self.simulated_error_previews.pop(preview_source, None)
            return destination

        # A queued event for this box must not fall through to the legacy
        # wildcard mapping: that would fire a future-round event too early.
        if has_queued_for_box:
            return None

        # Backward-compatible one-shot mappings created by older callers.
        destination = self.simulated_error_previews.get(source)
        if destination is None:
            return None
        if not self._can_place_simulated_error(source, destination):
            self.simulated_error_previews.pop(source, None)
            return None
        self.simulated_error_previews.pop(source, None)
        return tuple(destination)

    def toggle_simulation_error_mode(self):
        if not self._simulation_error_can_be_armed() and not self.simulation_error_mode:
            return False

        self.simulation_error_mode = not self.simulation_error_mode
        self.pending_simulated_error_source = None
        if self.simulation_error_mode:
            self.id_assignment_mode = False
            self.waiting_for_target = False
            self.temp_box_pos = None
            self.set_player_mode = False
            self.simulation_error_selected_round = self._simulation_error_target_round()
            # Choosing an event pauses both the path and an in-flight photo
            # timer; the user explicitly resumes by finishing the two clicks.
            if self.awaiting_residual_review:
                self.residual_capture_started_at = None
            print(
                f"模拟错误：第 {self.simulation_error_selected_round} 轮，"
                "先点击箱子，再点击错误位置（E/ESC）。"
            )
        else:
            print("已退出模拟错误选择。")
        return True

    def _can_place_simulated_error(self, source, destination, preview_owner=None):
        dx, dy = destination
        if not (0 <= dy < len(self.grid) and 0 <= dx < len(self.grid[dy])):
            return False
        if destination == source or destination == self.player:
            return False
        if self.grid[dy][dx] == '#' or destination in self.bombs:
            return False
        if destination in self.boxes:
            return False
        for preview_source, preview_destination in self.simulated_error_previews.items():
            if (
                preview_source not in {source, preview_owner}
                and preview_destination == destination
            ):
                return False
        return True

    def handle_simulated_error_click(self, pos):
        if not self.simulation_error_mode:
            return False

        x, y = pos[0] // BLOCK_SIZE, pos[1] // BLOCK_SIZE
        if not (0 <= y < len(self.grid) and 0 <= x < len(self.grid[y])):
            return True
        selected = (x, y)

        if self.pending_simulated_error_source is None:
            if selected not in self.boxes or selected in self.simulated_error_previews:
                print("模拟错误：请先选择一个当前仍在场、且尚未排队错误的箱子。")
                return True
            self.pending_simulated_error_source = selected
            print(f"模拟错误：已选择箱子 {selected}，现在选择错误位置。")
            return True

        source = self.pending_simulated_error_source
        if not self._can_place_simulated_error(source, selected):
            print("模拟错误：错误位置必须是未占用的非墙、非炸弹格。")
            return True

        if not self.queue_simulated_error_event(
            self._simulation_error_target_round(), source, selected
        ):
            print("模拟错误：事件未加入，箱子或错误位置已不可用。")
            self.pending_simulated_error_source = None
            return True
        self.pending_simulated_error_source = None
        self.simulation_error_mode = False
        if self.awaiting_residual_review:
            self.residual_capture_started_at = None
        print(
            f"模拟错误已设置：第 {self.simulation_error_selected_round} 轮，"
            f"箱子 {source} 将偏移到 {selected}。"
        )
        return True

    def _id_counts_and_surplus(self, box_ids, target_ids):
        common_count = 0
        box_surplus = []
        target_surplus = []
        for id_value in range(10):
            b_count = box_ids.count(id_value)
            t_count = target_ids.count(id_value)
            common_count += min(b_count, t_count)
            if b_count > t_count:
                box_surplus.extend([id_value] * (b_count - t_count))
            elif t_count > b_count:
                target_surplus.extend([id_value] * (t_count - b_count))
        return common_count, box_surplus, target_surplus

    def auto_assign_ids(self):
        """按 C 侧规则补全扫描 ID，只用于 demo 显示和消失判断。"""
        for pos, val in list(self.scanned_box_ids.items()):
            if pos in self.boxes and isinstance(val, int) and 0 <= val < 10:
                self.box_ids[pos] = val

        for pos, val in list(self.scanned_target_ids.items()):
            if pos in self.targets and isinstance(val, int) and 0 <= val < 10:
                self.target_ids[pos] = val

        if len(self.boxes) != len(self.targets):
            return False

        box_positions = sorted(self.boxes, key=lambda p: (p[1], p[0]))
        target_positions = sorted(self.targets, key=lambda p: (p[1], p[0]))
        missing_boxes = []
        missing_targets = []
        known_box_ids = []
        known_target_ids = []

        for pos in box_positions:
            val = self.box_ids.get(pos)
            if isinstance(val, int) and 0 <= val < 10:
                known_box_ids.append(val)
            else:
                missing_boxes.append(pos)

        for pos in target_positions:
            val = self.target_ids.get(pos)
            if isinstance(val, int) and 0 <= val < 10:
                known_target_ids.append(val)
            else:
                missing_targets.append(pos)

        _, box_surplus, target_surplus = self._id_counts_and_surplus(
            known_box_ids, known_target_ids
        )

        while box_surplus and missing_targets:
            id_value = box_surplus.pop(0)
            pos = missing_targets.pop(0)
            self.target_ids[pos] = id_value
            known_target_ids.append(id_value)

        while target_surplus and missing_boxes:
            id_value = target_surplus.pop(0)
            pos = missing_boxes.pop(0)
            self.box_ids[pos] = id_value
            known_box_ids.append(id_value)

        _, box_surplus, target_surplus = self._id_counts_and_surplus(
            known_box_ids, known_target_ids
        )

        if box_surplus or target_surplus:
            # A scanned ID is an observation, not a pairing hint.  Only fill
            # unobserved entities above; never rewrite a real mismatch into a
            # matching pair just to balance the display-side ledger.
            return False

        while missing_boxes and missing_targets:
            used_ids = set(known_box_ids) | set(known_target_ids)
            chosen_id = next((id_value for id_value in range(10) if id_value not in used_ids), None)
            if chosen_id is None:
                return False
            box_pos = missing_boxes.pop(0)
            target_pos = missing_targets.pop(0)
            self.box_ids[box_pos] = chosen_id
            self.target_ids[target_pos] = chosen_id
            known_box_ids.append(chosen_id)
            known_target_ids.append(chosen_id)

        if missing_boxes or missing_targets:
            return False

        for id_value in range(10):
            if known_box_ids.count(id_value) != known_target_ids.count(id_value):
                return False

        self.scanned_box_ids = dict(self.box_ids)
        self.scanned_target_ids = dict(self.target_ids)
        return True

    def _scan_waypoint_label(self, pos, tag=None):
        x, y = pos
        if isinstance(tag, int):
            if SCAN_WAYPOINT_BOX_TAG_BASE - MAX_BOXES < tag <= SCAN_WAYPOINT_BOX_TAG_BASE:
                return f"box#{SCAN_WAYPOINT_BOX_TAG_BASE - tag}@({x},{y}) tag={tag}"
            if SCAN_WAYPOINT_TARGET_TAG_BASE - MAX_TARGETS < tag <= SCAN_WAYPOINT_TARGET_TAG_BASE:
                return f"target#{SCAN_WAYPOINT_TARGET_TAG_BASE - tag}@({x},{y}) tag={tag}"
        if pos in self.boxes:
            return f"box@({x},{y}) tag={tag}"
        if pos in self.targets:
            return f"target@({x},{y}) tag={tag}"
        return f"unknown@({x},{y}) tag={tag}"

    def _scan_known_id_at(self, pos, is_box):
        primary = self.box_ids if is_box else self.target_ids
        scanned = self.scanned_box_ids if is_box else self.scanned_target_ids
        value = primary.get(pos)
        if value is None:
            value = scanned.get(pos)
        return value if isinstance(value, int) and 0 <= value < 10 else None

    def _box_matches_target_for_display(self, box_id, scanned_box_id, target_id, scanned_target_id, box_on_target):
        if not box_on_target:
            return False
        final_box_id = box_id if self._valid_entity_id(box_id) else scanned_box_id
        final_target_id = target_id if self._valid_entity_id(target_id) else scanned_target_id
        if self._is_identified_solution():
            # In scan mode, an unrecognized or mismatched pair is still a box.
            if self._valid_entity_id(final_box_id) and self._valid_entity_id(final_target_id):
                return final_box_id == final_target_id
            return False
        if not self.strict_mode:
            return True
        if self._valid_entity_id(final_box_id) and self._valid_entity_id(final_target_id):
            return final_box_id == final_target_id
        return final_box_id is None and final_target_id is None

    def _queue_box_merge_if_needed(self, pos, start_time, should_disappear):
        if should_disappear:
            self.pending_box_merge = {
                'pos': pos,
                'start_time': start_time,
            }
        else:
            self.pending_box_merge = None

    def _remove_absorbed_box_and_target(self, pos):
        bx, by = pos
        self.boxes.discard(pos)
        self.simulated_error_boxes.discard(pos)
        self.box_ids.pop(pos, None)
        self.scanned_box_ids.pop(pos, None)
        self.targets.discard(pos)
        self.target_ids.pop(pos, None)
        self.scanned_target_ids.pop(pos, None)
        if self.grid[by][bx] in '.abcdefghijklmnopqrstuvwxyz':
            self.grid[by][bx] = ' '
        self.visual_offsets.pop(('box', bx, by), None)
        if self.pending_box_merge and self.pending_box_merge['pos'] == pos:
            self.pending_box_merge = None

    def _resolve_identified_target_boxes(self):
        """Apply delayed absorption after both IDs become available."""
        if not self._is_identified_solution():
            return
        for pos in list(self.boxes & self.targets):
            box_id = self.box_ids.get(pos)
            scanned_box_id = self.scanned_box_ids.get(pos)
            target_id = self.target_ids.get(pos)
            scanned_target_id = self.scanned_target_ids.get(pos)
            if self._box_matches_target_for_display(
                box_id, scanned_box_id, target_id, scanned_target_id, True
            ):
                self._remove_absorbed_box_and_target(pos)

    def _resolve_direct_target_boxes(self):
        """Apply direct-mode absorption to boxes held during the scan prefix."""
        if self._is_identified_solution():
            return
        for pos in list(self.boxes & self.targets):
            self._remove_absorbed_box_and_target(pos)

    def scan_map(self):
        """生成扫描路径访问所有箱子和目标"""
        map_data = self.generate_wire_map()

        self.draw("生成扫描路径...")
        pygame.event.pump()

        solver = self.lib.solver_create()
        if not solver:
            self.draw("错误：创建求解器失败")
            pygame.time.wait(2000)
            return

        map_string = "|".join(map_data).encode('utf-8')
        if not self.lib.solver_load_map_from_string(solver, map_string):
            self.draw("错误：加载地图失败")
            self.lib.solver_destroy(solver)
            pygame.time.wait(2000)
            return

        start = time.time()
        success = self.lib.solver_generate_scan_path(solver)
        elapsed = time.time() - start
        simulated_elapsed, host_freq = get_simulated_time_with_host_freq(elapsed)

        if success:
            length = ctypes.c_uint16()
            path_ptr = self.lib.solver_get_solution(solver, ctypes.byref(length))

            if path_ptr:
                path = [(path_ptr[i].dx, path_ptr[i].dy) for i in range(length.value)]
                pause_count = sum(1 for dx, dy in path if dx == 0 and dy == 0)
                
                # 手动扫描重置计步器
                self.scan_executed_steps = 0
                self.solve_executed_steps = 0
                self.solve_planned_steps = 0
                self.scan_planned_total = len(path) - pause_count
                self.scan_planned_turns = self.count_turns_in_path(path)
                self.solve_planned_turns = 0
                
                print(f"扫描路径生成成功，实际耗时 {elapsed:.2f}秒 (主机CPU: {host_freq:.0f}MHz, 模拟600MHz单片机: {simulated_elapsed:.2f}秒): {len(path)} 步 (包含 {pause_count} 个扫描点)")
                self.auto_path = path
                self.is_playing = True
                self.is_scanning = True
                self._begin_simulation_error_route(0)
                self.show_trail = True
                self.update_path_trail()
            else:
                self.draw(f"扫描失败 (实际 {elapsed:.2f}秒, 模拟 {simulated_elapsed:.2f}秒)")
                pygame.time.wait(2000)
        else:
            self.draw(f"扫描失败 (实际 {elapsed:.2f}秒, 模拟 {simulated_elapsed:.2f}秒)")
            pygame.time.wait(2000)

        self.lib.solver_destroy(solver)

    def start_map_creation_choice(self):
        """显示地图编辑入口选择"""
        self.map_creation_choice_mode = True
        self.creation_choice_buttons = []
        print("请选择：编辑当前地图 或 编辑新地图")

    def cancel_map_creation_choice(self):
        self.map_creation_choice_mode = False
        self.creation_choice_buttons = []

    def _rows_to_creation_grid(self, rows):
        grid = [[' ' for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]
        for y in range(MAP_ROWS):
            source = rows[y] if y < len(rows) else ""
            for x in range(MAP_COLS):
                char = source[x] if x < len(source) else ' '
                if char in "#$.@B*":
                    grid[y][x] = char
                elif char.isdigit():
                    grid[y][x] = '$'
                elif char == '.' or ('a' <= char <= 'z'):
                    grid[y][x] = '.'
                else:
                    grid[y][x] = ' '
        return grid

    def _rows_to_residual_simulation_grid(self, rows):
        grid = self._rows_to_creation_grid(rows)
        for x, y in self.boxes & self.targets:
            if 0 <= x < MAP_COLS and 0 <= y < MAP_ROWS:
                grid[y][x] = RESIDUAL_OVERLAP_BOX
        return grid

    def _residual_snapshot_rows_from_grid(self):
        return ["".join(row) for row in self.residual_simulation_grid]

    @staticmethod
    def _validated_residual_snapshot_rows(rows):
        try:
            materialized = tuple(
                row if isinstance(row, str) else "".join(row) for row in rows
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("残局快照行必须是字符序列") from exc
        player_count = sum(row.count("@") for row in materialized)
        if player_count != 1:
            raise ValueError("残局快照必须包含唯一当前车位 @")
        return materialized

    def save_residual_simulation_snapshot(self, rows=None, label=None):
        """保存一张可按顺序提交的 map1/map2/... 残局快照。

        快照保存的是物理真相（``*`` 同时保留箱子和目的地）；提交给 C
        侧时仍通过 ``generate_residual_map`` 输出无 ID 相机视图。该列表
        不调用 ``load_map``，因此不会重置正在运行的恢复会话。
        """
        if self.awaiting_residual_review:
            self.residual_capture_started_at = None
        if rows is None:
            if self.residual_simulation_mode and self.residual_simulation_grid:
                rows = self._residual_snapshot_rows_from_grid()
            else:
                rows = self.generate_recovery_bootstrap_map()
        try:
            rows = self._validated_residual_snapshot_rows(rows)
            normalized = normalize_external_map_rows(
                rows, "残局快照", preserve_residual_overlap=True
            )
        except ValueError as exc:
            print(f"[残局快照] 保存失败: {exc}")
            return None
        snapshots = getattr(self, "residual_simulation_snapshots", None)
        if snapshots is None:
            snapshots = []
            self.residual_simulation_snapshots = snapshots
        snapshot = {
            "label": str(label or f"map{len(snapshots) + 1}"),
            "rows": tuple(normalized),
        }
        snapshots.append(snapshot)
        if not hasattr(self, "residual_simulation_snapshot_cursor"):
            self.residual_simulation_snapshot_cursor = 0
        print(
            f"[残局快照] 已保存 {snapshot['label']}，"
            f"队列共 {len(snapshots)} 张。"
        )
        return len(snapshots) - 1

    def load_next_residual_simulation_snapshot(self):
        """在独立编辑器中载入队列中的下一张快照。"""
        if not self.residual_simulation_mode:
            return False
        snapshots = getattr(self, "residual_simulation_snapshots", ())
        if not snapshots:
            print("[残局快照] 队列为空，请先按 F5 保存快照。")
            return False
        cursor = int(getattr(self, "residual_simulation_snapshot_cursor", 0))
        index = cursor % len(snapshots)
        rows = snapshots[index]["rows"]
        self.residual_simulation_grid = [list(row) for row in rows]
        self.residual_simulation_snapshot_cursor = (index + 1) % len(snapshots)
        print(f"[残局快照] 编辑器载入 {snapshots[index]['label']}。")
        return True

    def _apply_residual_snapshot_rows(self, rows):
        """Apply a snapshot to the live board without destroying recovery state."""
        rows = self._validated_residual_snapshot_rows(rows)
        normalized = normalize_external_map_rows(
            rows, "残局快照", preserve_residual_overlap=True
        )

        grid = []
        boxes = set()
        targets = set()
        bombs = set()
        box_ids = {}
        target_ids = {}
        player = None
        for y, row in enumerate(normalized):
            row_data = []
            for x, char in enumerate(row):
                if char == "#":
                    row_data.append("#")
                elif char in "$0123456789":
                    boxes.add((x, y))
                    if char.isdigit():
                        box_ids[(x, y)] = int(char)
                    row_data.append(" ")
                elif char == RESIDUAL_OVERLAP_BOX:
                    boxes.add((x, y))
                    targets.add((x, y))
                    row_data.append(".")
                elif char in ".abcdefghijklmnopqrstuvwxyz":
                    targets.add((x, y))
                    if char.isalpha() and char != ".":
                        target_ids[(x, y)] = ord(char) - ord("a")
                    row_data.append(char)
                elif char == "@":
                    player = (x, y)
                    row_data.append(" ")
                elif char == "B":
                    bombs.add((x, y))
                    row_data.append(" ")
                else:
                    row_data.append(" ")
            grid.append(row_data)

        if player is None:
            raise ValueError("残局快照缺少当前车位 @")
        self.grid = grid
        self.boxes = boxes
        self.targets = targets
        self.bombs = bombs
        self.player = player
        self.box_ids = box_ids
        self.target_ids = target_ids
        self.scanned_box_ids = {}
        self.scanned_target_ids = {}
        self.simulated_error_previews = {}
        self.simulated_error_boxes = set()
        self.simulation_error_mode = False
        self.pending_simulated_error_source = None
        self.auto_path = []
        self.is_playing = False
        self.is_scanning = False
        self.recovery_path_active = False
        self.recovery_next_status = None
        self.recovery_observation_pos = None
        self.recovery_observation_kind = SOKOBAN_RECOVERY_ENTITY_NONE
        self.recovery_entity_pos = None
        self.recovery_view_direction = None
        self.visual_offsets.clear()
        self.pending_box_merge = None
        self.last_move_time = time.time()
        return normalized

    def submit_next_residual_snapshot(self):
        """Submit the next saved snapshot to the existing recovery session.

        This is intentionally separate from ``start_residual_simulation_solve``:
        the latter is a new session entry point and resets the board, while this
        method keeps the C solver/recovery handles alive for map2/map3 replay.
        """
        if (
            not self._residual_snapshot_capture_available()
            or self.recovery_observation_count <= 0
        ):
            return False
        snapshots = getattr(self, "residual_simulation_snapshots", ())
        cursor = int(getattr(self, "residual_simulation_snapshot_cursor", 0))
        if cursor >= len(snapshots):
            print("[残局快照] 没有下一张可提交的快照，继续使用当前地图自动拍摄。")
            return False
        rows = list(snapshots[cursor]["rows"])
        try:
            self._apply_residual_snapshot_rows(rows)
        except ValueError as exc:
            print(f"[残局快照] 提交失败: {exc}")
            return False
        self.awaiting_residual_review = False
        self.residual_capture_started_at = None
        self.residual_capture_kind = None
        print(
            f"[残局快照] 提交 {snapshots[cursor]['label']} 作为第 "
            f"{self.recovery_observation_count + 1} 张残局观测。"
        )
        observation_count_before = self.recovery_observation_count
        submitted = bool(self._submit_residual_map_for_review())
        # RETRY_OBSERVATION asks for a new photo, so a snapshot that reached C
        # is consumed even when it was rejected.  Pre-call/API failures keep
        # the same queue entry available.
        if self.recovery_observation_count > observation_count_before:
            self.residual_simulation_snapshot_cursor = cursor + 1
        elif not submitted:
            self.awaiting_residual_review = True
        return submitted

    def _residual_snapshot_capture_available(self):
        return bool(
            self.awaiting_residual_review
            and not self.is_playing
            and not getattr(self, "auto_path", None)
            and not self.pending_box_merge
            and not self.waiting_for_recovery_id_input
            and not self.simulation_error_mode
            and self.recovery_session
        )

    def _residual_snapshot_submission_available(self):
        return bool(
            self._residual_snapshot_capture_available()
            and self.recovery_observation_count > 0
            and int(getattr(self, "residual_simulation_snapshot_cursor", 0))
            < len(getattr(self, "residual_simulation_snapshots", ()))
        )

    def _new_creation_grid(self):
        grid = [[' ' for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                if r == 0 or r == MAP_ROWS - 1 or c == 0 or c == MAP_COLS - 1:
                    grid[r][c] = '#'
        return grid

    def start_map_creation(self, edit_current=False):
        """启动地图创建模式"""
        self.cancel_map_creation_choice()
        self.map_creation_mode = True
        self.creation_save_mode = "current" if edit_current else "new"
        self.creation_source_map = self.current_map if edit_current else None
        self.current_brush = '#'
        self.dragging = False
        self.selected_tile_coord = None
        self.selected_tile_coord_copied_until = 0.0
        self.last_tile_click_coord = None
        self.last_tile_click_time = 0.0

        if edit_current:
            self.creation_grid = self._rows_to_creation_grid(self.generate_current_map())
            print(f"进入地图编辑模式：编辑当前地图 {self.current_map}")
        else:
            self.creation_grid = self._new_creation_grid()
            print("进入地图编辑模式：创建新地图")

    def _current_map_filepath(self):
        if self.creation_source_map and self.creation_source_map in self.custom_maps:
            return os.path.join(
                project_map_directory(), f"{self.creation_source_map}.txt"
            )
        return None

    def _next_map_filepath(self):
        map_folder = project_map_directory()
        if not os.path.exists(map_folder):
            os.makedirs(map_folder)

        map_num = 1
        while os.path.exists(os.path.join(map_folder, f"map{map_num}.txt")):
            map_num += 1
        return os.path.join(map_folder, f"map{map_num}.txt"), f"map{map_num}"

    def exit_map_creation(self):
        """退出地图创建模式"""
        self.map_creation_mode = False
        self.creation_grid = []
        self.creation_save_mode = "new"
        self.creation_source_map = None
        print("退出地图创建模式")

    def export_creation_map(self, save_as_new=False):
        """导出创建的地图"""
        # 生成地图数据
        output = [f'    "{"".join(row)}"' for row in self.creation_grid]

        filepath = self._current_map_filepath()
        saved_map_name = self.creation_source_map
        if save_as_new or filepath is None:
            filepath, saved_map_name = self._next_map_filepath()

        try:
            with open(filepath, "w", encoding='utf-8') as f:
                f.write("[\n" + ",\n".join(output) + "\n]")
            print(f"\n✅ 地图已保存至 {filepath}")
            self.load_custom_maps()
            if saved_map_name:
                self.current_map = saved_map_name
                self.load_map(self.current_map)
                self.creation_source_map = saved_map_name
                self.creation_save_mode = "current"
        except Exception as e:
            print(f"保存失败: {e}")

    def clear_creation_map(self):
        """清空创建的地图（保留外围墙）"""
        self.creation_grid = self._new_creation_grid()
        print("地图已清空")

    def export_maps_to_othermap(self):
        """将当前箱子数目的所有地图导出到 othermap 目录下，恢复为原始的 12x16 格式"""
        if self.selected_box_count is None:
            return
        
        box_count = self.selected_box_count
        catalog = self._get_map_catalog()
        maps_to_export = catalog.get(box_count, [])
        
        if not maps_to_export:
            print(f"没有 {box_count} 箱的地图可导出。")
            return
            
        base_export_dir = os.path.join(ROOT_DIR, "othermap", f"{box_count}箱")
        os.makedirs(base_export_dir, exist_ok=True)
        
        export_count = 0
        for entry in maps_to_export:
            map_data = entry["map_data"]
            # 顺时针旋转 90 度以恢复 12x16
            rotated = ["".join(row) for row in zip(*map_data[::-1])]
            # 替换空格为 '-'，'B' 为 '*'
            formatted = [line.replace(' ', '-').replace('B', '*') for line in rotated]
            
            map_key = str(entry["key"])
            filename = f"{map_key}.txt" if not map_key.lower().endswith('.txt') else map_key
            filepath = os.path.join(base_export_dir, filename)
            
            try:
                with open(filepath, 'w', encoding='utf-8') as f:
                    for line in formatted:
                        f.write(line + '\n')
                export_count += 1
            except Exception as e:
                print(f"导出 {filename} 失败: {e}")
                
        print(f"\n✅ 成功导出 {export_count} 张 {box_count} 箱地图到 {base_export_dir}")

    def start_residual_simulation(self):
        """Open a disposable editor whose map is submitted straight to recovery."""
        if (
            self.is_playing
            or self.scan_solve_mode
            or self.auto_solve_all
            or self.awaiting_residual_review
            or self.recovery_session
            or self.map_creation_mode
            or self.map_creation_choice_mode
        ):
            return False

        self.id_assignment_mode = False
        self.waiting_for_target = False
        self.temp_box_pos = None
        self.set_player_mode = False
        self.simulation_error_mode = False
        self.pending_simulated_error_source = None
        self.residual_simulation_mode = True
        self.residual_simulation_snapshots = []
        self.residual_simulation_snapshot_cursor = 0
        self.residual_simulation_grid = self._rows_to_residual_simulation_grid(
            self.generate_wire_map()
        )
        self.residual_simulation_brush = '#'
        self.residual_simulation_buttons = []
        self.dragging = False
        self.selected_tile_coord = None
        self.selected_tile_coord_copied_until = 0.0
        self.last_tile_click_coord = None
        self.last_tile_click_time = 0.0
        print(
            "进入残局模拟模式：F5 按 map1/map2/... 保存；"
            "D/S 从队首开始，恢复等待时按 F6 提交下一张。"
        )
        return True

    def exit_residual_simulation(self):
        self.residual_simulation_mode = False
        self.residual_simulation_grid = []
        self.residual_simulation_brush = '#'
        self.residual_simulation_buttons = []
        self.dragging = False
        print("退出残局模拟模式")

    def clear_residual_simulation(self):
        self.residual_simulation_grid = self._new_creation_grid()
        print("模拟残局已清空")

    def handle_residual_simulation_key(self, key):
        if not self.residual_simulation_mode:
            return False

        brush_keys = {
            pygame.K_1: '#',
            pygame.K_2: '$',
            pygame.K_3: '.',
            pygame.K_4: '@',
            pygame.K_5: 'B',
            pygame.K_6: RESIDUAL_OVERLAP_BOX,
            pygame.K_0: ' ',
        }
        if key in brush_keys:
            self.residual_simulation_brush = brush_keys[key]
            return True
        if key == pygame.K_c:
            self.clear_residual_simulation()
            return True
        if key == pygame.K_F5:
            return self.save_residual_simulation_snapshot() is not None
        if key == pygame.K_F6:
            return self.load_next_residual_simulation_snapshot()
        if key == pygame.K_d:
            return self.start_residual_simulation_solve("direct")
        if key == pygame.K_s:
            return self.start_residual_simulation_solve("identified")
        if key == pygame.K_ESCAPE:
            self.exit_residual_simulation()
            return True
        return False

    def handle_residual_simulation_click(self, pos):
        if not self.residual_simulation_mode:
            return False

        col = pos[0] // BLOCK_SIZE
        row = pos[1] // BLOCK_SIZE
        if not (0 <= col < MAP_COLS and 0 <= row < MAP_ROWS):
            return False

        if self.residual_simulation_brush == '@':
            for r in range(MAP_ROWS):
                for c in range(MAP_COLS):
                    if self.residual_simulation_grid[r][c] == '@':
                        self.residual_simulation_grid[r][c] = ' '

        self.residual_simulation_grid[row][col] = self.residual_simulation_brush
        return True

    def _load_runtime_map_rows(self, rows):
        """Load an in-memory editor map through the normal map parser/reset path."""
        normalized = normalize_external_map_rows(
            rows, "模拟残局", preserve_residual_overlap=True
        )
        self.residual_simulation_runtime_rows = normalized
        self.current_map = RESIDUAL_SIMULATION_MAP_KEY
        self.load_map(self.current_map)
        return True

    def start_residual_simulation_solve(self, origin_mode):
        """Submit the edited map as the first observation of an existing recovery flow."""
        if origin_mode not in ("direct", "identified"):
            return False
        if not self.residual_simulation_mode or not self.residual_simulation_grid:
            return False

        rows = ["".join(row) for row in self.residual_simulation_grid]
        try:
            rows = self._validated_residual_snapshot_rows(rows)
            normalized = normalize_external_map_rows(
                rows, "模拟残局", preserve_residual_overlap=True
            )
            snapshots = getattr(self, "residual_simulation_snapshots", None)
            if snapshots is None:
                snapshots = []
                self.residual_simulation_snapshots = snapshots
            current_rows = tuple(normalized)
            if not snapshots:
                snapshots.append({"label": "map1", "rows": current_rows})
            elif all(
                tuple(entry.get("rows", ())) != current_rows for entry in snapshots
            ):
                # Once map1 has been saved, the queue order is authoritative.
                # An unsaved current edit is the next map, never a replacement
                # that silently reverses map1/map2.
                snapshots.append(
                    {"label": f"map{len(snapshots) + 1}", "rows": current_rows}
                )
            first_rows = tuple(snapshots[0]["rows"])
            self._validated_residual_snapshot_rows(first_rows)
            self.residual_simulation_snapshot_cursor = 1
            self._load_runtime_map_rows(first_rows)
        except ValueError as exc:
            print(f"[残局模拟] 地图无效: {exc}")
            return False

        self.residual_simulation_mode = False
        self.residual_simulation_grid = []
        self.residual_simulation_buttons = []
        self.auto_path = []
        self.is_playing = False
        self.is_scanning = False
        self.show_trail = False
        self.path_trail = []
        self.pending_box_merge = None
        self.visual_offsets.clear()

        if origin_mode == "direct":
            self.solution_origin_mode = origin_mode
            self._resolve_direct_target_boxes()
        bootstrap_map = self.generate_recovery_bootstrap_map()
        self._prepare_residual_review(origin_mode, bootstrap_map)
        mode_label = "直接残局求解" if origin_mode == "direct" else "扫描残局求解"
        print(f"[残局模拟] {mode_label}：直接提交第 1 张残局地图。")
        return self._submit_residual_map_for_review()

    def handle_residual_simulation_panel_click(self, pos):
        for action, button_rect in self.residual_simulation_buttons:
            if not button_rect.collidepoint(pos):
                continue
            if action == "direct":
                return self.start_residual_simulation_solve("direct")
            if action == "identified":
                return self.start_residual_simulation_solve("identified")
            if action == "save_snapshot":
                return self.save_residual_simulation_snapshot() is not None
            if action == "load_snapshot":
                return self.load_next_residual_simulation_snapshot()
            if action == "clear":
                self.clear_residual_simulation()
                return True
            if action == "exit":
                self.exit_residual_simulation()
                return True
        return False

    def handle_creation_click(self, pos):
        """处理地图创建模式下的点击"""
        x, y = pos
        col = x // BLOCK_SIZE
        row = y // BLOCK_SIZE

        if 0 <= row < MAP_ROWS and 0 <= col < MAP_COLS:
            # 如果放置玩家，先清除地图上原有的玩家
            if self.current_brush == '@':
                for r in range(MAP_ROWS):
                    for c in range(MAP_COLS):
                        if self.creation_grid[r][c] == '@':
                            self.creation_grid[r][c] = ' '

            self.creation_grid[row][col] = self.current_brush
            return True
        return False

    def update_selected_tile_coord(self, pos):
        col = pos[0] // BLOCK_SIZE
        row = pos[1] // BLOCK_SIZE
        if 0 <= col < MAP_COLS and 0 <= row < MAP_ROWS:
            self.selected_tile_coord = (col, row)
            return True
        return False

    def handle_tile_click(self, pos, force_copy=False):
        col = pos[0] // BLOCK_SIZE
        row = pos[1] // BLOCK_SIZE
        if not (0 <= col < MAP_COLS and 0 <= row < MAP_ROWS):
            return False

        self.selected_tile_coord = (col, row)

        now = time.time()
        current_coord = (col, row)
        is_double_click = force_copy or (
            self.last_tile_click_coord == current_coord and (now - self.last_tile_click_time) <= 0.5
        )
        if is_double_click:
            coord_text = f"({col},{row})"
            if copy_text_to_clipboard(coord_text):
                self.selected_tile_coord_copied_until = now + 1.2
            self.last_tile_click_coord = None
            self.last_tile_click_time = 0.0
        else:
            self.last_tile_click_coord = current_coord
            self.last_tile_click_time = now

        return True

    def start_set_player_mode(self):
        if self.auto_solve_all:
            self.cancel_auto_solve_all("已取消")
        if self.is_playing or self.scan_solve_mode or self.map_creation_mode:
            return
        self.id_assignment_mode = False
        self.waiting_for_target = False
        self.temp_box_pos = None
        self.set_player_mode = not self.set_player_mode
        print("设置玩家模式：点击地图空地放置玩家" if self.set_player_mode else "退出设置玩家模式")

    def set_player_at_click(self, pos):
        col = pos[0] // BLOCK_SIZE
        row = pos[1] // BLOCK_SIZE
        if not (0 <= col < MAP_COLS and 0 <= row < MAP_ROWS):
            return False
        if self.grid[row][col] == '#' or (col, row) in self.boxes or (col, row) in self.bombs:
            print(f"不能把玩家放在 ({col},{row})：该位置被墙、箱子或炸弹占用")
            return False

        self.player = (col, row)
        self.auto_path = []
        self.is_playing = False
        self.is_scanning = False
        self.show_trail = False
        self.path_trail = []
        self.visual_offsets.clear()
        self.star_particles = []
        self.last_move_time = 0
        self.next_manual_move_time = 0
        self.manual_active_key = None
        self.last_solve_time = None
        self.last_solve_simulated_time = None
        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.solve_planned_steps = 0
        self.set_player_mode = False
        print(f"玩家位置已设置为 ({col},{row})")
        return True

    def _get_hover_tile_coord(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        if 0 <= mouse_x < MAP_COLS * BLOCK_SIZE and 0 <= mouse_y < MAP_ROWS * BLOCK_SIZE:
            return (mouse_x // BLOCK_SIZE, mouse_y // BLOCK_SIZE)
        return None

    def _draw_hover_badge(self, text, x, y, base_color=(40, 120, 165)):
        text_surface = self.small_font.render(text, True, (245, 250, 255))
        badge_w = text_surface.get_width() + 16
        badge_h = text_surface.get_height() + 8
        rect = pygame.Rect(int(x), int(y), badge_w, badge_h)
        self.draw_glass_rect(self.screen, rect, base_color=base_color, alpha=150, border_radius=10, blur_scale=0.08)
        pygame.draw.rect(self.screen, (215, 245, 255), rect, 1, border_radius=10)
        text_rect = text_surface.get_rect(center=rect.center)
        self.screen.blit(text_surface, text_rect)

    def draw_hover_guides(self):
        hover_coord = self._get_hover_tile_coord()
        if hover_coord is None:
            return None

        col, row = hover_coord
        map_w = MAP_COLS * BLOCK_SIZE
        map_h = MAP_ROWS * BLOCK_SIZE
        cell_x = col * BLOCK_SIZE
        cell_y = row * BLOCK_SIZE
        center_x = cell_x + BLOCK_SIZE // 2
        center_y = cell_y + BLOCK_SIZE // 2

        overlay = pygame.Surface((map_w, map_h), pygame.SRCALPHA)
        pygame.draw.rect(overlay, (90, 180, 255, 22), (0, cell_y, map_w, BLOCK_SIZE))
        pygame.draw.rect(overlay, (90, 180, 255, 22), (cell_x, 0, BLOCK_SIZE, map_h))
        pygame.draw.line(overlay, (130, 215, 255, 175), (center_x, 0), (center_x, map_h - 1), 2)
        pygame.draw.line(overlay, (130, 215, 255, 175), (0, center_y), (map_w - 1, center_y), 2)
        pygame.draw.rect(
            overlay,
            (160, 230, 255, 65),
            (cell_x + 1, cell_y + 1, BLOCK_SIZE - 2, BLOCK_SIZE - 2),
            border_radius=8
        )
        pygame.draw.rect(
            overlay,
            (235, 250, 255, 235),
            (cell_x + 2, cell_y + 2, BLOCK_SIZE - 4, BLOCK_SIZE - 4),
            2,
            border_radius=8
        )
        pygame.draw.circle(overlay, (255, 255, 255, 220), (center_x, center_y), 4)
        self.screen.blit(overlay, (0, 0))

        col_label = f"列 {col}"
        row_label = f"行 {row}"

        col_surface = self.small_font.render(col_label, True, (245, 250, 255))
        col_x = max(6, min(map_w - col_surface.get_width() - 22, center_x - (col_surface.get_width() + 16) // 2))
        self._draw_hover_badge(col_label, col_x, 6, base_color=(35, 120, 165))

        row_surface = self.small_font.render(row_label, True, (245, 250, 255))
        row_y = max(6, min(map_h - row_surface.get_height() - 14, center_y - (row_surface.get_height() + 8) // 2))
        self._draw_hover_badge(row_label, map_w - row_surface.get_width() - 22, row_y, base_color=(30, 105, 145))

        return hover_coord

    def draw_selected_tile_coord(self):
        display_coord = self._get_hover_tile_coord() or self.selected_tile_coord
        if display_coord is None:
            return

        col, row = display_coord
        copied = time.time() < self.selected_tile_coord_copied_until and display_coord == self.selected_tile_coord
        label = f"坐标: ({col},{row})"
        if copied:
            label += " 已复制"

        coord_text = self.font.render(label, True, (255, 245, 180))
        badge_rect = pygame.Rect(10, 10, coord_text.get_width() + 20, 28)
        self.draw_glass_rect(self.screen, badge_rect, base_color=(20, 30, 40), alpha=165, border_radius=10, blur_scale=0.08)
        self.screen.blit(coord_text, (18, 16))

    def solve(self):
        """切换到独立进程运行 C 求解器。"""
        if self.residual_simulation_mode:
            return self.start_residual_simulation_solve("direct")

        pygame.event.clear()

        if self.is_playing or self.scan_solve_mode or self.pending_start_solve or self.set_player_mode or self.awaiting_residual_review or self.recovery_session:
            return
        self._restore_manual_playback_speed()

        has_manual_ids = len(self.box_ids) > 0 and len(self.target_ids) > 0
        map_data = self.generate_wire_map()
        scope = map_scope(map_data)
        if not scope["eligible"]:
            reason = scope["reason"]
            print(
                f"当前地图不纳入自动/性能求解范围（最多 {ACTIVE_MAX_BOXES} 箱、"
                f"{ACTIVE_MAX_BOMBS} 个炸弹）: {reason}"
            )
            if self.auto_solve_all:
                self._skip_auto_solve_current(reason, status="skipped_out_of_scope")
            return
        use_strict_mode = has_manual_ids
        self._prepare_residual_review("direct", map_data)

        print("\n" + "=" * 50)
        print(f"开始常规求解 (严格模式: {use_strict_mode}) - 地图快照：")
        for line in map_data:
            print(line)
        print("=" * 50 + "\n")

        self.draw(f"启动求解进程 (严格模式: {use_strict_mode})...")
        pygame.event.pump()

        main_exe = os.path.join(os.path.dirname(__file__), "main.exe")
        if not os.path.exists(main_exe):
            print("错误：找不到main.exe！无法启动进程模式。请编译生成 main.exe。")
            if self.auto_solve_all:
                self._skip_auto_solve_current("找不到 main.exe", status="error")
            return

        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.solve_planned_steps = 0
        self.scan_pause_start = 0
        self.waiting_for_id_input = False
        self.current_target_position = None
        self.pending_start_solve = False
        self.scan_auto_default_ids = False
        self.scan_solve_compute_time = 0.0
        self.scan_solve_compute_started_at = None

        try:
            self._reset_protocol_phase_mirror()
            self.main_process = subprocess.Popen(
                [main_exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, encoding=C_SUBPROCESS_ENCODING,
                errors="replace", bufsize=1,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)
            )

            def read_output():
                for line in self.main_process.stdout:
                    self.main_output_queue.put(line.strip())

            threading.Thread(target=read_output, daemon=True).start()

            self.send_to_main(f"LOAD_MAP:{('|'.join(map_data))}")
            time.sleep(0.1)
            self.process_main_responses()
            if has_manual_ids:
                for command in self._manual_id_commands():
                    self.send_to_main(command)
            self.send_to_main("START_SOLVE")

            self.scan_solve_mode = True
            print("求解进程已启动")
        except Exception as e:
            print(f"启动进程失败: {e}")
            if self.main_process:
                self.main_process.kill()
                self.main_process = None
            if self.auto_solve_all:
                self._skip_auto_solve_current(f"启动进程失败: {e}", status="error")

    def build_auto_solve_queue(self):
        builtin_items = [(key, f"demo{key}") for key in sorted(MAPS.keys(), key=lambda x: int(x))]
        custom_items = [
            (name, f"{name}.txt")
            for name in sorted(self.custom_maps.keys(), key=natural_map_sort_key)
        ]
        queue = []
        exclusions = []
        for map_key, display_name in builtin_items + custom_items:
            map_data = self.custom_maps.get(map_key) or MAPS.get(map_key)
            scope = map_scope(map_data)
            if scope["eligible"]:
                queue.append((map_key, display_name))
                continue
            exclusions.append({
                "map_key": str(map_key),
                "display_name": display_name,
                "box_count": scope["box_count"],
                "bomb_count": scope["bomb_count"],
                "reason": scope["reason"],
            })
        self.auto_solve_scope_exclusions = exclusions
        return queue

    def build_auto_scan_solve_queue(self):
        queue = []
        for map_key, display_name in self.build_auto_solve_queue():
            for start_pos in ((6, 1), (6, 14)):
                queue.append({
                    "map_key": map_key,
                    "display_name": f"{display_name} @({start_pos[0]},{start_pos[1]})",
                    "start_pos": start_pos,
                })
        return queue

    def _log_auto_solve_scope_exclusions(self):
        exclusions = getattr(self, "auto_solve_scope_exclusions", [])
        if not exclusions:
            return
        print(
            "[一键任务] 已按当前规则跳过 "
            f"{len(exclusions)} 张地图（最多 {ACTIVE_MAX_BOXES} 箱、"
            f"{ACTIVE_MAX_BOMBS} 个炸弹）:"
        )
        for item in exclusions:
            print(f"  - {item['display_name']}: {item['reason']}")

    def _parse_auto_solve_queue_item(self, item):
        if isinstance(item, dict):
            return item.get("map_key"), item.get("display_name", str(item.get("map_key"))), item.get("start_pos")
        map_key, display_name = item
        return map_key, display_name, None

    def _set_player_for_auto_start(self, start_pos):
        if not start_pos:
            return True
        col, row = start_pos
        if not (0 <= col < MAP_COLS and 0 <= row < MAP_ROWS):
            print(f"自动起点 ({col},{row}) 超出地图范围")
            return False
        if self.grid[row][col] == '#' or (col, row) in self.boxes or (col, row) in self.bombs or (col, row) in self.targets:
            print(f"自动起点 ({col},{row}) 不是空地")
            return False

        self.player = (col, row)
        self.auto_path = []
        self.is_playing = False
        self.is_scanning = False
        self.show_trail = False
        self.path_trail = []
        self.visual_offsets.clear()
        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.solve_planned_steps = 0
        return True

    def _record_auto_solve_case(self, status, reason="", path="", elapsed=None, simulated_elapsed=None):
        if not self.auto_solve_all or self.auto_solve_index >= len(self.auto_solve_queue):
            return
        if self.auto_solve_current_recorded:
            return

        map_key, display_name, start_pos = self._parse_auto_solve_queue_item(self.auto_solve_queue[self.auto_solve_index])
        scan_path_len = self.scan_executed_steps if self.auto_solve_scan_mode else 0
        case_wall_time = None
        if self.auto_solve_current_started_at is not None:
            case_wall_time = time.perf_counter() - self.auto_solve_current_started_at

        item = {
            "index": self.auto_solve_index + 1,
            "map_key": str(map_key),
            "display_name": display_name,
            "start_pos": list(start_pos) if start_pos else None,
            "mode": "scan_solve" if self.auto_solve_scan_mode else "solve",
            "status": status,
            "reason": reason,
            "started_at": self.auto_solve_current_started_text,
            "solve_time_sec": round(float(elapsed), 6) if elapsed is not None else None,
            "simulated_600mhz_time_sec": round(float(simulated_elapsed), 6) if simulated_elapsed is not None else None,
            "case_wall_time_sec": round(case_wall_time, 6) if case_wall_time is not None else None,
            "scan_path_len": scan_path_len,
            "path_len": len(path) if path else 0,
            "total_path_len": scan_path_len + (len(path) if path else 0),
            "path": path,
        }
        self.auto_solve_results.append(item)
        self.auto_solve_current_recorded = True

    def start_auto_solve_all_maps(self):
        if self.is_playing or self.scan_solve_mode or self.id_assignment_mode or self.map_creation_mode or self.set_player_mode:
            return

        self.load_custom_maps()
        self.auto_solve_queue = self.build_auto_solve_queue()
        self._log_auto_solve_scope_exclusions()
        if not self.auto_solve_queue:
            print("一键求解队列为空")
            return

        self.auto_solve_all = True
        self.auto_solve_index = 0
        self.auto_solve_results = []
        self.auto_solve_started_at = time.perf_counter()
        self.auto_solve_started_text = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.auto_solve_current_started_at = None
        self.auto_solve_current_started_text = ""
        self.auto_solve_current_recorded = False
        self.auto_solve_scan_mode = False
        self.scan_auto_default_ids = False
        self.auto_solve_base_delay = getattr(self, "move_delay", 0.1)
        self.auto_solve_base_move_duration = self.move_duration
        self._start_auto_solve_current()

    def start_auto_scan_solve_all_maps(self):
        if self.is_playing or self.scan_solve_mode or self.id_assignment_mode or self.map_creation_mode or self.set_player_mode:
            return

        self.load_custom_maps()
        self.auto_solve_queue = self.build_auto_scan_solve_queue()
        self._log_auto_solve_scope_exclusions()
        if not self.auto_solve_queue:
            print("一键扫描求解队列为空")
            return

        self.auto_solve_all = True
        self.auto_solve_scan_mode = True
        self.auto_solve_index = 0
        self.auto_solve_results = []
        self.auto_solve_started_at = time.perf_counter()
        self.auto_solve_started_text = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.auto_solve_current_started_at = None
        self.auto_solve_current_started_text = ""
        self.auto_solve_current_recorded = False
        self.auto_solve_base_delay = getattr(self, "move_delay", 0.1)
        self.auto_solve_base_move_duration = self.move_duration
        self._start_auto_solve_current()

    def _start_auto_solve_current(self):
        if not self.auto_solve_all:
            return

        if self.auto_solve_index >= len(self.auto_solve_queue):
            self.cancel_auto_solve_all("全部完成", restore_current_map=False)
            return

        map_key, display_name, start_pos = self._parse_auto_solve_queue_item(self.auto_solve_queue[self.auto_solve_index])
        mode_label = "一键扫描求解" if self.auto_solve_scan_mode else "一键求解"
        print(f"[{mode_label}] {self.auto_solve_index + 1}/{len(self.auto_solve_queue)} 开始: {display_name}")
        self.auto_solve_current_started_at = time.perf_counter()
        self.auto_solve_current_started_text = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.auto_solve_current_recorded = False
        self.current_map = map_key
        self.load_map(map_key)
        if not self._set_player_for_auto_start(start_pos):
            self._skip_auto_solve_current("自动起点不可用", status="skipped")
            return
        self.move_delay = self.auto_solve_base_delay / self.auto_solve_speed_multiplier
        self.move_duration = self.auto_solve_base_move_duration / self.auto_solve_speed_multiplier
        if self.auto_solve_scan_mode:
            self.start_scan_solve(auto_default_ids=True)
        else:
            self.solve()

    def _advance_auto_solve_after_playback(self):
        if not self.auto_solve_all:
            return False

        self.auto_solve_index += 1
        self._start_auto_solve_current()
        return True

    def _skip_auto_solve_current(self, reason, status="skipped", elapsed=None, simulated_elapsed=None):
        if not self.auto_solve_all:
            return

        map_key, display_name, start_pos = self._parse_auto_solve_queue_item(self.auto_solve_queue[self.auto_solve_index])
        mode_label = "一键扫描求解" if self.auto_solve_scan_mode else "一键求解"
        print(f"[{mode_label}] 跳过 {display_name}: {reason}")
        self._record_auto_solve_case(status, reason=reason, elapsed=elapsed, simulated_elapsed=simulated_elapsed)
        if self.main_process:
            self.stop_scan_solve()
        self.auto_solve_index += 1
        self._start_auto_solve_current()

    def cancel_auto_solve_all(self, reason="已取消", restore_current_map=True):
        if not self.auto_solve_all:
            return False

        if self.auto_solve_index < len(self.auto_solve_queue) and not self.auto_solve_current_recorded:
            self._record_auto_solve_case("cancelled", reason=reason)
        mode_label = "一键扫描求解" if self.auto_solve_scan_mode else "一键求解"
        self.auto_solve_all = False
        self.auto_solve_scan_mode = False
        self.scan_auto_default_ids = False
        self.auto_solve_queue = []
        self.auto_solve_index = 0
        self.move_delay = self.auto_solve_base_delay
        self.move_duration = self.auto_solve_base_move_duration
        self.auto_path = []
        self.is_playing = False
        self.is_scanning = False
        self.show_trail = False
        self.path_trail = []
        self.pending_start_solve = False
        self.pending_box_merge = None
        self.visual_offsets.clear()

        if self.main_process:
            self.stop_scan_solve()

        if restore_current_map:
            self.load_map(self.current_map)
        print(f"{mode_label}{reason}")
        return True

    def _restore_manual_playback_speed(self):
        if self.auto_solve_all:
            return
        self.move_delay = self.auto_solve_base_delay
        self.move_duration = self.auto_solve_base_move_duration

    def update_path_trail(self):
        """更新路径轨迹 - 只显示到下一个箱子/炸弹到达目标的路径"""
        if not self.show_trail or not self.auto_path:
            self.path_trail = []
            self.trail_type = "box"
            self.bomb_target_wall = None
            return

        px, py = self.player
        trail = [(px, py)]
        temp_boxes = set(self.boxes)
        temp_bombs = set(self.bombs)
        rows = len(self.grid)
        cols = len(self.grid[0]) if rows > 0 else 0
        self.trail_type = "box"
        self.bomb_target_wall = None

        for dx, dy in self.auto_path:
            if dx == 0 and dy == 0:
                trail.append((px, py))
                continue

            nx, ny = px + dx, py + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            if self.grid[ny][nx] == '#':
                continue

            # 检查推箱子
            if (nx, ny) in temp_boxes:
                bx, by = nx + dx, ny + dy
                if not (0 <= bx < cols and 0 <= by < rows):
                    continue
                if self.grid[by][bx] == '#' or (bx, by) in temp_boxes or (bx, by) in temp_bombs:
                    continue
                trail.append((nx, ny))
                temp_boxes.remove((nx, ny))
                temp_boxes.add((bx, by))
                if (bx, by) in self.targets:
                    trail.append((bx, by))
                    self.trail_type = "box"
                    break

            # 检查推炸弹
            elif (nx, ny) in temp_bombs:
                bx, by = nx + dx, ny + dy
                if not (0 <= bx < cols and 0 <= by < rows):
                    continue
                is_destructible_wall = self.grid[by][bx] == '#' and (0 < bx < cols - 1 and 0 < by < rows - 1)
                if (bx, by) in temp_boxes or (bx, by) in temp_bombs:
                    continue
                if self.grid[by][bx] == '#' and not is_destructible_wall:
                    continue
                trail.append((nx, ny))
                temp_bombs.remove((nx, ny))
                temp_bombs.add((bx, by))
                if self.grid[by][bx] == '#':
                    trail.append((bx, by))
                    self.trail_type = "bomb"
                    self.bomb_target_wall = (bx, by)
                    break

            else:
                trail.append((nx, ny))

            px, py = nx, ny

        self.path_trail = trail
    
    def _move_player_legacy_shadow(self, dx, dy):
        """移动玩家"""
        px, py = self.player
        nx, ny = px + dx, py + dy
        rows = len(self.grid)
        cols = len(self.grid[0]) if rows > 0 else 0

        if not (0 <= nx < cols and 0 <= ny < rows):
            print(f"[PYTHON-物理引擎] ❌ 移动失败! 越界了: 坐标({nx}, {ny})")
            return False

        if self.grid[ny][nx] == '#':
            print(f"[PYTHON-物理引擎] ❌ 移动失败! 撞墙了: 坐标({nx}, {ny})")
            return False

        if False:
            pass

        if False:
            pass
        """

        # 1. 检查是否撞墙
        if self.grid[ny][nx] == '#':
            print(f"[PYTHON-物理引擎] ❌ 移动失败! 撞墙了: 坐标({nx}, {ny})")
            return False

        # 推炸弹
        """
        if (nx, ny) in self.bombs:
            bx, by = nx + dx, ny + dy
            if not (0 <= bx < cols and 0 <= by < rows):
                print(f"[PYTHON-物理引擎] ❌ 移动失败! 炸弹推不动: 炸弹({nx},{ny}) -> 障碍({bx},{by}) (是否可炸:False)")
                return False

            is_destructible_wall = self.grid[by][bx] == '#' and (0 < bx < cols - 1 and 0 < by < rows - 1)

            if (bx, by) in self.boxes or (bx, by) in self.bombs or (self.grid[by][bx] == '#' and not is_destructible_wall):
                print(
                    f"[PYTHON-物理引擎] ❌ 移动失败! 炸弹推不动: 炸弹({nx},{ny}) -> 障碍({bx},{by}) "
                    f"(是否可炸:{is_destructible_wall})"
                )
                return False

            self.visual_offsets.clear()
            self.bombs.remove((nx, ny))

            if is_destructible_wall:
                self.explode_bomb(bx, by)
                print(f"[PYTHON-物理引擎] 💥 物理引爆炸弹成功! 墙壁({bx},{by})及周围被摧毁")
            else:
                self.bombs.add((bx, by))
                self.set_visual_offset(('bomb', bx, by), dx, dy)

            self.player = (nx, ny)
            self.set_visual_offset('player', dx, dy)
            self.last_move_time = time.time()
            return True

        if (nx, ny) in self.bombs:
            bx, by = nx + dx, ny + dy
            if self.grid[by][bx] == '#':
                self.visual_offsets.clear()
                self.bombs.remove((nx, ny))
                self.explode_bomb(bx, by)
                self.player = (nx, ny)
                self.set_visual_offset('player', dx, dy)
                self.last_move_time = time.time()
                return True
            if (bx, by) in self.bombs or (bx, by) in self.boxes:
                return False
            self.visual_offsets.clear()
            self.bombs.remove((nx, ny))
            self.bombs.add((bx, by))
            self.set_visual_offset(('bomb', bx, by), dx, dy)
            self.player = (nx, ny)
            self.set_visual_offset('player', dx, dy)
            self.last_move_time = time.time()
            return True

        # 推箱子
        if (nx, ny) in self.boxes:
            bx, by = nx + dx, ny + dy
            if not (0 <= bx < cols and 0 <= by < rows):
                print(f"[PYTHON-物理引擎] ❌ 移动失败! 箱子被死死挡住: 箱子({nx},{ny}) -> 障碍物({bx},{by})")
                return False
            if self.grid[by][bx] == '#' or (bx, by) in self.boxes or (bx, by) in self.bombs:
                print(f"[PYTHON-物理引擎] ❌ 移动失败! 箱子被死死挡住: 箱子({nx},{ny}) -> 障碍物({bx},{by})")
                return False

            box_id = self.box_ids.get((nx, ny))
            scanned_box_id = self.scanned_box_ids.get((nx, ny))
            self.visual_offsets.clear()
            self.boxes.remove((nx, ny))
            self.box_ids.pop((nx, ny), None)
            self.scanned_box_ids.pop((nx, ny), None)

            box_on_target = (bx, by) in self.targets

            self.boxes.add((bx, by))
            self.set_visual_offset(('box', bx, by), dx, dy)
            if box_id is not None:
                self.box_ids[(bx, by)] = box_id
            if scanned_box_id is not None:
                self.scanned_box_ids[(bx, by)] = scanned_box_id

            should_disappear = box_on_target
            if should_disappear:
                self.pending_box_merge = {
                    'pos': (bx, by),
                    'start_time': time.time(),
                }
            else:
                self.pending_box_merge = None
            # if box_on_target:
            #     final_box_id = box_id if box_id is not None else scanned_box_id
            #     final_target_id = target_id if target_id is not None else scanned_target_id

            #     if self.strict_mode:
            #         # 銆愭瀬绠€榄旀硶銆戯細ID鐩哥瓑鍗虫秷澶憋紒
            #         # 瀹岀編鍖呭鏅€氱瀛愯繘鏅€氬潙 (None == None -> True)
            #         # 瀹岀編闃茶寖甯D绠卞瓙璺繃鏅€氬潙 (1 == None -> False)
            #         should_disappear = (final_box_id == final_target_id)
            #     else:
            #         should_disappear = True

            self.boxes.add((bx, by))
            self.set_visual_offset(('box', bx, by), dx, dy)
            if box_id is not None:
                self.box_ids[(bx, by)] = box_id
            if scanned_box_id is not None:
                self.scanned_box_ids[(bx, by)] = scanned_box_id

            if should_disappear:
                self.pending_box_merge = {
                    'pos': (bx, by),
                    'start_time': time.time(),
                }
            else:
                self.pending_box_merge = None

            self.player = (nx, ny)
            self.set_visual_offset('player', dx, dy)
            self.last_move_time = time.time()
            return True

        if (nx, ny) in self.boxes:
            bx, by = nx + dx, ny + dy
            if self.grid[by][bx] == '#' or (bx, by) in self.boxes or (bx, by) in self.bombs:
                return False

            box_id = self.box_ids.get((nx, ny))
            scanned_box_id = self.scanned_box_ids.get((nx, ny))
            self.visual_offsets.clear()
            self.boxes.remove((nx, ny))
            self.box_ids.pop((nx, ny), None)
            self.scanned_box_ids.pop((nx, ny), None)

            target_id = self.target_ids.get((bx, by))
            scanned_target_id = self.scanned_target_ids.get((bx, by))
            box_on_target = (bx, by) in self.targets

            self.boxes.add((bx, by))
            self.set_visual_offset(('box', bx, by), dx, dy)
            if box_id is not None:
                self.box_ids[(bx, by)] = box_id
            if scanned_box_id is not None:
                self.scanned_box_ids[(bx, by)] = scanned_box_id

            should_disappear = box_on_target
            if should_disappear:
                self.pending_box_merge = {
                    'pos': (bx, by),
                    'start_time': time.time(),
                }
            else:
                self.pending_box_merge = None
            # if box_on_target:
            #     final_box_id = box_id if box_id is not None else scanned_box_id
            #     final_target_id = target_id if target_id is not None else scanned_target_id

            #     if self.strict_mode:
            #         # 【极简魔法】：ID相等即消失！
            #         # 完美包容普通箱子进普通坑 (None == None -> True)
            #         # 完美防范带ID箱子路过普通坑 (1 == None -> False)
            #         should_disappear = (final_box_id == final_target_id)
            #     else:
            #         should_disappear = True

            self.boxes.add((bx, by))
            self.set_visual_offset(('box', bx, by), dx, dy)
            if box_id is not None:
                self.box_ids[(bx, by)] = box_id
            if scanned_box_id is not None:
                self.scanned_box_ids[(bx, by)] = scanned_box_id

            if should_disappear and True:
                self.pending_box_merge = {
                    'pos': (bx, by),
                    'start_time': time.time(),
                }
            else:
                self.pending_box_merge = None

            self.player = (nx, ny)
            self.set_visual_offset('player', dx, dy)
            self.last_move_time = time.time()
            return True

        self.visual_offsets.clear()
        self.player = (nx, ny)
        self.set_visual_offset('player', dx, dy)
        self.last_move_time = time.time()
        return True

    def move_player(self, dx, dy):
        """移动玩家"""
        px, py = self.player
        nx, ny = px + dx, py + dy
        rows = len(self.grid)
        cols = len(self.grid[0]) if rows > 0 else 0

        if not (0 <= nx < cols and 0 <= ny < rows):
            print(f"[PYTHON-物理引擎] ❌ 移动失败! 越界了: 坐标({nx}, {ny})")
            return False

        if self.grid[ny][nx] == '#':
            print(f"[PYTHON-物理引擎] ❌ 移动失败! 撞墙了: 坐标({nx}, {ny})")
            return False

        if (nx, ny) in self.boxes:
            bx, by = nx + dx, ny + dy
            if not (0 <= bx < cols and 0 <= by < rows):
                print(f"[PYTHON-物理引擎] ❌ 移动失败! 箱子被死死挡住: 箱子({nx},{ny}) -> 障碍物({bx},{by})")
                return False
            if self.grid[by][bx] == '#' or (bx, by) in self.boxes or (bx, by) in self.bombs:
                print(f"[PYTHON-物理引擎] ❌ 移动失败! 箱子被死死挡住: 箱子({nx},{ny}) -> 障碍物({bx},{by})")
                return False

            move_started_at = time.time()
            box_id = self.box_ids.get((nx, ny))
            scanned_box_id = self.scanned_box_ids.get((nx, ny))
            simulated_destination = self._take_simulation_error_for_push(
                (nx, ny), box_id, scanned_box_id
            )

            if simulated_destination is not None:
                target_id = self.target_ids.get(simulated_destination)
                scanned_target_id = self.scanned_target_ids.get(simulated_destination)
                box_on_target = simulated_destination in self.targets

                self.visual_offsets.clear()
                self.boxes.remove((nx, ny))
                self.simulated_error_boxes.discard((nx, ny))
                self.box_ids.pop((nx, ny), None)
                self.scanned_box_ids.pop((nx, ny), None)
                self.boxes.add(simulated_destination)
                self.simulated_error_boxes.add(simulated_destination)
                if box_id is not None:
                    self.box_ids[simulated_destination] = box_id
                if scanned_box_id is not None:
                    self.scanned_box_ids[simulated_destination] = scanned_box_id
                self._migrate_pending_simulation_error_events(
                    (nx, ny), simulated_destination, box_id, scanned_box_id
                )

                should_disappear = self._box_matches_target_for_display(
                    box_id, scanned_box_id, target_id, scanned_target_id, box_on_target
                )
                self._queue_box_merge_if_needed(
                    simulated_destination, move_started_at, should_disappear
                )
                self.player = (nx, ny)
                self.set_visual_offset('player', dx, dy)
                self.last_move_time = move_started_at
                self._note_simulation_box_push()
                print(
                    f"[模拟错误] 第 {getattr(self, 'simulation_error_route_round', 0)} 轮箱子 "
                    f"({nx},{ny}) 已偏移到 {simulated_destination}，后续按当前残局继续推进。"
                )
                return True

            target_id = self.target_ids.get((bx, by))
            scanned_target_id = self.scanned_target_ids.get((bx, by))
            box_on_target = (bx, by) in self.targets
            is_simulated_error_box = (nx, ny) in self.simulated_error_boxes

            self.visual_offsets.clear()
            self.boxes.remove((nx, ny))
            self.box_ids.pop((nx, ny), None)
            self.scanned_box_ids.pop((nx, ny), None)
            if is_simulated_error_box:
                self.simulated_error_boxes.discard((nx, ny))

            self.boxes.add((bx, by))
            self.set_visual_offset(('box', bx, by), dx, dy)
            if is_simulated_error_box:
                self.simulated_error_boxes.add((bx, by))
            if box_id is not None:
                self.box_ids[(bx, by)] = box_id
            if scanned_box_id is not None:
                self.scanned_box_ids[(bx, by)] = scanned_box_id
            self._migrate_pending_simulation_error_events(
                (nx, ny), (bx, by), box_id, scanned_box_id
            )

            should_disappear = self._box_matches_target_for_display(
                box_id, scanned_box_id, target_id, scanned_target_id, box_on_target
            )
            self._queue_box_merge_if_needed((bx, by), move_started_at, should_disappear)

            self.player = (nx, ny)
            self.set_visual_offset('player', dx, dy)
            self.last_move_time = move_started_at
            self._note_simulation_box_push()
            return True

        if (nx, ny) in self.bombs:
            bx, by = nx + dx, ny + dy
            if not (0 <= bx < cols and 0 <= by < rows):
                print(f"[PYTHON-物理引擎] ❌ 移动失败! 炸弹推不动: 炸弹({nx},{ny}) -> 障碍({bx},{by}) (是否可炸:False)")
                return False

            is_destructible_wall = self.grid[by][bx] == '#' and (0 < bx < cols - 1 and 0 < by < rows - 1)

            if (bx, by) in self.boxes or (bx, by) in self.bombs or (self.grid[by][bx] == '#' and not is_destructible_wall):
                print(
                    f"[PYTHON-物理引擎] ❌ 移动失败! 炸弹推不动: 炸弹({nx},{ny}) -> 障碍({bx},{by}) "
                    f"(是否可炸:{is_destructible_wall})"
                )
                return False

            self.visual_offsets.clear()
            self.bombs.remove((nx, ny))
            if is_destructible_wall:
                self.explode_bomb(bx, by)
                print(f"[PYTHON-物理引擎] 💥 物理引爆炸弹成功! 墙壁({bx},{by})及周围被摧毁")
            else:
                self.bombs.add((bx, by))
                self.set_visual_offset(('bomb', bx, by), dx, dy)

            self.player = (nx, ny)
            self.set_visual_offset('player', dx, dy)
            self.last_move_time = time.time()
            return True

        self.visual_offsets.clear()
        self.player = (nx, ny)
        self.set_visual_offset('player', dx, dy)
        self.last_move_time = time.time()
        return True

    def handle_manual_movement(self):
        """按住方向键时按固定节奏触发平滑移动。"""
        if self.is_playing or self.id_assignment_mode or self.scan_solve_mode or self.map_creation_mode or self.residual_simulation_mode or self.awaiting_residual_review or self.waiting_for_recovery_id_input:
            self.manual_active_key = None
            self.next_manual_move_time = 0
            return

        now = time.time()

        keys = pygame.key.get_pressed()
        directions = (
            (pygame.K_UP, 0, -1),
            (pygame.K_DOWN, 0, 1),
            (pygame.K_LEFT, -1, 0),
            (pygame.K_RIGHT, 1, 0),
        )

        pressed_direction = None
        for key, dx, dy in directions:
            if keys[key]:
                pressed_direction = (key, dx, dy)
                break

        if pressed_direction is None:
            self.manual_active_key = None
            self.next_manual_move_time = now
            return

        key, dx, dy = pressed_direction

        # First press moves immediately; holding the same key repeats more slowly.
        if self.manual_active_key != key:
            moved = self.move_player(dx, dy)
            self.manual_active_key = key
            self.next_manual_move_time = now + self.manual_first_repeat_delay
            if moved and self.show_trail:
                self.update_path_trail()
            return

        if now < self.next_manual_move_time:
            return

        moved = self.move_player(dx, dy)
        self.next_manual_move_time = now + self.manual_move_interval
        if moved and self.show_trail:
            self.update_path_trail()

    def explode_bomb(self, bomb_x, bomb_y):
        """在3x3区域爆炸炸弹"""
        print(f"爆炸！炸弹在 ({bomb_x}, {bomb_y}) 爆炸")
        self.explosion_effect = (bomb_x, bomb_y, time.time())

        # 生成火焰粒子
        self.flame_particles = []
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                px = (bomb_x + dx) * BLOCK_SIZE + BLOCK_SIZE // 2
                py = (bomb_y + dy) * BLOCK_SIZE + BLOCK_SIZE // 2
                for _ in range(15):
                    self.flame_particles.append({
                        'x': px + random.randint(-15, 15),
                        'y': py + random.randint(-15, 15),
                        'vx': random.uniform(-2, 2),
                        'vy': random.uniform(-3, -1),
                        'size': random.randint(4, 10),
                        'lifetime': random.uniform(0.3, 0.8),
                        'age': 0
                    })

        # 摧毁3x3区域的墙壁（边界墙除外）
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                x, y = bomb_x + dx, bomb_y + dy
                if 0 < x < MAP_COLS - 1 and 0 < y < MAP_ROWS - 1 and self.grid[y][x] == '#':
                    self.grid[y][x] = ' '

    def set_visual_offset(self, key, dx, dy):
        """设置平滑移动的起始视觉偏移量"""
        self.visual_offsets[key] = (-dx * BLOCK_SIZE, -dy * BLOCK_SIZE)

    def update_visual_transitions(self):
        """鍦ㄥ姩鐢诲畬鎴愬悗鎻愪氦闇€瑕佸欢杩熻惤鍦扮殑瑙嗚鐘舵€?"""
        if not self.pending_box_merge:
            for bx, by in list(self.boxes):
                box_id = self.box_ids.get((bx, by))
                scanned_box_id = self.scanned_box_ids.get((bx, by))
                target_id = self.target_ids.get((bx, by))
                scanned_target_id = self.scanned_target_ids.get((bx, by))
                should_disappear = self._box_matches_target_for_display(
                    box_id, scanned_box_id, target_id, scanned_target_id, (bx, by) in self.targets
                )
                if should_disappear and ('box', bx, by) in self.visual_offsets:
                    self.pending_box_merge = {
                        'pos': (bx, by),
                        'start_time': self.last_move_time,
                    }
                    break
        if not self.pending_box_merge:
            return

        if time.time() - self.pending_box_merge['start_time'] < self.move_duration:
            return

        bx, by = self.pending_box_merge['pos']
        box_id = self.box_ids.get((bx, by))
        scanned_box_id = self.scanned_box_ids.get((bx, by))
        target_id = self.target_ids.get((bx, by))
        scanned_target_id = self.scanned_target_ids.get((bx, by))
        if not self._box_matches_target_for_display(
            box_id,
            scanned_box_id,
            target_id,
            scanned_target_id,
            (bx, by) in self.targets,
        ):
            self.pending_box_merge = None
            return

        self._remove_absorbed_box_and_target((bx, by))

    def _draw_centered_text(self, text, x, y, font=None, color=(0, 0, 0)):
        """绘制居中文本"""
        font = font or self.font
        text_surface = font.render(str(text), True, color)
        text_rect = text_surface.get_rect(center=(x, y))
        self.screen.blit(text_surface, text_rect)

    def _draw_cell(self, rect, color, border_color=None, border_width=1):
        """绘制单元格"""
        pygame.draw.rect(self.screen, color, rect)
        if border_color:
            pygame.draw.rect(self.screen, border_color, rect, border_width)

    def _draw_id_on_cell(self, x, y, scanned_ids, manual_ids, cell=None, px=None, py=None):
        """绘制单元格ID"""
        if px is None or py is None:
            center_x = x * BLOCK_SIZE + BLOCK_SIZE // 2
            center_y = y * BLOCK_SIZE + BLOCK_SIZE // 2
        else:
            center_x = px + BLOCK_SIZE // 2
            center_y = py + BLOCK_SIZE // 2

        if (x, y) in scanned_ids:
            self._draw_centered_text(scanned_ids[(x, y)], center_x, center_y)
        elif (x, y) in manual_ids:
            self._draw_centered_text(manual_ids[(x, y)], center_x, center_y)
        elif cell and cell != '.':
            self._draw_centered_text(cell, center_x, center_y, self.small_font)
    
    def create_premium_background(self):
        """生成极简暗黑蓝图背景 (预渲染提升性能)"""
        bg_surf = pygame.Surface((self.screen_w, self.screen_h), pygame.SRCALPHA)
        
        for y in range(self.screen_h):
            progress = y / self.screen_h
            r = int(22 * (1 - progress) + 5)
            g = int(28 * (1 - progress) + 8)
            b = int(40 * (1 - progress) + 12)
            pygame.draw.line(bg_surf, (r, g, b), (0, y), (self.screen_w, y))
            
        grid_color = (255, 255, 255, 8) 
        grid_spacing = 20
        for x in range(0, self.screen_w, grid_spacing):
            pygame.draw.line(bg_surf, grid_color, (x, 0), (x, self.screen_h))
        for y in range(0, self.screen_h, grid_spacing):
            pygame.draw.line(bg_surf, grid_color, (0, y), (self.screen_w, y))
            
        return bg_surf

    def draw_star(self, surface, color, x, y, size, angle=0):
        """绘制一个五角星"""
        points = []
        for i in range(5):
            outer_angle = angle + i * (2 * math.pi / 5)
            points.append((x + math.sin(outer_angle) * size, y - math.cos(outer_angle) * size))
            inner_angle = outer_angle + (math.pi / 5)
            points.append((x + math.sin(inner_angle) * size * 0.4, y - math.cos(inner_angle) * size * 0.4))
        if len(points) >= 3:
            pygame.draw.polygon(surface, color, points)
    
    def draw_glass_rect(self, surface, rect, base_color=(255, 255, 255), alpha=60, border_radius=15, blur_scale=0.15):
        """核心渲染：高品质液态玻璃/毛玻璃特效面板"""
        x, y, w, h = rect
        if w <= 0 or h <= 0: return
        
        bg_rect = pygame.Rect(x, y, w, h)
        try:
            bg = surface.subsurface(bg_rect).copy().convert_alpha()
        except ValueError:
            return
        
        sw, sh = max(1, int(w * blur_scale)), max(1, int(h * blur_scale))
        try:
            small = pygame.transform.smoothscale(bg, (sw, sh))
            bg = pygame.transform.smoothscale(small, (w, h))
        except ValueError:
            pass
            
        mask = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(mask, (255, 255, 255, 255), (0, 0, w, h), border_radius=border_radius)
        bg.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
        
        overlay = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(overlay, (*base_color[:3], alpha), (0, 0, w, h), border_radius=border_radius) 
        pygame.draw.rect(overlay, (255, 255, 255, 120), (0, 0, w, h), width=2, border_radius=border_radius) 
        pygame.draw.rect(overlay, (255, 255, 255, 40), (2, 2, w-4, h-4), width=1, border_radius=border_radius) 
        
        bg.blit(overlay, (0, 0))
        surface.blit(bg, (x, y))
    
    def draw_apple_glass_button(self, surface, rect, text, button_id, is_active=False):
        """渲染苹果 VisionOS 风格的动效玻璃按钮"""
        if isinstance(rect, tuple):
            rect = pygame.Rect(rect)
            
        mouse_pos = pygame.mouse.get_pos()
        is_hovered = rect.collidepoint(mouse_pos)

        target_hover = 1.0 if is_hovered else 0.0
        current_hover = self.button_hover_states.get(button_id, 0.0)
        current_hover += (target_hover - current_hover) * 0.15 
        self.button_hover_states[button_id] = current_hover

        base_color = (100, 200, 255) if is_active else (255, 255, 255)
        base_alpha = 60 if is_active else 20
        alpha = int(base_alpha + 50 * current_hover)

        self.draw_glass_rect(surface, rect, base_color=base_color, alpha=alpha - 10, border_radius=12, blur_scale=0.08)

        overlay = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
        border_alpha = int(80 + 120 * current_hover)
        pygame.draw.rect(overlay, (255, 255, 255, border_alpha), overlay.get_rect(), width=1, border_radius=12)
        pygame.draw.line(overlay, (255, 255, 255, 180), (10, 1), (rect.width - 10, 1), 1)
        surface.blit(overlay, rect.topleft)

        y_offset = int(-2 * current_hover) 
        text_color = (255, 255, 255) if is_active or is_hovered else (200, 200, 200)
        
        text_surf = self.small_font.render(text, True, text_color)
        text_rect = text_surf.get_rect(center=(rect.centerx, rect.centery + y_offset))
        surface.blit(text_surf, text_rect)

    def _update_right_panel_scroll_metrics(self, view_rect, content_height):
        self.right_panel_view_rect = pygame.Rect(view_rect)
        self.right_panel_content_height = max(content_height, view_rect.height)
        self.right_panel_max_scroll = max(0, self.right_panel_content_height - view_rect.height)

        if self.right_panel_max_scroll <= 0:
            self.map_scroll_offset = 0
        else:
            self.map_scroll_offset = max(0, min(self.map_scroll_offset, self.right_panel_max_scroll))

        track_x = view_rect.right + 6
        track_w = 8
        self.right_panel_scrollbar_rect = pygame.Rect(track_x, view_rect.y, track_w, view_rect.height)

        if self.right_panel_max_scroll <= 0:
            self.right_panel_scrollbar_thumb_rect = pygame.Rect(track_x, view_rect.y, track_w, view_rect.height)
            return

        thumb_h = max(40, int(view_rect.height * view_rect.height / self.right_panel_content_height))
        thumb_h = min(thumb_h, view_rect.height)
        travel = max(1, view_rect.height - thumb_h)
        ratio = self.map_scroll_offset / self.right_panel_max_scroll
        thumb_y = view_rect.y + int(travel * ratio)
        self.right_panel_scrollbar_thumb_rect = pygame.Rect(track_x, thumb_y, track_w, thumb_h)

    def _right_panel_contains_point(self, pos):
        return self.right_panel_view_rect.collidepoint(pos) or self.right_panel_scrollbar_rect.collidepoint(pos)

    def _set_right_panel_thumb_top(self, thumb_top):
        if self.right_panel_max_scroll <= 0:
            self.map_scroll_offset = 0
            return

        track = self.right_panel_scrollbar_rect
        thumb_h = self.right_panel_scrollbar_thumb_rect.height
        travel = max(1, track.height - thumb_h)
        clamped_top = max(track.y, min(thumb_top, track.bottom - thumb_h))
        ratio = (clamped_top - track.y) / travel
        self.map_scroll_offset = int(round(ratio * self.right_panel_max_scroll))

    def scroll_right_panel(self, delta):
        if self.right_panel_max_scroll <= 0:
            self.map_scroll_offset = 0
            return False

        new_offset = max(0, min(self.map_scroll_offset + delta, self.right_panel_max_scroll))
        if new_offset == self.map_scroll_offset:
            return False

        self.map_scroll_offset = new_offset
        return True

    def handle_right_panel_scroll(self, pos, wheel_delta):
        if not self._right_panel_contains_point(pos):
            return False
        return self.scroll_right_panel(-wheel_delta * 48)

    def handle_right_panel_mouse_down(self, pos):
        if self.right_panel_max_scroll <= 0:
            return False

        if self.right_panel_scrollbar_thumb_rect.collidepoint(pos):
            self.right_panel_dragging_scrollbar = True
            self.right_panel_drag_offset = pos[1] - self.right_panel_scrollbar_thumb_rect.y
            return True

        if self.right_panel_scrollbar_rect.collidepoint(pos):
            thumb_half = self.right_panel_scrollbar_thumb_rect.height // 2
            self._set_right_panel_thumb_top(pos[1] - thumb_half)
            self.right_panel_dragging_scrollbar = True
            self.right_panel_drag_offset = thumb_half
            return True

        return False

    def handle_right_panel_drag(self, mouse_y):
        if not self.right_panel_dragging_scrollbar:
            return False

        self._set_right_panel_thumb_top(mouse_y - self.right_panel_drag_offset)
        return True

    def stop_right_panel_drag(self):
        self.right_panel_dragging_scrollbar = False
        self.right_panel_drag_offset = 0

    def _update_map_list_scroll_metrics(self, view_rect, content_height):
        self.map_list_view_rect = pygame.Rect(view_rect)
        self.map_list_content_height = max(content_height, view_rect.height)
        self.map_list_max_scroll = max(0, self.map_list_content_height - view_rect.height)

        if self.map_list_max_scroll <= 0:
            self.map_list_scroll_offset = 0
        else:
            self.map_list_scroll_offset = max(0, min(self.map_list_scroll_offset, self.map_list_max_scroll))

        track_x = view_rect.right + 3
        track_w = 6
        self.map_list_scrollbar_rect = pygame.Rect(track_x, view_rect.y, track_w, view_rect.height)

        if self.map_list_max_scroll <= 0:
            self.map_list_scrollbar_thumb_rect = pygame.Rect(track_x, view_rect.y, track_w, view_rect.height)
            return

        thumb_h = max(32, int(view_rect.height * view_rect.height / self.map_list_content_height))
        thumb_h = min(thumb_h, view_rect.height)
        travel = max(1, view_rect.height - thumb_h)
        ratio = self.map_list_scroll_offset / self.map_list_max_scroll
        thumb_y = view_rect.y + int(travel * ratio)
        self.map_list_scrollbar_thumb_rect = pygame.Rect(track_x, thumb_y, track_w, thumb_h)

    def _map_list_contains_point(self, pos):
        return self.map_list_view_rect.collidepoint(pos) or self.map_list_scrollbar_rect.collidepoint(pos)

    def _set_map_list_thumb_top(self, thumb_top):
        if self.map_list_max_scroll <= 0:
            self.map_list_scroll_offset = 0
            return

        track = self.map_list_scrollbar_rect
        thumb_h = self.map_list_scrollbar_thumb_rect.height
        travel = max(1, track.height - thumb_h)
        clamped_top = max(track.y, min(thumb_top, track.bottom - thumb_h))
        ratio = (clamped_top - track.y) / travel
        self.map_list_scroll_offset = int(round(ratio * self.map_list_max_scroll))

    def scroll_map_list(self, delta):
        if self.map_list_max_scroll <= 0:
            self.map_list_scroll_offset = 0
            return False

        new_offset = max(0, min(self.map_list_scroll_offset + delta, self.map_list_max_scroll))
        if new_offset == self.map_list_scroll_offset:
            return False

        self.map_list_scroll_offset = new_offset
        return True

    def handle_map_list_scroll(self, pos, wheel_delta):
        if not self._map_list_contains_point(pos):
            return False
        return self.scroll_map_list(-wheel_delta * 48)

    def handle_map_list_mouse_down(self, pos):
        if self.map_list_max_scroll <= 0:
            return False

        if self.map_list_scrollbar_thumb_rect.collidepoint(pos):
            self.map_list_dragging_scrollbar = True
            self.map_list_drag_offset = pos[1] - self.map_list_scrollbar_thumb_rect.y
            return True

        if self.map_list_scrollbar_rect.collidepoint(pos):
            thumb_half = self.map_list_scrollbar_thumb_rect.height // 2
            self._set_map_list_thumb_top(pos[1] - thumb_half)
            self.map_list_dragging_scrollbar = True
            self.map_list_drag_offset = thumb_half
            return True

        return False

    def handle_map_list_drag(self, mouse_y):
        if not self.map_list_dragging_scrollbar:
            return False

        self._set_map_list_thumb_top(mouse_y - self.map_list_drag_offset)
        return True

    def stop_map_list_drag(self):
        self.map_list_dragging_scrollbar = False
        self.map_list_drag_offset = 0

    def stop_all_right_panel_drag(self):
        self.stop_right_panel_drag()
        self.stop_map_list_drag()

    def draw_map_creation(self):
        """绘制地图创建模式界面"""
        CREATION_COLORS = {
            '#': (100, 100, 100),  ' ': (240, 240, 240),  
            '$': (255, 165, 0),  '.': (200, 0, 200),  
            '@': (0, 0, 255),  'B': (255, 0, 0),  'GRID': (200, 200, 200)
        }
        
        NAMES = {'#': '墙', '$': '箱子', '.': '目标', '@': '玩家', 'B': '炸弹', ' ': '空地'}

        map_rect = pygame.Rect(0, 0, MAP_COLS * BLOCK_SIZE, MAP_ROWS * BLOCK_SIZE)
        panel_x = MAP_COLS * BLOCK_SIZE
        panel_w = self.screen_w - panel_x
        right_panel_rect = pygame.Rect(panel_x, 0, panel_w, self.screen_h)
        bottom_panel_rect = pygame.Rect(0, MAP_ROWS * BLOCK_SIZE, MAP_COLS * BLOCK_SIZE, self.screen_h - MAP_ROWS * BLOCK_SIZE)

        pygame.draw.rect(self.screen, (18, 22, 30), right_panel_rect)
        pygame.draw.rect(self.screen, (18, 22, 30), bottom_panel_rect)
        pygame.draw.line(self.screen, (80, 90, 110), (panel_x, 0), (panel_x, self.screen_h), 1)
        pygame.draw.line(self.screen, (80, 90, 110), (0, MAP_ROWS * BLOCK_SIZE), (MAP_COLS * BLOCK_SIZE, MAP_ROWS * BLOCK_SIZE), 1)
        
        old_clip = self.screen.get_clip()
        self.screen.set_clip(map_rect)
        for r in range(MAP_ROWS):
            for c in range(MAP_COLS):
                char = self.creation_grid[r][c]
                rect = pygame.Rect(c * BLOCK_SIZE, r * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE)
                self._draw_cell(rect, CREATION_COLORS[char], CREATION_COLORS['GRID'])
        self.screen.set_clip(old_clip)
        self.draw_hover_guides()
        self.draw_selected_tile_coord()

        brush_y = MAP_ROWS * BLOCK_SIZE + 10
        brush_text = f"当前笔刷: {NAMES[self.current_brush]}"
        brush_color = CREATION_COLORS[self.current_brush]
        
        brush_bg_rect = pygame.Rect(10, brush_y, MAP_COLS * BLOCK_SIZE - 20, 40)
        pygame.draw.rect(self.screen, (40, 40, 40), brush_bg_rect)
        pygame.draw.rect(self.screen, (100, 100, 100), brush_bg_rect, 2)
        
        color_sample_rect = pygame.Rect(20, brush_y + 8, 24, 24)
        pygame.draw.rect(self.screen, brush_color, color_sample_rect)
        pygame.draw.rect(self.screen, (150, 150, 150), color_sample_rect, 1)
        
        text_render = self.font.render(brush_text, True, (255, 255, 255))
        self.screen.blit(text_render, (50, brush_y + 10))
        
        info_y = 50
        mode_label = "编辑当前地图" if self.creation_save_mode == "current" else "编辑新地图"
        save_label = "S - 保存当前地图" if self.creation_save_mode == "current" else "S - 保存为新地图"
        
        info_texts = [
            f"=== 地图编辑模式 ===", "", f"模式: {mode_label}", f"当前笔刷: {NAMES[self.current_brush]}", "",
            "按键说明:", "1 - 墙壁", "2 - 箱子", "3 - 目标", "4 - 玩家", "5 - 炸弹", "0 - 擦除", "",
            save_label
        ]
        
        if self.creation_save_mode == "current":
            info_texts.append("Z - 另存为新地图")
            
        info_texts.extend([
            "C - 清空内部", "ESC - 退出模式"
        ])
        
        for i, text in enumerate(info_texts):
            if text.startswith("==="):
                color = (255, 255, 100); font = self.font
            elif text.startswith("模式"):
                color = (120, 220, 255); font = self.font
            elif text.startswith("当前"):
                color = (100, 255, 100); font = self.font
            else:
                color = (200, 200, 200); font = self.small_font
            
            text_render = font.render(text, True, color)
            self.screen.blit(text_render, (panel_x + 10, info_y + i * 25))
        
        pygame.display.flip()

    def draw_residual_simulation(self):
        """Draw the disposable residual-map editor and its recovery-only actions."""
        editor_colors = {
            '#': (100, 100, 100),
            ' ': (240, 240, 240),
            '$': (255, 165, 0),
            '.': (200, 0, 200),
            RESIDUAL_OVERLAP_BOX: (200, 0, 200),
            '@': (0, 0, 255),
            'B': (255, 0, 0),
            'GRID': (200, 200, 200),
        }
        names = {
            '#': '墙',
            '$': '箱子',
            '.': '目的地',
            RESIDUAL_OVERLAP_BOX: '重叠箱子',
            '@': '玩家',
            'B': '炸弹',
            ' ': '空地',
        }

        map_width = MAP_COLS * BLOCK_SIZE
        map_height = MAP_ROWS * BLOCK_SIZE
        map_rect = pygame.Rect(0, 0, map_width, map_height)
        panel_x = map_width
        panel_w = self.screen_w - panel_x
        right_panel_rect = pygame.Rect(panel_x, 0, panel_w, self.screen_h)
        bottom_panel_rect = pygame.Rect(0, map_height, map_width, self.screen_h - map_height)

        pygame.draw.rect(self.screen, (18, 22, 30), right_panel_rect)
        pygame.draw.rect(self.screen, (18, 22, 30), bottom_panel_rect)
        pygame.draw.line(self.screen, (80, 90, 110), (panel_x, 0), (panel_x, self.screen_h), 1)
        pygame.draw.line(self.screen, (80, 90, 110), (0, map_height), (map_width, map_height), 1)

        old_clip = self.screen.get_clip()
        self.screen.set_clip(map_rect)
        for row in range(MAP_ROWS):
            for col in range(MAP_COLS):
                char = self.residual_simulation_grid[row][col]
                rect = pygame.Rect(col * BLOCK_SIZE, row * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE)
                self._draw_cell(
                    rect,
                    editor_colors.get(char, editor_colors[' ']),
                    editor_colors['GRID'],
                )
                if char == RESIDUAL_OVERLAP_BOX:
                    self._draw_centered_text(
                        "x",
                        col * BLOCK_SIZE + BLOCK_SIZE // 2,
                        row * BLOCK_SIZE + BLOCK_SIZE // 2,
                    )
        self.screen.set_clip(old_clip)
        self.draw_hover_guides()
        self.draw_selected_tile_coord()

        brush_name = names.get(self.residual_simulation_brush, '空地')

        self.residual_simulation_buttons = []
        title = self.font.render("残局模拟", True, (255, 255, 150))
        self.screen.blit(title, title.get_rect(center=(panel_x + panel_w // 2, 34)))

        info_lines = [
            f"当前笔刷: {brush_name}",
            "1 墙壁   2 箱子",
            "3 目的地 4 玩家",
            "5 炸弹   6 重叠箱",
            "0 擦除",
            f"快照队列: {len(getattr(self, 'residual_simulation_snapshots', ())) } 张",
            "F5 保存   F6 载入下一张",
            "D/S 从队首 map1 开始",
        ]
        for index, text in enumerate(info_lines):
            color = (110, 230, 255) if index == 0 else (205, 215, 225)
            rendered = self.small_font.render(text, True, color)
            self.screen.blit(rendered, (panel_x + 14, 75 + index * 24))

        button_x = panel_x + 12
        button_w = max(1, panel_w - 24)
        button_h = 38
        button_y = 75 + len(info_lines) * 24 + 12
        actions = [
            ("direct", "直接残局求解  D", True),
            ("identified", "扫描残局求解  S", False),
            ("save_snapshot", "保存快照  F5", False),
            ("load_snapshot", "载入下一快照  F6", False),
            ("clear", "清空残局  C", False),
            ("exit", "退出残局模拟  ESC", False),
        ]
        for action, label, is_active in actions:
            button_rect = pygame.Rect(button_x, button_y, button_w, button_h)
            self.residual_simulation_buttons.append((action, button_rect))
            self.draw_apple_glass_button(
                self.screen,
                button_rect,
                label,
                button_id=f"residual_{action}",
                is_active=is_active,
            )
            button_y += button_h + 10

        pygame.display.flip()

    def _draw_scan_focus_overlay(self):
        recovery_focus = (
            self.waiting_for_recovery_id_input
            and self.recovery_entity_pos is not None
        )
        scan_focus = self.scan_solve_mode and self.current_target_position is not None
        if not recovery_focus and not scan_focus:
            return

        tx, ty = (
            self.recovery_entity_pos
            if recovery_focus
            else self.current_target_position
        )
        cx = tx * BLOCK_SIZE + BLOCK_SIZE // 2
        cy = ty * BLOCK_SIZE + BLOCK_SIZE // 2
        pulse = abs((time.time() * 4.5) % 2 - 1)
        overlay = pygame.Surface((MAP_COLS * BLOCK_SIZE, MAP_ROWS * BLOCK_SIZE), pygame.SRCALPHA)

        is_focus_anim = (
            not recovery_focus
            and self.scan_pause_start > 0
            and not self.waiting_for_id_input
        )
        if recovery_focus:
            frame_color = (255, 190, 70, 230)
        else:
            frame_color = (80, 255, 180, 230) if is_focus_anim else (50, 255, 50, 230)
        fill_alpha = 55 if is_focus_anim else 85

        pygame.draw.rect(
            overlay,
            (frame_color[0], frame_color[1], frame_color[2], fill_alpha),
            (tx * BLOCK_SIZE, ty * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE),
            border_radius=10
        )

        outer_radius = int(BLOCK_SIZE * (0.55 + 0.12 * pulse))
        inner_radius = int(BLOCK_SIZE * (0.33 + 0.05 * (1.0 - pulse)))
        pygame.draw.circle(overlay, (*frame_color[:3], 110), (cx, cy), outer_radius, 3)
        pygame.draw.circle(overlay, (*frame_color[:3], 180), (cx, cy), inner_radius, 2)

        observer = self.recovery_observation_pos if recovery_focus else self.player
        if observer:
            px, py = observer
            player_cx = px * BLOCK_SIZE + BLOCK_SIZE // 2
            player_cy = py * BLOCK_SIZE + BLOCK_SIZE // 2
            pygame.draw.line(overlay, (*frame_color[:3], 120), (player_cx, player_cy), (cx, cy), 2)
            pygame.draw.circle(overlay, (*frame_color[:3], 150), (player_cx, player_cy), 5)

        if is_focus_anim:
            focus_progress = min((time.time() - self.scan_pause_start) / self.scan_focus_duration, 1.0)
            sweep_y = int((ty * BLOCK_SIZE) + focus_progress * BLOCK_SIZE)
            pygame.draw.line(
                overlay,
                (255, 255, 255, 180),
                (tx * BLOCK_SIZE + 4, sweep_y),
                ((tx + 1) * BLOCK_SIZE - 4, sweep_y),
                3
            )

        s = BLOCK_SIZE // 2 + 6
        l = 12
        t = 4
        pygame.draw.line(overlay, frame_color, (cx - s, cy - s), (cx - s + l, cy - s), t)
        pygame.draw.line(overlay, frame_color, (cx - s, cy - s), (cx - s, cy - s + l), t)
        pygame.draw.line(overlay, frame_color, (cx + s, cy - s), (cx + s - l, cy - s), t)
        pygame.draw.line(overlay, frame_color, (cx + s, cy - s), (cx + s, cy - s + l), t)
        pygame.draw.line(overlay, frame_color, (cx - s, cy + s), (cx - s + l, cy + s), t)
        pygame.draw.line(overlay, frame_color, (cx - s, cy + s), (cx - s, cy + s - l), t)
        pygame.draw.line(overlay, frame_color, (cx + s, cy + s), (cx + s - l, cy + s), t)
        pygame.draw.line(overlay, frame_color, (cx + s, cy + s), (cx + s, cy + s - l), t)

        self.screen.blit(overlay, (0, 0))

        if recovery_focus:
            label = "RECOVERY ID"
        else:
            label = "SCAN LOCK" if is_focus_anim else "ID READY"
        label_color = (255, 255, 180) if is_focus_anim else (180, 255, 180)
        label_text = self.small_font.render(label, True, label_color)
        label_rect = label_text.get_rect(center=(cx, max(14, ty * BLOCK_SIZE - 10)))
        self.screen.blit(label_text, label_rect)

    def draw(self, message=""):
        self.screen.blit(self.premium_bg, (0, 0))
        
        if self.map_creation_mode:
            self.draw_map_creation()
            return

        if self.residual_simulation_mode:
            self.draw_residual_simulation()
            return

        progress = 1.0
        if self.last_move_time > 0:
            progress = (time.time() - self.last_move_time) / self.move_duration
            if progress > 1.0: progress = 1.0
        
        eased_progress = 1.0 - (1.0 - progress) ** 2
        offset_multiplier = 1.0 - eased_progress

        for y in range(len(self.grid)):
            for x in range(len(self.grid[0])):
                rect = (x * BLOCK_SIZE, y * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE)
                cell = self.grid[y][x]

                if cell == '#':
                    self._draw_cell(rect, COLOR_WALL, COLOR_GRID)
                elif cell in '.abcdefghijklmnopqrstuvwxyz':
                    self._draw_cell(rect, COLOR_TARGET, COLOR_GRID)
                    overlap_marker = self._target_overlap_marker((x, y))
                    if overlap_marker:
                        self._draw_centered_text(
                            overlap_marker,
                            x * BLOCK_SIZE + BLOCK_SIZE // 2,
                            y * BLOCK_SIZE + BLOCK_SIZE // 2,
                        )
                    else:
                        self._draw_id_on_cell(x, y, self.scanned_target_ids, self.target_ids, cell)
                else:
                    self._draw_cell(rect, COLOR_BG, COLOR_GRID)
        
        if self.bomb_target_wall:
            wx, wy = self.bomb_target_wall
            center_x = wx * BLOCK_SIZE + BLOCK_SIZE // 2
            center_y = wy * BLOCK_SIZE + BLOCK_SIZE // 2
            
            crosshair_surface = pygame.Surface((MAP_COLS * BLOCK_SIZE, MAP_ROWS * BLOCK_SIZE), pygame.SRCALPHA)
            
            pulse = abs((time.time() * 3) % 2 - 1) 
            crosshair_color = (255, int(50 + 150 * pulse), 0, 220) 
            crosshair_size = 18 
            line_width = 4
            
            pygame.draw.line(crosshair_surface, crosshair_color, (center_x - crosshair_size, center_y), (center_x + crosshair_size, center_y), line_width)
            pygame.draw.line(crosshair_surface, crosshair_color, (center_x, center_y - crosshair_size), (center_x, center_y + crosshair_size), line_width)
            outer_radius = int(15 + 5 * pulse)
            pygame.draw.circle(crosshair_surface, crosshair_color, (center_x, center_y), outer_radius, 3)
            pygame.draw.circle(crosshair_surface, (255, 0, 0, 255), (center_x, center_y), 4)
            self.screen.blit(crosshair_surface, (0, 0))
            
            
            # 👇👇👇 核心新增：扫描目标高亮锁定框 👇👇👇
        if False and self.waiting_for_id_input and self.scan_solve_mode and self.current_scan_pause_index < len(self.scan_target_positions):
            tx, ty = self.scan_target_positions[self.current_scan_pause_index]
            cx = tx * BLOCK_SIZE + BLOCK_SIZE // 2
            cy = ty * BLOCK_SIZE + BLOCK_SIZE // 2
            
            # 呼吸灯效果
            pulse = abs((time.time() * 6) % 2 - 1)
            color = (50, 255, 50, int(100 + 155 * pulse)) 
            
            target_surf = pygame.Surface((self.screen_w, self.screen_h), pygame.SRCALPHA)
            s = BLOCK_SIZE // 2 + 4  # 框的大小
            t = 4  # 线条粗细
            l = 12 # 拐角长度
            
            # 绘制科幻风格的四角锁定框
            # 左上
            pygame.draw.line(target_surf, color, (cx - s, cy - s), (cx - s + l, cy - s), t)
            pygame.draw.line(target_surf, color, (cx - s, cy - s), (cx - s, cy - s + l), t)
            # 右上
            pygame.draw.line(target_surf, color, (cx + s, cy - s), (cx + s - l, cy - s), t)
            pygame.draw.line(target_surf, color, (cx + s, cy - s), (cx + s, cy - s + l), t)
            # 左下
            pygame.draw.line(target_surf, color, (cx - s, cy + s), (cx - s + l, cy + s), t)
            pygame.draw.line(target_surf, color, (cx - s, cy + s), (cx - s, cy + s - l), t)
            # 右下
            pygame.draw.line(target_surf, color, (cx + s, cy + s), (cx + s - l, cy + s), t)
            pygame.draw.line(target_surf, color, (cx + s, cy + s), (cx + s, cy + s - l), t)
            
            self.screen.blit(target_surf, (0, 0))
        # 👆👆👆 核心新增结束 👆👆👆
        
        if self.explosion_effect:
            ex, ey, start_time = self.explosion_effect
            elapsed = time.time() - start_time
            
            if elapsed < self.explosion_duration:
                explosion_surface = pygame.Surface((MAP_COLS * BLOCK_SIZE, MAP_ROWS * BLOCK_SIZE), pygame.SRCALPHA)
                center_x = ex * BLOCK_SIZE + BLOCK_SIZE // 2
                center_y = ey * BLOCK_SIZE + BLOCK_SIZE // 2
                progress = elapsed / self.explosion_duration
                dt = 1.0 / 120.0 
                
                active_particles = []
                for particle in self.flame_particles:
                    particle['age'] += dt
                    if particle['age'] < particle['lifetime']:
                        particle['x'] += particle['vx']
                        particle['y'] += particle['vy']
                        particle['vy'] += -0.2 
                        particle['vx'] *= 0.98 
                        
                        life_progress = particle['age'] / particle['lifetime']
                        
                        if life_progress < 0.2:
                            t = life_progress / 0.2
                            color = (255, 255, int(255 * (1 - t * 0.3)))
                        elif life_progress < 0.5:
                            t = (life_progress - 0.2) / 0.3
                            color = (255, int(255 - 100 * t), 0)
                        elif life_progress < 0.8:
                            t = (life_progress - 0.5) / 0.3
                            color = (int(255 - 55 * t), int(155 - 55 * t), 0)
                        else:
                            t = (life_progress - 0.8) / 0.2
                            color = (int(200 * (1 - t)), int(100 * (1 - t)), int(100 * (1 - t)))
                        
                        alpha = int(255 * (1 - life_progress))
                        
                        if life_progress < 0.3:
                            size = int(particle['size'] * (1 + life_progress))
                        else:
                            size = int(particle['size'] * (1.3 - life_progress * 0.5))
                        
                        if size > 0:
                            glow_size = size + 4
                            glow_color = (*color, alpha // 3)
                            pygame.draw.circle(explosion_surface, glow_color, (int(particle['x']), int(particle['y'])), glow_size)
                            main_color = (*color, alpha)
                            pygame.draw.circle(explosion_surface, main_color, (int(particle['x']), int(particle['y'])), size)
                            if life_progress < 0.4:
                                core_size = max(1, size // 2)
                                core_color = (255, 255, 200, alpha)
                                pygame.draw.circle(explosion_surface, core_color, (int(particle['x']), int(particle['y'])), core_size)
                        
                        active_particles.append(particle)
                
                self.flame_particles = active_particles
                
                if progress < 0.15:
                    flash_alpha = int(255 * (1 - progress / 0.15))
                    flash_radius = int(60 * (progress / 0.15))
                    pygame.draw.circle(explosion_surface, (255, 255, 255, flash_alpha), (center_x, center_y), flash_radius)
                    pygame.draw.circle(explosion_surface, (255, 200, 100, flash_alpha // 2), (center_x, center_y), flash_radius + 10)
                
                self.screen.blit(explosion_surface, (0, 0))
            else:
                self.explosion_effect = None
                self.flame_particles = []

        for bx, by in self.boxes:
            if self._box_overlaps_target((bx, by)):
                continue
            box_px, box_py = bx * BLOCK_SIZE, by * BLOCK_SIZE
            if ('box', bx, by) in self.visual_offsets:
                ox, oy = self.visual_offsets[('box', bx, by)]
                box_px += ox * offset_multiplier
                box_py += oy * offset_multiplier
            
            rect = (box_px, box_py, BLOCK_SIZE, BLOCK_SIZE)
            if (bx, by) in self.simulated_error_boxes:
                color = COLOR_SIMULATED_ERROR_BOX
            elif self.id_assignment_mode and self.temp_box_pos == (bx, by):
                color = (255, 255, 100)
            else:
                color = COLOR_BOX
            self._draw_cell(rect, color)
            self._draw_id_on_cell(bx, by, self.scanned_box_ids, self.box_ids, px=box_px, py=box_py)

        for source, destination in self.simulated_error_previews.items():
            if destination in self.boxes:
                continue
            if destination in self.targets and self._box_target_ids_mismatch(source, destination):
                continue
            dx, dy = destination
            rect = (dx * BLOCK_SIZE, dy * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE)
            self._draw_cell(rect, COLOR_SIMULATED_ERROR_BOX)
            preview_id = self._box_id_at(source)
            if preview_id is not None:
                self._draw_centered_text(
                    preview_id,
                    dx * BLOCK_SIZE + BLOCK_SIZE // 2,
                    dy * BLOCK_SIZE + BLOCK_SIZE // 2,
                )

        for bx, by in self.bombs:
            bomb_px, bomb_py = bx * BLOCK_SIZE, by * BLOCK_SIZE
            if ('bomb', bx, by) in self.visual_offsets:
                ox, oy = self.visual_offsets[('bomb', bx, by)]
                bomb_px += ox * offset_multiplier
                bomb_py += oy * offset_multiplier

            rect = (bomb_px, bomb_py, BLOCK_SIZE, BLOCK_SIZE)
            self._draw_cell(rect, COLOR_BOMB)
            center = (bomb_px + BLOCK_SIZE // 2, bomb_py + BLOCK_SIZE // 2)
            pygame.draw.circle(self.screen, (0, 0, 0), center, 5)

        if self.player:
            px, py = self.player
            player_px, player_py = px * BLOCK_SIZE, py * BLOCK_SIZE
            
            is_moving = False
            ox, oy = 0, 0
            if 'player' in self.visual_offsets:
                ox, oy = self.visual_offsets['player']
                player_px += ox * offset_multiplier
                player_py += oy * offset_multiplier
                if progress < 1.0 and (ox != 0 or oy != 0):
                    is_moving = True

            rect = (player_px, player_py, BLOCK_SIZE, BLOCK_SIZE)
            self._draw_cell(rect, COLOR_PLAYER)
            
            if is_moving:
                center_x = player_px + BLOCK_SIZE / 2
                center_y = player_py + BLOCK_SIZE / 2
                drift_x, drift_y = (ox / BLOCK_SIZE) * 2, (oy / BLOCK_SIZE) * 2
                
                for _ in range(random.randint(1, 2)):
                    self.star_particles.append({
                        'x': center_x + random.uniform(-8, 8),
                        'y': center_y + random.uniform(-8, 8),
                        'vx': random.uniform(-0.5, 0.5) - drift_x,
                        'vy': random.uniform(-0.5, 0.5) - drift_y,
                        'size': random.uniform(3, 7),
                        'color': random.choice([(255, 255, 0), (255, 215, 0), (255, 255, 255), (0, 255, 255)]),
                        'life': 0.5, 'max_life': 0.5,
                        'angle': random.uniform(0, math.pi * 2),
                        'rot_speed': random.uniform(-0.3, 0.3)
                    })
        
        if self.star_particles:
            star_surf = pygame.Surface((MAP_COLS * BLOCK_SIZE, MAP_ROWS * BLOCK_SIZE), pygame.SRCALPHA)
            dt = 1.0 / 120.0
            active_stars = []
            for p in self.star_particles:
                p['x'] += p['vx']
                p['y'] += p['vy']
                p['life'] -= dt
                p['angle'] += p['rot_speed']
                
                if p['life'] > 0:
                    active_stars.append(p)
                    alpha = int(255 * (p['life'] / p['max_life']))
                    color = (*p['color'], alpha)
                    self.draw_star(star_surf, color, p['x'], p['y'], p['size'], p['angle'])
                    
            self.star_particles = active_stars
            self.screen.blit(star_surf, (0, 0))

        self._draw_scan_focus_overlay()
        self.draw_hover_guides()
        self.draw_selected_tile_coord()

        # --- Draw UI Panel ---
        ui_y = MAP_ROWS * BLOCK_SIZE
        
        # 扩大后的液态玻璃层
        self.draw_glass_rect(self.screen, (5, ui_y + 5, MAP_COLS * BLOCK_SIZE - 10, 110), base_color=(30, 40, 55), alpha=160, border_radius=15)

        mode_str = "严格" if self.strict_mode else "普通"
        id_mode_str = " | ID模式: 开" if self.id_assignment_mode else ""
        set_player_mode_str = " | 设置玩家: 开" if self.set_player_mode else ""
        status = f"地图: {self.current_map} | 模式: {mode_str}{id_mode_str}{set_player_mode_str} | 箱子: {len(self.boxes)} | 炸弹: {len(self.bombs)}"

        text1 = self.font.render(status, True, (255, 255, 255))
        self.screen.blit(text1, (15, ui_y + 15))

        if message:
            text2 = self.font.render(message, True, (255, 255, 0))
            self.screen.blit(text2, (15, ui_y + 40))
        elif self.waiting_for_recovery_id_input:
            text2 = self.font.render(
                f"残局复查识别 {self.recovery_entity_pos} "
                f"方向 {self.recovery_view_direction} - 请在右侧选择ID",
                True,
                (255, 220, 120),
            )
            self.screen.blit(text2, (15, ui_y + 40))
        elif self.scan_pause_start > 0:
            text2 = self.font.render("扫描锁定中 - 正在高亮识别当前箱子/目标", True, (120, 255, 200))
            text2 = self.font.render("扫描已锁定 - 点击右侧按钮选择ID [ESC退出]", True, (255, 255, 100))
            self.screen.blit(text2, (15, ui_y + 40))
        elif self.waiting_for_id_input:
            text2 = self.font.render("扫描已锁定 - 点击右侧按钮选择ID [ESC退出]", True, (255, 255, 100))
            text2 = self.font.render(f"扫描暂停 - 请点击右侧按钮选择ID [ESC退出]", True, (255, 255, 100))
            self.screen.blit(text2, (15, ui_y + 40))
        elif self.scan_solve_mode:
            text2 = self.font.render("扫描求解模式运行中... [ESC退出]", True, (100, 255, 255))
            self.screen.blit(text2, (15, ui_y + 40))
        elif self.serial_bridge.connected:
            text2 = self.small_font.render(
                f"MCU: {self.serial_status_text}", True, (120, 220, 255)
            )
            self.screen.blit(text2, (15, ui_y + 40))
        elif self.id_assignment_mode:
            if self.waiting_for_target:
                text2 = self.font.render(f"现在点击目标位置 (ID {self.current_id}) [ESC取消]", True, (255, 100, 255))
            else:
                text2 = self.font.render(f"点击箱子 (ID {self.current_id}) [ESC退出ID模式]", True, (100, 255, 100))
            self.screen.blit(text2, (15, ui_y + 40))
            progress = f"箱子: {len(self.box_ids)}/{len(self.boxes)} | 目标: {len(self.target_ids)}/{len(self.targets)}"
            progress_text = self.small_font.render(progress, True, (200, 200, 100))
            self.screen.blit(progress_text, (15, ui_y + 65))
        elif self.set_player_mode:
            text2 = self.font.render("点击地图空地设置玩家位置 [ESC退出]", True, (120, 220, 255))
            self.screen.blit(text2, (15, ui_y + 40))
        else:
            text2 = self.small_font.render("[空格] 求解 | [方向键] 手动移动 | 严格模式: 箱子ID必须匹配目标ID", True, (200, 200, 200))
            self.screen.blit(text2, (15, ui_y + 40))
        
        # 显示求解时间
        if not message and self.scan_pause_start > 0:
            text2 = self.font.render("扫描锁定中 - 正在高亮识别当前箱子/目标", True, (120, 255, 200))
            self.screen.blit(text2, (15, ui_y + 40))
        elif not message and self.waiting_for_id_input:
            text2 = self.font.render("扫描已锁定 - 点击右侧按钮选择ID [ESC退出]", True, (255, 255, 100))
            self.screen.blit(text2, (15, ui_y + 40))

        if self.last_solve_time is not None:
            if self.serial_mode in ("direct", "identified"):
                time_info = f"MCU正式求解: {self.last_solve_time:.2f}s"
                if self.serial_recognition_compute_time is not None:
                    time_info += f" | 识别计算: {self.serial_recognition_compute_time:.2f}s"
                if self.serial_recognition_total_time is not None:
                    time_info += f" | 含等待总计: {self.serial_recognition_total_time:.2f}s"
            else:
                time_info = f"协议墙钟(不含人工等待): {self.last_solve_time:.2f}s"
            time_text = self.small_font.render(time_info, True, (100, 255, 100))
            self.screen.blit(time_text, (15, ui_y + 65))
            
        # --- 核心：总步数统计渲染 ---
        step_info = ""
        turn_info = ""
        if self.scan_planned_total > 0 or self.solve_planned_steps > 0:
            total_planned = self.scan_planned_total + self.solve_planned_steps
            total_executed = self.scan_executed_steps + self.solve_executed_steps
            total_turns = self.scan_planned_turns + self.solve_planned_turns
            
            scan_str = f"{self.scan_executed_steps}/{self.scan_planned_total}" if self.scan_planned_total > 0 else "0"
            solve_str = f"{self.solve_executed_steps}/{self.solve_planned_steps}" if self.solve_planned_steps > 0 else ("等待..." if self.scan_planned_total > 0 else "0")
            scan_turn_str = str(self.scan_planned_turns) if self.scan_planned_total > 0 else "0"
            solve_turn_str = str(self.solve_planned_turns) if self.solve_planned_steps > 0 else ("等待..." if self.scan_planned_total > 0 else "0")
            
            step_info = f"总执行/计划: {total_executed}/{total_planned}步 (扫描: {scan_str} | 求解: {solve_str})"
            turn_info = f"拐弯次数: {total_turns}次 (扫描: {scan_turn_str} | 求解: {solve_turn_str})"
            
        if step_info:
            step_text = self.small_font.render(step_info, True, (100, 255, 255))
            self.screen.blit(step_text, (15, ui_y + 85))
        if turn_info:
            turn_text = self.small_font.render(turn_info, True, (255, 220, 120))
            self.screen.blit(turn_text, (15, ui_y + 103))

        self.draw_right_panel()
        self.draw_map_creation_choice_overlay()
        pygame.display.flip()

    def draw_right_panel(self):
        panel_x = MAP_COLS * BLOCK_SIZE
        panel_w = self.map_panel_width
        panel_h = self.screen_h

        self.draw_glass_rect(self.screen, (panel_x + 5, 10, panel_w - 15, panel_h - 20), base_color=(20, 30, 40), alpha=120, border_radius=20)

        title = self.font.render("地图选择", True, (255, 255, 255))
        title_rect = title.get_rect(center=(panel_x + panel_w // 2, 20))
        self.screen.blit(title, title_rect)

        self.map_buttons = [] 
        button_w = (panel_w - 30) // 2 
        button_h = 28
        button_x_left = panel_x + 10
        button_x_right = panel_x + 10 + button_w + 10 
        start_y = 45
        
        if self.show_custom_maps:
            if self.custom_maps:
                for i, (map_name, map_data) in enumerate(sorted(self.custom_maps.items(), key=lambda item: natural_map_sort_key(item[0]))):
                    row = i // 2
                    col = i % 2
                    button_x = button_x_left if col == 0 else button_x_right
                    y_pos = start_y + row * (button_h + 3)
                    
                    button_rect = pygame.Rect(button_x, y_pos, button_w, button_h)
                    self.map_buttons.append((map_name, button_rect))
                    is_current = map_name == self.current_map
                    self.draw_apple_glass_button(self.screen, button_rect, map_name, button_id=f"map_{map_name}", is_active=is_current)
                custom_row = (len(self.custom_maps) + 1) // 2
            else:
                no_maps_text = self.small_font.render("暂无自定义地图", True, (150, 150, 150))
                self.screen.blit(no_maps_text, (button_x_left, start_y))
                custom_row = 1
        else:
            map_keys = sorted(MAPS.keys(), key=lambda x: int(x))
            max_maps_visible = len(map_keys)

            for i, key in enumerate(map_keys[:max_maps_visible]):
                row = i // 2 
                col = i % 2 
                button_x = button_x_left if col == 0 else button_x_right
                y_pos = start_y + row * (button_h + 3)
                button_rect = pygame.Rect(button_x, y_pos, button_w, button_h)
                self.map_buttons.append((key, button_rect))
                is_current = key == self.current_map
                map_name = f"地图 {key}"
                self.draw_apple_glass_button(self.screen, button_rect, map_name, button_id=f"map_{key}", is_active=is_current)
            
            custom_row = (max_maps_visible + 1) // 2
        
        custom_y = start_y + custom_row * (button_h + 3)
        custom_button_rect = pygame.Rect(button_x_left, custom_y, panel_w - 20, button_h)
        self.map_buttons.append(("custom", custom_button_rect))
        
        if self.show_custom_maps:
            button_text = "← 返回内置地图"
            is_active = True
        else:
            button_text = f"自定义地图 ({len(self.custom_maps)}) →"
            is_active = False
        
        self.draw_apple_glass_button(self.screen, custom_button_rect, button_text, button_id="btn_custom_toggle", is_active=is_active)
        
        control_y = custom_y + button_h + 20
        control_title = self.font.render("控制", True, (255, 255, 255))
        control_title_rect = control_title.get_rect(center=(panel_x + panel_w // 2, control_y))
        self.screen.blit(control_title, control_title_rect)
        control_y += 30

        self.control_buttons = [] 
        control_button_w = panel_w - 20
        control_button_x = panel_x + 10

        controls = [
            ("map_creation", "编辑地图", not self.is_playing and not self.id_assignment_mode and not self.scan_solve_mode),
            ("scan_solve", "扫描求解", not self.is_playing and not self.id_assignment_mode and not self.scan_solve_mode),
            ("toggle_mode", "切换模式" if not self.id_assignment_mode else "切换模式", not self.is_playing and not self.id_assignment_mode and not self.scan_solve_mode),
            ("assign_ids", "分配ID" if not self.id_assignment_mode else "退出ID模式", not self.is_playing and not self.scan_solve_mode),
            ("reset_map", "重置地图", not self.is_playing),
            ("quit", "退出", True)
        ]

        for action, label, enabled in controls:
            button_rect = pygame.Rect(control_button_x, control_y, control_button_w, button_h)
            self.control_buttons.append((action, button_rect, enabled))

            if enabled:
                self.draw_apple_glass_button(self.screen, button_rect, label, button_id=f"btn_{action}", is_active=False)
            else:
                self.draw_glass_rect(self.screen, button_rect, base_color=(50, 50, 50), alpha=40, border_radius=12, blur_scale=0.15)
                text = self.small_font.render(label, True, (100, 100, 100))
                text_rect = text.get_rect(center=button_rect.center)
                self.screen.blit(text, text_rect)

            control_y += button_h + 5

        mode_y = control_y + 10
        mode_text = f"当前模式: {'严格' if self.strict_mode else '普通'}"
        mode_render = self.small_font.render(mode_text, True, (200, 200, 100))
        mode_rect = mode_render.get_rect(center=(panel_x + panel_w // 2, mode_y))
        self.screen.blit(mode_render, mode_rect)
        
        if self.waiting_for_id_input:
            # 1. 修复变量未定义问题：直接使用上方已经完美计算好的 custom_y
            id_section_y = custom_y + button_h + 20
            
            # 2. 绘制一个深色的毛玻璃遮罩，盖住原来的控制按钮，防止文字和按钮丑陋地重叠！
            cover_rect = pygame.Rect(panel_x + 8, id_section_y - 12, panel_w - 16, panel_h - id_section_y + 10)
            self.draw_glass_rect(self.screen, cover_rect, base_color=(15, 20, 25), alpha=240, border_radius=12)
            
            # 3. 渲染标题
            id_title = self.font.render("扫描暂停 - 请选择ID", True, (255, 255, 100))
            id_title_rect = id_title.get_rect(center=(panel_x + panel_w // 2, id_section_y))
            self.screen.blit(id_title, id_title_rect)
            
            id_y = id_section_y + 25
            self.id_buttons = []
            
            id_button_w = (control_button_w - 5) // 2
            id_button_h = 22 
            
            for i in range(10):
                col = i % 2
                row = i // 2
                x = control_button_x + col * (id_button_w + 5)
                y = id_y + row * (id_button_h + 2)
                
                button_rect = pygame.Rect(x, y, id_button_w, id_button_h)
                self.id_buttons.append((str(i), button_rect))
                self.draw_apple_glass_button(self.screen, button_rect, str(i), button_id=f"id_{i}", is_active=False)
            
            id_y += 5 * (id_button_h + 2) + 5
            half_w = (control_button_w - 5) // 2
            
            no_button_rect = pygame.Rect(control_button_x, id_y, half_w, id_button_h + 5)
            self.id_buttons.append(("no", no_button_rect))
            self.draw_apple_glass_button(self.screen, no_button_rect, "无ID", button_id="id_no", is_active=True)
            
            fail_button_rect = pygame.Rect(control_button_x + half_w + 5, id_y, half_w, id_button_h + 5)
            self.id_buttons.append(("?", fail_button_rect))
            self.draw_apple_glass_button(self.screen, fail_button_rect, "识别失败(?)", button_id="id_fail", is_active=False)
            
            return

    def handle_game_click(self, pos):
        if not self.id_assignment_mode:
            return

        x, y = pos[0] // BLOCK_SIZE, pos[1] // BLOCK_SIZE
        if x < 0 or x >= MAP_COLS or y < 0 or y >= MAP_ROWS:
            return

        if not self.waiting_for_target:
            if (x, y) in self.boxes and (x, y) not in self.box_ids:
                self.temp_box_pos = (x, y)
                self.waiting_for_target = True
                print(f"已选择箱子于 ({x}, {y})，现在点击目标 (ID {self.current_id})")
        else:
            if (x, y) in self.targets and (x, y) not in self.target_ids:
                self.box_ids[self.temp_box_pos] = self.current_id
                self.target_ids[(x, y)] = self.current_id
                print(f"分配ID {self.current_id}: 箱子于 {self.temp_box_pos} -> 目标于 ({x}, {y})")
                self.current_id += 1
                self.waiting_for_target = False
                self.temp_box_pos = None
                if len(self.box_ids) == len(self.boxes) and len(self.target_ids) == len(self.targets):
                    print("所有ID已分配！按空格键求解。")

    def start_id_assignment(self):
        self.id_assignment_mode = True
        self.waiting_for_target = False
        self.current_id = 0
        self.box_ids = {}
        self.target_ids = {}
        self.temp_box_pos = None
        self.strict_mode = True
        print("ID分配模式：点击箱子，然后点击其目标")

    def cancel_id_assignment(self):
        if self.waiting_for_target:
            self.waiting_for_target = False
            self.temp_box_pos = None
            print(f"已取消。点击箱子 (ID {self.current_id})")

    def start_scan_solve(self, auto_default_ids=False):
        if self.residual_simulation_mode:
            return self.start_residual_simulation_solve("identified")
        if self.scan_solve_mode or self.awaiting_residual_review or self.recovery_session:
            return
        self._restore_manual_playback_speed()

        main_exe = os.path.join(os.path.dirname(__file__), "main.exe")
        if not os.path.exists(main_exe):
            print(f"错误：找不到main.exe")
            return

        # 启动前还原步数
        self.scan_executed_steps = 0
        self.solve_executed_steps = 0
        self.scan_planned_total = 0
        self.solve_planned_steps = 0
        self.scan_planned_turns = 0
        self.solve_planned_turns = 0
        self.scan_pause_start = 0
        self.waiting_for_id_input = False
        self.current_target_position = None
        self.pending_start_solve = False
        self.pending_finalize_scan_ids = False
        self.scan_auto_default_ids = bool(auto_default_ids)
        self.strict_mode = False
        self.scan_solve_compute_time = 0.0
        self.scan_solve_compute_started_at = None
        map_data = self.generate_wire_map()
        scope = map_scope(map_data)
        if not scope["eligible"]:
            reason = scope["reason"]
            print(
                f"当前地图不纳入扫描/性能范围（最多 {ACTIVE_MAX_BOXES} 箱、"
                f"{ACTIVE_MAX_BOMBS} 个炸弹）: {reason}"
            )
            if self.auto_solve_all:
                self._skip_auto_solve_current(reason, status="skipped_out_of_scope")
            return
        self._prepare_residual_review("identified", map_data)

        try:
            self._reset_protocol_phase_mirror()
            # UI wall-time starts before the child process and LOAD_MAP.  It
            # still pauses while the operator supplies IDs and is not a
            # low-level CPU benchmark.
            self.scan_solve_compute_started_at = time.perf_counter()
            self.main_process = subprocess.Popen(
                [main_exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, encoding=C_SUBPROCESS_ENCODING,
                errors="replace", bufsize=1,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)
            )

            def read_output():
                for line in self.main_process.stdout:
                    self.main_output_queue.put(line.strip())

            threading.Thread(target=read_output, daemon=True).start()

            print("\n" + "=" * 50)
            print("启动扫描+求解混合推演 - 底层地图快照：")
            for line in map_data:
                print(line)
            print("=" * 50 + "\n")
            self.send_to_main(f"LOAD_MAP:{('|'.join(map_data))}")
            time.sleep(0.1)
            self.process_main_responses()
            self.send_to_main("START_SCAN")
            self.scan_solve_mode = True
            self.current_scan_pause_index = 0
        except Exception as e:
            print(f"启动main.exe失败: {e}")
            if self.main_process:
                self.main_process.kill()
                self.main_process = None
            self.scan_auto_default_ids = False

    def _reset_protocol_phase_mirror(self):
        self.protocol_phase_mirror = "empty"
        self.protocol_pending_commands = []
        self.protocol_phase_mirror_active = False

    def _set_protocol_phase_mirror(self, next_phase, reason):
        previous = getattr(self, "protocol_phase_mirror", "empty")
        self.protocol_phase_mirror = next_phase
        if previous != next_phase:
            print(f"[阶段镜像] {previous} -> {next_phase} ({reason})")

    def _queue_protocol_command_mirror(self, command):
        if not hasattr(self, "protocol_pending_commands"):
            self._reset_protocol_phase_mirror()
        command_name = command.split(":", 1)[0]
        if command_name != "EXIT":
            self.protocol_phase_mirror_active = True
            self.protocol_pending_commands.append(command_name)

    def _mirror_protocol_response(self, resp_type, resp_data):
        if not hasattr(self, "protocol_pending_commands"):
            self._reset_protocol_phase_mirror()
        if not self.protocol_pending_commands:
            if self.protocol_phase_mirror_active:
                print(f"[阶段镜像] 未配对响应 RESP:{resp_type}:{resp_data}")
            return

        command = self.protocol_pending_commands.pop(0)
        phase = self.protocol_phase_mirror
        allowed = {
            "START_SCAN": {"map_ready", "solve_done"},
            "SCAN_ID": {"scanning"},
            "SET_ID_AT": {"map_ready", "ids_ready", "solve_done"},
            "FINALIZE_SCAN_IDS": {"scan_ids_pending"},
            "START_SOLVE": {
                "map_ready",
                "scan_direct_ready",
                "ids_ready",
                "solve_done",
            },
        }
        expected = allowed.get(command)
        phase_valid = expected is None or phase in expected
        if not phase_valid:
            print(
                f"[阶段镜像] 顺序异常 command={command} current={phase} "
                f"expected={','.join(sorted(expected))}"
            )
        if resp_type == 1 or not phase_valid:
            return

        if command == "LOAD_MAP" and resp_type == 0:
            self._set_protocol_phase_mirror("map_ready", command)
        elif command == "RESET" and resp_type == 0:
            self._set_protocol_phase_mirror("empty", command)
        elif command == "START_SCAN" and resp_type == 2:
            next_phase = "scanning" if "|" in resp_data else "scan_ids_pending"
            self._set_protocol_phase_mirror(next_phase, command)
        elif command == "SCAN_ID":
            if resp_type == 4 and resp_data == "no_id":
                self._set_protocol_phase_mirror("scan_direct_ready", command)
            elif resp_type == 4 and resp_data == "with_id":
                self._set_protocol_phase_mirror("scan_ids_pending", command)
        elif command == "FINALIZE_SCAN_IDS" and resp_type == 0:
            self._set_protocol_phase_mirror("ids_ready", command)
        elif command == "START_SOLVE" and resp_type == 5:
            self._set_protocol_phase_mirror("solve_done", command)
        elif command == "SET_ID_AT" and resp_type == 0 and phase == "solve_done":
            self._set_protocol_phase_mirror("map_ready", command)

    def send_to_main(self, command):
        if self.main_process and self.main_process.stdin:
            try:
                if command in ("START_SCAN", "START_SOLVE", "FINALIZE_SCAN_IDS") or command.startswith("SCAN_ID:"):
                    print(f"[扫描协议] -> {command}")
                self.main_process.stdin.write(command + "\n")
                self.main_process.stdin.flush()
                self._queue_protocol_command_mirror(command)
                if command == "START_SCAN":
                    if self.scan_solve_compute_started_at is None:
                        self.scan_solve_compute_started_at = time.perf_counter()
                elif command == "START_SOLVE" or command.startswith("SCAN_ID:"):
                    if self.scan_solve_compute_started_at is None:
                        self.scan_solve_compute_started_at = time.perf_counter()
            except Exception as e:
                print(f"发送命令失败: {e}")

    def _pause_scan_solve_compute_timer(self):
        if self.scan_solve_compute_started_at is None:
            return
        self.scan_solve_compute_time += time.perf_counter() - self.scan_solve_compute_started_at
        self.scan_solve_compute_started_at = None

    def _finish_scan_solve_compute_timer(self):
        self._pause_scan_solve_compute_timer()
        return self.scan_solve_compute_time

    def process_main_responses(self):
        while not self.main_output_queue.empty():
            response = self.main_output_queue.get()
            if not response.startswith("RESP:"):
                continue
            self._pause_scan_solve_compute_timer()
            parts = response.split(":", 2)
            if len(parts) < 3:
                continue
            resp_type = int(parts[1])
            resp_data = parts[2] if len(parts) > 2 else ""
            self._mirror_protocol_response(resp_type, resp_data)
            if resp_type == 0:  
                if resp_data == "continue":
                    print("[扫描协议] <- RESP_OK continue，继续播放当前扫描段")
                    self.is_playing = True
                    self.last_move_time = time.time()
                elif resp_data == "IDs finalized" and self.pending_start_solve:
                    print("[扫描协议] <- RESP_OK IDs finalized，开始最终求解")
                    self.pending_start_solve = False
                    self.send_to_main("START_SOLVE")
            elif resp_type == 1: 
                print(f"❌ 发生致命错误: {resp_data}")
                if self.auto_solve_all:
                    elapsed = self._finish_scan_solve_compute_timer()
                    simulated_elapsed, host_freq = get_simulated_time_with_host_freq(elapsed)
                    print(
                        f"[UI计时] 失败协议墙钟 {elapsed:.2f}秒 "
                        "(不含人工等待，不等同完整协议 CPU 门禁)"
                    )
                    self._skip_auto_solve_current(
                        resp_data or "底层错误",
                        status="error",
                        elapsed=elapsed,
                        simulated_elapsed=simulated_elapsed,
                    )
                else:
                    self.stop_scan_solve()
            elif resp_type == 2:  # RESP_SCAN_STEP
                parts = resp_data.split("|")
                self.scan_path_string = parts[0]
                
                # 动态追加新的扫描计划
                remaining_moves = len([c for c in self.scan_path_string if c in 'UDLR'])
                self.scan_planned_total = self.scan_executed_steps + remaining_moves
                
                self.scan_target_positions = []
                self.scan_target_tags = []
                for i in range(1, len(parts)):
                    coords = parts[i].split(",")
                    if len(coords) >= 2:
                        x, y = int(coords[0]), int(coords[1])
                        self.scan_target_positions.append((x, y))
                        tag = int(coords[2]) if len(coords) >= 3 else None
                        self.scan_target_tags.append(tag)

                self.current_scan_pause_index = 0
                if self.scan_solve_mode and self.scan_path_string and not self.scan_target_positions:
                    self.pending_finalize_scan_ids = True
                self.start_scan_playback()
            elif resp_type == 4:  
                if resp_data == "no_id":
                    print("[扫描协议] <- RESP_SCAN_COMPLETE no_id，直接进入最终求解")
                    self.auto_path = []
                    self.is_playing = False
                    self.is_scanning = False
                    self.strict_mode = False  
                    self.solution_origin_mode = "direct"
                    self._resolve_direct_target_boxes()
                    self.send_to_main("START_SOLVE")
                elif resp_data == "with_id":
                    print("[扫描协议] <- RESP_SCAN_COMPLETE with_id，扫描识别完成")
                    if not self.auto_assign_ids():
                        print("扫描 ID 配对失败：demo 将保留箱子和目标，不做自动消失显示。")
                    else:
                        self._resolve_identified_target_boxes()
                    self.strict_mode = True
                    # The C protocol keeps IDs pending until the explicit
                    # FINALIZE_SCAN_IDS acknowledgement.  Arm the existing
                    # playback-complete finalization path instead of sending
                    # START_SOLVE directly from scan_ids_pending.
                    self.pending_finalize_scan_ids = True
                    self.pending_start_solve = False
                    self.is_playing = True
                    self.last_move_time = time.time()
            elif resp_type == 5:  # RESP_SOLUTION
                elapsed = self._finish_scan_solve_compute_timer()
                simulated_elapsed, host_freq = get_simulated_time_with_host_freq(elapsed)
                print(f"[扫描协议] <- RESP_SOLUTION len={len(resp_data)}")
                print(
                    f"✅ 终极求解完成: 找到 {len(resp_data)} 步方案！协议墙钟 {elapsed:.2f}秒 "
                    "(至结果、不含人工等待；完整一秒门禁以子进程 CPU 为准)"
                )
                print(f"[求解路径] len={len(resp_data)} path={resp_data}")
                self.last_solve_time = elapsed
                self.last_solve_simulated_time = simulated_elapsed
                self.primary_solution_received = True
                
                # 更新求解计步器
                self.solve_planned_steps = len(resp_data)
                self.solve_executed_steps = 0
                self.solve_planned_turns = self.count_turns_in_path(resp_data)
                
                if self.auto_solve_all:
                    self._record_auto_solve_case(
                        "ok",
                        path=resp_data,
                        elapsed=elapsed,
                        simulated_elapsed=simulated_elapsed,
                    )
                self.play_solution(resp_data)
                self.stop_scan_solve(preserve_residual_review=True)
            elif resp_type == 6:  
                elapsed = self._finish_scan_solve_compute_timer()
                simulated_elapsed, host_freq = get_simulated_time_with_host_freq(elapsed)
                print(
                    f"\n❌ [终局判定] 绝对无解！算法宣告放弃。协议墙钟 {elapsed:.2f}秒 "
                    "(不含人工等待，不等同完整协议 CPU 门禁)\n"
                )
                self.last_solve_time = elapsed
                self.last_solve_simulated_time = simulated_elapsed
                self.is_playing = False
                self.is_scanning = False
                self.auto_path = []
                self.update_path_trail()
                if self.auto_solve_all:
                    self._skip_auto_solve_current(
                        "无解",
                        status="no_solution",
                        elapsed=elapsed,
                        simulated_elapsed=simulated_elapsed,
                    )
                else:
                    self.stop_scan_solve()

    def _finalize_scan_ids_after_playback(self):
        if not self.pending_finalize_scan_ids:
            return False

        self.pending_finalize_scan_ids = False
        print("[扫描协议] 扫描路径播放完毕，准备自动补全 ID 并 FINALIZE_SCAN_IDS")
        if not self.auto_assign_ids():
            print("扫描 ID 配对失败：demo 将保留箱子和目标，不做自动消失显示。")
        else:
            self._resolve_identified_target_boxes()
        self.strict_mode = True
        self.pending_start_solve = True
        self.send_to_main("FINALIZE_SCAN_IDS")
        return True

    def start_scan_playback(self):
        if not self.scan_path_string:
            return

        path = []
        for char in self.scan_path_string:
            if char in DIRECTION_MAP:
                path.append(DIRECTION_MAP[char])
            elif char == '?':
                path.append((0, 0))

        if self.is_scanning or self.scan_solve_mode:
            ok, reason = self.scan_path_is_physically_valid(path)
            if not ok:
                print(reason)
                print("[扫描提示] 保持执行 C 返回的原始扫描段，物理冲突步将被跳过。")

        self.auto_path = path
        
        self.is_playing = True
        self.is_scanning = True
        self._begin_simulation_error_route(0)
        self.scan_pause_start = 0
        self.waiting_for_id_input = False
        self.current_target_position = None
        self.show_trail = True
        self.update_path_trail()

    def scan_path_is_physically_valid(self, path):
        px, py = self.player
        boxes = set(self.boxes)
        bombs = set(self.bombs)
        grid = [row[:] for row in self.grid]

        def fmt_path(start_index):
            chars = []
            for sx, sy in path[start_index:]:
                if sx == 0 and sy == 0:
                    chars.append("?")
                elif sx == 0 and sy == -1:
                    chars.append("U")
                elif sx == 0 and sy == 1:
                    chars.append("D")
                elif sx == -1 and sy == 0:
                    chars.append("L")
                elif sx == 1 and sy == 0:
                    chars.append("R")
                else:
                    chars.append(f"({sx},{sy})")
            return "".join(chars)

        def fail(step_index, reason, nx=None, ny=None, extra=""):
            target = "" if nx is None or ny is None else f" target=({nx},{ny})"
            scan_targets = ",".join(f"({x},{y})" for x, y in self.scan_target_positions)
            return (
                False,
                "[扫描警告] C 返回的扫描段从当前物理状态预演会失败: "
                f"step={step_index + 1}/{len(path)} player=({px},{py}) "
                f"move=({path[step_index][0]},{path[step_index][1]}){target} "
                f"reason={reason}{extra} remaining='{fmt_path(step_index)}' "
                f"scan_targets=[{scan_targets}]"
            )

        for step_index, (dx, dy) in enumerate(path):
            if dx == 0 and dy == 0:
                continue

            nx, ny = px + dx, py + dy
            if not (0 <= ny < len(grid) and 0 <= nx < len(grid[ny])):
                return fail(step_index, "out_of_bounds", nx, ny)
            if grid[ny][nx] == '#':
                return fail(step_index, "wall", nx, ny)

            if (nx, ny) in boxes:
                bx, by = nx + dx, ny + dy
                if not (0 <= by < len(grid) and 0 <= bx < len(grid[by])):
                    return fail(step_index, "box_push_out_of_bounds", nx, ny, f" box_to=({bx},{by})")
                if grid[by][bx] == '#' or (bx, by) in boxes or (bx, by) in bombs:
                    return fail(step_index, "box_blocked", nx, ny, f" box_to=({bx},{by})")
                boxes.remove((nx, ny))
                if (bx, by) not in self.targets:
                    boxes.add((bx, by))
                px, py = nx, ny
                continue

            if (nx, ny) in bombs:
                bx, by = nx + dx, ny + dy
                if not (0 <= by < len(grid) and 0 <= bx < len(grid[by])):
                    return fail(step_index, "bomb_push_out_of_bounds", nx, ny, f" bomb_to=({bx},{by})")
                if (bx, by) in boxes or (bx, by) in bombs:
                    return fail(step_index, "bomb_blocked", nx, ny, f" bomb_to=({bx},{by})")
                bombs.remove((nx, ny))
                if grid[by][bx] == '#':
                    for oy in range(-1, 2):
                        for ox in range(-1, 2):
                            wx, wy = bx + ox, by + oy
                            if 0 < wy < len(grid) - 1 and 0 < wx < len(grid[wy]) - 1 and grid[wy][wx] == '#':
                                grid[wy][wx] = ' '
                else:
                    bombs.add((bx, by))
                px, py = nx, ny
                continue

            px, py = nx, ny

        return True, ""

    def handle_scan_pause(self):
        self.current_target_position = None
        if self.current_scan_pause_index < len(self.scan_target_positions):
            self.current_target_position = self.scan_target_positions[self.current_scan_pause_index]
        # 停靠步本身不移动，先收掉上一帧残留的插值，避免暂停时把刚刚那一步重播一遍。
        self.visual_offsets.clear()
        self.is_playing = False
        tag = self.scan_target_tags[self.current_scan_pause_index] if self.current_scan_pause_index < len(self.scan_target_tags) else None
        if self.current_target_position:
            print(
                f"[扫描暂停] {self.current_scan_pause_index + 1}/{len(self.scan_target_positions)} "
                f"{self._scan_waypoint_label(self.current_target_position, tag)}"
            )
        else:
            print(f"[扫描暂停] {self.current_scan_pause_index + 1}: no waypoint")
        if self.scan_auto_default_ids:
            self.scan_pause_start = 0
            self.waiting_for_id_input = True
            print(f"[一键扫描] 自动输入 ID={self.scan_auto_default_id}")
            self.handle_id_input(self.scan_auto_default_id)
            return

        self.scan_pause_start = time.time()
        self.waiting_for_id_input = True
        self.is_playing = False
        print(f"扫描暂停点 {self.current_scan_pause_index + 1}，请点击右侧按钮选择ID")

    def handle_id_input(self, id_input):
        if not self.waiting_for_id_input:
            return

        serial_numeric_input = (
            getattr(self, "serial_mode", None) == "identified"
            and getattr(self, "serial_scan_playback_started", False)
        )
        if serial_numeric_input and (
            not isinstance(id_input, str)
            or len(id_input) != 1
            or not id_input.isdigit()
        ):
            self.serial_status_text = "串口识别仅接受数字 ID 0-9"
            print(f"[串口扫描] 忽略非数字 ID: {id_input}")
            return

        waypoint_label = None
        if self.current_scan_pause_index < len(self.scan_target_positions):
            target_pos = self.scan_target_positions[self.current_scan_pause_index]
            tag = self.scan_target_tags[self.current_scan_pause_index] if self.current_scan_pause_index < len(self.scan_target_tags) else None
            waypoint_label = self._scan_waypoint_label(target_pos, tag)

            if id_input != "no":
                if id_input.isdigit() or id_input == "?":
                    val = int(id_input) if id_input.isdigit() else "?"
                    if target_pos in self.boxes:
                        self.scanned_box_ids[target_pos] = val
                        if serial_numeric_input:
                            self.box_ids[target_pos] = val
                    elif target_pos in self.targets:
                        self.scanned_target_ids[target_pos] = val
                        if serial_numeric_input:
                            self.target_ids[target_pos] = val
                    if id_input.isdigit():
                        self.strict_mode = True
                        self._resolve_identified_target_boxes()

        print(
            f"[扫描ID] pause={self.current_scan_pause_index + 1} "
            f"waypoint={waypoint_label or 'unknown'} input={id_input}"
        )

        if serial_numeric_input:
            try:
                self.serial_bridge.write(id_input + "\r\n")
            except Exception as exc:
                self.serial_busy = False
                self.serial_status_text = f"ID 发送失败: {exc}"
                print(f"[串口扫描] ID 发送失败: {exc}")
                return

            completed = (
                self.current_scan_pause_index + 1
                >= len(getattr(self, "serial_waypoints", ()))
            )
            if completed:
                if not self.auto_assign_ids():
                    print("[串口扫描] 剩余 ID 自动配对失败，保留当前显示状态")
                self._resolve_identified_target_boxes()

            self.scan_pause_start = 0
            self.waiting_for_id_input = False
            self.current_target_position = None
            self.current_scan_pause_index += 1
            self.is_playing = not completed
            self.is_scanning = not completed
            self.serial_playback_active = not completed
            self.last_move_time = time.time()
            self.serial_status_text = (
                "ID 已全部发送，等待正式求解"
                if completed else
                f"ID 已发送，继续扫描 {self.current_scan_pause_index + 1}/"
                f"{len(self.serial_waypoints)}"
            )
            if self.show_trail:
                self.update_path_trail()
            return

        self.send_to_main(f"SCAN_ID:{id_input}")
        self.scan_pause_start = 0
        self.waiting_for_id_input = False
        self.current_target_position = None
        self.current_scan_pause_index += 1

    def _advance_auto_path_step(self):
        if not self.is_playing or not self.auto_path:
            return False
        if self.simulation_error_mode or self.pending_simulated_error_source is not None:
            return False

        dx, dy = self.auto_path[0]
        if dx == 0 and dy == 0 and self.is_scanning:
            self.auto_path.pop(0)
            if getattr(self, "serial_playback_active", False):
                self._handle_serial_scan_pause()
                return True
            print(f"[PYTHON-执行器] 🛑 遇到停靠点 '?'，完美暂停执行！等待输入 ID...")
            self.handle_scan_pause()
            return True

        moved = self.move_player(dx, dy)
        if not moved:
            if self.recovery_path_active:
                failed_player = self.player
                self.auto_path = []
                self.is_playing = False
                self.is_scanning = False
                self.show_trail = False
                self.path_trail = []
                print(
                    f"[残局复查] 恢复路径在({failed_player[0]},{failed_player[1]})"
                    f"执行指令({dx},{dy})失败；已中止会话，不发布后续观测或返航完成。"
                )
                self._abort_recovery_session("恢复路径执行失败")
                return False
            self.auto_path.pop(0)
            print(
                f"[PYTHON-执行器] ⚠️ 小车在({self.player[0]},{self.player[1]})"
                f"执行指令({dx},{dy})失败，已跳过该步并继续后续路径。"
            )
            return False

        self.auto_path.pop(0)
        if self.is_playing:
            if self.is_scanning:
                self.scan_executed_steps += 1
            else:
                self.solve_executed_steps += 1

        if self.show_trail:
            self.update_path_trail()
        return True

    def play_solution(self, solution_str):
        path = [DIRECTION_MAP[char] for char in solution_str if char in DIRECTION_MAP]
        self.auto_path = path
        self.is_playing = True
        self.is_scanning = False
        self._begin_simulation_error_route(0)
        self.show_trail = True
        self.update_path_trail()

    def stop_scan_solve(self, preserve_residual_review=False):
        if self.main_process:
            self.send_to_main("EXIT")
            try:
                self.main_process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.main_process.kill()
            self.main_process = None
        self.scan_solve_mode = False
        self.pending_start_solve = False
        self.pending_finalize_scan_ids = False
        self.scan_auto_default_ids = False
        self.scan_pause_start = 0
        self.waiting_for_id_input = False
        self.current_target_position = None
        self.scan_path_string = ""
        self.scan_target_positions = []
        self.scan_target_tags = []
        self.scan_solve_compute_time = 0.0
        self.scan_solve_compute_started_at = None
        self.protocol_pending_commands = []
        self.protocol_phase_mirror_active = False
        if not preserve_residual_review:
            self.auto_path = []
            self.is_playing = False
            self.is_scanning = False
            self.show_trail = False
            self.path_trail = []
            self.pending_box_merge = None
            self.visual_offsets.clear()
            self._discard_residual_review()
        pygame.event.clear()
        print("[状态] 底层推演进程已关闭。")

    def draw_right_panel(self):
        panel_x = MAP_COLS * BLOCK_SIZE
        panel_w = self.map_panel_width
        panel_h = self.screen_h

        panel_rect = pygame.Rect(panel_x + 5, 10, panel_w - 15, panel_h - 20)
        self.draw_glass_rect(
            self.screen,
            panel_rect,
            base_color=(20, 30, 40),
            alpha=120,
            border_radius=20,
        )

        self.right_panel_view_rect = pygame.Rect(panel_x, 0, panel_w, panel_h)
        self.right_panel_scrollbar_rect = pygame.Rect(0, 0, 0, 0)
        self.right_panel_scrollbar_thumb_rect = pygame.Rect(0, 0, 0, 0)
        self.right_panel_max_scroll = 0
        self.right_panel_dragging_scrollbar = False

        title = self.font.render("地图选择", True, (255, 255, 255))
        title_rect = title.get_rect(center=(panel_x + panel_w // 2, 20))
        self.screen.blit(title, title_rect)

        catalog = self._get_map_catalog()
        # 1 箱地图仍会被加载，但不在右侧箱数筛选中显示；隐藏时优先切到 7 箱。
        box_counts = [box_count for box_count in sorted(catalog) if box_count != 1]
        selected_box_count = getattr(self, "selected_box_count", None)
        if selected_box_count not in box_counts:
            selected_box_count = 7 if 7 in box_counts else (box_counts[0] if box_counts else None)
            self.selected_box_count = selected_box_count

        content_x = panel_x + 10
        content_w = panel_w - 36
        count_title = self.small_font.render("箱子数量", True, (190, 210, 225))
        self.screen.blit(count_title, (content_x, 42))

        self.box_count_buttons = []
        count_button_y = 58
        count_button_h = 24
        count_gap = 3
        count_columns = min(4, max(1, len(box_counts)))
        count_button_w = max(1, (content_w - count_gap * (count_columns - 1)) // count_columns)
        for index, box_count in enumerate(box_counts):
            row = index // count_columns
            col = index % count_columns
            button_rect = pygame.Rect(
                content_x + col * (count_button_w + count_gap),
                count_button_y + row * (count_button_h + count_gap),
                count_button_w,
                count_button_h,
            )
            self.box_count_buttons.append((box_count, button_rect))
            self.draw_apple_glass_button(
                self.screen,
                button_rect,
                f"{box_count}箱",
                button_id=f"box_count_{box_count}",
                is_active=box_count == selected_box_count,
            )

        count_rows = max(1, (len(box_counts) + count_columns - 1) // count_columns)
        map_items = catalog.get(selected_box_count, [])
        map_header_y = count_button_y + count_rows * (count_button_h + count_gap) + 7
        map_header_text = f"{selected_box_count or 0} 箱 · {len(map_items)} 张 · 炸弹升序"
        map_header = self.small_font.render(
            fit_button_text(self.small_font, map_header_text, content_w),
            True,
            (190, 210, 225),
        )
        self.screen.blit(map_header, (content_x, map_header_y))

        button_gap = 8
        button_h = 28
        row_step = button_h + 3
        map_view_y = map_header_y + 20
        control_section_height = 254
        map_view_bottom = panel_rect.bottom - control_section_height
        map_view_rect = pygame.Rect(content_x, map_view_y, content_w, max(120, map_view_bottom - map_view_y))
        button_w = (map_view_rect.width - button_gap) // 2
        button_x_left = map_view_rect.x
        button_x_right = button_x_left + button_w + button_gap
        map_rows = max(1, (len(map_items) + 1) // 2)
        map_content_height = map_rows * row_step
        self._update_map_list_scroll_metrics(map_view_rect, map_content_height)
        map_scroll_offset = self.map_list_scroll_offset

        self.map_buttons = []
        self.control_buttons = []
        self.id_buttons = []

        self.draw_glass_rect(self.screen, map_view_rect.inflate(4, 4), base_color=(10, 16, 24), alpha=70, border_radius=10)
        old_clip = self.screen.get_clip()
        self.screen.set_clip(map_view_rect)

        if map_items:
            for i, entry in enumerate(map_items):
                row = i // 2
                col = i % 2
                button_x = button_x_left if col == 0 else button_x_right
                y_pos = map_view_rect.y + row * row_step - map_scroll_offset
                button_rect = pygame.Rect(button_x, y_pos, button_w, button_h)
                if button_rect.colliderect(map_view_rect):
                    self.map_buttons.append((entry["key"], button_rect.clip(map_view_rect)))
                    is_current = entry["key"] == self.current_map
                    self.draw_apple_glass_button(
                        self.screen,
                        button_rect,
                        fit_button_text(self.small_font, entry["label"], button_rect.width - 8),
                        button_id=f"map_{entry['key']}",
                        is_active=is_current,
                    )
        else:
            no_maps_text = self.small_font.render("该箱数暂无地图", True, (150, 150, 150))
            self.screen.blit(no_maps_text, (button_x_left, map_view_rect.y + 8))

        self.screen.set_clip(old_clip)

        track_rect = self.map_list_scrollbar_rect
        thumb_rect = self.map_list_scrollbar_thumb_rect
        if track_rect.width > 0 and track_rect.height > 0:
            track_overlay = pygame.Surface((track_rect.width, track_rect.height), pygame.SRCALPHA)
            pygame.draw.rect(track_overlay, (255, 255, 255, 32), track_overlay.get_rect(), border_radius=3)
            pygame.draw.rect(track_overlay, (255, 255, 255, 75), track_overlay.get_rect(), 1, border_radius=3)
            self.screen.blit(track_overlay, track_rect.topleft)

            if self.map_list_max_scroll > 0:
                mouse_pos = pygame.mouse.get_pos()
                thumb_color = (110, 210, 255) if self.map_list_dragging_scrollbar or thumb_rect.collidepoint(mouse_pos) else (210, 220, 235)
                thumb_alpha = 230 if self.map_list_dragging_scrollbar else 180
                thumb_overlay = pygame.Surface((thumb_rect.width, thumb_rect.height), pygame.SRCALPHA)
                pygame.draw.rect(thumb_overlay, (*thumb_color, thumb_alpha), thumb_overlay.get_rect(), border_radius=3)
                pygame.draw.rect(thumb_overlay, (255, 255, 255, 180), thumb_overlay.get_rect(), 1, border_radius=3)
                self.screen.blit(thumb_overlay, thumb_rect.topleft)

        control_y = map_view_rect.bottom + 18
        control_title = self.font.render("控制", True, (255, 255, 255))
        control_title_rect = control_title.get_rect(center=(panel_x + panel_w // 2 - 6, control_y))
        self.screen.blit(control_title, control_title_rect)

        control_button_w = map_view_rect.width
        control_button_x = map_view_rect.x
        idle_for_batch = not self.is_playing and not self.id_assignment_mode and not self.scan_solve_mode and not self.map_creation_mode and not self.map_creation_choice_mode and not self.residual_simulation_mode and not self.set_player_mode and not self.awaiting_residual_review
        serial_idle = idle_for_batch and not self.serial_busy and not self.main_process
        simulation_error_available = self._simulation_error_can_be_armed() or self.simulation_error_mode
        serial_label = self.serial_selected_port or "自动"
        controls = [
            ("serial_port", f"串口: {serial_label}", not self.serial_bridge.connected and not self.serial_busy),
            ("serial_connect", "断开串口" if self.serial_bridge.connected else "连接串口", not self.serial_busy or self.serial_bridge.connected),
            ("serial_solve_direct", "串口直接求解", serial_idle and self.serial_bridge.connected),
            ("serial_solve_identified", "串口识别求解(逐个ID)", serial_idle and self.serial_bridge.connected),
            ("auto_solve_all", "停止一键" if self.auto_solve_all and not self.auto_solve_scan_mode else "一键求解", (self.auto_solve_all and not self.auto_solve_scan_mode) or idle_for_batch),
            ("auto_scan_solve_all", "停止扫描" if self.auto_solve_all and self.auto_solve_scan_mode else "一键扫描求解", (self.auto_solve_all and self.auto_solve_scan_mode) or idle_for_batch),
            ("map_creation", "编辑地图", idle_for_batch and not self.auto_solve_all),
            ("simulate_residual", "模拟残局", idle_for_batch and not self.auto_solve_all),
            ("scan_solve", "扫描求解", idle_for_batch and not self.auto_solve_all),
            ("simulate_error", "模拟错误", simulation_error_available),
            ("toggle_mode", "切换模式", idle_for_batch and not self.auto_solve_all),
            ("assign_ids", "分配ID" if not self.id_assignment_mode else "退出ID模式", not self.is_playing and not self.scan_solve_mode and not self.auto_solve_all and not self.awaiting_residual_review),
            ("set_player", "退出设置玩家" if self.set_player_mode else "设置玩家", not self.is_playing and not self.scan_solve_mode and not self.auto_solve_all and not self.map_creation_mode and not self.awaiting_residual_review),
            ("reset_map", "重置地图", not self.is_playing and not self.auto_solve_all and not self.awaiting_residual_review),
            ("quit", "退出", True),
        ]

        if self.awaiting_residual_review and not self.waiting_for_recovery_id_input:
            controls.extend(
                [
                    (
                        "snapshot_save",
                        "保存残局快照 F5",
                        self._residual_snapshot_capture_available(),
                    ),
                    (
                        "snapshot_next",
                        "提交下一快照 F6",
                        self._residual_snapshot_submission_available(),
                    ),
                ]
            )

        if self.waiting_for_id_input or self.waiting_for_recovery_id_input:
            is_recovery_id_input = self.waiting_for_recovery_id_input
            id_section_y = control_y + 30
            cover_rect = pygame.Rect(control_button_x - 2, id_section_y - 12, control_button_w + 4, panel_rect.bottom - id_section_y)
            self.draw_glass_rect(self.screen, cover_rect, base_color=(15, 20, 25), alpha=240, border_radius=12)
            id_title_text = "残局复查 - 请选择ID" if is_recovery_id_input else "扫描暂停 - 请选择ID"
            id_title = self.font.render(id_title_text, True, (255, 255, 100))
            id_title_rect = id_title.get_rect(center=(panel_x + panel_w // 2 - 6, id_section_y))
            self.screen.blit(id_title, id_title_rect)

            id_y = id_section_y + 25
            id_button_w = (control_button_w - 5) // 2
            id_button_h = 22
            for i in range(10):
                col = i % 2
                row = i // 2
                x = control_button_x + col * (id_button_w + 5)
                y = id_y + row * (id_button_h + 2)
                button_rect = pygame.Rect(x, y, id_button_w, id_button_h)
                self.id_buttons.append((str(i), button_rect))
                self.draw_apple_glass_button(self.screen, button_rect, str(i), button_id=f"id_{i}", is_active=False)

            serial_numeric_only = (
                not is_recovery_id_input
                and self.serial_mode == "identified"
                and self.serial_scan_playback_started
            )
            if not serial_numeric_only:
                id_y += 5 * (id_button_h + 2) + 5
                half_w = (control_button_w - 5) // 2
                no_button_rect = pygame.Rect(control_button_x, id_y, half_w, id_button_h + 5)
                self.id_buttons.append(("no", no_button_rect))
                no_label = "跳过" if is_recovery_id_input else "无ID"
                self.draw_apple_glass_button(self.screen, no_button_rect, no_label, button_id="id_no", is_active=True)

                fail_button_rect = pygame.Rect(control_button_x + half_w + 5, id_y, half_w, id_button_h + 5)
                self.id_buttons.append(("?", fail_button_rect))
                fail_label = "补充识别(?)" if is_recovery_id_input else "识别失败(?)"
                self.draw_apple_glass_button(self.screen, fail_button_rect, fail_label, button_id="id_fail", is_active=False)
            return

        button_y = control_y + 30
        control_button_gap = 3
        control_columns = 2
        control_rows = (len(controls) + control_columns - 1) // control_columns
        control_column_gap = 5
        control_available_height = max(0, panel_rect.bottom - button_y - 34)
        control_button_h = max(
            20,
            min(
                button_h,
                (control_available_height - control_button_gap * (control_rows - 1)) // control_rows,
            ),
        )
        control_column_w = (control_button_w - control_column_gap) // control_columns
        for index, (action, label, enabled) in enumerate(controls):
            row = index // control_columns
            col = index % control_columns
            button_rect = pygame.Rect(
                control_button_x + col * (control_column_w + control_column_gap),
                button_y + row * (control_button_h + control_button_gap),
                control_column_w,
                control_button_h,
            )
            self.control_buttons.append((action, button_rect, enabled))

            if enabled:
                is_active = (
                    (action == "auto_solve_all" and self.auto_solve_all and not self.auto_solve_scan_mode) or
                    (action == "auto_scan_solve_all" and self.auto_solve_all and self.auto_solve_scan_mode) or
                    (action == "set_player" and self.set_player_mode) or
                    (action == "simulate_error" and self.simulation_error_mode)
                )
                button_label = fit_button_text(self.small_font, label, button_rect.width - 8)
                self.draw_apple_glass_button(self.screen, button_rect, button_label, button_id=f"btn_{action}", is_active=is_active)
            else:
                self.draw_glass_rect(self.screen, button_rect, base_color=(50, 50, 50), alpha=40, border_radius=12, blur_scale=0.15)
                label = fit_button_text(self.small_font, label, button_rect.width - 8)
                text = self.small_font.render(label, True, (100, 100, 100))
                text_rect = text.get_rect(center=button_rect.center)
                self.screen.blit(text, text_rect)

        mode_text = f"当前模式: {'严格' if self.strict_mode else '普通'}"
        if self.auto_solve_all and self.auto_solve_queue:
            prefix = "自动扫描" if self.auto_solve_scan_mode else "自动"
            mode_text = f"{prefix}: {self.auto_solve_index + 1}/{len(self.auto_solve_queue)}"
        if self.serial_bridge.connected:
            serial_state = "忙" if self.serial_busy else "就绪"
            mode_text += f" | MCU {serial_state}"
        mode_render = self.small_font.render(mode_text, True, (200, 200, 100))
        mode_y = button_y + control_rows * (control_button_h + control_button_gap) - control_button_gap + 10
        mode_rect = mode_render.get_rect(center=(panel_x + panel_w // 2 - 6, mode_y))
        self.screen.blit(mode_render, mode_rect)

    def draw_map_creation_choice_overlay(self):
        if not self.map_creation_choice_mode:
            return

        self.creation_choice_buttons = []
        overlay = pygame.Surface((self.screen_w, self.screen_h), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 145))
        self.screen.blit(overlay, (0, 0))

        dialog_w = 320
        dialog_h = 190
        dialog_rect = pygame.Rect(
            (self.screen_w - dialog_w) // 2,
            (self.screen_h - dialog_h) // 2,
            dialog_w,
            dialog_h,
        )
        self.draw_glass_rect(self.screen, dialog_rect, base_color=(18, 28, 40), alpha=235, border_radius=16)

        title = self.font.render("地图编辑", True, (255, 255, 255))
        self.screen.blit(title, title.get_rect(center=(dialog_rect.centerx, dialog_rect.y + 28)))

        hint = self.small_font.render("选择编辑来源，进入后按 S 保存", True, (190, 210, 225))
        self.screen.blit(hint, hint.get_rect(center=(dialog_rect.centerx, dialog_rect.y + 56)))

        button_w = dialog_w - 56
        button_h = 34
        current_rect = pygame.Rect(dialog_rect.x + 28, dialog_rect.y + 78, button_w, button_h)
        new_rect = pygame.Rect(dialog_rect.x + 28, dialog_rect.y + 120, button_w, button_h)

        self.creation_choice_buttons.append(("edit_current", current_rect))
        self.creation_choice_buttons.append(("edit_new", new_rect))
        self.draw_apple_glass_button(self.screen, current_rect, "编辑当前地图", button_id="edit_current_map", is_active=True)
        self.draw_apple_glass_button(self.screen, new_rect, "编辑新地图", button_id="edit_new_map", is_active=False)

        esc = self.small_font.render("ESC 取消", True, (170, 180, 190))
        self.screen.blit(esc, esc.get_rect(center=(dialog_rect.centerx, dialog_rect.bottom - 18)))

    def handle_right_panel_click(self, pos):
        if self.waiting_for_recovery_id_input:
            for id_value, button_rect in self.id_buttons:
                if button_rect.collidepoint(pos):
                    self.handle_recovery_id_input(id_value)
                    return True
            return self._right_panel_contains_point(pos)
        if self.awaiting_residual_review:
            for action, button_rect, enabled in getattr(self, "control_buttons", ()):
                if not enabled or not button_rect.collidepoint(pos):
                    continue
                if action == "simulate_error":
                    return self.toggle_simulation_error_mode()
                if action == "snapshot_save":
                    return self.save_residual_simulation_snapshot() is not None
                if action == "snapshot_next":
                    return self.submit_next_residual_snapshot()
            return self._right_panel_contains_point(pos)
        if self.waiting_for_id_input:
            for id_value, button_rect in self.id_buttons:
                if button_rect.collidepoint(pos):
                    self.handle_id_input(id_value)
                    return True
            return self._right_panel_contains_point(pos)

        if self.map_creation_choice_mode:
            return False

        for box_count, button_rect in getattr(self, "box_count_buttons", ()):
            if button_rect.collidepoint(pos):
                if box_count != getattr(self, "selected_box_count", None):
                    self.selected_box_count = box_count
                    self.map_list_scroll_offset = 0
                return True

        for map_key, button_rect in self.map_buttons:
            if button_rect.collidepoint(pos):
                if self.auto_solve_all:
                    self.cancel_auto_solve_all("已取消")
                self.current_map = map_key
                self.load_map(self.current_map)
                self.is_playing = False
                return True

        for action, button_rect, enabled in self.control_buttons:
            if button_rect.collidepoint(pos) and enabled:
                if action == "serial_port":
                    self.cycle_serial_port()
                elif action == "serial_connect":
                    self.toggle_serial_connection()
                elif action == "serial_solve_direct":
                    self.start_serial_solver("direct")
                elif action == "serial_solve_identified":
                    self.start_serial_solver("identified")
                elif action == "map_creation":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    self.start_map_creation_choice()
                elif action == "simulate_residual":
                    self.start_residual_simulation()
                elif action == "auto_solve_all":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    else:
                        self.start_auto_solve_all_maps()
                elif action == "auto_scan_solve_all":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    else:
                        self.start_auto_scan_solve_all_maps()
                elif action == "scan_solve":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    self.start_scan_solve()
                elif action == "simulate_error":
                    self.toggle_simulation_error_mode()
                elif action == "toggle_mode":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    self.strict_mode = not self.strict_mode
                    print(f"模式切换为 {'严格' if self.strict_mode else '普通'}")
                elif action == "assign_ids":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    if self.id_assignment_mode:
                        self.id_assignment_mode = False
                        self.waiting_for_target = False
                        self.temp_box_pos = None
                        print("退出ID分配模式")
                    else:
                        self.set_player_mode = False
                        self.start_id_assignment()
                elif action == "set_player":
                    self.start_set_player_mode()
                elif action == "reset_map":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消")
                    self.load_map(self.current_map)
                    print("地图已重置")
                elif action == "quit":
                    if self.auto_solve_all:
                        self.cancel_auto_solve_all("已取消", restore_current_map=False)
                    return "quit"
                return True

        return self._right_panel_contains_point(pos)

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if not self.map_creation_choice_mode and not self.residual_simulation_mode and not self.map_creation_mode and not self.id_assignment_mode and not self.set_player_mode and not self.simulation_error_mode:
                        if event.key == pygame.K_s:
                            if getattr(self, 's_key_down_time', 0.0) == 0.0:
                                self.s_key_down_time = time.time()
                                print("长按 S 5秒可批量导出当前箱子数的地图...")
                    
                    if self.map_creation_choice_mode:
                        if event.key == pygame.K_ESCAPE:
                            self.cancel_map_creation_choice()
                    elif self.residual_simulation_mode:
                        self.handle_residual_simulation_key(event.key)
                    elif self.map_creation_mode:
                        if event.key == pygame.K_ESCAPE:
                            self.exit_map_creation()
                        elif event.key == pygame.K_1:
                            self.current_brush = '#'
                        elif event.key == pygame.K_2:
                            self.current_brush = '$'
                        elif event.key == pygame.K_3:
                            self.current_brush = '.'
                        elif event.key == pygame.K_4:
                            self.current_brush = '@'
                        elif event.key == pygame.K_5:
                            self.current_brush = 'B'
                        elif event.key == pygame.K_0:
                            self.current_brush = ' '
                        elif event.key == pygame.K_s:
                            self.export_creation_map(save_as_new=False)
                        elif event.key == pygame.K_z:
                            self.export_creation_map(save_as_new=True)
                        elif event.key == pygame.K_c:
                            self.clear_creation_map()
                    elif event.key == pygame.K_e and self._simulation_error_can_be_armed():
                        self.toggle_simulation_error_mode()
                    elif event.key == pygame.K_F5 and self._residual_snapshot_capture_available():
                        self.save_residual_simulation_snapshot()
                    elif event.key == pygame.K_F6 and self._residual_snapshot_submission_available():
                        self.submit_next_residual_snapshot()
                    elif event.key == pygame.K_ESCAPE:
                        if self.auto_solve_all:
                            self.cancel_auto_solve_all("已取消")
                        elif self.set_player_mode:
                            self.set_player_mode = False
                            print("退出设置玩家模式")
                        elif self.simulation_error_mode:
                            self.simulation_error_mode = False
                            self.pending_simulated_error_source = None
                            print("已取消模拟错误选择")
                        elif self.id_assignment_mode:
                            if self.waiting_for_target:
                                self.cancel_id_assignment()
                            else:
                                self.id_assignment_mode = False
                                print("退出ID分配模式")
                        elif self.scan_solve_mode:
                            self.stop_scan_solve()
                    elif not self.is_playing and not self.id_assignment_mode and not self.scan_solve_mode and not self.set_player_mode and not self.map_creation_choice_mode and not self.awaiting_residual_review and not self.recovery_session:
                        if event.key == pygame.K_SPACE:
                            self.solve()
                            
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_s:
                        self.s_key_down_time = 0.0

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    is_double_click = getattr(event, "clicks", 1) >= 2
                    if self.map_creation_choice_mode:
                        handled_choice = False
                        for action, button_rect in self.creation_choice_buttons:
                            if button_rect.collidepoint(event.pos):
                                self.start_map_creation(edit_current=(action == "edit_current"))
                                handled_choice = True
                                break
                        if not handled_choice:
                            self.cancel_map_creation_choice()
                    elif self.residual_simulation_mode:
                        if event.pos[0] >= MAP_COLS * BLOCK_SIZE:
                            self.handle_residual_simulation_panel_click(event.pos)
                        elif event.pos[1] < MAP_ROWS * BLOCK_SIZE:
                            self.handle_tile_click(event.pos, force_copy=is_double_click)
                            self.dragging = True
                            self.handle_residual_simulation_click(event.pos)
                    elif self.map_creation_mode:
                        self.handle_tile_click(event.pos, force_copy=is_double_click)
                        self.dragging = True
                        self.handle_creation_click(event.pos)
                    elif event.pos[0] >= MAP_COLS * BLOCK_SIZE:
                        if self.handle_map_list_mouse_down(event.pos):
                            result = True
                        elif self.handle_right_panel_mouse_down(event.pos):
                            result = True
                        else:
                            result = self.handle_right_panel_click(event.pos)
                        if result == "quit":
                            running = False
                    elif event.pos[1] < MAP_ROWS * BLOCK_SIZE:
                        if self.simulation_error_mode:
                            self.handle_simulated_error_click(event.pos)
                        elif self.set_player_mode:
                            self.set_player_at_click(event.pos)
                        else:
                            self.handle_tile_click(event.pos, force_copy=is_double_click)
                        if self.id_assignment_mode:
                            self.handle_game_click(event.pos)
                    elif self.id_assignment_mode:
                        self.handle_game_click(event.pos)

                if event.type == pygame.MOUSEBUTTONDOWN and event.button in (4, 5):
                    if self.map_creation_choice_mode or self.residual_simulation_mode:
                        continue
                    wheel_delta = 1 if event.button == 4 else -1
                    if not self.handle_map_list_scroll(event.pos, wheel_delta):
                        self.handle_right_panel_scroll(event.pos, wheel_delta)

                if event.type == pygame.MOUSEWHEEL:
                    if self.map_creation_choice_mode or self.residual_simulation_mode:
                        continue
                    mouse_pos = pygame.mouse.get_pos()
                    if not self.handle_map_list_scroll(mouse_pos, event.y):
                        self.handle_right_panel_scroll(mouse_pos, event.y)

                if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    self.stop_all_right_panel_drag()
                    if self.map_creation_mode or self.residual_simulation_mode:
                        self.dragging = False

                if event.type == pygame.MOUSEMOTION:
                    if self.map_creation_choice_mode:
                        continue
                    if self.residual_simulation_mode:
                        if self.dragging:
                            self.update_selected_tile_coord(event.pos)
                            self.handle_residual_simulation_click(event.pos)
                        continue
                    if self.map_list_dragging_scrollbar:
                        self.handle_map_list_drag(event.pos[1])
                    if self.right_panel_dragging_scrollbar:
                        self.handle_right_panel_drag(event.pos[1])
                    if self.map_creation_mode and self.dragging:
                        self.update_selected_tile_coord(event.pos)
                        self.handle_creation_click(event.pos)

            self.update_visual_transitions()
            self._advance_residual_capture()
            self.handle_manual_movement()
            current_time = time.time()
            
            if getattr(self, 's_key_down_time', 0.0) > 0.0 and (current_time - self.s_key_down_time) >= 5.0:
                self.s_key_down_time = 0.0
                self.export_maps_to_othermap()

            if self.scan_solve_mode and self.scan_pause_start > 0 and not self.waiting_for_id_input:
                if current_time - self.scan_pause_start >= self.scan_focus_duration:
                    self.scan_pause_start = 0
                    self.waiting_for_id_input = True
                    print(f"Scan lock {self.current_scan_pause_index + 1} ready: choose ID on the right panel.")

            if self.is_playing and self.auto_path:
                move_delay = getattr(self, "move_delay", 0.1)
                if current_time - self.last_move_time > move_delay:
                    self._advance_auto_path_step()
            elif self.is_playing and not self.auto_path:
                self.is_playing = False
                self.is_scanning = False
                self.show_trail = False
                self.path_trail = []
                if self.recovery_path_active:
                    self._finish_recovery_path_playback()
                elif self._finalize_scan_ids_after_playback():
                    pass
                elif self.auto_solve_all:
                    self._advance_auto_solve_after_playback()
                else:
                    self._simulation_error_round_finished()
                    self._queue_residual_review_after_primary_solution()

            if self.scan_solve_mode:
                self.process_main_responses()

            self.poll_serial_bridge()
            self.draw()
            self.clock.tick(120)

        if getattr(self, "serial_bridge", None) is not None:
            self.serial_bridge.close()
        pygame.quit()

def main():
    print("=" * 60)
    print("推箱子求解器 - C算法 + Python可视化")
    print("=" * 60)

    if not ensure_c_solver_built():
        print("\nC 求解器自动编译失败，demo 未启动")
        return 1

    lib = load_c_library()
    if not lib:
        print("\n未找到C库（请先编译）")
        return 1

    if not HAS_PYGAME:
        print("\nGUI已禁用（请安装pygame）")
        return 1

    try:
        demo = GameDemo(lib)
        demo.run()
    except Exception as e:
        print(f"致命错误崩溃: {e}")
        return 1

    return 0

if __name__ == "__main__":
    if relaunch_detached_from_ide_if_needed():
        sys.exit(0)
    sys.exit(main())
