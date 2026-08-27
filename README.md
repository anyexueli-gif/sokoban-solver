# 16×12 Sokoban Solver

[![Build and smoke test](https://github.com/anyexueli-gif/sokoban-solver/actions/workflows/build-smoke.yml/badge.svg)](https://github.com/anyexueli-gif/sokoban-solver/actions/workflows/build-smoke.yml)

<p align="right">中文 · <a href="README_EN.md">English</a></p>

这是用于第21届全国大学生智能汽车竞赛智能视觉组的推箱子核心求解器的项目，可以直接移植到RT1064。作为一名新手，我在开发过程中使用了大量 AI 工具参与方案设计、代码实现和问题排查，因此项目中难免存在一些代码结构不够优雅的“屎山”、潜在漏洞以及尚未完善的地方。项目仍会持续改进，欢迎大家在使用和阅读过程中提出意见、报告问题或贡献修改，也希望大家多一些理解与包容。

<a id="中文"></a>

## 中文

### 项目特点

- 16×12 位图地图，支持最多 10 个箱子、10 个目标和 5 个炸弹。
- A* 导航与箱子路径规划，支持多箱、多目标场景。
- 支持炸弹推动、爆炸和可破坏墙体的动态状态推进。
- 支持普通目标匹配，以及箱子/目标 ID 严格匹配。
- 提供扫描路径、识别点 ID 录入和残局恢复接口。
- `solver_solve_robust()` 会在返回结果前校验路径回放。

求解器以生成可执行路径为目标，不承诺全局最短路径。地图尺寸和容量定义在 [`Driver/sokoban_types.h`](Driver/sokoban_types.h)。

> **容量与运行边界：** 当前核心和 PC 演示配置最多支持 10 个箱子、10 个目标和 5 个炸弹；是否能在给定平台和时间内完成求解，还取决于地图结构与可用内存。PC 演示是主机侧运行环境，不等同于 RT1064 板上的性能证明。

### 目录结构

| 路径 | 说明 |
| --- | --- |
| [`Driver/`](Driver/) | 可移植核心：A*、状态推进、炸弹规则、扫描、恢复和缓存接口 |
| [`Driver/rt1064_porting_example.c`](Driver/rt1064_porting_example.c) | RT1064 硬件移植例程；展示扫描、识别、求解和恢复接口的调用顺序，不参与 PC 默认构建 |
| [`User/`](User/) | Windows 构建脚本、协议入口、Pygame 演示和 PC Flash 适配 |
| [`map/`](map/) | 示例地图和自定义地图 |
| [`map_eligibility.py`](map_eligibility.py) | 演示端地图容量筛选规则 |

### 快速开始（Windows）

环境要求：PowerShell、GCC（`gcc` 位于 `PATH` 中）和 Python 3。

```powershell
# 安装已验证的演示依赖（含 Pygame，以及可选的串口/主机信息支持）
python -m pip install -r .\requirements-demo.txt

# 构建
powershell -NoProfile -ExecutionPolicy Bypass -File .\User\build_fast.ps1 --profile mcu-fast

# 启动 Pygame 演示
python .\User\demo.py
```

`requirements-demo.txt` 固定了本仓库验证过的 GUI 依赖，并同时列出可选的串口和主机信息依赖。

也可以运行 `User/start_demo.ps1`。如果只需要协议入口，构建后运行：

```powershell
.\User\main.exe
```

构建生成的 `main.exe`、`sokoban_solver.dll` 和中间文件仅用于本地运行，已被 Git 忽略。

首次启动 `User/demo.py` 会自动调用 `User/build_fast.ps1` 做增量构建。Flash 扫描缓存默认关闭（`g_sokoban_flash_cache_enabled = 0`）；只有在上层移植工程显式开启后，PC 适配器才会创建被忽略的 `FLASH/scan_cache.bin`。演示中的地图编辑和常规保存会写入 `map/`；“导出为原始 12×16 格式”功能会按需写入被忽略的 `othermap/`。程序也可能为可解析但文件名不以 `map` 开头的地图补充编号文件名，使用前请保留自己的备份。

`Driver/rt1064_porting_example.c` 是独立的移植参考例程，不是可直接运行的 PC 程序，也不会被上述构建脚本编译。例程中的 `map1`、`map2`、摄像头识别和电机控制函数需要由具体 RT1064 工程提供；它用于说明如何串联扫描、正式求解和残局恢复接口。

### 本地验证与 CI

完成构建后，可运行 `python -m py_compile User/demo.py map_eligibility.py` 进行 Python 语法检查，并向 `User/main.exe` 输入 `WARMUP`、`EXIT` 验证协议入口。GitHub Actions 会在推送和拉取请求时执行同等的 Python 检查、GCC 构建和协议冒烟测试，配置见 [`.github/workflows/build-smoke.yml`](.github/workflows/build-smoke.yml)。

### 地图格式

`map/` 下的 `.txt` 文件支持 JSON 字符串数组或逐行纯文本。规范化地图为 16 行、每行 12 个 ASCII 字符；空格是有效地面格，请保留行尾空格。

部分历史源地图采用 12 行、每行 16 列的格式（例如 `map_2_b8_a1.txt`）。这类源地图使用 `-` 表示地面、`*` 表示炸弹，演示端加载时会逆时针旋转为 16×12 布局；源文件没有 `@` 时，演示端会在默认空位或第一个可用空位补入玩家起点。

| 字符 | 含义 |
| --- | --- |
| `#` | 墙 |
| 空格 | 地面 |
| `@` | 玩家起点 |
| `$` | 未指定 ID 的箱子 |
| `.` | 未指定 ID 的目标 |
| `*` | 箱子位于目标 |
| `B` | 炸弹 |
| `0`–`9` | 箱子 ID（严格模式） |
| `a`–`j` | 目标 ID 0–9（严格模式） |

传给 C API 的地图字符串格式为 `row1|row2|...|row16`。

### C API 最小示例

```c
#include "sokoban_solver.h"

SokobanSolver *solver = solver_create();
const char *map = "row1|row2|...|row16";

if (solver && solver_load_map_from_string(solver, map) &&
    solver_solve_robust(solver)) {
    uint16_t length = 0;
    Direction *path = solver_get_solution(solver, &length);
    /* path[0..length) 为方向步。 */
}

solver_destroy(solver);
```

扫描识别可使用 `solver_generate_scan_path()`、`solver_assign_next_scan_id()` 和 `solver_set_identified_solve_mode()`；残局观测恢复可使用 `sokoban_recovery_*()` 接口。

### PC 协议入口

`User/pc_main.c` 从标准输入读取一行一条命令，并输出 `RESP:<类型>:<内容>`。

| 命令 | 作用 |
| --- | --- |
| `LOAD_MAP:<地图字符串>` | 加载地图 |
| `WARMUP` | 初始化求解器查找表和运行时缓存 |
| `FLASH_CLEAR` | 清除 Flash 扫描缓存（需先启用 Flash 缓存并提供适配器） |
| `START_SOLVE` | 直接求解 |
| `START_SCAN` | 生成扫描路径 |
| `SCAN_ID:<0-9\|no\|?>` | 提交扫描点识别结果 |
| `SET_ID_AT:x,y,id` | 设置坐标处实体 ID |
| `FINALIZE_SCAN_IDS` | 完成 ID 分配并进入严格模式 |
| `RESET` / `EXIT` | 重置或退出 |

### MCU 串口（可选）

演示默认使用 115200 波特率，可通过环境变量预选端口：

```powershell
$env:SOKOBAN_SERIAL_PORT = "COM3"
python .\User\demo.py
```

### 移植注意事项

- `Driver/` 使用共享静态工作区，默认按单实例、串行方式调用；多任务环境请在上层加锁。
- MCU 工程需要为 `Driver` 中的自定义 BSS 段配置链接脚本，并提供目标平台的 Flash 适配。
- `Driver` 的 C/H 文件保留原始 CP936 编码；编辑或批量格式化前请确认工具链不会擅自转码。
- 方向步使用 `U`、`D`、`L`、`R`；扫描路径还可能包含识别点暂停标记。
- `User/build_fast.ps1 --force-close` 仅应在确认项目进程占用输出文件时使用；该选项会强制结束匹配的本地进程，请勿在有未保存工作的情况下使用。

### 许可证

本项目按 [MIT License](LICENSE) 开源。使用、复制、修改和分发时，请保留其中的版权与许可声明。
