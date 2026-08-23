# BK7258 openvela 移植调试全记录（新手可读版）

> 目标：把 openvela（小米开源 AIoT OS，基于 NuttX）移植到博通集成 **BK7258 DevKit**
> （AI 玩具开发板，双 GC9D01 圆屏），参加"2026 openvela AI 硬件开发者大赛 · 新硬件平台适配赛道"。
>
> 本文记录从 0 到"撞墙"的完整调试过程：**用了什么方法、什么命令、为什么用、打印了什么、得到什么结论**。
> 面向新手，尽量把每一步的来龙去脉写清楚，方便回溯与答疑。

---

## 0. 关键背景与硬件事实

| 项目 | 结论 | 怎么知道的 |
| ---- | ---- | ---- |
| 芯片 | BK7258，双核 **Armv8-M STAR-MC1**（Cortex-M33 兼容，带 FPU），最高 480MHz | Datasheet |
| 核 | **CP 核(CPU1)=主核**（跑 WiFi/BT/RF），**AP 核(CPU0)=应用核** | 逆向 + 日志 |
| 安全 | 带 **TrustZone**（安全/非安全，地址差 0x10000000） | reg_base.h |
| 内存 | 640KB 共享 SRAM(0x28000000) + 8/16MB PSRAM(0x60000000) + Flash XIP(0x02000000) | Datasheet + SDK |
| 串口 | 板载 CH340 = **UART0**（既是 bootROM 下载口，也是 CP 核日志口），115200 | 实测 bk_loader + 日志 |
| 屏幕 | 双 SPI LCD **GC9D01 160x160** | ARMINO 文档 |

**最重要的一条**：BK7258 是"待适配"板 —— openvela/NuttX 内核里**原本没有** BK7258 的芯片支持，一切要从零建。

---

## 1. 环境与工具

- 源码：`/home/zhangyan68/miwear-main/vendor/openvela`（openvela 全量，含 nuttx 内核）
- ARMINO SDK（博通官方，作模板）：`/home/zhangyan68/miwear-main/vendor/armino/bk_avdk_smp`
- 交叉工具链：`/home/zhangyan68/openvela/gcc-arm-none-eabi-10.3-2021.10`（ARM 官方 10.3，ARMINO 指定版本）
  以及 openvela 自带 `prebuilts/gcc/.../arm-none-eabi`（GCC 13.4，编 NuttX 用）
- 烧录器：`/home/zhangyan68/openvela/BEKEN_BKFIL_V2.1.11.8_20240509/bk_loader`（Linux 版 BKFIL）
- 串口工具：picocom / 直接 `cat /dev/ttyUSB0`

---

## 2. 调试方法论（贯穿全程的套路）

1. **先读官方文档**：搞清赛道、流程、支持的硬件、镜像格式。
2. **拿"标准答案"当模板**：编译 ARMINO 官方例程 `spi_lcd_example`，它编出的 `all-app.bin` 就是
   合法可启动镜像的模板；我们照它的格式/布局做。
3. **逆向 + 对照**：用 `readelf`/`nm`/`objdump`/`xxd` 分析二进制，确定入口、向量表、内存布局、镜像格式。
4. **分里程碑（M0→M4）**：脚手架 → 芯片起步 → 串口 NSH → 点屏 → UI。
5. **串口日志是唯一真相**：一切"是否启动/走到哪一步"都靠串口打印判断；打不出就加"生命体征"标记。
6. **一次只改一个变量**：每次只动一处，重编译→重打包→烧录→看串口，逐个排除。

---

## 3. 第一阶段：拉代码 + 编译模拟器（验证环境）

**为什么**：先确认 openvela 源码能编、能跑，再碰硬件。

```bash
# 官方 repo 工具拉取（分支 dev-ai-contest-2026）
repo init -u https://github.com/open-vela/manifests.git -b dev-ai-contest-2026 -m openvela.xml \
  --repo-url=https://mirrors.tuna.tsinghua.edu.cn/git/git-repo/ --git-lfs
repo sync -c -j8

# 编译并运行 goldfish 模拟器（验证工具链）
./build.sh vendor/openvela/boards/vela/configs/goldfish-arm64-v8a-ap/ --cmake -j$(nproc)
./emulator.sh cmake_out/vela_goldfish-arm64-v8a-ap/
```

**结果**：模拟器起来了（黑屏 = 正常，没 app 画屏），**证明编译环境 OK**。
（注意：模拟器用 `adb`，但真机 BK7258 是 RTOS MCU，**不走 adb，只走 UART 串口**——这是新手常见误区。）

---

## 4. 第二阶段：搭 BSP 脚手架（M0）

BK7258 在 `nuttx/arch/arm/src/` 下**没有芯片目录**，`vendor/beken` 是空壳。参照已适配板
`vendor/bes/boards/best1700_ep`（同为 Armv8-M）和 `vendor/espressif/boards/esp32s3` 建骨架：

```
vendor/beken/boards/bk7258/bk7258-devkit/   # 板级
  README_zh-cn.md / Kconfig / configs/nsh/defconfig / include/board.h
  src/{bk7258_bringup.c, bk7258_appinit.c, bk7258_boardinitialize.c, CMakeLists.txt, Make.defs, scripts/ld.script}
nuttx/arch/arm/src/bk7258/                   # 芯片层
  hardware/{bk7258_uart.h, bk7258_memorymap.h}
  bk7258_{start,lowputc,irq,timerisr,allocateheap,serial}.c / chip.h / Make.defs / CMakeLists.txt / Kconfig
nuttx/arch/arm/include/bk7258/{irq.h, chip.h}
```

---

## 5. 第三阶段：拿手册和 SDK，抽取寄存器/内存映射

**为什么**：写驱动必须要真实的寄存器地址，不能瞎猜。

- **Datasheet**（`docs/BK7258 Datasheet.pdf`）用 `pdftotext` 提取：
  ```bash
  pdftotext -layout "BK7258 Datasheet.pdf" /tmp/bk7258_ds.txt
  grep -niE "cortex|armv8|SRAM|PSRAM|UART|MHz" /tmp/bk7258_ds.txt
  ```
  得到：STAR-MC1 双核@480MHz、26MHz 晶振、640KB SRAM、3×UART。
  **但 datasheet 没有寄存器基址表**（在 TRM 或 SDK 里）。

- **ARMINO SDK** 提供了寄存器基址和内存映射（关键文件）：
  ```
  ap/include/soc/bk7258/reg_base.h          # 外设基址(UART0=0x44820000 等)
  ap/middleware/soc/bk7258_ap/soc/uart_struct.h  # UART 寄存器布局(config@0x10 等)
  ap/middleware/soc/bk7258_ap/hal/uart_ll.h      # 收发/波特率逻辑
  ap/middleware/soc/bk7258_ap/bk7258_ap_bsp.ld   # AP app 内存布局
  ```
  抽出：UART0=0x44820000/UART1=0x45830000；fifo_status@0x18(wr_ready=bit20)；
  fifo_port@0x1c(写发/读收)；`clk_div = 26MHz/波特率 - 1`（115200→225）。

---

## 6. 第四阶段：写芯片层 + 首次编译（逐个清错）

**方法**：先写最小串口输出 `arm_lowputc`（轮询 fifo_wr_ready 后写 fifo_port），再补
启动/中断/时钟/堆。然后 `./build.sh ... --cmake` 编译，把**报错当路标**逐个解决。

编译踩过的坑（每个都是真实报错→定位→修复）：

| 报错 | 原因 | 修复 |
| ---- | ---- | ---- |
| `No CMakeLists.txt at .../bk7258-devkit` | 板根缺 CMakeLists | 建板根 `CMakeLists.txt`（`add_subdirectory(src)`）|
| 找不到板级目录 | `CONFIG_ARCH_BOARD_CUSTOM_DIR` 相对 nuttx/ 解析 | 路径加前缀 `../` |
| `arch/chip/irq.h 找不到` | 缺 arch 暴露头 | 建 `arch/arm/include/bk7258/{irq.h,chip.h}`（64 个外部中断，UART0=IRQ20）|
| `ARMV8M_PERIPHERAL_INTERRUPTS undeclared` | 向量表需要中断数 | src `chip.h` 定义 `=BK7258_IRQ_NEXTINT` |
| 链接到最后缺一堆符号 | 芯片层函数没实现 | 补 `up_irqinitialize/up_timer_initialize/g_idle_topstack/arm_serialinit/up_putc` 等 |

**关键命令**（看符号地址、确认布局）：
```bash
arm-none-eabi-nm cmake_out/.../nuttx | grep -E " _vectors| __start"   # 看向量表/入口地址
arm-none-eabi-readelf -l app.elf    # 看程序段的加载地址(LMA/VMA)
xxd cmake_out/.../nuttx.bin | head   # 看镜像头(向量表=SP+reset)
```

**进展**：所有源文件编译通过、链接生成 `nuttx.bin`（约 140KB），符号地址正确。

---

## 7. 第五阶段：逆向 AP app 镜像格式 + 改链接脚本

**为什么**：要让 bootloader 能加载我们的镜像，必须搞清它期望的格式。

用 `readelf -l` 看 ARMINO 的 `bk7258_ap/app.elf`、`xxd` 看 `app1.bin` 头：
```bash
arm-none-eabi-readelf -l bk7258_ap/app.elf   # 入口 0x216771d, LOAD 段 FLASH@0x02150000
xxd package/tmp/app1.bin | head              # 头部: SP=0x28064000, reset=0x0216771d
```
**结论（AP app 格式）**：
- **XIP 从 flash 运行**：物理分区 0x165000 经 flash 控制器 34/32 CRC 解码 → 映射到 XIP 虚拟地址 **0x02150000**；
- `app1.bin` 开头**直接是向量表**（SP + reset），**无额外镜像头**；bootloader 取 SP+reset 跳转。

据此把 NuttX 链接脚本从"跑 SRAM"改成**标准 flash 启动布局**：
```
FLASH(rx)  ORIGIN=0x02150000 LEN=0x110000   # .vectors + .text + .rodata + .data(LMA)
RAM(rwx)   ORIGIN=0x28010000 LEN=0x54000    # .data(VMA) + .bss + heap（AP 专属区，勿踩 CP）
```
**踩坑**：`--build-id` 生成的 `.note.gnu.build-id` 抢占了 flash 起始 0x02150000，把向量表挤到 0x02150200。
用 `/DISCARD/ : { *(.note.gnu.build-id) }` 丢弃，向量表回到 0x02150000。
用 `nm` 验证 `_vectors@0x02150000` ✓。

---

## 8. 第六阶段：打包成 AP app 段（含 CRC）

**为什么**：raw `nuttx.bin` 不能直接烧，要按 Beken 格式打包（每 32 字节加 2 字节 CRC）并和
bootloader+CP app 拼成 `all-app.bin`。

先编 ARMINO 例程拿到打包产物和配置：
```bash
# ARMINO 原生编译(绕过 Docker, 用 ARM 官方10.3工具链)
export COMPILER_TOOLCHAIN_PATH=/home/zhangyan68/openvela/gcc-arm-none-eabi-10.3-2021.10/bin
# (并把 ap/cp 的 soc_config.mk 里 /opt/gcc... 改成该路径)
make bk7258 PROJECT=spi_lcd_example
```
产物：`build/bk7258/spi_lcd_example/package/{all-app.bin, tmp/{bootloader.bin,app.bin(CP),app1.bin(AP)}}`
配置：`partitions/bk_package.json`（3 段：bootloader@0x0 / app(CP)@0x11000 / app1(AP)@0x165000，crc_enable=true）

复用 ARMINO 的打包器，用我们的 nuttx.bin 替换 app1 重新打包（脚本 `/tmp/bk_repack/repack.py`）：
```python
sys.path.insert(0, ".../tools/env_tools/bk_py_libs")
from bk_packager.bk_packager_linear_crc import bk_packager_linear_crc
bk_packager_linear_crc(workdir, "bk_package.json", "all-app-nuttx.bin").pack()
```
**验证**：`xxd -s 0x165000 all-app-nuttx.bin` 看到 AP 区正是我们的向量表，且每 32 字节后有 2 字节 CRC ✓。

---

## 9. 第七阶段：打通烧录链路（bk_loader）

**踩过的坑（新手必看）**：

1. **`/dev/ttyUSB0` 不出现**：Ubuntu 的 `brltty` 服务霸占了 CH340（udev 规则匹配 `1a86:7523`）。
   ```bash
   sudo apt remove -y brltty         # 或屏蔽 /etc/udev/rules.d/85-brltty.rules
   # 拔插 USB 后 ls -l /dev/ttyUSB* 就出现了
   ```
2. **端口号会变（ttyUSB0↔ttyUSB1）**：反复复位/拔插导致 USB 重新枚举、节点号递增。
   **只按 RST、别拔插 USB** 就稳定。命令里用 `N=$(ls /dev/ttyUSB*|grep -oE '[0-9]+$'|head -1)` 自动取号。
3. **进下载模式的时序**（最关键）：bk_loader 显示 `Getting Bus...` 时**点按 1-2 次 RST** 拿总线；
   **一旦出现 `Erasing/Writing %` 就彻底松手别碰**（继续按会复位芯片、传输从头再来）。
4. **bk_loader 即使失败也返回 0**：循环烧录不能用 `&& break`，要用 `grep "All Finished Successfully"` 判断成功。
5. **`read -f` 的输出路径**：相对 bk_loader 所在目录，且自动加 `_dump_时间_地址.bin` 后缀。

烧录命令：
```bash
./bk_loader download -p 0 -b 1500000 -i all-app-nuttx.bin   # 1.5Mbaud
# 先备份出厂固件(安全网):
./bk_loader read -p 0 -b 1500000 -f "factory@0-800000"
```
**进展**：先烧 ARMINO `spi_lcd_example` 验证 → 板子重启、串口出日志、双核 ap0/ap1、shell `$`，
**整条烧录链路验证通过**。出厂"眼睛"demo 已备份（`x1_dump_..._0x800000.bin`，8MB，可刷回）。

---

## 10. 第八阶段：启动调试（一系列"生命体征"排查）

烧我们的 NuttX 后串口只有 bootloader/CP 日志、**没有 NuttX 输出**。逐个假设排查：

### 10.1 加"生命体征"标记
在 `__start` 里每步打印 `A/B/C/D/E`（后改成可 grep 的 `<NX:1-uart>`…`<NX:7-nx_start>`），
用来判断启动走到哪一步。**方法：打不出就是没走到，能打出就往后推。**

### 10.2 MSPLIM（栈限制寄存器）—— 一个关键修复
**现象**：完全无输出。**分析**：bootloader 是"跳转"进 AP app（非真复位），
**MSPLIM 残留 bootloader 旧值**，NuttX 一压栈就触发栈限制违例 → 立即 fault → 无输出。
（对照 ARMINO `Reset_Handler_Cpu0` 开头就 `__set_MSPLIM(...)`。）
**修复**：把 `__start` 改成 naked 汇编，显式**从 vector[0] 设 SP + 清 MSPLIM=0**再进 C：
```asm
cpsid i
ldr r0,=_vectors ; ldr r1,[r0] ; mov sp,r1   ; 设 SP
movs r0,#0 ; msr MSPLIM,r0                     ; 清栈限制
b bk7258_cstart
```
**进展**：这次真机看到了启动标记（一度"跑到 E"）——**证明 AP 核能跑我们的代码到 nx_start**。

### 10.3 无 NSH → 缺 /dev/console
`arm_serialinit` 曾是空桩，没注册 `/dev/console`，NSH 打不开控制台。
**修复**：实现完整 UART 驱动 `bk7258_serial.c`（uart_lowerhalf，RX/TX 中断，注册 /dev/console+/dev/ttyS0），
defconfig 开 `CONFIG_STANDARD_SERIAL`。

### 10.4 看门狗复位循环（曾怀疑）
**现象**：日志里 bootloader 序列重复出现（`ef:I(4)` 多次）≈ 每 ~100ms 复位。
**分析**：怀疑 bootloader 启用了看门狗、要求 app 及时喂狗。
**尝试**：照 ARMINO `wdt_hal_close()` 在启动最早期关 AON WDT + 普通 WDT：
```c
putreg32(0x5a0000, 0x44000600); putreg32(0xa50000, 0x44000600);   // AON WDT
*(0x44800008)|=(1<<1); putreg32(0x5a0000,0x44800010); putreg32(0xa50000,0x44800010); // WDT
```
**结果**：循环停了（不再反复复位），但**仍无 NuttX 输出**——说明看门狗不是根因。

### 10.5 决定性测试：AP 核到底跑没跑？
在 naked `__start` 最开头（设完 SP/MSPLIM 后）**直接往 UART0(0x4482001c) 写 `@`、UART1(0x4583001c) 写 `#`**，
不做任何 setup（bootloader 已配好 UART），绕过一切 C 代码/驱动。
```bash
# 读日志文件(与主机同一 /tmp), 数 @ 和 #
grep -ao "@" /tmp/nuttx_boot2.log | wc -l   # 结果: 0
grep -ao "#" /tmp/nuttx_boot2.log | wc -l   # 结果: 0
```
**结论**：`@`/`#` 全 0 → **AP 核连第一条指令都没执行**。日志停在 `cal:E(107)` 静默、不循环。

### 10.6 定位卡点归属
查关键字符串在哪个核：
```bash
grep -ac "load polar tab" package/tmp/app.bin    # CP app: 2   ← CP 打印的
grep -ac "load polar tab" package/tmp/app1.bin   # AP app: 0
```
**结论**：`ef/psram/cal` 是 **CP 核(CPU1，我们保留的 ARMINO)** 打印的。CP 是主核、先跑，
做完 wifi 校准后**再启动 AP 核**。日志停在 `cal:E(107)` = **CP 卡在校准阶段，从没启动 AP 核**。

---

## 11. 撞墙点（当前结论）

**根因**：BK7258 是**双核 + TrustZone 紧耦合**。**CP 核(主)在启动 AP 核之前，会等 AP 核完成某个启动握手**
（共享内存标志 / mailbox / spinlock / core_id）。ARMINO 的 AP app 深度参与这套握手；我们的裸 NuttX 不参与，
导致 CP 主核在 ~107ms 处一直等 → 卡死 → 永不启动 AP 核 → 我们的 NuttX 一个字都跑不出来。

对比证据：烧 ARMINO 的 `spi_lcd_example`（AP 也是 ARMINO）能越过同一处 cal → 说明差异就在 AP app 是否参与握手。

**为什么难**：要跨过这一步，需要逆向 CP↔AP 的精确启动握手协议（CP 到底在等哪个标志/mailbox），
**没有 JTAG 硬件调试器很难定位**（看不到 CP 卡在哪行、AP 核是否被 release、安全态如何）。

---

## 12. 已完成的成果清单（可写进作品）

- ✅ 完整 openvela BK7258 芯片层 BSP：naked 复位(SP+MSPLIM)、NVIC 中断、SysTick、堆、UART 驱动、内存映射、向量表 —— **编译链接通过**
- ✅ 逆向出 AP app 完整镜像格式（XIP@0x02150000、向量表布局、34/32 CRC、分区偏移 0x165000）
- ✅ 打通整条烧录链路（ARMINO 原生编译绕 Docker、bk_loader 下载、RST 时序、repack 打包+CRC 合入 all-app.bin）
- ✅ 关键修复：MSPLIM、build-id 占位、看门狗、串口驱动、/dev/console
- ✅ 出厂固件已备份可恢复
- ⏳ 未过：AP 核未被 CP 主核启动（双核+TrustZone 启动握手）——已知后续攻坚点

---

## 13. 常用命令速查（本项目）

```bash
# 编译 NuttX
cd /home/zhangyan68/miwear-main/vendor/openvela
./build.sh vendor/beken/boards/bk7258/bk7258-devkit/configs/nsh/ --cmake -j$(nproc)

# 打包成 AP app 合入 all-app.bin
cp cmake_out/bk7258-devkit_nsh/nuttx.bin /tmp/bk_repack/app1.bin
cd /tmp/bk_repack && python3 repack.py

# 烧录(自己终端跑, 能看 bk_loader 输出卡 RST 时机)
bash /tmp/flash_and_log.sh

# 单独抓串口
N=$(ls /dev/ttyUSB*|grep -oE '[0-9]+$'|head -1)
stty -F /dev/ttyUSB$N 115200 raw -echo; cat /dev/ttyUSB$N

# 分析二进制
arm-none-eabi-nm nuttx | grep -E " _vectors| __start"
arm-none-eabi-readelf -l app.elf
xxd -s 0x165000 all-app-nuttx.bin | head
```

---

## 14. 下一步（尝试方向 1：攻坚双核启动握手）

思路：逆向 CP app（`package/tmp/app.bin` / CP 源码）里"wifi 校准之后、启动 AP 核之前"等待的
共享标志或 mailbox；在我们 NuttX 最早期（naked `__start`）设置 core_id 并写入 CP 期望的标志，
让 CP 认为 AP 已就绪、继续启动 AP。**风险**：无 JTAG，定位靠盲试，成功率有限。

---

## 15. 尝试 1 的深入发现（双核启动架构逆向）

**目标**：搞清 CP↔CPU1 的启动握手，尝试让我们的 NuttX（CPU1）跑起来。

**关键代码定位**（命令：`grep -rn start_cpu1_core / get_partition_addr / reset_cpu1_core`）：

1. **谁是主核**：`cp/` 侧是主核，先启动、打印 ef/psram/cal（这些字符串仅在 `app.bin`=CP 分区）。
   `ap/components/bk_startup/system_main.c` 的 `entry_main → rtos_init → start_app_main_thread → 调度器`。
2. **谁启动我们的 NuttX**：`start_cpu1_core()`（`cp/components/bk_cli/cli_main.c:511` 等）调用
   `reset_cpu1_core(offset,1)`，其中 `offset = get_partition_addr(1)`：
   ```c
   // system_main.c: 取 APPLICATION1 分区(物理 0x165000), 按 34/32 CRC 反算虚拟地址
   addr = (partition_start_addr / 34) * 32;   // 0x165000 -> 0x150000
   // + SOC_FLASH_DATA_BASE(0x02000000) = 0x02150000  ← 正是我们 NuttX 的 XIP 链接地址!
   ```
   → **CPU1(我们的 NuttX) 启动地址 = 0x02150000，与我们的向量表一致 ✓**。
3. **启动时机晚**：`start_cpu1_core` 在 `cli_main`（较晚）+ 有 **slave 心跳**（`mb_ipc_heartbeat`）。
   日志停在 cal(~107ms)、没有 shell/CLI 字符串 → CP 可能没跑到 cli_main 就停了 → CPU1 从没被启动。

**CPU1 复位处理**（`startup_cpu1.c: Reset_Handler_Cpu1`，我们 NuttX 该对齐的）：
```c
rtos_disable_int();        // cpsid i           —— 我们有
cpu1_set_core_id();        // REG_WRITE(CPU_ID_ADDR, CPU1_CORE_ID) —— 我们缺
__set_MSPLIM(&__STACK_LIMIT1);                  //  —— 我们有
SystemInitCpu1();          // CMSIS: VTOR/FPU/SAU等 —— 我们用自己的
__TCM_LOADER_START();      // 加载 TCM 代码       —— 我们缺
secondary_core_interrupt_init();
```

**为什么判定盲调到头**：
- 我们在 naked `__start` 设完 SP 后**第 2 条指令就直接写 UART0/UART1 的 `@`/`#`**，
  真机日志里 `@`/`#` **数量为 0**（命令：`grep -ao "@" log | wc -l`）。
- 说明 CPU1 **连我们复位代码的第 3 条指令都没到** → 要么没被 release，要么在取指/取栈瞬间就 fault。
- 这早于任何 core_id/TCM/SystemInit 的差异，**不是这些能解释的**。
- 最可能是 **CPU1 的安全态(TrustZone SAU)/boot 地址**由 CP 在 `reset_cpu1_core` 里配置，
  我们既看不到也改不了；**必须用 JTAG/SWD 观察 CPU1 的 PC、fault 类型、安全态**才能定位。

**结论**：BK7258 双核 + TrustZone 的从核启动，纯串口盲调无法突破。需要：
- (正道) **SWD/JTAG 调试器**：直接看 CPU1 状态；或
- (务实) 换 openvela 已支持的**单核板**（如黄山派 SF32LB52）快速跑通，BK7258 适配作为文档化深度案例。

**给其他团队的提示**：如果你们也想把 openvela 塞进 BK7258 的某个核，务必先准备 SWD 调试器；
双核 SMP + TrustZone 的从核 boot 握手（core_id / 安全态 / boot 地址 / 心跳）是最大的坑。

---

## 16. 硬件原理图 / 位号图分析（关键接口与调试口）

**资料**：`docs/AIDK_AI玩具开发板_{原理图,顶层位号图,底层位号图}.pdf`
**方法**：`pdftotext -layout` 抽原理图文字网名 + `pdftoppm -png -r 300` 渲染位号图看实际布局。

### 16.1 串口接口（重要）

| 接口 | 位置 | 说明 |
| ---- | ---- | ---- |
| **USB2**（USB-C，丝印 "USB TO UART"）| 板底部 | CH340 → **UART0**。既是 bootROM 下载口，也是 **CP 核日志口**。我们一直用它。 |
| **CN10**（4 脚排针）| 板右侧 | **UART1 引出**：丝印 `VDDOUT / TX1 / RX1 / VSS`。原理图 UART1 = 芯片 P0(1TX)/P1(1RX)。 |
| TX0 / RX0 / GND 测试点 | 板底层中部 | UART0 测试点 |

原理图佐证：
```
第20-21行:  Type C 5V -> UART0 -> CH340        (USB2 = CH340 = UART0)
第302,304行: P1/1RX/1SDA=UART1_RXD, P0/1TX/1SCL=UART1_TXD   (UART1 在 P0/P1)
第305行:     P11/DL_0TX/SD_D3                  (UART0 下载 TX = P11/DL_0TX)
```

### 16.2 SWD 调试口

芯片引脚具备 SWD：`pin83 P20/OSCL/SWCLK`、`pin84 P21/OSDA/SWDIO`、`pin43 SWD`、`pin44 CEN(复位)`。
但**位号图上没找到明确引出的 SWD 焊盘**，且 P20/P21 复用 D8/D9/OSDA/OSCL，**不确定是否可直接接 SWD 调试器**（可能需飞线到 P20/P21 芯片引脚）。

### 16.3 其他器件（位号图）

- U1 = BK7258（中央）；LCD1 / LCD2 = 双 GC9D01 圆屏；U7 = SD NAND
- USB1（左）、USB2（底，USB TO UART）；CN10（UART1）、CN5、CN1
- K1（右上）= 复位键；S1/S2/S3（底）= 按键；2.4G 天线（顶）；DVP/H1 = 摄像头（GC2145）
- CN2 = 电池（BAT+/GND/NTC）；CN6/7/8 = 电机/喇叭；MO+/MO- = 马达

### 16.4 由此得到的关键排查思路（无需 SWD）

**我们之前漏抓了 UART1！** 早期 `#`（写 UART1）测试的输出是从 **CN10** 出来的，
而我们**只抓了 USB2(UART0)**、CN10 没接线 → "没看到 #" **不能证明 NuttX 没运行**。

**推测**：NuttX 可能在 CPU1 上已经跑起来，但因 UART0 被 CP 占用，其输出实际走了 **UART1(CN10)**。

**下一步**：用 USB 转串口小板接 CN10（`GND↔VSS`、`适配器RXD↔TX1`），115200 抓 UART1。
若出现 `#` / `<NX:>` / NSH → 证明 NuttX 在 CPU1 上是活的、只是输出走 UART1，
则把控制台正式改到 UART1、从 CN10 交互即可，绕过 SWD 与双核握手难题。

> **状态**：串口小板已焊到 CN10 的 **TX1 / TX2(RX1)**（待抓取验证）。

---

## 17. 尝试 1 收尾：镜像对比 + core_id + 三路 UART 抓取

### 17.1 决定性对比：bootloader+CP 区字节完全一致
```bash
cmp <(head -c 0x165000 原版all-app.bin) <(head -c 0x165000 我们的all-app-nuttx.bin)
# -> 前 0x165000 字节完全一致
```
**结论**：CP 核跑的字节和原版 spi_lcd **完全相同** → CP **照常启动了 CPU1**。
问题被收窄为：**CPU1 被启动了，但我们的 NuttX 在输出第一个字符前就崩了**（不是 CP 卡死）。
（spi_lcd 日志能到 shell，是因为其 CPU1=ARMINO 会通过 mailbox 报"就绪"让 CP 继续；
我们的 CPU1=NuttX 崩了不报 → CP 一直等 → 日志停在 cal。）

### 17.2 尝试 cpu1_set_core_id（对齐 ARMINO CPU1 第一步）
`CPU_ID_ADDR=0x20000000`(DTCM)，`cpu1_set_core_id()` = 写 1 到该地址。
假设 SoC 总线依 CPU_ID 路由外设访问 → 在 naked `__start` 最前面加 `str #1,[0x20000000]`。
**结果**：无效，仍无任何输出。

### 17.3 三路 UART 同时抓取（含 CN10 的 UART1/UART2）
硬件：CN10 焊了 USB 转串口小板 → 主机新增 `/dev/ttyACM0`(UART1)、`/dev/ttyACM1`(UART2)。
```bash
for p in ttyUSB0 ttyACM0 ttyACM1; do stty -F /dev/$p 115200 raw -echo; done
cat /dev/ttyUSB0 >u0.log & cat /dev/ttyACM0 >a0.log & cat /dev/ttyACM1 >a1.log &
# 按 RST 后统计 @(UART0)/#(UART1)/<NX:>
```
**结果**：UART0 有 CP 日志但 `@`=0；UART1/UART2 **完全空**。三路都没有我们的标记。

### 17.4 尝试 1 最终结论
- CPU1 确实被 CP 启动（镜像对比证明），但 NuttX **在第一条 store 指令前就 fault**（连 UART0 的 `@` 都没有）。
- 最可能：CPU1 取指(flash XIP)/安全态(TrustZone)/向量表处理与我们的假设不同，**取指或第一次访存即 fault**。
- **纯串口盲调到此为止，必须用 SWD/JTAG** 看 CPU1 的 PC、fault 类型、安全态。
- SWD 焊盘：位号图无现成引出，SWDIO=P21(84脚)/SWCLK=P20(83脚)，需飞线到 QFN 引脚。

**给其他团队**：把 openvela 塞进 BK7258 的从核(CPU1)，卡点是 CPU1 上电后取指/访存即 fault，
纯串口无法定位，务必准备 SWD 调试器；或改用 openvela 已支持的单核板。

---

## 18. 尝试 2：SWD 硬件调试（DAP-Link + OpenOCD）【进行中·占位】

> **状态**：DAP-Link（USB SWD/JTAG 版，CMSIS-DAP 固件，约 ¥14.75）已下单，待到货。
> 配置文件已写好放在同级目录 `openocd/`（`bk7258.cfg` + `README_zh-cn.md`），到货即用。
> 本节先把接线表和成败判断标准记下来，实测结论到货后回填 18.3 及之后。

### 18.1 为什么上 SWD

第 17 节纯串口盲调的终点：CPU1 被 CP 启动，但 NuttX 在输出第一个字符前就 fault，
三路 UART 全空，**无法判断崩在取指还是访存、是否 TrustZone 安全态**。SWD 能把核
halt 住直接读 PC / 故障寄存器（CFSR/HFSR/MMFAR/BFAR），这是串口给不了的信息。

### 18.2 接线表（DAP-Link → BK7258）

DAP-Link 针脚（手册确认）：`TCK/CK`=SWCLK、`TMS/IO`=SWDIO、`nRST`=复位、`GND`、
`3V3/5V`=对外供电、`U_TX/U_RX`=虚拟串口（115200）。

| DAP-Link 针脚 | 接到 BK7258 | 芯片位置 | 说明 |
|---|---|---|---|
| **TMS/IO** | SWDIO = P21 | QFN **pin 84** | 数据线；也可试专用 `SWD`(pin43) 旁串阻 R2 焊盘 |
| **TCK/CK** | SWCLK = P20 | QFN **pin 83** | 时钟线 |
| **GND** | GND | 任意地焊盘 | 必接，务必共地 |
| **nRST** | CEN = pin 44 | 复位脚 | **必接**，connect-under-reset 要用 |
| ~~3V3 / 5V~~ | — | — | **不接**，板子自己 USB 供电，两电源顶牛会出事 |

飞线技巧：QFN 细脚难焊，优先找 P20/P21 走线上的**串阻焊盘**。
附带福利：`U_TX→板RXD`、`U_RX→板TXD`、共地，可当又一路 115200 串口。

### 18.3 关键一步：connect-under-reset（防脚被复用）

风险点（DAP-Link 手册也警告）：P20/P21 在 BK7258 上兼 I2C(0SCL/0SDA)/ADC/LCD，
固件一旦跑起来可能把这两个脚复用掉，SWD 就被禁用连不上。
对策：接 nRST(CEN)，OpenOCD 连接时先拉住复位，在固件重配脚之前抢先 halt。
`openocd/bk7258.cfg` 已默认开启 `reset_config srst_only srst_nogate connect_assert_srst`。

### 18.4 操作流程（到货后）

```bash
sudo apt install openocd            # 需 0.11+，建议 0.12（支持 Cortex-M33）
cd vendor/beken/boards/bk7258/bk7258-devkit/openocd
openocd -f bk7258.cfg               # 信号差可加 -c "adapter speed 200"
# 另开终端：
telnet localhost 4444
#   bkhalt      复位停在入口，打印 PC/SP/LR/xPSR
#   vt          核对 flash 头向量表（SP 范围、复位向量 LSB=1）
#   resume      放它跑
#   halt        手动停住
#   faultinfo   逐位解码故障寄存器（中文），直接定位崩溃类型/地址
```
源码级现场：`arm-none-eabi-gdb cmake_out/bk7258-devkit_nsh/nuttx -ex "target extended-remote :3333"`。

### 18.5 成败判断标准（决定是否继续死磕）

| 观察 | 含义 | 行动 |
|---|---|---|
| OpenOCD 启动后**读出 DPIDR / 成功 halt** | **SWD 没被锁**，成功一大半 | 继续 `faultinfo` 定位崩溃点 |
| 接线正确、connect-under-reset 仍进不去、读不出 DPIDR | 安全 bootloader / 读保护**锁了 SWD** | 基本无解，**转备选**（换单核带调试座的板子）|
| 能 halt 但 PC 是乱值 / 抓到的不是 AP 核 | 抓错核 | `vt` 核对，必要时 `dap info` 看有几个 AP |

> **底线**：能读出 DPIDR 才有戏；连不进就是被锁，别耗，果断转备选方案。

### 18.6 faultinfo 常见结论对照（预判）

- `PRECISERR + BFARVALID` → 精确总线错，看 BFAR：多半访问了没使能的外设/内存（时钟门控/XIP 未就绪）
- `INVSTATE` → 跳到非 Thumb 地址（复位向量 LSB 没置 1）
- `UNDEFINSTR` → 取到垃圾指令（XIP 映射/加载地址不对，取指位置不是我们的向量表）
- `IBUSERR` → 取指总线错（flash XIP 未映射到 0x02150000 / 分区偏移不对）
- `NOCP` → 碰了 FPU 但没使能 CPACR

### 18.7 实测结论【待回填】

- [ ] SWD 是否连通（DPIDR = ?）
- [ ] CPU1 崩溃时 PC / LR / SP / xPSR
- [ ] faultinfo 解码结果（崩溃类型 + 出错地址）
- [ ] 根因判断（取指 / 访存 / 安全态 / 向量表）
- [ ] 修复措施与结果

### 18.8 官方确认 + ARMINO 调试配置（关键突破，2026-07）

大赛群里 Beken 官方（罗工）确认：**BK7258 的 JLink/SWD 模式没有关**，通过给 CP、AP
都打开调试模式配置即可 debug；使用 JLink 前还可打开 `config dump` 抓完整异常现场。
据此在本地 ARMINO SDK（`vendor/armino/bk_avdk_smp`）逆向出确切机制：

**(1) 内核确认**：AP/CP 核目录为 `middleware/arch/cm33/` → 三个核都是 **ARM Cortex-M33**。
→ 我们选 CMSIS-DAP + OpenOCD 通用 `cortex_m` 驱动**完全正确**（M33 属 ARMv8-M，cortex_m 兼容）。

**(2) 三个关键 Kconfig（AP、CP 各一份，都要打开）**：
| 配置项 | 默认 | 作用（源码验证）|
|---|---|---|
| `CONFIG_SWD_DEBUG_MODE` | n | 调用 `bk_set_jtag_mode()` 使能调试、路由 SWD 到某核、拉 CPU 到 120M、**关看门狗**；且异常处理里变成 `while(1)` 自旋（`dump_system_info`），**让调试器能抓住 fault 现场** |
| `CONFIG_JTAG` | n | 使能 JTAG 引脚；同时影响 flash 访问路径（`flash_ll.h`）|
| `CONFIG_DUMP_ENABLE` (+`MEMDUMP_ALL`,`DUMP_UART_PRINT_PORT`=0) | n | 异常时把寄存器/内存**从 UART0 全量 dump**，**无需调试器**即可拿现场 |

来源：`{ap,cp}/middleware/arch/cm33/Kconfig`、`ap/components/bk_startup/system_main.c:290`、
`.../CMSIS_5/.../smp/startup_cpu0.c:411`。

**(3) SWD/JTAG 引脚组（`gpio_jtag_sel`, `cp/middleware/driver/bk7258/gpio_driver.c:179`）**：
- **GROUP0**：`GPIO20 → TCK(SWCLK)`、`GPIO21 → TMS(SWDIO)` ← **与原理图 P20(pin83)/P21(pin84) 完全一致，我们的接线表正确**
- GROUP1：GPIO0/GPIO1（备选）

**(4) SWD 是"单核复用"——必须路由到跑 NuttX 的那个核（最关键）**：
JTAG-DP 一次只连一个 ARM 核，由系统寄存器字段选择：
- 寄存器 `SYS_CPU_STORAGE_CONNECT_OP_SELECT` @ **0x44010008**
- 字段 `JTAG_CORE_SEL` 在 **bit[8:7]**（mask 0x3），写 cpu_id（0/1/2）
- 由 `sys_hal_set_jtag_mode(cpu_id)` → `sys_ll_set_..._jtag_core_sel(cpu_id)` 完成
- 另有 `ICU_R_JTAG_SELECT` 在 ARM 与 TL4(DSP) 间选择：`0x7111E968`=ARM
- **含义**：若 CP 把 JTAG 路由到 CPU0，而 NuttX 跑在 CPU1，则 halt 到的是错的核 → 必须确保 `JTAG_CORE_SEL` 指向 NuttX 所在核。

**(5) 据此修正的行动计划**：
1. **（不等调试器，现在就能做）** 重编 ARMINO **CP**（保留在镜像里的核）令 `CONFIG_SWD_DEBUG_MODE=y`、`CONFIG_JTAG=y`、`CONFIG_DUMP_ENABLE=y`；CP 会提前使能调试口、关看门狗、保持 GPIO20/21 为 SWD、并把 JTAG 路由到目标核。同时编一份**原厂 AP**（同样打开这些）作对照，验证 UART0 dump 与调试口可用。
2. 确认 NuttX 跑在哪个 cpu_id，保证 `JTAG_CORE_SEL(0x44010008 bit7-8)` 指向它；必要时在 NuttX `__start` 早期或 CP 侧写好该寄存器。
3. NuttX 侧：启动早期**不要重配 GPIO20/21**（保持 SWD），并考虑在 HardFault 处理里加 `while(1)` 自旋，配合调试器抓现场（对齐 ARMINO SWD_DEBUG_MODE 行为）。
4. 调试器到货：按 18.2 接线（P21/P20/GND/CEN），`openocd -f bk7258.cfg` → `bkhalt`/`faultinfo`。SWD 未锁，成功概率大幅提高。

> **意义**：18.5 里"SWD 可能被安全 bootloader 锁死"这个最大风险，已被官方**排除**。
> 现在瓶颈从"能不能连上"变成"路由到正确的核 + NuttX 别把调试脚复用掉"，都是可控的工程问题。

### 18.9 已动手：打开 ARMINO 调试开关（含 JTAG 风险规避）

参考群里发的 `jlink使用例子.docx`（方法：在 `startup_bk7236.c` 故障处理里加 `while(1)`，
连 JLink，触发异常后 suspend 看调用栈；用**硬件断点**，因代码在 XIP flash 上）——
这正是 `CONFIG_SWD_DEBUG_MODE` 内建的行为（异常处理 `while(1)` 自旋）。其调试用的是
`JLinkGDBServer + arm-none-eabi-gdb`，与我们 `OpenOCD(:3333) + gdb` 流程等价，`bt` 看栈。

**改了什么**（`vendor/armino/bk_avdk_smp/projects/spi_lcd_example/`）：
| 文件 | 改动 |
|---|---|
| `cp/config/bk7258/config` | `CONFIG_SWD_DEBUG_MODE=y`、`CONFIG_DUMP_ENABLE=y` |
| `ap/config/bk7258_ap/config` | `CONFIG_SWD_DEBUG_MODE=y`、`CONFIG_DUMP_ENABLE=y` |

**为什么没开 `CONFIG_JTAG`（重要，避免变砖）**：
`CONFIG_JTAG=y` 会在 flash 驱动里把 **flash 控制器 CRC 关掉**（`flash_ll.h`: `crc_en=0`；
`flash.c`: `value &= ~CRC_EN`）。而我们烧录的镜像是**带 34/32 CRC 编码**的（repack 时加的），
一旦 flash 控制器停止 CRC 解码，XIP 读回的就是错位的原始字节 → **CP 跑起来即崩、直接不启动**。
且我们用 DAP-Link 走 **SWD（2 线）**，只需 `SWD_DEBUG_MODE` 即可，用不到全 JTAG（4 线）。
如果以后真要开 `CONFIG_JTAG`，必须同时把镜像改成 **crc_enable=false** 重新打包，属另一条路。

**`SWD_DEBUG_MODE` 打开后获得的能力**（源码 `bk_set_jtag_mode`, `system_main.c`）：
1. `sys_drv_set_jtag_mode(cpu_id)` 把 SWD-DP 路由到指定核（寄存器 0x44010008 bit7-8）
2. `gpio_jtag_sel(0)` 把 **GPIO20→SWCLK、GPIO21→SWDIO**（group0，正是我们要接的脚）
3. 拉 CPU 到 120M、**关看门狗**
4. 异常处理进 `while(1)` 自旋 → 调试器能抓住 fault 现场

**运行时激活（不用改代码）**：CP 端 UART0 控制台有 CLI 命令
```
setjtagmode {cpu0|cpu1|cpu2} {group1|group2}     # group1 = GPIO20/21
```
它调用 `bk_set_jtag_mode()`，但**仅在编译了 `SWD_DEBUG_MODE` 时才生效**（已打开）。
上电后在 CP 控制台敲 `setjtagmode cpu1 group1` 即把 SWD 路由到 CPU1 的 GPIO20/21。
（NuttX 具体跑在哪个 cpu_id 待确认；不确定时可依次试 cpu0/1/2。）

**重编与验证**：
```bash
cd /home/zhangyan68/miwear-main/vendor/armino/bk_avdk_smp
# 建议先清该工程的生成配置, 确保 config 改动被重新读取:
rm -rf build/bk7258/spi_lcd_example
make bk7258 PROJECT=spi_lcd_example
# 验证生成的 sdkconfig 里开关已生效:
grep -E "SWD_DEBUG_MODE|DUMP_ENABLE|CONFIG_JTAG" \
     build/bk7258/spi_lcd_example/bk7258/config/sdkconfig.json
```
编出的 CP app 用于替换/合入镜像（AP 仍替换成 NuttX）；也可先烧**全原厂**（含新开关）验证
`setjtagmode` + UART0 dump 通路，再换 NuttX。

> **NuttX 侧 TODO**：启动早期不要重配 GPIO20/21；可在 HardFault 处理加 `while(1)` 自旋，
> 对齐 ARMINO 行为，方便 SWD 抓现场。

### 18.10 NuttX 侧改造：故障自旋 + 保护 SWD 脚 + 早置 VTOR

配合 SWD 调试，对 NuttX 端口做了三处改动（已编译链接通过，nuttx.bin≈146KB）：

**(1) 故障处理改为死循环自旋**（新增 `CONFIG_BK7258_DEBUG_FAULT_SPIN`，默认 y）
- `arch/arm/src/bk7258/Kconfig`：新增调试开关 + `..._MARK`（自旋前打标记 `!!BKFAULT!!`）。
- `arch/arm/src/bk7258/bk7258_irq.c`：新增 `bk7258_fault_spin()`，把 context 存到全局
  `g_bk7258_fault_regs`，打标记后 `while(1)`。在 `up_irqinitialize()` 末尾用它**覆盖**默认的
  `arm_hardfault`，并单独 attach+enable Mem/Bus/Usage fault（不让它们升级为 HardFault，
  OpenOCD 里能直接分辨 fault 类型）。
- 好处：核停在故障现场自旋，硬件压栈的异常帧（PC/LR/xPSR…）原样保留在 MSP，
  SWD halt 后即可读；不走 `_alert/PANIC`（OS 早期未起来时可能再 fault 或无输出）。
  对齐 jlink 例子"故障处加 while(1) 后 suspend 看栈"的做法。

**(2) 早置 VTOR**（`bk7258_start.c` 的 `__start`）
- 在清 MSPLIM 之后、写 '@' 之前，`SCB->VTOR(0xE000ED08) = _vectors(0x02150000)`。
- 原因：`up_irqinitialize()` 之前发生的异常本会走 bootloader 残留向量表，落到别处、
  SWD 看不到我们的自旋。早置后，**最早期的 fault 也进入本工程的故障处理**。

**(3) 保护 SWD 脚（防复用）**
- `bk7258_start.c` 顶部加醒目注释：GPIO20=SWCLK、GPIO21=SWDIO，本端口无 GPIO/pinmux
  驱动、全程不复用这两脚（已 grep 确认芯片层+板级 src 无任何 GPIO 复用代码）；
  后续加引脚复用时严禁动 GPIO20/21。

**gdb/OpenOCD 里读故障现场**：
```
halt
faultinfo                       # 解码 CFSR/HFSR/MMFAR/BFAR (bk7258.cfg 内置)
# 或用暴露的全局帧:
p/x *(uint32_t(*)[21])g_bk7258_fault_regs
reg pc ; reg lr ; reg sp ; reg xpsr
```
关掉自旋恢复默认：`menuconfig` 里取消 `BK7258_DEBUG_FAULT_SPIN`。

### 18.11 SWD 首次连接：DP 通了，MEM-AP 待路由（里程碑）

接好 3 线(SWDIO→P21/pin84、SWCLK→P20/pin83、GND;3V3/5V/nRST 不接)、板子自供电后,
`openocd -f bk7258.cfg -c "adapter speed 200"` 输出:
```
Info : CMSIS-DAP: FW Version = 0254 ... Interface Initialised (SWD)
Info : SWD DPIDR 0x1be12aeb
Error: Could not find MEM-AP to control the core
Warn : target bk7258.cpu examination failed
```

**解读**:
- **DPIDR 读出 = SWD 物理层 + 调试端口(DP)全通**,飞线正确、共地正确,**DP 未被锁**。
  这是关键里程碑:排除了"安全 bootloader 锁死 SWD"。
- DPIDR `0x1be12aeb` 解码:VERSION=2(**DPv2, 支持 multidrop 多核共享 SWD**)、MIN=1、
  designer≠ARM 的 0x23B → 说明当前 DP 连的**不是我们要的 ARM 核**(疑似 TL4/DSP 侧)。
- `Could not find MEM-AP` = 能跟 DP 对话但够不到 CPU 内核的内存访问口。多核 DPv2 上典型
  表现:没选对目标核时 DPIDR 能读、AP 却访问不到(与 RP2040 multidrop 现象一致)。

**根因**:板子当前跑的是**未打开 `SWD_DEBUG_MODE` 的旧固件**,CP 没把调试口路由到跑
NuttX 的 ARM 核;而 `ICU_R_JTAG_SELECT`(ARM/TL4 选择)与 `JTAG_CORE_SEL`(0x44010008,
ARM 多核选择)只能由**芯片上运行的固件**设置——SWD 现在够不到内存,无法自行写入。

**解锁步骤(下一步)**:
1. 重编带开关的 ARMINO(配置已改, 见 18.9)+ 重烧(新 CP + 带 fault-spin 的新 NuttX)。
2. 上电后在 CP 的 UART0 控制台敲 `setjtagmode cpu0 group1`(group1=GPIO20/21;
   cpu0/1/2 依次试),把 SWD 路由到跑 NuttX 的 ARM 核。
3. 再 `openocd -f bk7258.cfg` → 应能 examine 成功 → `halt` / `faultinfo` 看崩溃点。

**备选(若路由后仍不行)**:BK7258 是 DPv2 multidrop,可能需要 OpenOCD 侧
`swd multidrop` + `TARGETSEL`(目标 DP 实例 ID)。目标 ID 需从 Beken/官方 JLink 配置获取
或扫描;优先走上面的固件路由方案(官方推荐)。

> 进展:SWD 物理层 100% 打通,问题从"能不能连"缩小为"路由到正确的 ARM 核"。

### 18.12 关键校正：核编号 + 路由改为"代码里开机路由"

烧入带 `SWD_DEBUG_MODE=y` 的固件后,CP 串口(UART0/CH340)打出:
```
ef:I(4):ENV start address is 0x007FA000 ... EasyFlash V4.1.0 init success
psram:W(10):psram type(16MB) not match CONFIG_PSRAM_CAPACITY 0X00800000
I(10):driver_init: psram init ok
... rwnx_cal_set_rfconfig(0x102) phy on; rf off
cal:E(108):load polar tab magic code error 0xffffffff   <-- 日志停在这里, 无命令行
```

**校正三点**:
1. **核编号**:`cp_main.c` 的 `main()` → `bk_init()`，之后 `user_app_main()` 里
   `bk_pm_module_vote_boot_cp1_ctrl()` 才启动 AP 核(cpu1/cpu2)。即 **CP = CPU0**,
   AP(跑 NuttX) = CPU1。
2. **没有命令行**:CP 日志停在 `cal:E(108)`,到不了 shell → `setjtagmode` 命令这条路
   **走不通**(之前 README 步骤 5 作废)。
3. **MEM-AP 找不到的真因**:本工程**从没调用 `bk_set_jtag_mode()`**(只在 cp_main.c
   声明了 extern)。所以即便 `SWD_DEBUG_MODE:true` 编进去了,GPIO20/21 也没被复用成
   JTAG、核也没被选中 → openocd 只看到 DP、找不到 MEM-AP。

**修复**:在 `projects/spi_lcd_example/cp/cp_main.c` 的 `user_app_main()` 里,启动 AP 核
之前加一行 `bk_set_jtag_mode(1, 0)`(路由到 CPU1=AP,group0=GPIO20/21)。
`gpio_jtag_sel` 依赖已初始化的 gpio 驱动(`s_gpio.hal`),所以放在 `bk_init()` 之后的
`user_app_main` 里执行是安全的。

> 备注:`bk_set_jtag_mode` 不动 ARM/TL4 的 ICU 选择(`ICU_V_JTAG_SEL_WR_ARM` 全代码未写,
> 默认即 ARM);只设 `jtag_core_sel`(0x44010008)+ GPIO 复用。

**下一步(重编 CP → repack → 重烧 → RST → attach)**:
- 重烧后**不用敲任何命令**,上电后 CP 会自动路由 SWD 到 CPU1。
- `openocd -f bk7258.cfg` 应能 examine 成功 → `halt` → 看 PC:
  - 落在 `bk7258_fault_spin` / 0x0215xxxx(NuttX flash)→ **正在调 NuttX**,`faultinfo` 定位崩溃。
  - 仍 `Could not find MEM-AP` → 把 `bk_set_jtag_mode(1,0)` 的 1 改成 0(先验证能连 CPU0/CP),
    或改 2 试另一 AP 核。

### 18.13 深挖 MEM-AP 找不到：排除 ICU，改早期路由 CPU0

重烧带路由的固件后 openocd 仍 `Could not find MEM-AP`。在本机核查:
- 反汇编 CP 的 `app.elf` 确认 `user_app_main` **确实调用了** `bk_set_jtag_mode(1,0)`
  (`bl bk_set_jtag_mode`, r0=1/r1=0)→ 代码已编入烧进,`rm -rf` 无必要,不是编译问题。
- DPIDR `0x1be12aeb` 低 12 位 `0xaeb` ≠ 标准 ARM SW-DP 的 `0x477`。曾怀疑是路由到了
  TL4/DSP 侧,需写 `ICU_R_JTAG_SELECT=0x7111E968` 切 ARM。**但核查发现**
  `SOC_ICU_REG_BASE = 0xdead0add`(毒值)→ **BK7258 没有 ICU 块**,ARM/TL4 select 是
  老芯片(BK7231)遗留、在 7258 上无效。故排除该路径;`0x1be12aeb` 就是 Beken 自家 DP 的 ID。

**据此推断**:最可能是 `bk_init()` 卡在 `cal:E(108)`,在 `user_app_main` 之前就停了 →
`bk_set_jtag_mode(1,0)` 根本没机会执行 → 没选核 → 没 MEM-AP;或核选择寄存器默认指向了
一个没上电的核。

**改法**(`projects/spi_lcd_example/cp/cp_main.c`):在 `main()` 最开头、`bk_init()` 之前,
加一条**纯寄存器写**把 JTAG_CORE_SEL 选到 CPU0:
```c
volatile uint32_t *jtag_sel = (volatile uint32_t *)0x44010008;
*jtag_sel = (*jtag_sel & ~(0x3u<<7)) | (0u<<7);   // JTAG_CORE_SEL=CPU0
```
不依赖任何驱动,一定执行。组合效果:
- `bk_init` 卡住 → 停留在 CPU0 路由 → attach CPU0 看 CP 卡在哪(直捣墙根);
- `bk_init` 跑完 → `user_app_main` 再把路由改到 CPU1 → attach 看 NuttX。

attach 后 `reg pc` 判断连的是哪个核:
- PC 在 0x02xxxxxx 的 CP wifi/cal 代码 → CP 卡在 cal,找到了不启动 AP 的墙;
- PC 在 0x0215xxxx / `bk7258_fault_spin` → 已连到 NuttX,`faultinfo` 定位崩溃。

### 18.14 定位到工具端：OpenOCD 0.11 太老，换 pyOCD / OpenOCD 0.12

即便在 `bk_init()` 之前就把 JTAG_CORE_SEL 路由到**一定在跑的 CPU0**,openocd 仍
`Could not find MEM-AP`。→ 排除"选错核/核没上电",问题在**调试器主机端够不到 AP**。

- 环境:Ubuntu 22.04,apt 只有 **OpenOCD 0.11.0(2021)**。
- BK7258 = Cortex-M33,DP = **DPv2**(DPIDR 0x1be12aeb, version 字段=2, 支持 multidrop)。
- OpenOCD 0.11 对较新 DAP 的 AP 枚举 / multidrop 支持差 → "读得到 DP、找不到 MEM-AP"
  是其典型表现。Beken 官方用 JLink(带 BK7258 器件定义)能连,佐证是**工具端适配**问题。

**下一步(换现代工具)**:
1. pyOCD(pip 装, 最快):`pip install --user pyocd` → `pyocd list` → `pyocd commander -t cortex_m -f 200000`(可加 `--connect under-reset`)。
2. 备选 OpenOCD 0.12(xPack 预编译):支持 ADIv6 + 更好的 multidrop,用同一个 bk7258.cfg。

> 结论演进:SWD 物理层/DP 已通;瓶颈从"固件路由"进一步收敛到"主机端调试器版本"。
> 待验证:换工具后能否 examine 成功、halt 住核、读 PC。

### 18.15 决定性突破：NuttX 其实一直在正常运行！问题是串口输出口选错

pyOCD 连上后(OpenOCD 0.11 换 pyOCD 即通),`reset halt` → `go` → `halt` → `reg`:
```
pc = 0x0215eb9a   -> up_idle (符号确认: 0215eb9a T up_idle, 内容仅 nop;bx lr)
lr = 0x02150f22   -> nx_start 内部 (nx_start=0x02150ec8)
sp = 0x28011c48   -> AP RAM, 有效
xpsr = 0x89000000 -> IPSR=0, 线程模式, 不在异常里
CFSR/HFSR/ICSR = 0 -> 无任何故障
```

**结论:NuttX 完整启动、跑到了 idle 循环(up_idle)。** 即 `nx_start`→`up_initialize`→
`nx_bringup`(建 init 任务)→ 进入 idle 全部成功。

→ **推翻第 10/11 节的旧结论**("AP 核第一条指令都没执行 / CP 卡死不启动 AP")。真相是:
AP 核一直在正常跑 NuttX,只是**一个字符都没从我们监视的串口出来**。之前纯串口盲调因为
看不到内部状态,误判成"没启动"。SWD 一上来就把真相摆出来了。

**根因锁定:控制台 UART 选错。**
- `bk7258_putc_uart`(反汇编)轮询 TX-ready(base+0x18 bit20)带**超时**(~200000 次),
  超时后仍写 base+0x1c —— 所以 UART 不就绪时**不会 hang、只是丢字节**,完美吻合"能跑、无输出"。
- defconfig 选了 `CONFIG_UART0_SERIAL_CONSOLE=y`(理由:CH340=UART0,烧录日志同一根线)。
- 但 memorymap 注释与 ARMINO 惯例:**UART0 = CP 核的口;AP 核的日志口是 UART1(0x45830000)**。
  NuttX 跑在 AP 核,写 UART0 大概率驱动不了(UART0 归 CP / 需 mailbox 转发)→ 全部超时丢弃。

**验证手段(SWD 直写,无需重编)**:`cat /dev/ttyUSB0` 看 CH340,pyocd 里
`write32 0x4482001c 0x55`(UART0 TX)与 `write32 0x4583001c 0x55`(UART1 TX),看哪个口冒出 'U'。

**预期修复**:defconfig 控制台从 UART0 改 UART1(0x45830000),监视 UART1(CN10 引出),NSH 应出。

### 18.16 根因修复：AP 核自己使能 UART1 时钟 + 复用 GPIO0/1 引脚

SWD 读寄存器确认:两个 UART 从 AP 核都能读、config 都是 0x0000e119(TX 使能+波特率分频,
正是 `bk7258_uart_config` 写的值)、fifo_status 都 TX 就绪;但直写 FIFO(0x4482001c/0x4583001c)
都无物理输出。

**根因**:`bk7258_lowsetup`/`bk7258_uart_config` 只做了 UART 的 baud/TX-enable,**漏了两步**:
1. 使能 UART 外设时钟;
2. 把 TX/RX 引脚复用成 UART 功能。
当初假设"bootloader 已配好 UART0"——但那是 CP 给自己配的(UART0=GPIO11/10=CH340,CP 占用),
AP 核驱动不了 UART0。AP 核要用串口,必须**自己开时钟 + 自己 mux 引脚**(对照 ARMINO
`uart_driver.c`: `sys_drv_dev_clk_pwr_up` + `uart_init_gpio`→`gpio_dev_map`)。

**修复**(`bk7258_lowputc.c` 新增 `bk7258_uart1_hwsetup()`,在 `bk7258_lowsetup` 最前调用):
- 使能 UART1 时钟:`0x44010030` 置 bit10(UART1_CKEN)。
- 引脚功能选择:`0x440100C0` 清 [7:0](GPIO0→[3:0]=0=UART1_TXD, GPIO1→[7:4]=0=UART1_RXD)。
- 每脚第二功能使能:`0x44000400`(GPIO0)、`0x44000404`(GPIO1)置 bit6。
寄存器/引脚全部来自 ARMINO SDK(UART1_TX=GPIO0, UART1_RX=GPIO1, 引出到 CN10)。

编译通过(nuttx.bin 146316B),已 repack 出 all-app-nuttx.bin。

**验证**:重烧后监视 **UART1 = /dev/ttyACM0**(`stty -F /dev/ttyACM0 115200 raw -echo; cat /dev/ttyACM0`)。
`arm_lowputc` 本就同时写 UART0+UART1,故启动进度标记应开始从 ttyACM0 冒出。
若出现 → 时钟+引脚就是缺的那一步,下一步把 NSH 控制台正式切到 UART1。

### 18.17 定位:halt 门控 UART 时钟 + 控制台切到 UART1

之前所有"SWD halt 时写 FIFO 无输出"的测试**全是假象**:BK7258 在调试 halt 时会**门控外设时钟**,
UART 停摆。决定性验证:`halt` → 往 UART1 FIFO 写 5 个 0x55 → `go`,一 resume,**ttyACM0 立刻冒出
一串 'U'**(FIFO 里的字节在时钟恢复后发出)。

**结论**:
- UART1 从 AP 核**完全可用**(时钟/引脚/config 全对,18.16 的设置正确)。
- 之前 register-poking 看不到输出,是 halt 门控时钟所致,不是硬件不通。
- SWD 寄存器读写仍有效(APB 常开),但**外设的"运行"(如 UART 移位发送)在 halt 时暂停**。

**据此的最后一步**:把 NuttX 的 NSH 控制台从 UART0(CP 占用、AP 打不出)切到 **UART1**。
- `bk7258_serial.c`: `CONSOLE_BASE` = `BK7258_UART1_BASE`,`CONSOLE_IRQ` = `BK7258_IRQ_UART1`(EXTINT+15)。
- UART1 时钟+引脚已由 `bk7258_lowsetup()->bk7258_uart1_hwsetup()` 配好。
编译通过、已 repack。

**验证**:重烧后**断开 pyOCD**(否则 halt 会停时钟)、按 RST、`cat /dev/ttyACM0`(115200)。
应看到 NuttX 启动日志 + NSH 提示符。这是通关验证。

### 18.18 根因定论(官方文档):CPU1 无直连日志 UART,走 mailbox

官方文档 https://docs.bekencorp.com/arminodoc/bk_idk/bk7258/zh_CN/v2.0.1/developer-guide/debug_trace/
明确 BK7258 三核日志架构:
- **CPU0** → 直接 UART0(DL_UART0),默认 115200。
- **CPU1** → **MAILBOX 转发给 CPU0**,再由 CPU0 经 UART0 输出(CPU1 **无直连日志 UART**);
  可用 `CONFIG_SYS_PRINT_DEV_UART=y`+`CONFIG_UART_PRINT_PORT=0` 改为 CPU1 直出 UART0。
- **CPU2** → 直接 UART2。

**我们的 NuttX 跑在 CPU1**(user_app_main 启动它,jtag_core_sel=1 即 pyOCD 所连的核)。
按硬件设计 CPU1 直写 UART0/UART1 寄存器不会物理发送 → 与探测循环结果一致('A'/'B' 均不出)。

**运行态探测(不碰 SWD,避开 halt 门控)结论**:NuttX(CPU1)一启动就死循环写 UART0='A'、
UART1='B';真机 RST 后 CH340 只见 CPU0 的 cal 日志、无 'A',ttyACM0 无 'B' → **CPU1 直驱两个 UART 都不出**。
排除 secure/非secure 地址嫌疑:AP CONFIG_SPE=1 → 用 secure 地址(0x4482/0x4583),与我们一致,地址没错。

**里程碑再确认**:NuttX 在 BK7258(CPU1)上**完整启动并运行到 idle**,移植主体成功;
剩余是"日志从哪个口出"的架构问题。

**下一步选项**:
- **A(推荐)**:把 NuttX 改跑在 **CPU0**(直连 UART0=CH340)。需逆向 CPU0(app@0x11000)的
  XIP/RAM 布局并重链接,烧入 app 分区替换 ARMINO CP。裸 NSH 不依赖 CP 的 psram/cal 初始化。
- B:实现 ARMINO mailbox 协议,CPU1 日志交 CPU0 打印(复杂)。
- C:CPU1 驱动 UART2(GPIO31/41),需飞线引出。

已移除临时探测死循环,恢复正常启动路径(控制台暂留 UART1)。

### 18.19 方向 A 落地:把 NuttX 改跑在 CPU0(直连 UART0)

因 CPU1 无直连日志 UART(18.18),改为让 NuttX 作为**主核 CPU0** 运行,替换 ARMINO CP,
直接拥有 UART0=CH340。

**逆向 CP(CPU0)布局**(`build/bk7258/spi_lcd_example/bk7258/app.elf`):
- `.vectors@0x02010000`(XIP 基址,对应 flash app 分区 @0x11000)
- 初始 SP=0x2809f700,reset=0x0207cef4;CP 用 SRAM 0x28064000+,DTCM 0x20000000,IRAM 0x08064000
- app 分区逻辑空间 0x02010000..0x02150000(1280K,正好接到 AP 的 0x02150000)

**改动**:
- `scripts/ld.script`:`flash ORIGIN 0x02150000→0x02010000, LEN 0x110000→0x140000`;RAM 保持
  `0x28010000 LEN 0x54000`(不启动 AP,该区空闲且已验证;NuttX 自设 SP,不依赖 CP 的 0x2809f700)。
- `bk7258_serial.c`:控制台 `CONSOLE_BASE/IRQ` 改回 **UART0**(CPU0 直连,bootloader 已配好)。
- `tools/repack.py`:把 nuttx.bin 放进 **app 分区(0x11000)** 替换 ARMINO CP;app1 保留原厂 AP 占位。

**验证**:编译通过;`_vectors@0x02010000`、`__start@0x02010140`;nuttx.bin 头 = SP 0x28011c68 +
reset 0x0201045d(CPU0 XIP,Thumb);打包后 flash 0x11000 头部即该向量表。产物 all-app-nuttx.bin。

**依赖评估(task3)**:NuttX-on-CPU0 只依赖 bootloader(时钟/XIP/UART0 引脚均已配),不用 PSRAM
(RAM 在 SRAM),启动早期关看门狗、清 MSPLIM;ARMINO CP 的 wifi-cal/psram/多核启动对裸 NSH 不需要。

**待验证(task5)**:烧录后 `cat /dev/ttyUSB0`(CH340,115200)按 RST,期待 NuttX 启动日志 + `nsh>`。
这是最干净的控制台路径(CPU0 直驱 UART0),预计能出 NSH。

### 18.20 CPU0 早期 LOCKUP 根因:__start 写 DTCM(0x20000000)

NuttX-on-CPU0 烧入后 CH340 无任何输出。给 NuttX 加了自路由 SWD(bk7258_swd_route_cpu0:
jtag_core_sel=0 + GPIO20/21 复用 JTAG),pyOCD 连上后:
```
Connected to CoreSightTarget [Lockup]      <- 核处于 LOCKUP(双重故障)
pc = 0xeffffffe   <- Cortex-M 锁死特征 PC
xpsr = 0x09000003 <- IPSR=3 = HardFault
sp = 0x2802f790   <- 高于我们的栈顶 0x28011c68 => fault 发生在 __start 设 SP 之前
lr = 0xfffffff9   <- EXC_RETURN
```
**根因**:`__start` 第一条 store 是给 CPU1 写 core-id 到 **DTCM 0x20000000**(多核尝试遗留)。
CPU0 冷启动时 DTCM 未使能,该 store 立即 fault;此时 SP 未设、MSPLIM 为 bootloader 残留值,
fault 压栈失败 → 双重故障 → **LOCKUP**。这解释了"CPU0 完全无输出"(连 __start 里的 '@' 都到不了)。

**修复**:删除 `__start` 里对 0x20000000 的三条写(CPU0 无需且致命)。现 __start 仅:
设 SP(from vector[0]) → 清 MSPLIM → 设 VTOR → 写 '@' 到 UART0 → 进 cstart。编译+repack 通过。

**待验证**:烧录后 `cat /dev/ttyUSB0` 按 RST,期待 `@@@@` + `<NX:1..>` 进度标记 + NSH。
（附:已给 NuttX 内建 SWD 自路由到 CPU0,替换 CP 后仍可用 pyOCD 观测本核。）

### 18.21 CPU0 第二个 LOCKUP 根因:FPU 未使能(NOCP)

删掉 DTCM 写后仍 LOCKUP。pyOCD 读故障寄存器给出确切原因:
```
CFSR = 0x00180000  -> bit19 NOCP(协处理器/FPU 未使能) + bit20 STKOF(栈限制)
HFSR = 0x40000000  -> FORCED(可配置 fault 升级为 HardFault)
pc=0xeffffffe (lockup), IPSR=3(HardFault), r9=0x0201049c(reset 跳板 start)
```
**根因**:NuttX 以硬浮点 ABI 编译(-mfpu=fpv5-sp-d16 -mfloat-abi=hard),`arm_fpuconfig()`
(0x02010608, cstart 后段才调)之前的早期 C 代码就可能用到 FPU 指令。CPU0 冷启动 FPU 未开
→ NOCP UsageFault → 升级 HardFault → 双重故障 → LOCKUP。CPU1 之前由 ARMINO CP 预先开过
FPU/CPACR,所以在 CPU1 上没暴露。

**修复**:在 `__start` 里尽早使能 FPU —— 写 `SCB->CPACR(0xE000ED88)` bit[23:20]=0xF
(CP10/CP11 全访问)+ dsb/isb,放在设 SP/MSPLIM/VTOR 之后、进 cstart 之前。编译+repack 通过。

**待验证**:烧录后 `cat /dev/ttyUSB0` 按 RST,期待 `@@@@` + `<NX:1..>` 标记 + NSH。

### 18.22 CPU0 LOCKUP 精确定位:SWD 单步抓到 __start 顺序问题

pyOCD 关键操作:`reset halt` 停在 bootloader(0x020001c0);`break <app addr>`+`go` 让 bootloader
交接到 app 后停在断点;**坐在断点上时 step/go 不前进(会重复命中该 bp),须先 `rmbreak` 掉当前 bp
再 step**。删掉后单步实测:
```
0x02010146 mov sp, r1  (sp=r1=0x28011c68)
step -> pc=0x02010930 (HardFault 处理), IPSR=3, lr=0xfffffff9, sp=0x2802f800(bootloader 栈区)
```
即在设 SP 附近触发 HardFault, 压栈落到 bootloader 栈; 配合 CFSR=0x00180000(**NOCP+STKOF**)判定:

**根因(顺序错误)**:`__start` 先 `mov sp`(设成 0x28011c68)再 `msr MSPLIM=0`。bootloader 残留的
MSPLIM 较高, 我们的 SP 在其之下; 在"设 SP 后、清 MSPLIM 前"的窗口一旦发生异常压栈 -> SP<MSPLIM ->
**STKOF**; 同时异常压栈做 FPU 惰性保存而 FPU 未开 -> **NOCP**; 叠加 -> 双重故障 -> LOCKUP。

**修复**:把 `__start` 顺序改为——先 `msr MSPLIM=0`、再使能 FPU(CPACR)、**然后**才 `mov sp` 设栈、
设 VTOR。即"清限制+开 FPU"提到设 SP 之前。编译+repack 通过。

**pyOCD 调试要点(记录备用)**:
- `reset halt` 停在 bootloader, 不是 app; 要用 `break <app_addr>`+`go` 到 app。
- 坐在断点上 step/go 不动 -> 先 `rmbreak` 当前地址。
- LOCKUP 后 `reg` 报 "Core is not halted"; 需 `reset halt` 重来。

**待验证**:烧录后 CH340 应出 `@@@@` + `<NX:1..>` + NSH。

### 18.23 里程碑:NuttX 在 CPU0 完整启动到 idle;补 UART0 时钟+引脚

修好 __start 顺序(18.22)后,用 SWD `break+go+rmbreak` 逐段确认(全程无 fault):
```
start(0x020104b4) -> __start(0x02010140) -> [MSPLIM清+FPU开+SP+VTOR] ->
cstart(0x02010208) -> lowsetup(0x02010378) -> nx_start(0x02010f64) -> up_idle(0x0201ec36)
```
→ **NuttX 作为 CPU0 主核完整启动、跑到 idle 循环(0x0201ec36),零 fault!** 移植在 CPU0 成功。
`go` 后核在 idle 正常运行(reg 报 "not halted" = 运行中, 非 lockup)。

**唯一剩余**:UART0(控制台=CH340)无物理输出。与 UART1 同一类问题——bootloader 交接前
很可能把 UART0 拆了(关时钟/复位引脚),而 NuttX 的 `bk7258_uart_config` 只设波特率+TX使能,
未开时钟/复用引脚。

**修复**:新增 `bk7258_uart0_hwsetup()` 在 `lowsetup` 最前调用——使能 UART0 时钟(0x44010030 bit2)+
时钟源 XTAL(cksel_uart0 bit10 of 0x44010020=0)+ 复用 GPIO11→UART0_TXD、GPIO10→UART0_RXD
(func-sel 0x440100C4 bits[15:8]=0 + 每脚 cfg bit6)。编译+repack 通过。

**待验证**:烧录后 `cat /dev/ttyUSB0` 按 RST,期待 `@@@@` + `<NX:1..5>` + NuttX 启动日志 + `nsh>`。

### 18.24 UART0 出字符了(乱码→修 soft-reset 破坏 bootloader 状态)

补上 UART0 时钟+引脚(18.23)后 CH340 冒出**一个乱码字节** `�`——说明 UART0 开始发送了,
但只出一个字节、不是一串乱码。SWD 读 bootloader 交接时(app 入口 0x020104b4)的 UART0 状态:
```
UART0 config(0x44820010) = 0x0000e119  (clk_div=225, TX_EN, 8bit)  <- 与 NuttX 相同
CLK_DIV_MODE1(0x44010020) = 0x00000033  (clkdiv_uart0=0, cksel_uart0=0=XTAL)
```
即**波特率配置与 bootloader 完全一致**(都 div=225/XTAL/26M → 115200)。既然配置相同却出乱码,
且只出一个字节 → 不是波特率问题, 而是 **NuttX 的 `bk7258_uart_config`/`bk7258_setup` 做了
`soft reset`, 把 bootloader 已工作的 UART0 TX 状态清掉了 → TX 不再排空 → putc 超时丢字符**。

**修复**:去掉对 UART0 的 soft reset;改为**读现有 config、只改数据位/波特率/使能 TX(+RX)**,
保留 bootloader 已工作的时钟/FIFO 状态(`bk7258_lowputc.c` 的 `bk7258_uart_config` 与
`bk7258_serial.c` 的 `bk7258_setup` 都改)。编译+repack 通过。

**待验证**:烧录后 `cat /dev/ttyUSB0`(115200)按 RST,期待可读的 `<NX:...>` + NuttX 日志 + `nsh>`。

### 18.25 ✅ 通关:NuttX NSH 在 BK7258 DevKit 跑通

烧录后 CH340(UART0, 115200)完整输出:
```
@@@@
<NX:1-uart>
<NX:2-wdt>
<NX:3-bss>
<NX:4-data>
<NX:5-fpu>
<NX:6-board>
<NX:7-nx_start>

NuttShell (NSH)
nsh>
```
**openvela/NuttX 作为 BK7258 CPU0 主核完整启动到交互式 NSH 提示符。移植成功。**

**CPU0 路线关键修复链(总结)**:
1. `__start` 删除对 DTCM(0x20000000)的 core-id 写(CPU0 冷启动 DTCM 未使能, 访问即 fault)。
2. `__start` 顺序:**先清 MSPLIM + 先开 FPU(CPACR)**, 再设 SP/VTOR。否则设 SP 后、清限制前的窗口
   一旦异常压栈 -> STKOF; 且压栈做 FPU 惰性保存而 FPU 未开 -> NOCP; 双故障 -> LOCKUP。
3. 链接脚本 `flash ORIGIN=0x02010000`(CPU0 XIP), `LEN=0x140000`; RAM 保持 0x28010000。
4. repack 把 nuttx.bin 放进 **app 分区(flash 0x11000)** 替换 ARMINO CP; 保留 bootloader + app1。
5. `bk7258_uart0_hwsetup`:使能 UART0 时钟(0x44010030 bit2)+ 时钟源 XTAL(0x44010020 bit10=0)+
   GPIO11→UART0_TXD/GPIO10→UART0_RXD 复用。
6. **去掉 UART0 的 soft-reset**:改为读现有 config、只改数据位/波特率/TX+RX 使能。soft-reset 会破坏
   bootloader 已工作的 UART0 TX 状态, 导致只冒一个乱码字节。
7. `bk7258_swd_route_cpu0`:NuttX 自己把 SWD 路由到 CPU0 + 复用 GPIO20/21, 替换 CP 后仍可 pyOCD 观测。

**调试方法学(SWD 决定性)**:OpenOCD 0.11 连不上(找不到 MEM-AP), 换 **pyOCD** 秒连。
`reset halt` 停在 bootloader; 用 `break <app_addr>`+`go` 到 app; 坐在断点上 step/go 不动需先 `rmbreak`;
`break+go+rmbreak+step` 逐段夹逼定位每一个 LOCKUP 的确切指令与 fault 类型(CFSR/HFSR)。
SWD 一上来就推翻了"AP 核没启动"的旧结论(其实一直跑到 idle), 并逐个清除 CPU0 的早期故障。

—— 至此 A 步(NuttX 上板 BK7258 出 NSH)完成。

### 18.26 修 NSH 输入(RX):FIFO 阈值太高,收到字符不触发中断

`nsh>` 出来后,picocom 里敲命令无回显/无响应。SWD 读 UART0(NuttX 运行中):
```
config(0x44820010)      = 0x0000e11b  -> RX_ENABLE(bit1)=1 ✓
fifo_status(0x44820018) = 0x00323c00  -> rx_fifo_count(bits15:8)=0x3c=60!, RD_READY=1
int_enable(0x44820020)  = 0x00000042  -> RX 中断位(bit1/bit6)已使能 ✓
```
即**输入字符确实收到了(RX FIFO 堆了 60 字节),但没人读走** → RX 中断没触发。

**根因**:18.24 为修 TX 去掉了 UART soft-reset,导致 `fifo_config` 沿用 bootloader 的**高 RX 阈值**
(为下载协议调的),交互式单字节输入达不到阈值 → `RX_NEED_READ` 不触发 → 字符堆积不被读。

**修复**(`bk7258_serial.c` `bk7258_setup`):显式写 `fifo_config(0x14)`——
RX 阈值设为 1(bits[15:8]=1)+ rx_stop_detect_time=2(bits[17:16]),保留 tx 阈值(读改写)。
这样任一按键就触发 RX 中断 → `bk7258_interrupt` → `uart_recvchars` → 读走并回显。编译+repack 通过。

**待验证**:烧录后 picocom(115200)敲 `help`/`uname -a`,应有回显与命令输出。

### 18.27 RX 中断真因:BK7258 有 SYS 级中断聚合器(NVIC 之前还有一道门)

设低 RX 阈值后,SWD 复读 UART0:`int_status(0x44820024)=0x42` —— RX 中断在 UART 侧**已 assert**
(RX_NEED_READ+RX_FINISH),但 int_status 一直是 0x42 不被清 → 处理函数从没运行 → NVIC 没收到该中断。
而 rx_fifo_count 涨到 0x65。

**根因**:BK7258 在 NVIC 之前还有一层 **SYS 级中断聚合器**(每 CPU 一个):
`SYS_CPU0_INT_0_31_EN @ 0x44010080`(源 0-31)、`...32_63 @ 0x44010084`(源 32-63),
**每 bit = 中断源号**(bit4=UART, bit3=TIMER0…)。外设中断必须在**这里 + NVIC 都使能**才会送达 CPU。
NuttX 只使能了 NVIC,漏了 SYS 聚合器 → UART 中断被这道门挡住。
(OS 仍能跑是因为 SysTick 是内核异常,不经这道门。)

**修复**(`bk7258_irq.c`):`up_enable_irq`/`up_disable_irq` 对外设 IRQ 额外设置/清除 SYS 聚合器对应位
(源<32 → 0x44010080,32-63 → 0x44010084;bit=源号=irq-EXTINT)。这是通用修复,UART/定时器等
所有外设中断都受益。编译+repack 通过。

**待验证**:烧录后 picocom 敲命令应有回显 + NSH 命令输出 → 完整交互式 NSH。

### 18.28 ✅✅ 完整交互式 NSH 跑通(RX+TX 双向)

补上 SYS 中断聚合器使能(18.27)后, picocom(115200)实测:
```
nsh> help          -> 列出完整命令表(ls/cat/echo/mount/uname/... )
nsh> uname -a      -> NuttX 0.0.0 dd92bcf4257-dirty Jul 28 2026 arm bk7258-devkit
nsh> free          -> free: command not found  (可选命令未编入, 非故障; shell 正常解析响应)
```
**输入(RX)、输出(TX)、命令解析执行全部正常。openvela/NuttX 在 BK7258 DevKit 上跑通完整的
双向交互式 NSH 控制台。** 比赛"新硬件平台适配"目标完整达成。

RX 修复链小结:UART0 时钟+引脚(18.23) -> 去 soft-reset 修 TX(18.24) -> SWD 确认字符进了
RX FIFO 但没被读(18.26) -> 设低 RX FIFO 阈值(18.26) -> 发现 SYS 中断聚合器这道门、在
up_enable_irq 里补上(18.27) -> RX 中断触发、回显正常(18.28)。

后续可选:在 defconfig 使能更多 NSH 命令(free/ps 等)、清理临时诊断(@@@@/<NX:> 标记、UART1 双写、
SWD 自路由)出干净版、点屏(双 GC9D01)。

---
## 19. 修改记录（干净版清理 / Change Log）

> 从"诊断版"清理为"提交版"的逐条改动。每条用**彩色方块**标记类别(GitHub 可见),
> 并用 HTML 上色(本地 VS Code 预览更醒目)。诊断版整套已备份至
> `~/bk7258_port_debug_backup_20260801/`(见其 `BACKUP_README.md`)。

**颜色图例**

| 标记 | 类别 | 含义 |
| ---- | ---- | ---- |
| 🟢 | <span style="color:#2ea043">**新增**</span> | 新增文件/函数/配置 |
| 🔴 | <span style="color:#d1242f">**删除**</span> | 移除诊断代码/临时探针 |
| 🟡 | <span style="color:#bf8700">**修改**</span> | 调整现有逻辑/条件 |
| 🔵 | <span style="color:#0969da">**说明**</span> | 备份/验证/提交等非代码动作 |

### 19.1 清理批次 · 2026-08-01

- 🔵 <span style="color:#0969da">**说明**</span> 诊断版整套备份到 `~/bk7258_port_debug_backup_20260801/`
  (chip 层 + 板级 + `DEBUG_JOURNAL` + `openocd/`;含还原的诊断版 `bk7258_start.c`/`bk7258_lowputc.c`)。
- 🔴 <span style="color:#d1242f">**删除**</span> `bk7258_start.c` · `__start`:移除直写 UART0/UART1 fifo_port 的
  `@@@@` / `####` 最小生命体征汇编。
- 🟡 <span style="color:#bf8700">**修改**</span> `bk7258_start.c`:`showprogress` 由"无条件输出"改回
  `#ifdef CONFIG_DEBUG_FEATURES` 条件(正常构建为空操作)。
- 🔴 <span style="color:#d1242f">**删除**</span> `bk7258_start.c`:移除 `bk_puts` 函数与 `<NX:1..7>` 分阶段进度标记,
  `bk7258_cstart` 改用 `showprogress('A'..'F')` 里程碑。
- 🔴 <span style="color:#d1242f">**删除**</span> `bk7258_lowputc.c`:`arm_lowputc` 去掉 UART1 双写,只写 UART0。
- 🔴 <span style="color:#d1242f">**删除**</span> `bk7258_lowputc.c`:移除 `bk7258_uart1_hwsetup`(UART1 诊断口)
  及 `bk7258_swd_route_cpu0`(SWD 自路由)与相关寄存器宏。
- 🟡 <span style="color:#bf8700">**修改**</span> `bk7258_lowputc.c`:`bk7258_lowsetup` 精简为仅
  `bk7258_uart0_hwsetup()` + `bk7258_uart_config(UART0)`。

- 🟡 <span style="color:#bf8700">**修改**</span> `chip/bk7258/Kconfig`:`CONFIG_BK7258_DEBUG_FAULT_SPIN`
  默认 `y` → `n`(干净版默认关闭故障自旋, 恢复 NuttX 默认故障处理; MARK 依赖它, 不变)。
- 🟢 <span style="color:#2ea043">**新增**</span> `nsh/defconfig`:`CONFIG_FS_PROCFS=y`(提供 `/proc`,
  自动启用 `ps`; bringup 已配套挂载 `/proc`)。
- 🟢 <span style="color:#2ea043">**新增**</span> `nsh/defconfig`:放开常用命令
  `date`、`mb`/`mh`/`mw`(内存查看); `free` 本就启用。

- 🔴 <span style="color:#d1242f">**删除**</span> `nsh/defconfig`:移除全部散文注释与空行。
  **原因(重要)**:openvela 的 CMake 预解析器 `nuttx_export_kconfig`(`nuttx/CMakeLists.txt`
  第 236/294 行)会逐行读 defconfig, 含中文/标点的散文注释会让 `string(REGEX MATCH ...)`
  拿到空参数而报错 `REGEX ... needs at least 5 arguments`, **导致全量(clean)构建配置失败**。
  之前增量构建复用了缓存 `.config` 没重解析 defconfig, 侥幸没暴露 —— 属真实潜在 bug,
  评委做干净构建即会命中。
- 🟡 <span style="color:#bf8700">**修改**</span> `nsh/defconfig`:用官方 `savedefconfig` 规范化为标准最小格式
  (排序 + 标准头 + 仅保留非默认项)。`UART0_SERIAL_CONSOLE`/`BK7258_UART0`/`UART0_BAUD`
  经确认为 no-op(依赖未满足, kconfig 本就丢弃; 控制台由 `STANDARD_SERIAL` + 自定义
  `bk7258_serial.c` 硬编码 UART0 提供), 产物字节数三版一致(179324 B)。
- 🔵 <span style="color:#0969da">**说明**</span> 干净构建验证通过:`build.sh ... --cmake` 0 报错、
  `build completed successfully`;`.config` 确认 `ARCH_FPU`/`STANDARD_SERIAL`/`FS_PROCFS`/`SYSTEM_NSH`
  齐全, `free`/`ps`/`date`/`mb`/`mh`/`mw` 全部启用。`repack.py` 产出
  `all-app-nuttx.bin`(2065126 B), nuttx.bin 179324 B。

- 🔵 <span style="color:#0969da">**说明**</span> 真机烧录验证通过(picocom 115200):`help` 打印完整命令表、
  `uname -a`、`free`(输出内存用量, 不再 command not found)、`ps`(列任务)、`date` 均正常。
  干净版功能完整。
  > 备注:连上串口后**第一条**命令偶发被插入一个杂散字节(表现为 `nsh: ◆help: command not found`),
  > 重敲即恢复。系开端口瞬间的一次性 RX 线路杂散/残留字节, 非代码缺陷。可选缓解:连上先敲回车,
  > 或在 UART setup 使能前 flush 一次 RX FIFO。

### 19.2 提交前修复 · 2026-08-01

- 🟡 <span style="color:#bf8700">**修改**</span> `nuttx/arch/arm/Kconfig`:修正 `source "arch/arm/src/bk7258/Kconfig"`
  的**放置 bug** —— 原来被误插进 `if ARCH_CHIP_NRF53` 块内, 导致构建 bk7258 时 chip
  Kconfig **从不被 source**(所有 `BK7258_*` 符号未定义)。现移到独立的
  `if ARCH_CHIP_BK7258 ... endif` 块。
  > 端口此前能跑属"侥幸": 自定义 `bk7258_serial.c` 硬编码 UART0、`bk7258_lowputc.c`
  > 用 `#ifndef` 回退到 UART0, 不依赖这些符号。评委做严格审阅会发现此 bug。
- 🟡 <span style="color:#bf8700">**修改**</span> `chip/bk7258/Kconfig`:把 UART 默认值与实现对齐 ——
  `BK7258_UART0` 默认 `y`(控制台/CH340)、`BK7258_UART1` 默认 `n`;
  `BK7258_CONSOLE_UART_BASE` 默认 `0x44820000`(UART0)。删除误导性的
  `select UART1_SERIALDRIVER`/`ARCH_HAVE_SERIAL_TERMIOS`(自定义驱动不走标准框架)。
- 🔵 <span style="color:#0969da">**说明**</span> 修复后干净构建 0 报错、`.config` 自洽
  (`BK7258_UART0=y`, `CONSOLE_UART_BASE=0x44820000`, 无 `UART1_SERIALDRIVER`),
  命令仍全启用, 产物字节数不变(179324 B, 与真机验证版一致 → 功能等价)。
  `savedefconfig` 确认 defconfig 已是稳定规范最小集。repack 刷新烧录镜像。

### 19.3 PR checkpatch 修复 · 2026-08-01

PR 到 `open-vela/nuttx` 后, CI 的 `checkpatch`(nxstyle)对芯片层文件报大量风格错误
(超长行 = 中文注释, 少量 brace/空行/括号/注释框)。选用**方案 B: 注释全改英文**(贴合上游)。

- 🟡 <span style="color:#bf8700">**修改**</span> chip 层 9 个文件注释**全部英文化**并压到 78 列内:
  `bk7258_start.c`/`bk7258_lowputc.{c,h}`/`bk7258_serial.c`/`bk7258_irq.c`/`chip.h`/
  `hardware/bk7258_uart.h`/`include/bk7258/{irq,chip}.h`。
- 🟡 <span style="color:#bf8700">**修改**</span> `bk7258_start.c`:`__start` 内大段说明移到 asm 块外
  (消除 asm 内多行注释的对齐告警); 两个空增量 `for(...; )` 循环改为增量写进头部(消除
  `Space precedes right parenthesis`)。
- 🟡 <span style="color:#bf8700">**修改**</span> `bk7258_serial.c`/`bk7258_irq.c`:把内嵌 `{ }` 块的局部变量
  提到函数顶部声明, 消除 brace 对齐告警。
- 🔴 <span style="color:#d1242f">**删除**</span> `bk7258_lowputc.c`:`arm_lowputc` **补删**残留的 UART1 双写
  (task2 当时漏改), 现只写 UART0(`CONSOLE_BASE`)。
- 🟡 <span style="color:#bf8700">**修改**</span> `hardware/bk7258_uart.h`:INT_ENABLE banner 对齐到 78 列。
- 🔵 <span style="color:#0969da">**说明**</span> 全部文件 `checkpatch -f` **0 error**; 干净构建成功,
  产物 179308 B(比前一版小 16 字节 = 去掉 UART1 多余写, 实质改进); repack 已刷新镜像。

> 待续:🔵 `git commit --amend` + force push 更新 PR;等组委会 review 合入。板级部分进专属仓
> `contest2026_098_zhanshangxingguang`。

### 19.4 板级英文化 + 过 checkpatch/cmake-format · 2026-08-01

为板级也能干净进专属仓/上游, 把板级**源码与配置**里的中文全部英文化并过风格检查
(`DEBUG_JOURNAL_zh-cn.md`/`README_zh-cn.md`/`openocd/README_zh-cn.md` 等**中文文档保留**)。

- 🟡 <span style="color:#bf8700">**修改**</span> 英文化: `include/board.h`、`scripts/ld.script`、
  `src/{bk7258_appinit,bk7258_boardinitialize,bk7258_bringup}.c`、`src/bk7258-devkit.h`、
  `src/{CMakeLists.txt,Make.defs}`、`Kconfig`、`.gitignore`、`openocd/bk7258.cfg`、`tools/repack.py`。
- 🟡 <span style="color:#bf8700">**修改**</span> `include/board.h`: 压 78 列、去掉末尾独立占位注释
  (消除 "Missing blank line after comment")。
- 🟡 <span style="color:#bf8700">**修改**</span> `src/CMakeLists.txt`: `set(SRCS ...)` 与注释按 cmake-format
  规范化(单行 TODO, 避免注释被 cmake-format 揉成一行)。
- 🔵 <span style="color:#0969da">**说明**</span> 校验: 板级 C/H `checkpatch -f` 0 error、两个 CMakeLists
  `cmake-format --check` 通过、源码/配置无中文、干净构建成功(nuttx.bin 179300 B)。

> 待续:🔵 板级进专属仓 `contest2026_098_zhanshangxingguang`(英文名 + Signed-off-by + logs/)。

---

## 20. M2 显示:GC9D01 双眼圆屏 bring-up

> 时间线:2026-08-08 ~ 08-11。目标从"点亮一块 160×160 圆屏"推进到"双眼表情动画"。

### 20.1 硬件事实确认(先勘察再动手)

从原理图 + ARMino SDK 交叉确认,避免猜引脚:

| 项 | 结论 | 出处 |
|---|---|---|
| 屏型号 | **GC9D01,160×160,SPI** | ARMino `lcd_spi_gc9d01.c` 内 `lcd_device_gc9d01{.width=160,.height=160,.type=LCD_TYPE_SPI}` |
| 双屏方案 | LCD1 + LCD2 各一路,12pin 座 CN5;**双屏方案无触摸**(触摸只在单屏 24pin 方案上) | 原理图 LCD 页 |
| 引脚 | SCLK=GPIO_2 / CS=GPIO_3 / MOSI=GPIO_4 / DC=GPIO_5(借 QSPI1 脚)、RST=GPIO_29、BL=GPIO_25(经 Q3) | 原理图 + ARMino `gpio_map.h` |
| 命令/数据 | GC9D01 用**独立 DC 脚**(不是 9-bit SPI):DC=L 命令,DC=H 数据;SPI mode 0,MSB first | GC9D01 datasheet + ARMino 驱动 |

- 🔵 <span style="color:#0969da">**说明**</span> openvela 自带 `nuttx/drivers/lcd/gc9a01.c`(同族),初期作为初始化序列的对照基底。

### 20.2 决策:先 bit-bang,不碰硬件 QSPI

- 🔵 <span style="color:#0969da">**说明**</span> BK7258 的 QSPI 控制器在 openvela 侧尚无驱动,若先做控制器会把
  "点屏"这件事的变量堆到两个(控制器 + 屏)。故**先用 bit-bang SPI 打通屏**,把引脚设为普通 GPIO 输出,
  只验证屏本身与初始化序列。控制器/DMA 留作后续性能优化。
- 🔵 <span style="color:#0969da">**说明**</span> 代价:帧率受限,做动画会吃紧(见 20.5 的优化)。

### 20.3 分阶段调试命令(本项目最有效的排障模式)

- 🟢 <span style="color:#1a7f37">**新增**</span> `app/lcdtest/` 与板级 `src/bk7258_gc9d01.c`,通过
  `CONFIG_EXAMPLES_LCDTEST` + `CONFIG_BUILTIN` + `CONFIG_NSH_BUILTIN_APPS` 暴露为 NSH 内建命令。
- 🔵 <span style="color:#0969da">**说明**</span> **不在开机路径调用 bring-up**:bit-bang 初始化耗时,
  放 `bk7258_bringup.c` 会阻塞启动、拖慢每次迭代。改为按需触发。

命令分层设计(后续外设沿用此模式):

```
lcdtest            分阶段:A=背光  B=初始化  C=红色方块(最小可见输出)
lcdtest go         一步到位的可靠上屏流程(生产路径)
lcdtest scan       GPIO 扫描,定位未知的 LCD 电源使能脚
lcdtest pwr lo hi  在给定 GPIO 区间内二分,缩小使能脚范围
```

- 🔵 <span style="color:#0969da">**说明**</span> `scan`/`pwr` 是为解决"背光亮但屏无内容"而加的:
  当怀疑存在未知的电源/使能脚时,用扫描与二分替代逐个试错,显著缩短定位时间。
  **经验**:这类"未知使能脚"在第三方开发板上很常见,值得把扫描能力做成常备工具。

### 20.4 表情动画(从静态方块到会发光的眼睛)

- 🟢 <span style="color:#1a7f37">**新增**</span> `lcdtest emo` / `lcdtest anim`:发光式 emoji 眼动画。
- 🟢 <span style="color:#1a7f37">**新增**</span> `lcdtest oeye`:接近原厂 demo 观感的"官方风格眼"。
- 🔵 <span style="color:#0969da">**说明**</span> 之所以要对齐原厂观感:原厂 demo 已证明 160×160 画眼睛效果好,
  以其为参照可先排除"是不是分辨率不够"的疑虑,把问题收敛到渲染实现上。

### 20.5 bit-bang 提速:缓存 GPIO 配置寄存器

- 🟡 <span style="color:#bf8700">**修改**</span> `src/bk7258_gc9d01.c`:引入 `gpio_cache_t`,
  在 setup 阶段预先算出并缓存每个引脚的 CFG 寄存器地址与"OUTPUT=0 / OUTPUT=1"两个基值。

```c
typedef struct
{
  uintptr_t addr;    /* BK7258_GPIO_CFG(pin) address */
  uint32_t  base_lo; /* CFG with OUTPUT bit cleared */
  uint32_t  base_hi; /* CFG with OUTPUT bit set     */
} gpio_cache_t;
```

- 🔵 <span style="color:#0969da">**说明**</span> 原实现每次 `gpio_write()` 都要**读-改-写**;
  改为置位只需一次 `putreg32(base_hi)`。bit-bang 场景下每帧有大量位翻转,这一项对帧率影响明显。
- 🔵 <span style="color:#0969da">**说明**</span> 仍属过渡方案。要支撑流畅双眼动画,**最终需换硬件 SPI/QSPI + DMA**。

### 20.6 当前状态与遗留

- ✅ 单屏点亮、初始化序列可靠、表情动画可跑
- 🟡 **双屏尚未同时驱动**(第二块屏的片选/复位与刷新调度未实现)
- 🟡 未接 `LCD_FRAMEBUFFER` / LVGL
- 🔴 **阻塞项**:`CONFIG_MM_REGIONS` 仍为默认 1,**PSRAM(`0x60000000`) 未纳入堆**。
  双屏双缓冲与后续摄像头帧缓冲都需要 PSRAM(GC2145 640×480 YUYV 单帧 ≈ 614KB,
  远超 AP 侧 336KB SRAM)。→ 下一步优先打通 PSRAM 初始化 + `mm_addregion()`。

> 待续:🔵 双屏并行刷新、framebuffer/LVGL 接入、硬件 SPI+DMA 替换 bit-bang。

---

## 21. 双麦音频采集与声源定位(寻声)

> 时间线:2026-08-10 ~ 08-11。硬件前提:原装单麦已拔除,换装 **2 颗同型号咪头**(6027 ECM),
> 分别接 M1 / M2 座(原理图 `CN7`/`CN9`,`HC-1.25-2PLT` 1.25mm)。

### 21.1 为什么必须换成两颗同型号

- 🔵 <span style="color:#0969da">**说明**</span> 出厂只装 1 颗麦(另一座空置),单麦无法判方位。
- 🔵 <span style="color:#0969da">**说明**</span> 且**两麦必须同型号**:互相关测向依赖两路的相位与灵敏度一致性,
  混用不同型号会引入固定相位偏差,直接劣化 TDOA 结果。故连原装那颗一并换掉,凑同款对。
- 🔵 <span style="color:#0969da">**说明**</span> 麦克风为**带线外接**,间距由结构决定而非 PCB 固定
  (座子间距仅 ~9.4mm,远不够)。设计目标:装到外壳两侧"耳朵"位,**间距 ≥60mm**。

### 21.2 ⚠️ 根因级踩坑:模拟寄存器不是普通 MMIO

这是本节最重要的发现,**漏掉会导致"代码看着全对但 ADC FIFO 恒空"**。

- 🔴 <span style="color:#d1242f">**坑**</span> BK7258 的模拟寄存器(`ana_reg*`)写操作**要经一条串行总线下发**,
  **每次写后必须轮询完成标志**,否则下一次写会把前一次挤掉 → 模拟前端始终配不上。
- 🟢 <span style="color:#1a7f37">**新增**</span> `ana_write()` / `ana_setbit()` 封装,内部强制轮询写完成。

```c
/* WRONG —— 前一次还没下发完就被覆盖 */
aud_putreg(v1, ANA_REG_X);
aud_putreg(v2, ANA_REG_Y);

/* RIGHT —— 每次写后轮询串行总线 */
ana_write(ANA_REG_X, v1);
ana_write(ANA_REG_Y, v2);
```

- 🔵 <span style="color:#0969da">**说明**</span> 症状极具误导性:寄存器**回读像是写进去了**,但模拟通路无效、
  FIFO 无数据。若不知道这条,很容易误判为"时钟没开"或"引脚错"而反复绕圈。
- 🔵 <span style="color:#0969da">**说明**</span> 出处:ARMino `aud_common_driver.c` + `aud_adc_driver.c` + `sys_ll.h`
  (Beken,Apache-2.0),移植时已在文件头保留出处。

### 21.3 实现内容

- 🟢 <span style="color:#1a7f37">**新增**</span> `src/bk7258_audio.{c,h}` —— 模拟音频 ADC **双通道**
  (L=M1,R=M2)采集,**寄存器级、polling,无 DMA 无中断**。
  - `audio_init()` / `audio_deinit()`:含 **APLL 锁定等待**
  - `audio_capture(n)`:抓 n 个采样
  - `compute_rms()`:RMS 能量
  - `bk7258_mic_energy(n)`:能量查询
  - `bk7258_mic_set_quiet(bool)`:静音门控
- 🔵 <span style="color:#0969da">**说明**</span> 先用 polling 是刻意的:bring-up 阶段变量越少越好,
  先证明"两路都能拿到波形"。**48kHz 双通道连续采集最终需上 DMA**,否则 CPU 占用过高。

### 21.4 声源定位(互相关 TDOA)

- 🟢 <span style="color:#1a7f37">**新增**</span> `mic_locate_process(n, *out_tau_q8)` /
  `bk7258_mic_locate(*out_tau_q8)`:对两路做**互相关**求时延差,输出 **Q8 定点**的 τ。
- 🟢 <span style="color:#1a7f37">**新增**</span> 关键参数:
  - `CORR_LAG_MAX 32` —— lag 搜索范围 `[-32, +32]` 采样
  - `RMS_GATE 50` —— **静音门限**:两路能量均低于该值时直接判为"居中",避免静音下输出随机方位
- 🔵 <span style="color:#0969da">**说明**</span> 只解**左右单轴**(双麦固有限制,存在前后镜像模糊)。
  对"左右双眼设备把视线转向说话人"这一用途**恰好够用**。
- 🟢 <span style="color:#1a7f37">**新增**</span> `bk7258_mic_main()` 暴露为命令,并接入 `lcdtest mic`,
  可把测向结果直接驱动屏上眼球偏移,形成"听到声音→眼睛转过去"的闭环。

### 21.5 当前状态与遗留

- ✅ 双通道采集通、RMS 正常、互相关测向可输出 τ
- 🟡 标注为 **WIP**:间距/角度标定未做(需外壳定位后按实际 60~65mm 间距标定)
- 🟡 采样率与分辨力待优化:**建议取 48kHz**(16kHz 下 60mm 间距仅约 2.8 个采样,
  分辨力不足);并加**抛物线亚采样插值**提升精度
- 🟡 polling → **DMA** 待改造
- 🟡 与喇叭同时工作时需验证回声/啸叫(结构上应隔离麦腔与喇叭腔;必要时启用芯片 AEC)

> 待续:🔵 外壳定位后标定间距 → 48kHz + 亚采样插值 → DMA 化 → 与表情引擎联动做"听觉引导视觉"。

---

## 22. PSRAM bring-up（S1: 上电 + ID 识别 + 数据通路验证）

日期: 2026-08-13

### 22.1 背景与阻塞项

AP 侧只有 336KB SRAM（0x28010000 ~ +336KB），双屏双缓冲 + 摄像头帧缓冲
（GC2145 640×480 YUYV 单帧 ~614KB）都放不下。PSRAM（0x60000000）完全未初始化，
是当前头号阻塞项。

### 22.2 参考移植来源

- 🟢 <span style="color:#1a7f37">**新增**</span> Armino SDK（Apache-2.0）:
  `psram_driver.c`（init 序列）、`psram_hal.c`（寄存器级操作）、
  `sys_psram_driver.c`（电源树）、`psram_hal.h`（ID 常量/模式值）
- 🔵 <span style="color:#0969da">**说明**</span> 去掉了 rtos_mutex / semaphore /
  psram_task / env 读写等 OS 依赖，只取寄存器序列、时序等待、ID 判定
- 芯片 ID → 容量: `0x8D09`=8MB (APS6408L)、`0x8D08`=16MB (APS128XXO)、
  `0x1C8F`=4MB (W955D8MKY)

### 22.3 实现内容

- 🟢 <span style="color:#1a7f37">**新增**</span> 芯片级驱动:
  `nuttx/arch/arm/src/bk7258/bk7258_psram.c` +
  `hardware/bk7258_psram.h`（寄存器定义）+
  `bk7258_psram.h`（公共头文件）
- 🟢 <span style="color:#1a7f37">**新增**</span> NSH app:
  `apps/examples/psram/`（`psram id` / `psram probe`）
- 🟡 <span style="color:#9a6700">**修改**</span> Kconfig:
  新增 `CONFIG_BK7258_PSRAM`（default n，显式在 defconfig 里 =y）
- 🟡 <span style="color:#9a6700">**修改**</span> defconfig:
  新增 `CONFIG_BK7258_PSRAM=y` + `CONFIG_EXAMPLES_PSRAM=y`

### 22.4 init 序列（寄存器级，修正版）

```
 1) ana_reg13 @ 0x44010134: 逐 bit ana_rmw 设 psldo_swb=1, vpsramsel=3
    → 1.95V（必须走 ana_write，禁止裸 putreg —— 见 22.8 踩坑）
 2) ana_rmw 设 enpsram=1 → LDO 使能，等 1ms
 3) cpu_power_sleep_wakeup @ 0x44010040: 清 pwd_ahbp (bit5)
    → AHB PSRAM 电源域上电
 4) cpu_clk_div_mode2 @ 0x44010024: cksel=0, ckdiv=1 → 80MHz init 时钟
 5) cpu_device_clk_enable @ 0x44010030: 置 psram_cken (bit19)
    → PSRAM 外设时钟使能
 6) REG2 @ 0x46080008: 读改写置 bit[0]=1 (sf_reset) + bit[1]=1 (bypass)
 7) 按优先级尝试三种 PSRAM（每种完整配置 MR 寄存器）:
    - APS6408L:  MODE6 → drv=0x380 → reset → 读 ID → 配 MR0/MR4
    - APS128XXO: MODE7 → drv=0x380 → reset → 读 ID → 配 MR0/MR4/MR8
    - W955D8MKY: MODE8 → drv=0x292 → reset → 读 ID → 配驱动
 8) 等 1ms → 切 120MHz 默认时钟（cksel=1, ckdiv=1）
```

### 22.5 数据通路验证

- 🔴 <span style="color:#d1242f">**坑**</span> ID 读取走的是命令寄存器通路（REG9/REG_A/REG_B），
  证明不了数据窗口 0x60000000 能不能用 — 那是 AXI 内存映射通路，另一条路
- 🟢 <span style="color:#1a7f37">**新增**</span> `psram probe` 在 init 成功后额外做:
  对 0x60000000 写 0xdeadbeef → 读回比对 → 恢复原值
  通过则说明 AXI 数据通路可用

### 22.6 设计决策

- 🔵 <span style="color:#0969da">**说明**</span> 驱动放芯片级（`nuttx/arch/arm/src/bk7258/`），
  非板级。理由: PSRAM 控制器是 SoC 片上外设（寄存器 @ 0x46080000），
  外置的只是 PSRAM 颗粒本身。先例: ESP32 spiram 在 arch/xtensa/src/esp32/、
  STM32 FSMC/FMC 在 arch/arm/src/stm32/
- 🔵 <span style="color:#0969da">**说明**</span> `CONFIG_BK7258_PSRAM` default n:
  PSRAM 上电序列是最可能挂核的一步，留一个能快速关掉的开关
- 🔵 <span style="color:#0969da">**说明**</span> 不在开机路径自动初始化，由 `psram probe` 命令触发。
  万一序列把核挂住，重启仍能进 NSH，不至于变砖
- 🔵 <span style="color:#0969da">**说明**</span> 未配置 MPU / D-cache（defconfig 无
  CONFIG_ARMV8M_DCACHE），cache 一致性暂不处理

### 22.7 当前状态与验收标准

- ✅ checkpatch 0 error（所有 5 个文件）
- ✅ 构建通过（flash 204028B, sram 38084B）
- ⬜ 待实测: `psram id` 打印 chip ID 与容量
- ⬜ 待实测: `psram probe` 数据通路 0xdeadbeef 读回验证

### 22.8 踩坑记录：模拟寄存器写协议（再犯）

- 🔴 <span style="color:#d1242f">**坑**</span> **模拟寄存器裸写** — S1 初版用
  `psram_putreg(PSRAM_VOLTAGE_LDO_EN, SYS_ANA_REG13)` 一次性写整个 ana_reg13。
  这违反了 `bk7258-armino-port/SKILL.md` 步骤 4 第 ④ 条的明确规则。
  两个错误:
  (a) 未轮询 SPI 总线完成（`ANA_SPI_STATE_REG @ 0x440100E8`），
      写操作可能未下发到模拟前端
  (b) 整寄存器盲写，破坏了 ana_reg13 内其它模拟块的控制位
- 🔴 <span style="color:#d1242f">**坑**</span> **漏掉 AHB 电源域和时钟门控** —
  Armino `psram_hal_power_clk_enable()` 的 8 步序列里有两步被遗漏:
  `bk_pm_module_vote_power_ctrl(AHBP_PSRAM, ON)`（清 `pwd_ahbp` bit5 @ 0x44010040）
  和 `sys_drv_dev_clk_pwr_up(CLK_PWR_ID_PSRAM)`（置 `psram_cken` bit19 @ 0x44010030）。
  不做这两步，PSRAM 控制器的总线时钟不通。
- 🔴 <span style="color:#d1242f">**坑**</span> **漏掉 REG2 bypass 位** —
  Armino `psram_hal_config_init()` 在 sf_reset(1) 之后还置了 bit[1] (psram bypass)。
  初版漏掉。
- 🔴 <span style="color:#d1242f">**坑**</span> **只读 ID 没配 MR 寄存器** —
  初版 `psram_try_detect()` 只做了 "写 mode → reset → 读 ID"，没有配置
  PSRAM 颗粒自身的模式寄存器（MR0/MR4/MR8）。
  后果: ID 可能读对，但 latency/drive/burst 没配，0x60000000 数据通路大概率不通。
- 🟢 <span style="color:#1a7f37">**新增**</span> 已修正所有问题，并在
  `bk7258-armino-port/SKILL.md` 的 ④ 条强化了硬规则:
  "凡地址落在 0x440101xx 的 ana_reg*，一律走 ana_write()，禁止裸 putreg"
- 🟢 <span style="color:#1a7f37">**新增**</span> NSH 命令入口 `bk7258_psram_main()`
  从 arch 层移到了 `apps/examples/psram/`，arch 层只保留纯驱动 API

> 待续:🟡 实测回贴串口输出 → 根据 ID 修正 board.h 的 BOARD_PSRAM_SIZE →
> S2 读写验证与容量实测 → S3 堆集成方案确认。

### 22.9 S1 实测通过 + 踩坑：repack 打包旧产物

- 🟢 <span style="color:#1a7f37">**新增**</span> S1 实测输出:
  ```
  nsh> psram id
  psram: init OK, ID=0x8d08, size=16384 KB
  psram: chip ID = 0x8d08
  psram: size    = 16384 KB (16 MB)

  nsh> psram probe
  psram: init OK, ID=0x8d08, size=16384 KB
  psram: data path OK (0x60000000 verified)
  ```
- 🟢 <span style="color:#1a7f37">**新增**</span> ID=0x8d08 → APS128XXO → 16MB。
  证实 board.h 里的 8MB 是错的，已修正为 16MB。
  data path OK 也证实颗粒 MR 配置那一步确实是必需的。
- 🔴 <span style="color:#d1242f">**坑**</span> **repack.py 打包旧产物** —
  首次烧录后 psram 命令在 NSH 里不存在。根因不是代码:
  `repack.py:37` 硬编码 `NUTTX_BIN = cmake_out/bk7258-devkit_nsh/nuttx.bin`，
  但改用专属仓 defconfig 后构建输出到 `cmake_out/contest2026_098_board_nsh/nuttx.bin`。
  repack 打的是 08-11 的旧 nuttx.bin，烧进去的固件里只有 lcdtest。
  现象极易误判成"代码没编进去"。
- 🟢 <span style="color:#1a7f37">**新增**</span> `repack.py` 已参数化:
  `--nuttx-bin <path>` / `NUTTX_BIN` 环境变量 / 自动推导 mtime 最新 / 兜底默认。
  打包前打印路径+大小+mtime，超过 24 小时的文件会打印显著警告。
  已同步加固 `bk7258-nuttx-bringup/SKILL.md`。

> 待续:🟡 S2 读写验证与容量实测 → S3 堆集成方案确认。

### 22.10 S2 实测通过：16MB 零错误 + 带宽发现

- 🟢 <span style="color:#1a7f37">**新增**</span> S2 实测输出:
  ```
  nsh> psram width
  psram: init OK, ID=0x8d08, size=16384 KB
    32-bit: OK
    16-bit: OK (RGB565 framebuffer safe)
    8-bit:  OK
  psram width: all widths OK (8/16/32-bit)

  nsh> psram alias
  psram: init OK, ID=0x8d08, size=16384 KB
    wrote 0x11111111 @ 0x60400000 (4 MB)
    wrote 0x22222222 @ 0x60800000 (8 MB)
    wrote 0x33333333 @ 0x60fff000 (16MB-4K)
  psram alias: OK — no aliasing detected (16 MB usable)

  nsh> psram test 1
  psram: init OK, ID=0x8d08, size=16384 KB
  psram test: range 0x60000000 - 0x600fffff (1024 KB, destructive)
  psram test: 262144 words, 0 errors, 1843.2 / 1865.5 KB/s (wr/rd), 1.10 s total

  nsh> psram test
  psram: init OK, ID=0x8d08, size=16384 KB
  psram test: range 0x60000000 - 0x60ffffff (16384 KB, destructive)
  psram test: 4194304 words, 0 errors, 1768.3 / 1802.1 KB/s (wr/rd), 18.24 s total
  ```
- 🟢 <span style="color:#1a7f37">**新增**</span> 三个关键疑虑全部排除:
  1. 窗口是否真映射 16MB → 是，无别名
  2. 半字是否可用 → 是，RGB565 safe
  3. 有无位错 → 4M words 零错误
- 🟡 <span style="color:#d4a017">**发现**</span> **CPU 逐字带宽仅 ~1.8 MB/s**。
  原因: BK7258 无 D-cache、无 burst 传输，每次 32-bit 读写都走完整
  PSRAM 总线事务（命令+地址+数据+等待），延迟被吃满。
  实测值: 写 1768 KB/s，读 1802 KB/s（16MB 全量测试）。

  > ⛔ **此结论已于 2026-08-20 被推翻，见 §25。**
  > 1.8 MB/s 不是 PSRAM 的能力上限，而是**当时 `-O0` 编译的搬运循环**的上限。
  > 同一个 `psram fbtest` 在 `-Os` 下实测 **写 20466 KB/s、读 12280 KB/s**
  > （约 11× / 6.8×）。上面「延迟被吃满」的归因是错的 —— 真正吃满的是
  > 每次循环迭代那一堆栈访问指令。
  > **本节下面基于 1.8 MB/s 做出的架构决策需要重新审视。**
- 🔴 <span style="color:#d1242f">**架构决策**</span> 基于带宽实测:
  - **LCD 帧缓冲留在 SRAM** — 双眼双缓冲 200KB，放得进 336KB SRAM，
    高频刷新需要快速访问
  - **PSRAM 专供摄像头大缓冲** — GC2145 640×480 YUYV 单帧 614KB，
    必须 PSRAM，大块低频访问对带宽不敏感
  - 结论: 不把帧缓冲迁到 PSRAM，改为 S4 用 psram_malloc 申请摄像头缓冲
- 🟢 <span style="color:#1a7f37">**新增**</span> 带宽显示 bug 修复:
  旧代码 `mbps = size * TICK_PER_SEC / elapsed / (1024*1024)` 两个问题:
  ① 写+读混合计时，实际带宽被稀释一倍
  ② 整数除法截断导致 ~0 MB/s 显示
  修复: 分别计时写/读，用 KB/s + 1 位小数

> 待续:🟡 S3 堆集成（方案 B: 独立 PSRAM 堆）→ S4 摄像头缓冲验证。

### 22.11 S3 实测通过：独立 PSRAM 堆可用

- 🟢 <span style="color:#1a7f37">**新增**</span> S3 实测输出:
  ```
  nsh> psram heap
  psram: init OK, ID=0x8d08, size=16384 KB
  PSRAM heap status:
    arena (total) : 16384 KB
    allocated     : 376 B (堆元数据)
    free chunks   : 1

  nsh> psram alloc 1024
  alloc: 1024 KB @ 0x60000178 (OK, slot 0)
    verify: malloc(1024)=0x2801c5a8 (SRAM OK)

  nsh> psram alloc 614
  alloc: 614 KB @ 0x60100180 (OK, slot 1)

  nsh> psram heap
    allocated : 1638 KB (= 1024+614，精确吻合)

  nsh> psram freeall
  freeall: released 2 block(s)

  nsh> psram heap
    allocated : 376 B (回落到初始值，无碎片)
  ```
- 🟢 <span style="color:#1a7f37">**新增**</span> 方案 B 达成:
  PSRAM 独立堆可用，主堆仍在 SRAM（malloc 返回 0x28xxxxxx）。
  heap_init 内部自保（自动调 psram_init）修复生效。
- 🟡 <span style="color:#d4a017">**接口契约教训**</span>
  S2 的 heap_init 要求"调用方先 psram_init()"，多入口场景（heap/alloc）
  必然漏掉。正确做法: 被调方内部幂等自保（g_psram_init_done 守卫）。
  这是典型的隐式前置条件 bug，写 API 时要假设调用方什么都不知道。

### 22.12 S4: 对齐分配 + 摄像头缓冲验证

- 🟢 <span style="color:#1a7f37">**新增**</span> 新增 API:
  `bk7258_psram_memalign(alignment, size)` — 封装 NuttX mm_memalign()，
  支持 32/64 字节对齐，DVP DMA 必需。
- 🟢 <span style="color:#1a7f37">**新增**</span> NSH 新命令:
  - `psram align <A> <KB>` — 按指定对齐分配并验证
  - `psram fbtest` — 摄像头场景一次性验证:
    32 字节对齐 + 64 字节对齐各分配 614KB，读写压测，报告带宽
- 🟡 <span style="color:#d4a017">**D-cache 前提**</span>
  当前 defconfig 无 CONFIG_ARMV8M_DCACHE，无需 cache 维护。
  已在 memalign 注释中标注: 一旦启用 D-cache，
  DMA 写入 PSRAM 后必须 invalidate（arm_dcache_invalidate），
  否则 CPU 读到旧 cache 行 = 数据损坏。
- 🟢 <span style="color:#1a7f37">**架构决策确认**</span>
  S2 带宽实测 ~1.8 MB/s 决定了分工:
  - LCD 帧缓冲 → SRAM（高频刷新，200KB 放得下 336KB SRAM）
  - 摄像头缓冲 → PSRAM（614KB 大块，DMA 写入+CPU 低频读取）

  > ⚠️ **依据已失效（2026-08-20，见 §25）**:1.8 MB/s 是 `-O0` 循环上限而非
  > PSRAM 上限，`-Os` 下实测写 20.5 / 读 12.3 MB/s。
  > 不过**这个分工结论本身仍然成立**,只是理由换了:
  > SRAM 与 PSRAM 的读性能差异在真实转换循环里 **<2%**（§25.3），
  > 所以「LCD 用 SRAM」的真实理由不是带宽，而是 **DVP DMA 必须写进
  > PSRAM 那块 614KB 连续区**、而 SRAM 装不下整帧。
  > 结论对、原因错，这种情况尤其危险 —— 下次照原因去推广就会出错。

> ✅ 已完成:DVP 驱动移植见 §24（用 psram_memalign 申请摄像头缓冲）。

---

## 23. M2 续:硬件 SPI 双屏 + 15× 吞吐提升

> 源码侧的完整记录见 `src/bk7258_gc9d01.c` 末尾的 `DEBUG_JOURNAL` 注释块，
> 本节是给人读的提炼版。

### 23.1 为什么必须从 bit-bang 转硬件 SPI

§20.2 当初决定先 bit-bang 是对的（先出图，不碰未知外设）。但到了要放
摄像头预览的阶段，bit-bang 的天花板挡住了路:

- 全屏 160×160 RGB565 = 51200 字节,bit-bang 刷一次 **740ms**(67 KB/s)
- 一帧预览光是"送到屏上"就要 0.74 秒,连 2fps 都到不了

所以这一阶段的目标很明确:把全屏刷新压进 100ms 以内。最终做到 **50ms**。

### 23.2 三个硬件级根因(本阶段最耗时的部分)

这三条都不是"代码写错",是**寄存器行为与直觉不符**,查手册也查不到:

- 🔴 <span style="color:#d1242f">**`trans_len` 在 `tx_en` 上升沿锁存**</span>
  多块传输时,每一块都必须**先清 `tx_en`、再置 `tx_en`**。
  如果把 `trans_len` 和 `tx_en=1` 写在同一次 store 里,第二块就没有上升沿,
  长度不会被重新锁存。
  症状:FIFO 卡在 ~70 字节不动,`INT_STATUS` 读出来全 0,
  看起来像"SPI 死了",实际是长度寄存器还是上一块的值。

- 🔴 <span style="color:#d1242f">**`CFG`(REG_0x05)必须读改写**</span>
  直接整写会把 `tx_finish_int_en`(bit2)清掉,
  于是 `tx_finish`(bit13)永远不置位 → 又是一个"SPI 停住"的假象。

- 🔴 <span style="color:#d1242f">**`trans_len=1` 不触发 `tx_finish_int`**</span>
  单字节传输走硬件 SPI 会永久等待。
  规避:驱动里加 `LCD_GC9D01_HWSPI_MIN_LEN` 阈值,短传输退回 bit-bang。

### 23.3 关键推理:FIFO 深度是 64,不是 48

这一条值得单独写,因为它是**靠推理而不是靠文档**定下来的。

起因:`SPI_FIFO_DEPTH` 一开始照抄 ARMino 的 `SPI_FIFO_INT_LEVEL_48` 写成 48。
但那个枚举定义的是**中断阈值**,不是 FIFO 容量。ARMino 的 `spi_ll.h` 写的是
"≥48 字节",被读成了"正好 48"。

三次实测现象:

| 阈值 level | 单次写入量 | 结果 |
|---|---|---|
| 16 | 16 字节 | 正常 |
| 32 | 32 字节 | 正常(刚好塞满) |
| 48 | 48 字节 | **溢出失败** |

唯一能同时解释这三条的模型:

```
tx_fifo_int_level 是"占用量"阈值:
  FIFO 占用 <= level 时触发中断
  → 空槽数 >= FIFO_DEPTH - level

安全写入量 = FIFO_DEPTH - level
  level=16 → 空槽 >= 48,写 16 → 安全 ✓
  level=32 → 空槽 >= 32,写 32 → 安全 ✓(恰好)
  level=48 → 空槽 >= 16,写 48 → 溢出 32 字节 ✗
```

只有 `FIFO_DEPTH = 64` 能让三条观测自洽。**手册没写,是推出来的。**

### 23.4 性能实测(2026-08-15 定档)

全屏 160×160 = 51200 字节,命令 `lcdtest chunk 4094 sram multi`:

| SPI 时钟 | Burst | 耗时 | 吞吐 | 说明 |
|---|---|---|---|---|
| 2.0 MHz | 1 | 740 ms | 67 KB/s | 逐字节轮询(旧默认) |
| 6.5 MHz | 1 | 390 ms | 128 KB/s | 逐字节轮询 |
| 6.5 MHz | 16 | 120 ms | 416 KB/s | `tx_fifo_int` level 1 |
| 6.5 MHz | 32 | 120 ms | 416 KB/s | `tx_fifo_int` level 2 |
| 6.5 MHz | 48 | **失败** | — | FIFO 溢出(见 §23.3) |
| **13 MHz** | **32** | **50 ms** | **1000 KB/s** | ← **最终默认** |
| 26 MHz | 32 | 50 ms | 1000 KB/s | 无额外收益 |

- **总提速 740ms → 50ms ≈ 15×**
- 26MHz 没有额外收益,说明已经撞到**轮询循环的天花板**,
  再快只能上 DMA。选 13MHz 是为了留时序余量。
- 计时精度 10ms,50ms 的真实范围是 45~55ms。
- 1 像素竖线图样(`lcdtest pat`)在 13MHz 和 26MHz 下都干净,排除了时序问题。

最终默认值(Kconfig + defconfig):

```
CONFIG_LCD_GC9D01_SPI_CLK_DIV       = 1   (13 MHz)
CONFIG_LCD_GC9D01_SPI_BURST         = 32
CONFIG_LCD_GC9D01_SPI_BYTE_INTERVAL = 0
```

### 23.5 双屏引脚缓存失同步

§20.5 为了给 bit-bang 提速,缓存了 GPIO 配置寄存器。上双屏之后这个缓存成了 bug 源:
切换左右屏时缓存没跟着走,于是往左屏写的数据出现在右屏上,或者两屏互相污染。

修法是**从根上重建**:引入 `g_cached_pins`,由 `lcd_set_pins()` 统一重建缓存,
而不是在每个调用点各自记得同步。

### 23.6 方法论教训(两条,都栽了不止一次)

- 🟡 <span style="color:#d4a017">**不要用"不反映真实状态的变量"做守卫**</span>
  同一个反模式在这一个阶段里造成了 **4 个 bug**:
  - `lcd_spi_pins_to_gpio()` 硬编码 `g_lcd_left`,污染 GPIO 缓存
  - `lcdtest_chunk()` 用 `g_active_pins != &g_lcd_left` 做守卫,
    而静态初值恰好就等于它 → 条件恒假 → 初始化被跳过
  - `lcd_hw_spi_usable()` 在 `lcd_setup_pins()` 之前就被调用
  - `preview_init` 用 `lcd_set_pins` 切屏,但 GPIO 缓存没跟上

  这四个表现各异,根子是同一个:**守卫变量与真实硬件状态脱钩**。

- 🟡 <span style="color:#d4a017">**区分"档位号"和"它代表的物理量"**</span>
  同一个错误犯了两次:
  - `SPI_FIFO_DEPTH` 被设成 48(最大档位枚举值),实际 FIFO 是 64 字节
  - `safe_write` 算成 `SPI_FIFO_DEPTH - level`(如 64-2=62),
    减的是**档位序号 2** 而不是**该档位的字节数 32**。
    正确:`64 - fifo_level_to_bytes(2) = 64 - 32 = 32`

  规则:寄存器字段通过查表映射到物理量时(档位→字节、枚举→频率、码值→电压),
  **一定要过一次转换函数**,绝不能把原始字段值直接拿去做算术。

### 23.7 诊断命令(排障主力)

沿用 §20.3 的分阶段套路,本阶段新增:

| 命令 | 用途 |
|---|---|
| `lcdtest chunk <N> <sram\|psram> [multi]` | 分块传输计时,性能测量主力 |
| `lcdtest clk <div>` | 运行时改 SPI 时钟,不用重烧 |
| `lcdtest burst <1\|16\|32>` | 运行时改批量写入档位 |
| `lcdtest pat` | 1 像素竖线图样,验证高频下时序是否干净 |
| `lcdtest cross [both\|left\|right]` | 十字准心 + 边界环,给 3D 外壳对位用 |
| `lcd_spidiag` | 寄存器级 SPI 状态转储 |

### 23.8 当前状态与遗留

- 🟢 <span style="color:#1a7f37">**双屏出图正常**</span>,全屏 50ms,4 个关键提交均已 bisect 验证可独立编译
- 🟡 <span style="color:#d4a017">**遗留**</span> 再快需要上 DMA。当前不做:
  真实路径是"局部刷新眼睛"(60×60 虹膜区 7200 字节 ≈ 7ms),50ms 全屏已够用

---

## 24. M4 摄像头:GC2145 DVP bring-up

> 源码侧完整记录见 `src/bk7258_camera.c` 末尾的 `DEBUG_JOURNAL` 注释块。

### 24.1 确认的硬件事实

- **SCCB**:bit-bang,开漏。GC2145 7 位地址 `0x3C`,
  ID 寄存器 `0xF0=0x21` / `0xF1=0x45`。
  "拉低" = `output_enable(bit3)=0` + `output_value(bit1)=0` + `second_func(bit6)=0`;
  "释放" = `output_enable=1`(禁用输出) + `input_enable(bit2)=1`。
  **绝不主动拉高**——总线有外部上拉,拉高会和其他器件顶牛。
- **MCLK**:必须在 **P27**,功能选择 = 1。别的脚都不行,时钟输出是硬连到该 pad 功能的。
- **DVP 数据总线**:P29–P39 共 11 根(PCLK/HSYNC/VSYNC/PXDATA0-7),功能选择 = 0。
  注意"功能选择 0"和"`second_func` = 1"是**两个不同寄存器里的两个不同字段**,
  两个都要设。
- **帧缓冲**:YUYV 640×480 = 614400 字节/帧,PSRAM 里开两块(0x60000000 起)。
  DVP DMA 要 32/64 字节对齐,普通 `mm_malloc` 保证不了 ——
  这就是 §22.12 加 `bk7258_psram_memalign()` 的原因。
  预览暂存区在 `0x6012C000`,160×160×2 = 51200 字节。

### 24.2 中断有两道门(和 §18.27 同一个坑)

- 🔴 <span style="color:#d1242f">**DVP 中断不直连 NVIC**</span>
  BK7258 在 NVIC 前面还有一级 **SYS 中断聚合器**,
  `cpu1_int_32_63_en`(`0x4401008C`)也必须使能。
  只开 NVIC 那一道,中断永远不来。

  这和 §18.27 修 UART RX 中断时踩的是**同一类**坑:
  **两道门串联,只开一道 = 完全静默**。
  本项目已经栽过两次,以后遇到"外设中断死活不来",第一反应就该查 SYS 级聚合器。

- 模块时钟门控:`sys_reserver_reg0xd`(`0x44010034`),bit[9] = `cis_auxs_cken`。

### 24.3 像素格式是 VYUY,不是 UYVY(也不是 YUYV)

- 🔴 <span style="color:#d1242f">**被推翻 (2026-08-21):下面第一版结论是错的**</span>

  ~~GC2145 寄存器 `0x84 = 0x02` 输出的是 **UYVY**,
  而 ARMino `dvp_gc2145.c` 把这个值标注成 "yuyv"。
  抓 PSRAM 内存 dump 确认,实际字节序是 `U Y0 V Y1`。~~

  **推翻依据**:纯红/纯蓝颜色靶交叉验证(2026-08-21):

  | 激励 | byte0 均值 | byte2 均值 | 理论 Cb | 理论 Cr |
  |------|-----------|-----------|---------|---------|
  | 纯红 | 230 (高)  | 101 (低)  | 85 (低) | 255 (高) |
  | 纯蓝 | 127 (中)  | 219 (高)  | 255 (高)| 107 (低) |

  两次测试独立一致:byte0 = Cr, byte2 = Cb。
  再用肤色做第三验证:byte2=114 < 128 < byte0=144,
  满足肤色不变量 Cb < 128 < Cr。
  **实际字节序是 `V Y0 U Y1` (VYUY),即 Cr 在前、Cb 在后。**

- 🟡 <span style="color:#d08c00">**第一版验证为什么是无效的**</span>

  当初的内存 dump 是在灰度场景下做的:Cb≈Cr≈128,
  两个色度字节的值几乎相等,无法区分谁是谁。
  dump 能确认"偏移 0 和 2 是色度、偏移 1 和 3 是亮度",
  但**确认不了色度的顺序**。

  > **方法论教训**:用对称量互判顺序,必须使用能让它们分离的激励。
  > 灰度场景下 U/V 顺序不可判;纯色(红/绿/蓝)才能打破对称。

- 为什么搞反了能活很久:byte0↔byte2 互换**不会出乱码**,
  而是出一张"看起来合理但颜色发飘"的图。
  互换后 `r` 由实际的 Cb 驱动、`b` 由实际的 Cr 驱动,
  肤色(Cb 低、Cr 高)会被渲染成偏蓝紫,但不熟悉色度空间的人不容易发现。

- � <span style="color:#1a7f37">**已验收 (2026-08-21)**</span>
  用**手机全屏纯红**充满视野做受控验收:

  | 项 | 结果 |
  |---|---|
  | `camera preview 30 0` | 屏幕显示红色 ✅ |
  | `camera uvhist` 同场景 | `Cb 均值 101`(低) / `Cr 均值 215`(高),峰值 `(97, 210)` |
  | 纯红理论值 | `Cb=85` / `Cr=255`,方向一致 |

  画面偏粉红而非纯红,与数值吻合(手机屏非理想原色,饱和度略欠)。
  **从传感器到屏幕的整条彩色链路至此全线验证通过。**

- 🔴 <span style="color:#d1242f">**方法论:受控激励是唯一可靠的判据**</span>
  这条线前后绕了四轮,每一轮的错误都是同一个毛病 ——
  **拿"场景不受控"的数据下结论**:
  1. 第一次测出色度全在中性,判为"色度通道是死的" → 实际只是场景没颜色
  2. 4 次运行只标了 3 个场景,把蓝色当成红色,推出错误的字节序
  3. 从复杂室内景的绿/洋红反推"色度幅值被放大几十倍" → 无从验证的推断
  4. 直到用**纯红/纯蓝**做激励,才一次定案

  同理,`camera dump` 的四格式并列诊断**一直存在**,却在灰度场景下毫无用处
  (四种解读看着都差不多);有了彩色数据它立刻成为最硬的逐像素证据。
  > **诊断工具的有效性取决于激励能否打破对称。**
  > 工具没用不代表工具不行,可能只是激励选错了。

- 代码里部分标识符还叫 YUYV/UYVY,是因为它对应移植来源的 ARMino 寄存器表;
  `uyvy_to_rgb565_scaled()` 已于 2026-08-21 修正为 VYUY 解析。

### 24.4 乒乓缓冲归属(消除 DVP 与消费者竞争)

三态归属:

| 变量 | 含义 |
|---|---|
| `g_cur_buf` | DVP 正在写入的缓冲 |
| `g_ready_buf` | 已完整、等待消费的帧(-1 = 无) |
| `g_busy_buf` | 消费者正在读的缓冲(-1 = 无) |

ISR 里收到 `YUV_ARV`(整帧写完)时,**仅当** `g_ready_buf < 0`
且 `next_buf != g_busy_buf` 才推进;否则 `g_drop_count++`,继续写同一块。

要点是:ISR 绝不把别人正在读的缓冲交给 DVP,也绝不覆盖尚未交付的帧。
**丢帧永远优于撕裂帧。** DVP 写地址寄存器在 ISR 内改,所以交接对消费者是原子的。

- 🟡 <span style="color:#d4a017">**教训**</span>
  改之前消费者读的是"DVP 恰好在填的那块",表现为间歇性横向撕裂,
  一开始被当成 SPI 时序问题查了一阵。
  **显示异常不等于显示的 bug**——数据源的并发问题一样会画到屏上。

### 24.5 性能与瓶颈(约 7fps)

`camera preview 30 left`:约 127.6 ms/帧

| 环节 | 耗时 | 说明 |
|---|---|---|
| blit | 约 50 ms | SPI 传输,13MHz/burst 32 |
| 降采样 + 格式转换 | 约 78 ms | **瓶颈** |

> ⛔ **以下这段归因已于 2026-08-20 被实测推翻,保留原文以记录教训。
> 正确结论见 §25。**

~~🔴 **78ms 不是算术开销,是 PSRAM 访问模式**~~
~~`uyvy_to_rgb565_scaled()` 每个输出像素要读 3 个源字节,160×160 输出~~
~~≈ 76800 次散读。PSRAM 挂在 QSPI 后面,单次访问开销占主导,~~
~~只有顺序突发才接近标称带宽。~~

~~真要优化,按优先级:~~
~~1. 先 `memcpy` 把源扫描行搬进 SRAM,把散读变顺序突发;~~
~~2. 配 DVP 硬件裁剪,让采集直接就是 160×160,转换变 1:1,散读从根上消失。~~

~~🟡 人脸检测选型时,决定"塞不塞得进"的是**算法的内存访问模式**,不是指令条数。~~

**实测结论(§25)**:瓶颈**完全不在内存**。PSRAM 散读与 SRAM 顺序读的差异
在 `-O0` 下 <2%、在 `-Os` 下 3%。真因是 `-O0` 编译产生的指令膨胀
(内层循环 177 条指令/像素,其中 37% 是栈访问)。上面「候选优化 1」
(SRAM staging) 经实测是**净亏**,已作废。

**故意不优化(这条仍然成立)。** 全屏预览是 bring-up 和演示路径,不是产品路径。
产品路径是:采集到 PSRAM → 在那儿做人脸检测 → 输出方向 → 只重绘眼睛。

- 🔴 <span style="color:#d1242f">**教训:不要在未优化的构建上做性能归因**</span>
  这次的错误链是:看到 78ms → 数出「76800 次散读」→ 认定 PSRAM 是瓶颈 →
  据此制定优化方向,甚至据此给出算法选型标准。
  数字本身没错,错在**没有先排除编译器**。`-O0` 下每像素 177 条指令、
  37% 在搬栈,任何「热点在哪」的判断都失真。
  **规则:性能归因之前,先确认优化级别;A/B 测量必须一次只变一个变量。**

### 24.6 踩坑

- 🟡 `0x44010028` / `0x44010030` 是 **LCD 与 DVP 共享**的时钟配置寄存器,
  必须读改写。整写会静默破坏另一个子系统的时钟,而故障点离写入处很远。
- 🟡 接管/释放引脚时要**保存并恢复完整的 4 位功能字段 + 引脚 CFG**
  (`gpio_state_t` / `mclk_state_t`)。恢复成猜的默认值,会把之前占用该脚的模块搞坏。
- 🟡 SDA 与 SCL 必须是不同引脚。代码里加了断言,
  因为 Kconfig 配错产生过一条"SDA=SCL"的总线,它什么都不 ACK,看起来像传感器坏了。
- 🟡 抢占 DVP 引脚前先查板级保留引脚:**P29 之前是左屏复位线**。
  子系统之间的引脚重分配是本板"昨天还好好的"类回归的头号来源。

### 24.7 当前状态与下一步

- 🟢 <span style="color:#1a7f37">**摄像头实时预览跑通**</span>,7fps,乒乓缓冲无撕裂
- 🟢 诊断命令:`camera still [left|right]`(冻帧,验证显示链路)、
  `camera still flat <hex16>`(纯色填充,把显示链路和摄像头数据解耦)
- 🟡 <span style="color:#d4a017">**下一步**</span> 人脸检测 + 方向输出。
  ~~开工前需先调研 PSRAM 散读性能约束~~ —— 该调研已于 2026-08-20 完成,
  结论见 §25:**内存不是约束,指令条数才是**。选型依据改为「每像素指令预算」。

> 待续:🟡 人脸检测 → 方向 → 眼睛局部刷新。

---

## 25. 性能归因实测:瓶颈是编译优化,不是 PSRAM

> 本节推翻了 §22.10 的「PSRAM 带宽 1.8 MB/s」和 §24.5 的「散读是瓶颈」两个
> 结论。源码侧见 `src/bk7258_camera.c` 的 `camera_bench_conversion()`。

### 25.1 为什么要做这个实验

§24.5 认定预览的 78ms 瓶颈是 PSRAM 散读,并据此排了优化方向,甚至据此给出了
人脸检测的算法选型标准(「访存模式是关键」)。但那个结论是**推理出来的,没有
被隔离验证过** —— 只知道「耗时 78ms」和「有 76800 次散读」,两者之间的因果
从未被测量。

### 25.2 实验设计:三条路径隔离两个变量

新增 `camera bench` 命令。三条路径输出像素数相同(25600)、走完全相同的指令
路径,只有源数据的位置与访问模式不同:

| 路径 | 源 | 访问模式 | 调用 |
|---|---|---|---|
| **A** | PSRAM | 散读(4:1/3:1 降采样,行跳 3840B、行内跳 8B) | `(psram, 640,480, 160,160)` |
| **C** | PSRAM | 顺序(1:1) | `(psram, 160,160, 160,160)` |
| **B** | SRAM | 顺序(1:1) | `(sram, 160,160, 160,160)` |

于是:

- **A vs C** 隔离**访问模式**(内存同为 PSRAM)
- **C vs B** 隔离**内存类型**(模式同为顺序)

- 🔴 <span style="color:#d1242f">**第一版设计是错的,值得记下来**</span>
  最初只有 A 和 B 两条:A 是「PSRAM + 散读」,B 是「SRAM + 顺序」。
  这**同时改变了两个变量**,测出 B 快也无法归因。补上 C 才拆得开。
  另外第一版把 `dst` 放在 PSRAM,三条路径都额外付一次 51200 字节的 PSRAM 写;
  虽然该代价在差值里会抵消,但会抬高 ns/px 绝对值,后改为 SRAM。

### 25.3 实测结果

`-O0`(`CONFIG_DEBUG_NOOPT=y`,50 轮累计):

| 路径 | ms/iter | ns/px | 源侧触及字节 |
|---|---|---|---|
| A PSRAM 散读 | 55 | 2156 | 204800 |
| C PSRAM 顺序 | 54 | 2132 | 51200 |
| B SRAM 顺序 | 54 | 2132 | 51200 |

`-Os`(`CONFIG_DEBUG_FULLOPT=y`,同样 50 轮):

| 路径 | ms/iter | ns/px | 相对 -O0 |
|---|---|---|---|
| A PSRAM 散读 | 13 | 523 | **4.2×** |
| C PSRAM 顺序 | 13 | 507 | 4.2× |
| B SRAM 顺序 | 12 | 500 | 4.3× |

- 🔴 <span style="color:#d1242f">**A ≈ C ≈ B,两个优化级别下都成立**</span>
  A 触及的地址空间是 C 的 **4 倍**、而且是 8 字节步长的散读,代价只多
  **1%(-O0)/ 3%(-Os)**;C 与 B **完全相同**。
  **PSRAM 与 SRAM 的读性能差异 <2%,访问模式的影响 ≤3%。内存不是瓶颈。**

### 25.4 真因:`-O0` 的指令膨胀

反汇编 `uyvy_to_rgb565_scaled` 内层循环:

| 指标 | -O0 | -Os |
|---|---|---|
| 循环体指令数/像素 | **177** | **43** |
| 其中 `[sp,...]` 栈访问 | **66(37%)** | 3 |
| 每像素周期数 @120MHz | 256 | 60 |
| 实测 CPI | 1.45 | 1.40 |

177 条指令去做「读 3 字节 + 十来个算术 + 写 2 字节」,是 `-O0` 把所有局部变量
摊到栈上的直接后果。

- 🟢 <span style="color:#1a7f37">**顺带结论:XIP flash 取指不是瓶颈**</span>
  CPI 实测 1.45(-O0)/1.40(-Os)。对一个无 cache、代码在 QSPI flash 上 XIP
  执行的核心来说,这个数说明**存在有效的预取缓冲**,取指停顿很小。
  所以「把热循环搬进 16KB ITCM」的收益有限,不必优先做。

- 🟡 <span style="color:#d4a017">**`-Os` 会用 `sdiv` 换体积**</span>
  `-O0` 时 `cb_i * 28 / 81` 被编译成 `smull` 倒数乘;`-Os` 改成了
  **循环内每像素一条 `sdiv`**(体积 4 字节 vs 倒数乘序列十几字节)。
  但实测代价很小(60 周期里只占几个周期,SDIV 对小操作数提前终止),
  故未处理。若要消除,源码改 `(cb_i * 88) >> 8` 即可
  (88/256=0.34375 vs 28/81=0.34568,G 分量最大偏差 0.25/255,不可见)。

### 25.5 被作废的优化方向

- ⛔ **SRAM staging(§24.5 候选优化 1)—— 净亏,作废。**
  memcpy 实测吞吐 4654 KB/s(-O0)/ 16000 KB/s(-Os);真实 staging 需搬
  204800 字节(降采样要触及 160 行 × 1280 字节),即 44ms(-O0)/ 12ms(-Os)。
  而它能省下的是访问模式那 1~3%,即 **0.4ms**。花 12ms 省 0.4ms。
- ⚠️ **DVP 硬件裁剪(§24.5 候选优化 2)—— 理由需重写。**
  原理由是「消除散读」,但散读只值 3%。它仍然有价值,但机制是
  **减少要处理的像素数**,与访存模式无关。

### 25.6 `-Os` 切换的连带影响

- 🟢 全系统提速,回归全过:`camera id` 仍读到 `0x2145`、双屏出图正常、
  全屏 SPI 传输 50ms → **30ms**(1000 → 1666 KB/s)、`psram fbtest` 零错误。
- 🟢 `.text` 249912 → **145760(-42%)**;`.bss` 因 bench 的两个 51200B
  静态缓冲增至 197092,SRAM 占用 57.5%。
- 🔴 <span style="color:#d1242f">**`CONFIG_BOARD_LOOPSPERMSEC=60000` 从未标定,高约 6.5 倍**</span>

  `up_mdelay()` / `up_udelay()` 都是 `volatile` 空转循环计时,依赖该值。
  **实测方法(零改动)**:不带参数的 `lcdtest` 标称延时合计约 1.6 秒
  (函数体内 50+1000+10+120+10+120 = 1310ms,加 `lcd_init_sequence()` ×2
  各含 120ms,再加两次全屏 SPI 约 60ms)。**秒表实测约 10 秒。**

  反汇编 `up_mdelay` 内层循环:7 条指令(`ldr` / `cmp` / `bls` / `ldr` /
  `adds` / `str` / `b`),含两次栈访问与两个跳转,约 13 周期/迭代。
  60000 迭代 = 78 万周期 = **6.5 ms @120MHz**。与 10s / 1.6s ≈ 6.5 吻合。

  | | 标称 | -O0 实际 | -Os 实际 |
  |---|---|---|---|
  | `up_mdelay(1)` | 1 ms | 约 10~15 ms | **6.5 ms** |
  | 面板 RST 高后 | 120 ms | 1.2~1.8 s | **780 ms** |
  | SCCB 半周期 | 5 µs | 约 50 µs | 约 32 µs(15 kHz) |

  正确值应约 **9200**。

- 🟢 <span style="color:#1a7f37">**修正:`-Os` 并未破坏时序**</span>
  切换优化级别确实使延时变短,但只是**从「过长约 15 倍」变成「过长约 6.5 倍」**,
  两者都远超需求,所以没有任何时序被破坏。
  曾据此怀疑「`lcdtest go` 一块屏显示纯蓝是初始化延时不足」——**已排除**:
  面板复位后实际获得 780ms,规格仅需 120ms,余量 6.5 倍。
  该现象最终判断为**青色虹膜的观感误判**(`r=60` 虹膜在可见半径 79 的圆屏上
  铺满 76%,黑边露出多少不同就会看成"一个有眼睛、一个纯蓝"),
  两块屏渲染实际完全一致。

- 🔴 <span style="color:#d1242f">**决定:暂不修正 LOOPSPERMSEC**</span>
  把 60000 改成 9200 会让**所有延时一次性缩短 6.5 倍**降到标称值,而那些
  标称值(如 `up_mdelay(120)`)在本板上从未被验证过——过去一直跑在 6.5 倍
  余量下。一改就可能暴露真实时序要求,出现"改了个配置屏幕就不亮"。
  当前代价仅是浪费时间(`lcdtest` 10s 而非 1.6s、SCCB 15kHz 而非 100kHz),
  **不影响正确性**。等真正在意启动速度时再改,届时须逐处重新验证延时。

- 🟡 <span style="color:#d4a017">**教训:先量基线,再谈变化**</span>
  本次预判「`-Os` 会使延时不足」时,默认了「原值是标定准的」这个前提,
  而该前提从未被验证。方向猜对了,结论完全错。
  **规则:讨论某个量的"变化"之前,先确认它的"绝对值"是否可信。**
- 🟡 <span style="color:#d4a017">**观察:摄像头会改动共享时钟寄存器**</span>
  跑过 `camera bench` 之后,LCD 打印的 `en_reg` 从 `0x00008284` 变成
  `0x00088284`(bit19 置位)。目前未造成故障,但印证了 §24.6 第一条。
  日后若出现「跑过摄像头后屏幕异常」,先查这里。

### 25.7 对人脸检测选型的结论(下一步的真实输入)

`-Os` 下完整 YUV→RGB565 转换的预算是 **500 ns/像素 = 60 周期 = 43 条指令**。

- 🟢 <span style="color:#1a7f37">**选型依据是「每像素指令数」,不是访存模式**</span>
  访存不规则的算法(如在积分图上跳查的 Haar/LBP cascade)**不再是劣势**。
- 🟢 **方向检测不需要 RGB。** 只要对 U、V 做两次阈值比较,估约 10 条指令:

  | 方案 | 每像素 | 160×160 全图一遍 |
  |---|---|---|
  | 现状(完整转换) | 500 ns | 12.8 ms |
  | 仅 UV 阈值 | 约 120 ns | 约 3 ms |
  | UV 阈值 + 1/4 抽样 | — | **< 1 ms** |

  **算力已经不是约束**,剩下的是算法设计问题。
- 🟡 尚未动用的余量:真 `-O2`(`FULLOPT` 映射的是 `-Os`,会关循环展开)、
  提频(datasheet 标称上限 480MHz,现跑 120MHz = DPLL/4)、DSP/SIMD 指令
  (手册称支持,当前未启用)。现阶段不需要,记录备用。

> 待续:🟡 UV 色度域方向检测 → 眼睛局部刷新。

---
## 26. M5 人脸跟随双眼：从"检测老掉 0"到"两屏干净跟手"

> 承接 §25.7 的"待续"。目标：摄像头看到人脸 → 双圆屏的瞳孔跟着人脸左右转；无人脸 → 回中。
> 这一章是**一条完整的排障链**，每一步都由现场日志（`camera detect` / `camera track` / `camera uvhist`）驱动，
> 教训是：**先把"能观测"做出来，再谈修**。相关代码：`src/bk7258_camera.c`（检测+注视）、`src/bk7258_gc9d01.c`（画眼+SPI）、`app/camera/camera_main.c`（命令）。

### 26.1 先补诊断字段，否则全是瞎猜
最初 `camera detect` 只打印 `0 blob hits < 96 min`，无法区分两种完全不同的故障：
- **采样阶段一个肤色像素都没命中**（传感器/曝光/门限问题）
- **命中够了但团块提取归零**（代码 bug）

**方法**：在失败路径补打 `total_hits / total_samples` 和实际生效的门限区间；`track` 状态行补 `total=` 和 `cr>=`（实际用的自适应阈值），并把 `raw=` 从 `dir.dx` 改成取反后的 `raw_dx`（否则 `invert` 的作用被掩盖）。
- 🟢 **结论**：诊断字段是排障的前提。补上后一眼看出是 `total_hits=0`——采样阶段就没命中，方向立刻从"查代码"转向"查门限/数据"。
- 顺带修了两个确凿 bug：EMA 负数侧的 `>>2` 改 `/4`（算术右移向负无穷取整会把 `smooth` 卡在 −1 回不了中）；补 `#include <stdlib.h>`（`abs()` 靠间接包含）。

### 26.2 检测：固定肤色阈值行不通 → 自适应
用 `camera uvhist`（同 detect 的采样与取字节）分表面实测 Cr 分布：

| 表面 | 背景 Cr 峰 | 肤色/自身 Cr | 关键结论 |
|---|---|---|---|
| 白墙 | 127 | ≤138 | 纯背景 |
| 木板（手机照） | 127 | 主体 137~147、尾到 154 | **木板比想象的红** |
| 真人脸 | 127 | 簇 142~167 | 肤色是背景之上的"红尾巴" |

- 固定 `Cr≥158` 卡在肤色簇最上沿，只捞到极少数、且随光照漂移在"能测/全 MISS"间跳变。
- 🟢 **决定**：改**自适应阈值** = 本帧背景 Cr 峰 + 余量。背景中性峰在所有帧都稳定在 **127**，是可靠参照。
- 演示场景**无木板**（用户确认）→ 不用纠结肤色/木板分离，"最红的区域"就是脸。

**两遍法**（`camera_detect_frame`）：第一遍统计 Cr 直方图找背景峰 `bg_cr` 与稳健最大值 `max_cr`（要求 ≥3 样本，忽略单像素离群）；阈值 `cr_lo = bg_cr + 60%×(max_cr − bg_cr)`，钳到 `[140,156]`，让门限落在肤色簇上半段（团块紧、质心稳），随光照/肤色强度自动跟随。第二遍才按 `cr_lo` 数肤色列。

- 🟡 **踩坑（阈值定太低）**：一度用固定 `bg+16`（≈143），blob 涨到 1000+（脸+脖子+背景一大片），质心乱跳、方向噪声。→ 改成 60% 比例式收紧到脸核心。
- 🟡 **稳健化夹值**：现场发现 `cr>=156` 一直顶上限、全 MISS——原来板上**橙色排线/铜线圈 Cr≥175，比脸(≤164)还红**，把 `max_cr` 顶高、阈值窜到 156 上限卡掉人脸。加 `#define SKIN_CR_CEIL 166`，`max_cr` 先夹到它再算，比肤色更红的物体顶不动阈值。

### 26.3 检测稳定性：预热 + MIN_HITS
- **预热**：`detect` 原本无预热、`track` 只 10 帧。冷启动首帧 AE/AWB 未收敛（曾出现 `out-of-range 2072` 的过曝坏帧）。→ **detect 加 20 帧、track 提到 20 帧**丢弃预热。
- **命中门限**：`DETECT_MIN_HITS_PER_FRAME` 96 → **60**（无木板，放宽提升移动中的连续性；空场景实测 total 仅 0~29，60 仍能拒掉无脸帧）。

### 26.4 注视：方向、对称、消抖
1. **检测方向是镜像**：人脸左→右，`dir.dx` 反向变化 → `invert=1` 才是"盯着人看"。设为默认（`camera_main.c` 里 `inv=1`）。
2. **幅度**：`gaze = smooth × 28 / SPAN`。人脸质心 `raw` 实测最多 ±60，原来除以 127 只用了半程 → 引入按符号的**左右独立自适应量程** `dx_span_pos/neg`（各自超峰即扩、否则向下限 `TRACK_SPAN_MIN=30` 缓衰），让两侧到各自极限都能到满 `±TRACK_GAZE_MAX(28)`。
3. **左右不对称**（右比左远）：根因是取景偏移——人脸中立位不在画面几何中心。加**缓慢自适应基线** `dx_baseline=(baseline×31+d)/32`，`raw_dx=d−baseline`，让注视围绕人的中立位对称。首个有效帧初始化基线避免开局突跳；只在有效帧更新，无脸不漂移。
4. **消抖**：EMA 权重 3/4 → **7/8**（`TRACK_EMA_WEIGHT=7`），死区 2px → **3px**。静止时瞳孔纹丝不动。
   - 🟡 代价：EMA 7/8 约半秒跟随延迟；嫌黏可降到 5/6。

### 26.5 持续模式
`camera track 0` = 一直跟踪、**按回车停**（非阻塞 stdin：`fcntl` O_NONBLOCK + `read` 轮询，退出时恢复终端）。运行中无脸回中、背光不灭；停止才回中灭屏。`camera track 100` 仍是跑满 100 帧即停。

### 26.6 SPI 单次初始化（去掉每帧重复 init）
`track` 每次更新画左右两眼，左眼(sclk=2)每次都触发 `lcd_setup_pins → lcd_spi_init`，把整个 SPI1 外设重置一遍 + 刷屏。右眼位操作用自己引脚，根本不碰 SPI1。
- 🟢 **修法**：`lcd_setup_pins` 里给 `lcd_spi_init` 加 `&& !g_hw_spi_capable` 守卫——只初始化一次，`lcd_spi_deinit` 会清标志，下次会话再初始化。`drop` 明显下降。

### 26.7 显示消损：横纹 + 黑斑（最费劲的一段，逐层剥）
现象：肉眼可见两屏在瞳孔移动后出现**横纹 + 黑色微斑**（纯色填充里出现 = 传输时字节丢/错位）。逐层定位：

| 层 | 手段 | 发现 | 处理 |
|---|---|---|---|
| 像素时钟 | `lcdtest clk 5`（运行时改分频，即时生效） | 开局全屏画眼变干净，但移动后仍坏 | HW SPI 像素流默认分频 `clk_div` 1→**3**（6.5MHz） |
| 命令路径 | 读 `lcd_fill_circle` | **逐行**调 `fill_rect`，每行发 CASET/RASET/RAMWR，命令走位操作**无延时** | `spi_write_byte` 加回 `spi_delay()`（SCLK 两相各一次） |
| 并发 | `lcdtest anim`（无相机同款动作）对照 | 横纹**少一大半**——证明主因是**相机并发**（DVP 搬帧/读 PSRAM 抢总线+中断抢占） | 重绘前后 `up_disable_irq(BK7258_IRQ_YUV_BUF)` 屏蔽 DVP 中断；`spi_delay` 4→2 NOP（右眼恢复跟手） |
| 固有残留 | 仍剩一点点（anim 也有） | 逐行命令风暴本身 | **重构 `eye_move_pupil`：开一次窗口、一次性流式刷整块** |

**根治重构**（`eye_move_pupil`）：算出覆盖新旧瞳孔+高光的包围盒（夹到虹膜/屏幕），逐像素按 **高光>瞳孔>虹膜>背景** 合成到一块 `static` 缓冲，一次 `CASET/RASET/RAMWR` + 一次 `lcd_send_data` 写完。
- 🟢 **效果**：上百组逐行命令 → 一次连续传输。命令风暴消除、并发抢占窗口大幅缩短，两屏移动时**彻底干净**。小步移动时"带"很窄（≈位移+44px）比原来还快；大跳最宽一个虹膜宽度，始终一次传输。代价：13KB 静态合成缓冲。

### 26.8 方法论教训（本章反复栽的）
1. 🟢 **先补可观测字段再修**：`total_hits` / `cr>=` 一补，检测问题当场分流，省掉无数瞎猜。
2. 🟢 **用对照实验隔离变量**：`lcdtest anim`（无相机）把"渲染固有 vs 相机并发"一刀切开，直接定主因。
3. 🟡 **别被表象带偏**：一度把"白墙"当木板参照、把亚克力罩物理反光当"高光不动"、把拍照摩尔纹当显示故障——每次都靠"回到原始数据/肉眼直接看"纠正。
4. 🟡 **固定阈值是环境的函数**：白平衡一变就翻车。自适应（相对背景峰）+ 稳健夹值才扛得住不可控光照。

### 26.9 最终参数与命令
- 检测：`Y[50,230]`、`Cb[40,150]`、自适应 `cr_lo=bg_cr+60%×(min(max_cr,166)−bg_cr)` 钳 `[140,156]`；`MIN_HITS=60`；预热 detect/track 各 20 帧。
- 注视：`invert=1` 默认；基线 `×31/32`；左右独立量程下限 `SPAN_MIN=30`、`GAZE_MAX=28`；EMA `7/8`；死区 `3px`。
- 显示：`clk_div=3`、`spi_delay=2 NOP`、重绘屏蔽 DVP 中断、`eye_move_pupil` 单窗口流式、SPI 只初始化一次。
- 命令：`camera init` → `camera buf` → `camera track 0`（持续，回车停）；诊断 `camera detect [n]`、`camera uvhist [n] [lo] [hi]`、`lcdtest anim`、`lcdtest clk <div>`。

- 🟢 <span style="color:#1a7f37">**当前状态：人脸跟随双眼全链路打通——检测在不可控光照下稳定、双眼对称跟随且无抖动、持续模式、两屏显示干净。**</span>
