/****************************************************************************
 * arch/arm/src/bk7258/bk7258_gc9d01.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Bit-bang SPI driver for the GC9D01 160x160 RGB565 LCD on BK7258 DevKit.
 *
 * Pin mapping (from schematic, confirmed against ARMINO gpio_map.h):
 *   SCLK = GPIO_2  — HW SPI: SPI1_SCK (func0)  / Bit-bang: GPIO output
 *   MOSI = GPIO_4  — HW SPI: SPI1_MOSI(func0)  / Bit-bang: GPIO output
 *   CS   = GPIO_3  — ALWAYS GPIO output (never SPI1_CSN — auto-toggle kills GC9D01)
 *   DC   = GPIO_5  — ALWAYS GPIO output
 *   RST  = GPIO_45 (LCD_RST on schematic — was wrongly P29/DVP_PCLK)
 *   BL   = GPIO_25 (LCD_BL_PWM via Q3)
 *
 *   HW SPI only muxes SCLK+MOSI to SPI1 peripheral.
 *   CS stays GPIO because SPI1_CSN auto-toggles per byte, which
 *   resets the GC9D01 command state machine.
 *
 * The GC9D01 uses a separate DC pin (not 9-bit SPI).  DC=LOW for command,
 * DC=HIGH for data.  SPI mode 0, MSB first.
 *
 * Staged test via NSH command "lcdtest":
 *   lcdtest          — staged bring-up (A=backlight, B=init, C=red sq)
 *   lcdtest go       — one-step reliable LCD bring-up (production flow)
 *   lcdtest pwr lo hi — power-on GPIO range for binary-search
 *   lcdtest scan     — GPIO pin scan for LCD power enable
 *   lcdtest spidiag  — minimal 4-byte HW SPI transfer with register dumps
 *   lcdtest cross    — AA-centre crosshair pattern for enclosure alignment
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/nuttx.h>

#include "bk7258_gpio.h"
#include "bk7258_audio.h"
#include "bk7258_psram.h"

#ifdef CONFIG_EXAMPLES_GC2145_ID
#  include <arch/board/bk7258_camera.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Shared pins */

#define LCD_PIN_BL        25
#define LCD_PIN_LDO33_EN  52   /* U3 ME6211C33 LDO enable — LCD 3V3 supply */

/* Per-panel pin descriptor */

typedef struct
{
  int sclk;
  int cs;
  int mosi;
  int dc;
  int rst;
} lcd_pins_t;

static const lcd_pins_t g_lcd_left =
{
  .sclk = 2, .cs = 3, .mosi = 4, .dc = 5, .rst = 45
};

static const lcd_pins_t g_lcd_right =
{
  .sclk = 22, .cs = 23, .mosi = 24, .dc = 7, .rst = 6
};

/* Currently active panel — all SPI functions use this.
 * g_active_pins tracks which panel's pin assignments are logically current.
 * g_cached_pins tracks which panel's GPIO caches (sclk/mosi/cs/dc) are
 * physically loaded.  lcd_set_pins() keeps both in sync.  If they ever
 * diverge, bit-bang writes go to the wrong GPIOs — the root cause of
 * four separate bugs (Bug A, lcdtest_chunk guard, hw_spi_usable order,
 * and preview_init cache desync).
 */

static const lcd_pins_t *g_active_pins = &g_lcd_left;
static const lcd_pins_t *g_cached_pins = NULL;  /* NULL = no cache loaded yet */

/****************************************************************************
 * Cached GPIO pin state for fast bit-bang
 *
 * Instead of read-modify-write on every gpio_write() call, we cache the
 * CFG base value (with OUTPUT=0) and register address at setup time.
 * Setting a pin HIGH is then a single putreg32(base | OUTPUT_BIT).
 ****************************************************************************/

typedef struct
{
  uintptr_t addr;   /* BK7258_GPIO_CFG(pin) address */
  uint32_t base_lo; /* CFG with OUTPUT bit cleared */
  uint32_t base_hi; /* CFG with OUTPUT bit set */
} gpio_cache_t;

static gpio_cache_t g_cache_sclk;
static gpio_cache_t g_cache_mosi;
static gpio_cache_t g_cache_cs;
static gpio_cache_t g_cache_dc;

/* Display geometry */

#define LCD_WIDTH      160
#define LCD_HEIGHT     160
#define LCD_BPP        16  /* RGB565 */

/* Test square size (Stage C) */

#define SQ_SIZE        40

/* SPI bit-bang delay: a few NOPs for timing margin. */

#define spi_delay() \
  do { \
    __asm__ volatile("nop; nop;"); \
  } while (0)

/****************************************************************************
 * Hardware SPI1 Register Definitions (BK7258)
 *
 * Sources (all Apache-2.0, Beken ARMino SDK):
 *   SPI1 base = SOC_SPI_REG_BASE(0x44870000) + 0x1010000 = 0x45880000
 *     (reg_base.h: SOC_SPI_REG_BASE; spi_ll.h: SPI_LL_REG_BASE(1))
 *
 *   SYS clock enable = SOC_SYS_REG_BASE(0x44010000) + 0xC*4 = 0x44010030
 *     SPI1 enable = bit[9]
 *     (sys_reg.h: SYS_CPU_DEVICE_CLK_ENABLE_ADDR, SPI1_CKEN_POS=9)
 *
 *   GPIO func select: BK7258_SYS_BASE(0x44010000) + 0xC0 + (pin/8)*4
 *     4 bits/pin, shift = (pin%8)*4
 *     (bk7258_gpio.h: BK7258_SYS_GPIO_FUNC, BK7258_GPIO_FUNC_SHIFT)
 *
 *   SPI1 pin mapping (gpio_map.h, index 0 = AF0):
 *     GPIO_2 = SPI1_SCK  (used for HW SPI)
 *     GPIO_3 = SPI1_CSN  (NOT used — auto-toggle kills GC9D01)
 *     GPIO_4 = SPI1_MOSI (used for HW SPI)
 *
 *   Register offsets (spi_struct.h: spi_hw_t, REG_0xN at offset N*4):
 *     REG_0x02 global_ctrl, REG_0x04 ctrl, REG_0x05 cfg, REG_0x06 int_status,
 *     REG_0x07 data
 *
 *   INT_STATUS bits (spi_struct.h):
 *     bit[1]  tx_fifo_wr_ready   — FIFO has space
 *     bit[13] tx_finish_int      — transfer complete
 *     bit[16] tx_fifo_clr        — write 1 to clear TX FIFO
 *
 *   Clock: XTAL = 26 MHz (defconfig: CONFIG_XTAL_FREQ=26000000)
 *     SPI_CLK_XTAL selects 26 MHz source (sys_hal.c: sys_hal_spi_select_clock)
 *     SPI clock = 26 MHz / (clk_rate + 1); clk_rate=0 → 26 MHz
 *
 *   FIFO depth: 64 bytes (empirically measured, see SPI_FIFO_DEPTH below)
 *   TX trans len: 12-bit field, max 4095 per transfer
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_GC9D01_HW_SPI

#define BK7258_SPI1_BASE             0x45880000u

/* SPI register offsets (word-indexed in spi_struct.h, *4 for byte addr) */

#define SPI_REG_GLOBAL_CTRL(base)    ((base) + 0x02 * 4)
#define SPI_REG_CTRL(base)           ((base) + 0x04 * 4)
#define SPI_REG_CFG(base)            ((base) + 0x05 * 4)
#define SPI_REG_INT_STATUS(base)     ((base) + 0x06 * 4)
#define SPI_REG_DATA(base)           ((base) + 0x07 * 4)

/* GLOBAL_CTRL bits */

#define SPI_SOFT_RESET               (1u << 0)
#define SPI_CLK_GATE_BYPASS          (1u << 1)

/* CTRL register bits (spi_struct.h: ctrl, REG_0x04) */

#define SPI_CTRL_ENABLE_BIT          (1u << 23)
#define SPI_CTRL_MASTER_BIT          (1u << 22)
#define SPI_CTRL_CPHA_BIT            (1u << 21)
#define SPI_CTRL_CPOL_BIT            (1u << 20)
#define SPI_CTRL_LSB_FIRST_BIT       (1u << 19)
#define SPI_CTRL_BIT_WIDTH_BIT       (1u << 18)
#define SPI_CTRL_CLK_RATE_SHIFT      8
#define SPI_CTRL_CLK_RATE_MASK       0xffu
#define SPI_CTRL_BYTE_INTERVAL_SHIFT 24
#define SPI_CTRL_BYTE_INTERVAL_MASK  0x3fu

/* CTRL bits[1:0] = tx_fifo_int_level: FIFO threshold for tx_fifo_int.
 * 0 = 1 empty slot, 1 = 16, 2 = 32, 3 = 48.
 * (ARMino hal_spi_types.h: SPI_FIFO_INT_LEVEL_1/16/32/48)
 */

#define SPI_CTRL_TX_FIFO_INT_LVL_SHIFT 0
#define SPI_CTRL_TX_FIFO_INT_LVL_MASK  0x3u

/* CFG register bits (spi_struct.h: cfg, REG_0x05) */


#define SPI_CFG_TX_EN_BIT            (1u << 0)
#define SPI_CFG_RX_EN_BIT            (1u << 1)
#define SPI_CFG_TX_FINISH_INT_EN_BIT (1u << 2)
#define SPI_CFG_RX_FINISH_INT_EN_BIT (1u << 3)
#define SPI_CFG_TX_TRANS_LEN_SHIFT   8
#define SPI_CFG_TX_TRANS_LEN_MASK    0xfffu

/* Maximum bytes per SPI chunk.
 * The tx_trans_len field is 12 bits (max 4095), but empirically chunk>=4095
 * causes the controller to stall: INT_STATUS reads 0x00000000 (FIFO full,
 * no tx_fifo_wr_ready), always stalls at sent=70/4095.  1024 is verified
 * stable across all existing call sites.  SPI_CFG_TX_TRANS_LEN_MASK is
 * used only for register bit-field operations, NOT as a length limit.
 * Use "lcdtest chunk <N>" to sweep and find the true hardware ceiling.
 */

#define SPI_MAX_CHUNK_BYTES          4095

/* Runtime-overridable chunk limit.  lcd_spi_write() uses this instead of
 * the compile-time macro so that "lcdtest chunk N" can probe values above
 * 1024.  Default = SPI_MAX_CHUNK_BYTES.  lcdtest_chunk() temporarily
 * overrides this and always restores it on exit (including timeout paths).
 */

static uint32_t g_spi_max_chunk = SPI_MAX_CHUNK_BYTES;

/* Runtime-adjustable SPI clock divider.
 * Default = CONFIG_LCD_GC9D01_SPI_CLK_DIV (from Kconfig, default 1 = 13 MHz).
 * "lcdtest clk <div>" sets this and re-initializes SPI1 CTRL.
 * Actual SPI clock = 26 MHz / (div + 1).
 * Range: 0..255 (CTRL register bits[8:15]).
 *
 * Measured (51200-byte blit, burst=32, BYTE_INTERVAL=0):
 *   div=12 (2.0 MHz) → 740ms / 67 KB/s   (burst=1)
 *   div=3  (6.5 MHz) → 120ms / 416 KB/s  (burst=32)
 *   div=1  (13 MHz)  →  50ms / 1000 KB/s (burst=32) ← default
 *   div=0  (26 MHz)  →  50ms / 1000 KB/s (no gain, polling ceiling)
 */

static uint32_t g_spi_clk_div = 3;   /* was CONFIG (1); 6.5MHz for
                                       * signal integrity under load */

/* Runtime-adjustable SPI burst write count.
 * Default = CONFIG_LCD_GC9D01_SPI_BURST (from Kconfig, default 32).
 * "lcdtest burst <n>" sets this.  Accepted values: 1, 16, 32.
 *
 * burst=1: classic per-byte polling via tx_fifo_wr_ready (INT_STATUS bit1).
 * burst=16/32: uses tx_fifo_int_level (CTRL bits[1:0]) to set the FIFO
 *   interrupt threshold, then polls tx_fifo_int (INT_STATUS bit8) which
 *   asserts when the FIFO occupancy <= level.  After the poll, writes
 *   N bytes to the FIFO in a tight loop (no per-byte ready check).
 *
 * Measured (51200-byte blit, lcdtest chunk 4094 sram multi):
 *   2.0 MHz  burst=1  →  740ms /   67 KB/s
 *   6.5 MHz  burst=1  →  390ms /  128 KB/s
 *   6.5 MHz  burst=16 →  120ms /  416 KB/s
 *   6.5 MHz  burst=32 →  120ms /  416 KB/s
 *   13  MHz  burst=32 →   50ms / 1000 KB/s  ← default combo
 *   26  MHz  burst=32 →   50ms / 1000 KB/s  (no further gain)
 *   burst=48 → FAIL (FIFO overflow, see below)
 *
 * The 4 hardware tx_fifo_int levels map to CTRL bits[1:0]:
 *   level 0 → occupancy ≤ 0  (burst=1, uses bit1 polling instead)
 *   level 1 → occupancy ≤ 16 (empty ≥ 48, safe to write 16)
 *   level 2 → occupancy ≤ 32 (empty ≥ 32, safe to write 32)
 *   level 3 → occupancy ≤ 48 (empty ≥ 16, safe to write only 16!)
 *
 * FIFO depth is 64, NOT 48.  The value 48 comes from ARMino's
 * SPI_FIFO_INT_LEVEL_48 enum which defines interrupt *thresholds*, not
 * capacity.  Safe write count = FIFO_DEPTH (64) - level:
 *   level=16 → safe=48, write 16 → OK
 *   level=32 → safe=32, write 32 → OK (exactly fits)
 *   level=48 → safe=16, write 48 → OVERFLOW (writes 32 extra bytes)
 *
 * IMPORTANT: tx_fifo_int (bit8) is write-1-to-clear.  It must be cleared
 * before each re-poll; otherwise a stale assertion causes an immediate
 * pass and the FIFO overflows.
 */

#ifdef CONFIG_LCD_GC9D01_SPI_BURST
static uint32_t g_spi_burst = CONFIG_LCD_GC9D01_SPI_BURST;
#else
static uint32_t g_spi_burst = 32;
#endif

/* Convert burst count (1/16/32) to CTRL tx_fifo_int_level field (0/1/2/3).
 * burst=1 → level 0 (not used for int polling, but valid for register).
 * burst=48 is intentionally NOT supported: level=3 only guarantees 16
 * empty slots (FIFO_DEPTH=64 minus level=48), but burst=48 writes 48.
 */

static uint32_t burst_to_fifo_level(uint32_t burst)
{
  if (burst >= 32) return 2;
  if (burst >= 16) return 1;
  return 0;
}

/* Convert CTRL tx_fifo_int_level register value (0/1/2/3) back to the
 * byte count that level represents.  NOT the same as the level number!
 *   level 0 →  1 byte  (burst=1, per-byte mode)
 *   level 1 → 16 bytes
 *   level 2 → 32 bytes
 *   level 3 → 48 bytes
 *
 * This mapping is the inverse of the CTRL bits[1:0] encoding.
 * Common mistake: using the level number directly as a byte count
 * (e.g. level=2 → 2 bytes instead of 32).  See DEBUG_JOURNAL §5.
 */

static uint32_t fifo_level_to_bytes(uint32_t level)
{
  switch (level)
    {
      case 0:  return 1;
      case 1:  return 16;
      case 2:  return 32;
      case 3:  return 48;
      default: return 1;
    }
}

/* INT_STATUS bits (spi_struct.h: int_status, REG_0x06) */

#define SPI_INT_TX_FIFO_WR_READY     (1u << 1)
#define SPI_INT_TX_FIFO_INT          (1u << 8)
#define SPI_INT_TX_UNDERFLOW         (1u << 11)
#define SPI_INT_TX_FINISH            (1u << 13)
#define SPI_INT_TX_FIFO_CLR          (1u << 16)
#define SPI_INT_RX_FIFO_CLR          (1u << 17)

/* System clock enable register for SPI1 */

#define BK7258_SYS_CLK_ENABLE_REG    (BK7258_SYS_BASE + 0x0c * 4)
#define SPI1_CLK_ENABLE_BIT          (1u << 9)

/* SPI clock source select register (ARMino: sys_hal_spi_select_clock).
 * Register: SYS_REG_0x0A at SYS_BASE + 0x0a * 4 = 0x44010028.
 *   bit4 = clksel_spi0 (0 = XTAL 26 MHz, 1 = APLL)
 *   bit5 = clksel_spi1 (0 = XTAL 26 MHz, 1 = APLL)
 */

#define BK7258_SYS_CLK_DIV_REG       (BK7258_SYS_BASE + 0x0a * 4)
#define SPI1_CLKSEL_BIT              (1u << 5)

/* SPI clock: XTAL 26 MHz / (clk_rate + 1).
 * CONFIG_LCD_GC9D01_SPI_CLK_DIV sets the clk_rate field (0-255).
 * Default=12 → 26/(12+1) = 2 MHz (conservative for bring-up).
 */

#define SPI1_CLK_RATE  CONFIG_LCD_GC9D01_SPI_CLK_DIV

/* SPI FIFO depth (bytes).
 *
 * IMPORTANT: The FIFO is 64 bytes deep, NOT 48.
 *
 * The value 48 comes from ARMino's SPI_FIFO_INT_LEVEL_48 enum (the largest
 * tx_fifo_int_level setting), which was mistakenly copied as the FIFO depth
 * in earlier versions.  The ARMino enum defines interrupt *thresholds*, not
 * capacity.
 *
 * Empirical proof (BK7258, clk_div=3 / 6.5 MHz, lcdtest chunk 4094 sram):
 *   tx_fifo_int_level=16  → safe to write 16 bytes  (FIFO empty ≥ 48)
 *   tx_fifo_int_level=32  → safe to write 32 bytes  (FIFO empty ≥ 32)
 *   tx_fifo_int_level=48  → OVERFLOW at write 48     (FIFO empty only ≥ 16)
 *
 * This only makes sense if FIFO depth = 64:
 *   tx_fifo_int asserts when FIFO *occupancy* ≤ level (empty ≥ 64 - level).
 *   level=16 → empty ≥ 48, write 16 → safe
 *   level=32 → empty ≥ 32, write 32 → safe (exactly fits)
 *   level=48 → empty ≥ 16, write 48 → overwrites 32 bytes → corruption
 *
 * ARMino's FIFO depth is documented as "≥48" (spi_ll.h comment), which is
 * consistent with 64 but was read as "exactly 48".  The actual depth is 64.
 *
 * Safe write count per level: safe = FIFO_DEPTH - level
 *   level 16 → safe = 48
 *   level 32 → safe = 32
 *   level 48 → safe = 16  (NOT enough for burst=48, hence the failure)
 */

#define SPI_FIFO_DEPTH               64

/* Tile buffer size for lcd_fill_rect() HW SPI path.
 * Configurable via CONFIG_LCD_GC9D01_TILE_BYTES (default 1024).
 * Must be ≤4095 (tx_trans_len 12-bit field) and even (RGB565 pixel pairs).
 */

#ifdef CONFIG_LCD_GC9D01_HW_SPI
#  define SPI_TILE_BYTES  CONFIG_LCD_GC9D01_TILE_BYTES
#else
#  define SPI_TILE_BYTES  SPI_FIFO_DEPTH
#endif

/* Timeout for SPI status polling loops.
 * Max chunk = 4095 bytes @ 2 MHz ≈ 16.4 ms.  Each poll iteration is a
 * single getreg32 (~2-3 bus cycles @ 26 MHz CPU) ≈ 0.1 μs.
 * 500,000 iterations ≈ 50 ms, which is ~3× the max transfer time.
 */

#define SPI_POLL_TIMEOUT             500000u

/* Minimum data length to use hardware SPI.
 * BK7258 SPI does NOT assert tx_finish_int when tx_trans_len == 1
 * (observed empirically: all len=1 transfers timeout with
 * INT_STATUS=0x00000003, while len>=2 always gets bit13).
 * Panel init commands are mostly 1-byte, so routing them through HW SPI
 * wastes ~3 ms timeout per byte.  Bit-bang is faster for short writes.
 * Set via CONFIG_LCD_GC9D01_HWSPI_MIN_LEN (default 8).
 */

#define SPI_HWSPI_MIN_LEN  CONFIG_LCD_GC9D01_HWSPI_MIN_LEN

/****************************************************************************
 * HW SPI State
 ****************************************************************************/

static bool g_hw_spi_capable;     /* SPI1 initialized successfully */
static bool g_pins_in_spi_mode;   /* false=GPIO, true=SPI peripheral */
static uint32_t g_saved_sys_clk_en;
static uint32_t g_saved_sys_clk_div;  /* SPI clock source select register */

/* HW SPI1 is physically wired only to the left screen (P2=SCLK, P4=MOSI).
 * The right screen (P22/P23/P24) has no SPI peripheral — it must always
 * use bit-bang.  This helper prevents accidental HW SPI on the wrong panel.
 */

static inline bool lcd_hw_spi_usable(void)
{
#ifdef CONFIG_LCD_GC9D01_HW_SPI
  return g_hw_spi_capable && g_active_pins == &g_lcd_left;
#else
  return false;
#endif
}

/* Forward declarations — called by lcd_spi_init / lcd_spidiag before definition */

static void lcd_spi_pins_to_spi(void);
static void lcd_spi_pins_to_gpio(void);
static void lcd_set_pins(const lcd_pins_t *pins);
static void lcd_setup_pins(const lcd_pins_t *pins);

#endif /* CONFIG_LCD_GC9D01_HW_SPI */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_set_output
 ****************************************************************************/

static void gpio_set_output(int pin)
{
  uint32_t cfg = getreg32(BK7258_GPIO_CFG(pin));

  cfg &= ~GPIO_CFG_SECOND_FUNC;  /* GPIO mode (bit6 = 0) */
  cfg &= ~GPIO_CFG_OUTPUT_EN;    /* output enable (bit3 active-low: 0 = on) */
  cfg &= ~GPIO_CFG_INPUT_EN;     /* input off (bit2: 0=off) */
  putreg32(cfg, BK7258_GPIO_CFG(pin));
}

/****************************************************************************
 * Name: gpio_write
 ****************************************************************************/

static void gpio_write(int pin, int val)
{
  uint32_t cfg = getreg32(BK7258_GPIO_CFG(pin));

  if (val)
    {
      cfg |= GPIO_CFG_OUTPUT;
    }
  else
    {
      cfg &= ~GPIO_CFG_OUTPUT;
    }

  putreg32(cfg, BK7258_GPIO_CFG(pin));
}

/****************************************************************************
 * Name: gpio_set_output_cached
 *
 * Description:
 *   Configure pin as GPIO output and cache the CFG register values
 *   for fast putreg32-only writes (no getreg32 read needed later).
 *
 ****************************************************************************/

static void gpio_set_output_cached(int pin, gpio_cache_t *c)
{
  uint32_t cfg;

  c->addr = BK7258_GPIO_CFG(pin);
  cfg = getreg32(c->addr);
  cfg &= ~GPIO_CFG_SECOND_FUNC;
  cfg &= ~GPIO_CFG_OUTPUT_EN;
  cfg &= ~GPIO_CFG_INPUT_EN;
  cfg &= ~GPIO_CFG_OUTPUT;   /* output = 0 */
  putreg32(cfg, c->addr);
  c->base_lo = cfg;            /* OUTPUT=0 */
  c->base_hi = cfg | GPIO_CFG_OUTPUT; /* OUTPUT=1 */
}

/****************************************************************************
 * Name: gpio_write_fast
 *
 * Description:
 *   Set pin via cached values — no read-modify-write, just a single
 *   putreg32.  ~2x faster than gpio_write().
 *
 ****************************************************************************/

static inline void gpio_write_fast(const gpio_cache_t *c, int val)
{
  putreg32(val ? c->base_hi : c->base_lo, c->addr);
}

/****************************************************************************
 * Name: gpio_set_second_func
 *
 * Description:
 *   Configure a GPIO pin for peripheral (second) function.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_GC9D01_HW_SPI

static void gpio_set_second_func(int pin, int func_sel)
{
  uint32_t cfg;
  uintptr_t func_reg;
  uint32_t shift;
  uint32_t val;

  cfg = getreg32(BK7258_GPIO_CFG(pin));
  cfg |= GPIO_CFG_SECOND_FUNC;    /* second function mode (bit6=1) */
  cfg &= ~GPIO_CFG_OUTPUT_EN;     /* ENABLE GPIO output (active-low: bit3=0=on) */
  cfg &= ~GPIO_CFG_INPUT_EN;      /* disable input (bit2=0) */
  cfg &= ~GPIO_CFG_PULL_EN;       /* disable pull (bit5=0) */

  /* Set drive capacity to 1 for SCK pins (matches ARMino SPI_SET_PIN).
   * GPIO capacity is bits[8:9], max=3.  SCK needs stronger drive.
   * MOSI uses default capacity=0.
   */

  if (pin == g_lcd_left.sclk)
    {
      cfg &= ~(0x3u << 8);        /* clear capacity bits */
      cfg |= (1u << 8);           /* capacity=1 for SCK */
    }

  putreg32(cfg, BK7258_GPIO_CFG(pin));

  func_reg = BK7258_SYS_GPIO_FUNC(pin);
  shift = BK7258_GPIO_FUNC_SHIFT(pin);
  val = getreg32(func_reg);
  val &= ~(BK7258_GPIO_FUNC_MASK << shift);
  val |= ((uint32_t)(func_sel & BK7258_GPIO_FUNC_MASK) << shift);
  putreg32(val, func_reg);
}

/****************************************************************************
 * Name: lcd_spi_init
 *
 * Description:
 *   Initialize BK7258 SPI1 for left-screen LCD data transfers.
 *   Returns 0 on success, negative errno on failure.
 *   On failure, g_hw_spi_capable stays false — caller falls back to bit-bang.
 *
 *   Init sequence (matching ARMino spi_id_init_common + spi_hal_init
 *     + spi_hal_configure + spi_hal_start_common):
 *     1. Enable SPI1 module clock, verify by read-back
 *     2. Soft-reset (NO CLK_GATE_BYPASS — ARMino doesn't use it)
 *     3. Configure CTRL: master, enable, mode 0, 8-bit, MSB first,
 *        byte_interval=1, rx_sample_edge=1, interrupt enables
 *     4. Configure CFG: TX enable, TX/RX finish int enable
 *     5. Clear all interrupt status
 *     6. Clear FIFOs
 *
 ****************************************************************************/

static int lcd_spi_init(const lcd_pins_t *pins)
{
  uint32_t ctrl_val;
  uint32_t cfg_val;
  uint32_t readback;
  uint32_t cfg2;
  uint32_t cfg4;
  uint32_t func2;
  uint32_t func4;
  uintptr_t func_reg;
  uint32_t shift;

  if (pins->sclk != 2 || pins->cs != 3 || pins->mosi != 4)
    {
      g_hw_spi_capable = false;
      return -EINVAL;
    }

  /* Step 1: Enable SPI1 module clock, verify by read-back */

  g_saved_sys_clk_en = getreg32(BK7258_SYS_CLK_ENABLE_REG);
  putreg32(g_saved_sys_clk_en | SPI1_CLK_ENABLE_BIT,
           BK7258_SYS_CLK_ENABLE_REG);

  readback = getreg32(BK7258_SYS_CLK_ENABLE_REG);

  if (!(readback & SPI1_CLK_ENABLE_BIT))
    {
      syslog(LOG_ERR,
             "[lcd] SPI1 clock enable FAILED: wrote 0x%08lx, "
             "read 0x%08lx\n",
             (unsigned long)(g_saved_sys_clk_en | SPI1_CLK_ENABLE_BIT),
             (unsigned long)readback);
      g_hw_spi_capable = false;
      return -EIO;
    }

  /* Step 1b: Select XTAL as SPI1 clock source.
   * ARMino: sys_hal_spi_select_clock(SPI_ID_1, SPI_CLK_XTAL)
   * Register 0x44010028 bit5: 0 = XTAL 26 MHz, 1 = APLL.
   * Without this, SPI1 may have no clock source and never runs.
   */

  g_saved_sys_clk_div = getreg32(BK7258_SYS_CLK_DIV_REG);
  putreg32(g_saved_sys_clk_div & ~SPI1_CLKSEL_BIT,
           BK7258_SYS_CLK_DIV_REG);

  /* Step 2: Soft-reset (ARMino spi_ll_init — NO CLK_GATE_BYPASS) */

  putreg32(SPI_SOFT_RESET, SPI_REG_GLOBAL_CTRL(BK7258_SPI1_BASE));

  /* Step 3: Configure CTRL — match ARMino spi_ll_init + spi_hal_configure
   *
   * spi_ll_init sets:
   *   soft_reset → byte_interval=1 → rx_sample_edge=1 → fifo_levels=0
   *
   * spi_hal_configure sets:
   *   tx_udf_int_en=1 → rx_ovf_int_en=1 → tx_fifo_int_en=0
   *   → rx_fifo_int_en=1 → clk_rate → bit_width → bit_order
   *   → cpol → cpha → master → wire3_en=0 → slave_release_int_en=0
   *   → tx_finish_int_en=1 → rx_finish_int_en=1 → clear_int_status
   *
   * spi_hal_start_common (LAST): enable=1
   *
   * CTRL REG_0x04 layout (spi_struct.h):
   *   bit[0:1]   tx_fifo_int_level   = 0
   *   bit[2:3]   rx_fifo_int_level   = 0
   *   bit[4]     tx_udf_int_en       = 1
   *   bit[5]     rx_ovf_int_en       = 1
   *   bit[6]     tx_fifo_int_en      = 0
   *   bit[7]     rx_fifo_int_en      = 1
   *   bit[8:15]  clk_rate            = CONFIG_LCD_GC9D01_SPI_CLK_DIV
   *   bit[16]    slave_release_int_en= 0
   *   bit[17]    wire3_en            = 0
   *   bit[18]    bit_width           = 0 (8-bit)
   *   bit[19]    lsb_first_en        = 0 (MSB first)
   *   bit[20]    cpol                = 0
   *   bit[21]    cpha                = 0
   *   bit[22]    master_en           = 1
   *   bit[23]    enable              = 1
   *   bit[24:29] byte_interval       = 1
   *   bit[30:31] rx_sample_edge      = 1
   */

  /* tx_fifo_int_level (CTRL bits[1:0]) and tx_fifo_int_en (bit6) are set
   * when g_spi_burst > 1.  tx_fifo_int_en gates the tx_fifo_int status bit.
   */

  uint32_t fifo_lvl = burst_to_fifo_level(g_spi_burst);
  uint32_t fifo_int_en = (g_spi_burst > 1) ? (1u << 6) : 0; /* tx_fifo_int_en */

  ctrl_val = (1u << 4)  |   /* tx_udf_int_en */
             (1u << 5)  |   /* rx_ovf_int_en */
             (1u << 7)  |   /* rx_fifo_int_en */
             fifo_int_en  |  /* tx_fifo_int_en (when burst>1) */
             (fifo_lvl << SPI_CTRL_TX_FIFO_INT_LVL_SHIFT) |
             (g_spi_clk_div << SPI_CTRL_CLK_RATE_SHIFT) |
             SPI_CTRL_MASTER_BIT |
             SPI_CTRL_ENABLE_BIT |
             ((uint32_t)CONFIG_LCD_GC9D01_SPI_BYTE_INTERVAL
                       << SPI_CTRL_BYTE_INTERVAL_SHIFT) |
             (1u << 30);     /* rx_sample_edge=1 (spi_ll_set_rx_sample_edge) */

  putreg32(ctrl_val, SPI_REG_CTRL(BK7258_SPI1_BASE));

  /* Step 4: Configure CFG — TX enable, TX/RX finish int enable
   * (matches spi_hal_configure: tx_en, tx_finish_int_en, rx_finish_int_en)
   */

  cfg_val = SPI_CFG_TX_EN_BIT |
            SPI_CFG_TX_FINISH_INT_EN_BIT |
            SPI_CFG_RX_FINISH_INT_EN_BIT;
  putreg32(cfg_val, SPI_REG_CFG(BK7258_SPI1_BASE));

  /* Step 5: Clear interrupt status (matches spi_hal_configure last step) */

  putreg32(0xFFFFFFFF, SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

  /* Step 6: Clear FIFOs */

  putreg32(SPI_INT_TX_FIFO_CLR | SPI_INT_RX_FIFO_CLR,
           SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

  /* Verify CTRL was accepted + check SPI mode config */

  readback = getreg32(SPI_REG_CTRL(BK7258_SPI1_BASE));

  if (!(readback & SPI_CTRL_ENABLE_BIT))
    {
      syslog(LOG_ERR,
             "[lcd] SPI1 CTRL enable FAILED: wrote 0x%08lx, "
             "read 0x%08lx\n",
             (unsigned long)ctrl_val, (unsigned long)readback);
      g_hw_spi_capable = false;
      return -EIO;
    }

  /* Verify SPI controller config matches GC9D01 requirements:
   *   mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit, master
   */

  if (readback & SPI_CTRL_CPHA_BIT)
    {
      syslog(LOG_WARNING,
             "[lcd] WARNING: CPHA=1 (expected 0 for mode 0)\n");
    }

  if (readback & SPI_CTRL_CPOL_BIT)
    {
      syslog(LOG_WARNING,
             "[lcd] WARNING: CPOL=1 (expected 0 for mode 0)\n");
    }

  if (readback & SPI_CTRL_LSB_FIRST_BIT)
    {
      syslog(LOG_WARNING,
             "[lcd] WARNING: LSB first (expected MSB first)\n");
    }

  if (readback & SPI_CTRL_BIT_WIDTH_BIT)
    {
      syslog(LOG_WARNING,
             "[lcd] WARNING: bit_width=1 (expected 0 for 8-bit)\n");
    }

  g_hw_spi_capable = true;
  g_pins_in_spi_mode = false;

  /* Mux readback diagnostics: print GPIO CFG and func select once.
   * Do this here (not per-frame) to avoid syslog overhead.
   */

  lcd_spi_pins_to_spi();

  cfg2 = getreg32(BK7258_GPIO_CFG(g_lcd_left.sclk));
  cfg4 = getreg32(BK7258_GPIO_CFG(g_lcd_left.mosi));

  func_reg = BK7258_SYS_GPIO_FUNC(g_lcd_left.sclk);
  shift = BK7258_GPIO_FUNC_SHIFT(g_lcd_left.sclk);
  func2 = (getreg32(func_reg) >> shift) & BK7258_GPIO_FUNC_MASK;

  func_reg = BK7258_SYS_GPIO_FUNC(g_lcd_left.mosi);
  shift = BK7258_GPIO_FUNC_SHIFT(g_lcd_left.mosi);
  func4 = (getreg32(func_reg) >> shift) & BK7258_GPIO_FUNC_MASK;

  /* Restore to GPIO mode — pins stay GPIO until data transfer time */

  lcd_spi_pins_to_gpio();

  syslog(LOG_INFO,
         "[lcd] SPI1 init OK: clk_div=%d (~%d.%d MHz), "
         "byte_interval=%d, CTRL=0x%08lX CFG=0x%08lX\n"
         "[lcd]   mux: P%d cfg=0x%05lX func=%lu, "
         "P%d cfg=0x%05lX func=%lu\n"
         "[lcd]   mode: %s, %s, %s, enable=%d\n"
         "[lcd]   clk: en_reg=0x%08lX div_reg=0x%08lX "
         "(SPI1_en=%d clksel=%s)\n",
         (int)g_spi_clk_div,
         26 / (int)(g_spi_clk_div + 1),
         (26 * 10 / (int)(g_spi_clk_div + 1)) % 10,
         CONFIG_LCD_GC9D01_SPI_BYTE_INTERVAL,
         (unsigned long)readback,
         (unsigned long)getreg32(SPI_REG_CFG(BK7258_SPI1_BASE)),
         g_lcd_left.sclk, (unsigned long)cfg2, (unsigned long)func2,
         g_lcd_left.mosi, (unsigned long)cfg4, (unsigned long)func4,
         (readback & SPI_CTRL_CPHA_BIT) ? "CPHA=1" : "CPHA=0",
         (readback & SPI_CTRL_CPOL_BIT) ? "CPOL=1" : "CPOL=0",
         (readback & SPI_CTRL_LSB_FIRST_BIT) ? "LSB first" : "MSB first",
         !!(readback & SPI_CTRL_ENABLE_BIT),
         (unsigned long)getreg32(BK7258_SYS_CLK_ENABLE_REG),
         (unsigned long)getreg32(BK7258_SYS_CLK_DIV_REG),
         !!(getreg32(BK7258_SYS_CLK_ENABLE_REG) & SPI1_CLK_ENABLE_BIT),
         (getreg32(BK7258_SYS_CLK_DIV_REG) & SPI1_CLKSEL_BIT)
           ? "APLL" : "XTAL");

  return 0;
}

/****************************************************************************
 * Name: lcd_spi_deinit
 ****************************************************************************/

static void lcd_spi_deinit(void)
{
  uint32_t val;

  if (!g_hw_spi_capable)
    {
      return;
    }

  putreg32(0, SPI_REG_CTRL(BK7258_SPI1_BASE));

  if (g_pins_in_spi_mode)
    {
      gpio_set_output_cached(g_active_pins->sclk, &g_cache_sclk);
      gpio_set_output_cached(g_active_pins->mosi, &g_cache_mosi);
      gpio_set_output_cached(g_active_pins->cs,   &g_cache_cs);
      g_pins_in_spi_mode = false;
    }

  /* Restore only the SPI1-related bits in shared clock registers.
   * 0x44010030 bit9 = SPI1 module clock enable
   * 0x44010028 bit5 = SPI1 clock source select (CLKSEL)
   * DVP camera also uses 0x44010028 for its own clock fields.
   * Full-word write with the init-time snapshot would clobber DVP bits.
   */

  val = getreg32(BK7258_SYS_CLK_ENABLE_REG);
  val &= ~SPI1_CLK_ENABLE_BIT;               /* clear SPI1 clock enable */
  val |= (g_saved_sys_clk_en & SPI1_CLK_ENABLE_BIT);  /* restore original */
  putreg32(val, BK7258_SYS_CLK_ENABLE_REG);

  val = getreg32(BK7258_SYS_CLK_DIV_REG);
  val &= ~SPI1_CLKSEL_BIT;                   /* clear SPI1 CLKSEL */
  val |= (g_saved_sys_clk_div & SPI1_CLKSEL_BIT);     /* restore original */
  putreg32(val, BK7258_SYS_CLK_DIV_REG);

  g_hw_spi_capable = false;
}

/****************************************************************************
 * Name: lcd_spi_pins_to_spi / lcd_spi_pins_to_gpio
 *
 * Description:
 *   Only SCLK (P2) and MOSI (P4) are muxed to SPI1 peripheral.
 *   CS (P3) and DC (P5) STAY as GPIO outputs — software controls them.
 *
 *   Why CS stays GPIO: SPI1_CSN is auto-toggled by the controller
 *   per byte/transfer.  GC9D01 requires CS low across the entire
 *   command+data sequence.  Auto-toggling CS resets the panel state
 *   machine and all commands are lost.
 *
 ****************************************************************************/

static void lcd_spi_pins_to_spi(void)
{
  /* Hardware SPI1 is only wired to the left screen (P2/P4).
   * If the active panel is not the left one, do nothing — the caller
   * must not attempt HW SPI on the right screen.
   */

  if (g_pins_in_spi_mode)
    {
      return;
    }

  if (g_active_pins != &g_lcd_left)
    {
      return;
    }

  /* Only SCLK and MOSI → SPI1 peripheral.  CS stays GPIO. */

  gpio_set_second_func(g_active_pins->sclk, 0);  /* SPI1_SCK  func0 */
  gpio_set_second_func(g_active_pins->mosi, 0);  /* SPI1_MOSI func0 */

  g_pins_in_spi_mode = true;
}

static void lcd_spi_pins_to_gpio(void)
{
  if (!g_pins_in_spi_mode)
    {
      return;
    }

  /* Restore SCLK and MOSI to GPIO output mode using the ACTIVE panel's
   * pins, not hardcoded g_lcd_left.  This prevents corrupting the GPIO
   * cache when the active panel has changed since to_spi() was called.
   */

  gpio_set_output_cached(g_active_pins->sclk, &g_cache_sclk);
  gpio_set_output_cached(g_active_pins->mosi, &g_cache_mosi);

  g_pins_in_spi_mode = false;
}

/****************************************************************************
 * Name: lcd_spi_write
 *
 * Description:
 *   Write data bytes via hardware SPI1 with timeout-protected polling.
 *   Returns 0 on success, -ETIMEDOUT if FIFO stalls.
 *   If sent_out is non-NULL, *sent_out receives the number of bytes
 *   successfully written before return (0 on success = full len).
 *
 * HARDWARE NOTE 1: BK7258 SPI does NOT assert tx_finish_int (bit13) when
 *   tx_trans_len == 1.  Observed empirically: all len=1 transfers timeout
 *   with INT_STATUS=0x00000003, while len>=2 always get bit13.
 *   Callers should route len < SPI_HWSPI_MIN_LEN through bit-bang instead.
 *
 * HARDWARE NOTE 2: tx_en rising edge latches trans_len.
 *   The controller loads tx_trans_len into its internal counter on the
 *   rising edge of tx_en (0→1).  If tx_en stays 1 across chunks, the
 *   new trans_len is never loaded and the FIFO stalls after ~70 bytes.
 *   Therefore each chunk MUST: (1) clear tx_en, (2) set trans_len,
 *   (3) set tx_en (rising edge).  At the end of each chunk, tx_en must
 *   be cleared again so the next chunk gets a fresh rising edge.
 *   ARMino reference: spi_driver.c spi_hal_enable_tx() is always a
 *   separate register write after spi_hal_set_tx_trans_len().
 *
 ****************************************************************************/

static int lcd_spi_write(const uint8_t *data, size_t len,
                         size_t *sent_out)
{
  size_t remaining = len;
  const uint8_t *p = data;
  uint32_t cfg;

  /* Guard: HW SPI only works with left screen pins (P2/P3/P4).
   * If caches point to right screen, CS/DC would go to P23/P7 —
   * left screen ignores all data, right screen gets no clock.
   */

  if (g_cached_pins != &g_lcd_left)
    {
      syslog(LOG_ERR,
             "[lcd] lcd_spi_write called with wrong pins "
             "(cached=%s, need left)\n",
             g_cached_pins ? "right" : "none");
      return -EINVAL;
    }
#ifdef CONFIG_LCD_GC9D01_SPI_VERIFY
  size_t bytes_written = 0;
  size_t chunks_sent = 0;
#endif

  if (sent_out != NULL)
    {
      *sent_out = 0;
    }

  while (remaining > 0)
    {
      size_t chunk = remaining;
      uint32_t poll_cnt;
      uint32_t st;

      if (chunk > g_spi_max_chunk)
        {
          chunk = g_spi_max_chunk;
        }

      /* [A] Clear TX/RX FIFO (bit16, bit17).
       * ARMino: spi_hal_clear_tx_fifo → spi_ll_clear_tx_fifo (bit16=1)
       */

      putreg32(SPI_INT_TX_FIFO_CLR | SPI_INT_RX_FIFO_CLR,
               SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

      /* [A2] Clear status bits 8..14 only (write-1-to-clear).
       * Do NOT write 0xFFFFFFFF — that would pulse bit16/17 (FIFO clear)
       * again, which is unnecessary and could disturb the FIFO state.
       * Bits: 8=tx_fifo_int, 9=rx_fifo_int, 10=rx_finish,
       *       11=tx_underflow, 12=tx_finish, 13=rx_underflow, 14=cs_done
       */

      putreg32(0x7F00u, SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

      /* [B] Set TX transfer length with tx_en = 0.
       * This must be a SEPARATE step from [C] — the controller latches
       * trans_len on the tx_en rising edge.  If we set trans_len and
       * tx_en=1 in the same write, there is no rising edge for the
       * second chunk (tx_en was already 1 from the previous chunk).
       *
       * ARMino: spi_hal_set_tx_trans_len(size) — only touches trans_len
       *         field, tx_en is left as-is (0 at this point).
       */

      cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
      cfg &= ~SPI_CFG_TX_EN_BIT;  /* tx_en = 0 — prerequisite for rising edge */
      cfg &= ~(SPI_CFG_TX_TRANS_LEN_MASK << SPI_CFG_TX_TRANS_LEN_SHIFT);
      cfg |= (chunk << SPI_CFG_TX_TRANS_LEN_SHIFT);
      cfg |= SPI_CFG_TX_FINISH_INT_EN_BIT; /* preserve bit2 */
      putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));

      /* [C] Enable TX — rising edge (0→1) latches trans_len.
       * This MUST be a separate putreg32 from [B].
       * ARMino: spi_hal_enable_tx() — separate register write.
       */

      cfg |= SPI_CFG_TX_EN_BIT;
      putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));

      /* [D] Feed FIFO.
       *
       * g_spi_burst=1 (default): per-byte polling via tx_fifo_wr_ready
       *   (INT_STATUS bit1).  Matches ARMino spi_id_write_bytes_common.
       *
       * g_spi_burst=16/32: uses tx_fifo_int (INT_STATUS bit8).
       *   CTRL bits[1:0] = level (1→16, 2→32).
       *   tx_fifo_int asserts when FIFO occupancy <= level
       *   (i.e. empty >= FIFO_DEPTH(64) - level).
       *   bit8 is write-1-to-clear: must clear before each re-poll,
       *   otherwise a stale assertion causes immediate pass → overflow.
       *
       *   burst=48 is NOT supported: level=3 only guarantees 16 empty
       *   slots, writing 48 overflows.  See SPI_FIFO_DEPTH comment.
       */

      size_t i;

      for (i = 0; i < chunk; )
        {
          size_t burst = chunk - i;
          size_t b;

          if (burst > g_spi_burst)
            {
              burst = g_spi_burst;
            }

          if (g_spi_burst <= 1)
            {
              /* ---- burst=1: per-byte tx_fifo_wr_ready polling ---- */

              poll_cnt = SPI_POLL_TIMEOUT;

              while (!(getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE)) &
                       SPI_INT_TX_FIFO_WR_READY))
                {
                  if (--poll_cnt == 0)
                    {
                      size_t done = (size_t)(p - data) + i;

                      st = getreg32(SPI_REG_INT_STATUS(
                                      BK7258_SPI1_BASE));

                      syslog(LOG_ERR,
                             "[lcd] SPI TX FIFO timeout, "
                             "INT_STATUS=0x%08lx sent=%lu/%lu "
                             "total_done=%lu/%lu\n",
                             (unsigned long)st,
                             (unsigned long)i,
                             (unsigned long)chunk,
                             (unsigned long)done,
                             (unsigned long)len);
                      cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
                      cfg &= ~SPI_CFG_TX_EN_BIT;
                      putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));
                      lcd_spi_pins_to_gpio();
                      if (sent_out != NULL)
                        {
                          *sent_out = done;
                        }

                      return -ETIMEDOUT;
                    }
                }

              putreg32(p[i], SPI_REG_DATA(BK7258_SPI1_BASE));
              i++;
            }
          else
            {
              /* ---- burst>1: tx_fifo_int level polling ---- */

              /* Clear tx_fifo_int (write-1-to-clear bit8) BEFORE polling.
               * A stale assertion from a previous iteration would cause an
               * immediate pass and FIFO overflow.
               */

              putreg32(SPI_INT_TX_FIFO_INT,
                       SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

              poll_cnt = SPI_POLL_TIMEOUT;

              while (!(getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE)) &
                       SPI_INT_TX_FIFO_INT))
                {
                  if (--poll_cnt == 0)
                    {
                      size_t done = (size_t)(p - data) + i;

                      st = getreg32(SPI_REG_INT_STATUS(
                                      BK7258_SPI1_BASE));

                      syslog(LOG_ERR,
                             "[lcd] SPI TX FIFO INT timeout, "
                             "INT_STATUS=0x%08lx sent=%lu/%lu "
                             "total_done=%lu/%lu burst=%lu\n",
                             (unsigned long)st,
                             (unsigned long)i,
                             (unsigned long)chunk,
                             (unsigned long)done,
                             (unsigned long)len,
                             (unsigned long)g_spi_burst);
                      cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
                      cfg &= ~SPI_CFG_TX_EN_BIT;
                      putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));
                      lcd_spi_pins_to_gpio();
                      if (sent_out != NULL)
                        {
                          *sent_out = done;
                        }

                      return -ETIMEDOUT;
                    }
                }

              /* Write burst bytes to FIFO (no per-byte ready check).
               * The FIFO has >= burst empty slots at this point.
               */

              for (b = 0; b < burst; b++)
                {
                  putreg32(p[i + b], SPI_REG_DATA(BK7258_SPI1_BASE));
                }

              i += burst;
            }
        }

#ifdef CONFIG_LCD_GC9D01_SPI_VERIFY
      bytes_written += chunk;
      chunks_sent++;
#endif

      /* [E] Wait for TX finish (bit13) — the controller has clocked out
       * all tx_trans_len bytes.
       * ARMino: ISR checks spi_ll_is_tx_finish_int_triggered(status, BIT(13)),
       *         then calls bk_spi_clr_tx → disable_tx.
       * We poll bit13 directly.  Also accept bit11 (tx_underflow) as a
       * secondary indicator — it fires when the FIFO drains faster than
       * expected, which is benign and means data was sent.
       *
       * CRITICAL: do NOT clear INT_STATUS between feeding and waiting.
       */

      poll_cnt = SPI_POLL_TIMEOUT;

      for (;;)
        {
          st = getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

          if (st & (SPI_INT_TX_FINISH | SPI_INT_TX_UNDERFLOW))
            {
              break;
            }

          if (--poll_cnt == 0)
            {
              size_t done = (size_t)(p - data) + chunk;

              syslog(LOG_ERR,
                     "[lcd] SPI TX finish timeout, "
                     "INT_STATUS=0x%08lx sent=%lu/%lu "
                     "total_done=%lu/%lu\n",
                     (unsigned long)st,
                     (unsigned long)chunk,
                     (unsigned long)chunk,
                     (unsigned long)done,
                     (unsigned long)len);
              cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
              cfg &= ~SPI_CFG_TX_EN_BIT;
              putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));
              lcd_spi_pins_to_gpio();
              if (sent_out != NULL)
                {
                  *sent_out = done;
                }

              return -ETIMEDOUT;
            }
        }

      /* tx_finish_int (bit13) fires when the last byte enters the shift
       * register, NOT when it has fully shifted out.  The final 8 clocks
       * (4 μs @ 2 MHz) may still be in progress.  Disabling tx_en
       * immediately truncates that byte, causing cumulative byte
       * misalignment — visible as diagonal seams on structured content.
       *
       * Cost: 10 μs × 50 tiles/frame = 0.5 ms per frame @ tile=1024,
       * which is 0.24% of 206 ms frame time.  Negligible.  DO NOT
       * remove this delay again without scope-verifying the MOSI line.
       */

      up_udelay(10);

      /* [F] Clear completion bits (write-1-to-clear) and disable tx_en.
       *
       * tx_en MUST be cleared here — it is the prerequisite for the next
       * chunk's rising edge.  Without this, the next iteration's
       * [B]+[C] cannot produce a 0→1 transition, and the new trans_len
       * is never latched.  This was previously removed as "redundant"
       * and caused multi-chunk stalls (INT_STATUS=0, sent=70).
       * ARMino: spi_hal_disable_tx() is called after every chunk.
       */

      putreg32(st & (SPI_INT_TX_FINISH | SPI_INT_TX_UNDERFLOW),
               SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

      cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
      cfg &= ~SPI_CFG_TX_EN_BIT;
      putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));

      p += chunk;
      remaining -= chunk;
    }

#ifdef CONFIG_LCD_GC9D01_SPI_VERIFY
  if (bytes_written != len)
    {
      syslog(LOG_ERR,
             "[lcd] SPI VERIFY FAIL: requested=%zu written=%zu "
             "delta=%zd chunks=%zu\n",
             len, bytes_written,
             (ssize_t)len - (ssize_t)bytes_written,
             chunks_sent);
    }
#endif

  if (sent_out != NULL)
    {
      *sent_out = len;
    }

  return 0;
}

#endif /* CONFIG_LCD_GC9D01_HW_SPI */

/****************************************************************************
 * Name: lcd_spidiag
 *
 * Description:
 *   Minimal SPI1 diagnostic: send 1 cmd byte + 4 data bytes, printing
 *   register snapshots at every decision point.  Invoked as "lcd spidiag".
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_GC9D01_HW_SPI

static void spidiag_decode_ctrl(uint32_t ctrl)
{
  syslog(LOG_INFO,
         "[spidiag]   master=%d enable=%d cpol=%d cpha=%d "
         "clk_rate=%lu tx_udf_int_en=%d\n",
         !!(ctrl & SPI_CTRL_MASTER_BIT),
         !!(ctrl & SPI_CTRL_ENABLE_BIT),
         !!(ctrl & SPI_CTRL_CPOL_BIT),
         !!(ctrl & SPI_CTRL_CPHA_BIT),
         (unsigned long)((ctrl >> SPI_CTRL_CLK_RATE_SHIFT) & SPI_CTRL_CLK_RATE_MASK),
         !!(ctrl & (1u << 4)));
}

static void spidiag_decode_cfg(uint32_t cfg)
{
  syslog(LOG_INFO,
         "[spidiag]   tx_en=%d rx_en=%d tx_finish_int_en=%d "
         "rx_finish_int_en=%d tx_trans_len=%lu\n",
         !!(cfg & SPI_CFG_TX_EN_BIT),
         !!(cfg & SPI_CFG_RX_EN_BIT),
         !!(cfg & SPI_CFG_TX_FINISH_INT_EN_BIT),
         !!(cfg & SPI_CFG_RX_FINISH_INT_EN_BIT),
         (unsigned long)((cfg >> SPI_CFG_TX_TRANS_LEN_SHIFT) & SPI_CFG_TX_TRANS_LEN_MASK));
}

static int lcd_spidiag(void)
{
  uint32_t ctrl;
  uint32_t cfg;
  uint32_t st;
  uint32_t poll_cnt;
  uint32_t cfg_before;
  uint32_t cfg_after;
  uint32_t func_before;
  uint32_t func_after;
  uintptr_t func_reg;
  uint32_t shift;
  int i;

  static const uint8_t diag_data[4] = {0xaa, 0x55, 0x33, 0xcc};

  /* Self-contained: ensure left screen pins + SPI1 are initialized.
   * lcd_setup_pins is idempotent (skips if already done for this panel).
   * This allows running "lcdtest spidiag" without a prior "lcdtest go".
   * We only init pins+SPI, NOT the panel display sequence.
   */

  lcd_setup_pins(&g_lcd_left);

  if (!g_hw_spi_capable)
    {
      syslog(LOG_ERR, "[spidiag] HW SPI not capable, abort\n");
      return -ENODEV;
    }

  /* [F-before] GPIO pin state BEFORE mux */

  cfg_before = getreg32(BK7258_GPIO_CFG(g_lcd_left.sclk));
  func_reg = BK7258_SYS_GPIO_FUNC(g_lcd_left.sclk);
  shift = BK7258_GPIO_FUNC_SHIFT(g_lcd_left.sclk);
  func_before = (getreg32(func_reg) >> shift) & BK7258_GPIO_FUNC_MASK;

  syslog(LOG_INFO,
         "[spidiag] GPIO BEFORE mux: P%d cfg=0x%05lX func=%lu\n",
         g_lcd_left.sclk, (unsigned long)cfg_before,
         (unsigned long)func_before);

  /* Mux to SPI */

  lcd_spi_pins_to_spi();

  /* [F-after] GPIO pin state AFTER mux */

  cfg_after = getreg32(BK7258_GPIO_CFG(g_lcd_left.sclk));
  func_reg = BK7258_SYS_GPIO_FUNC(g_lcd_left.sclk);
  shift = BK7258_GPIO_FUNC_SHIFT(g_lcd_left.sclk);
  func_after = (getreg32(func_reg) >> shift) & BK7258_GPIO_FUNC_MASK;

  syslog(LOG_INFO,
         "[spidiag] GPIO AFTER  mux: P%d cfg=0x%05lX func=%lu\n",
         g_lcd_left.sclk, (unsigned long)cfg_after,
         (unsigned long)func_after);

  /* [A] Post-init register dump */

  ctrl = getreg32(SPI_REG_CTRL(BK7258_SPI1_BASE));
  cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
  st = getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

  syslog(LOG_INFO,
         "[spidiag] [A] post-init: CTRL=0x%08lX CFG=0x%08lX "
         "INT_STATUS=0x%08lX\n",
         (unsigned long)ctrl, (unsigned long)cfg, (unsigned long)st);
  spidiag_decode_ctrl(ctrl);
  spidiag_decode_cfg(cfg);

  /* --- Start a minimal transfer: 4 data bytes --- */

  /* Clear FIFO + interrupts */

  putreg32(SPI_INT_TX_FIFO_CLR | SPI_INT_RX_FIFO_CLR,
           SPI_REG_INT_STATUS(BK7258_SPI1_BASE));
  putreg32(0xFFFFFFFF, SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

  /* Set trans_len via read-modify-write */

  cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
  cfg &= ~(SPI_CFG_TX_TRANS_LEN_MASK << SPI_CFG_TX_TRANS_LEN_SHIFT);
  cfg |= (4u << SPI_CFG_TX_TRANS_LEN_SHIFT);
  cfg |= SPI_CFG_TX_EN_BIT;
  cfg |= SPI_CFG_TX_FINISH_INT_EN_BIT;
  putreg32(cfg, SPI_REG_CFG(BK7258_SPI1_BASE));

  /* [B] Post-translen CFG */

  cfg = getreg32(SPI_REG_CFG(BK7258_SPI1_BASE));
  syslog(LOG_INFO,
         "[spidiag] [B] after set trans_len: CFG=0x%08lX\n",
         (unsigned long)cfg);
  spidiag_decode_cfg(cfg);

  /* Assert CS (DC=1 for data) */

  gpio_write_fast(&g_cache_dc, 1);
  gpio_write_fast(&g_cache_cs, 0);

  /* [C] Feed 4 bytes one at a time */

  for (i = 0; i < 4; i++)
    {
      poll_cnt = SPI_POLL_TIMEOUT;

      while (!(getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE)) &
               SPI_INT_TX_FIFO_WR_READY))
        {
          if (--poll_cnt == 0)
            {
              st = getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE));
              syslog(LOG_ERR,
                     "[spidiag] FIFO timeout at byte %d, "
                     "INT_STATUS=0x%08lX\n",
                     i, (unsigned long)st);
              gpio_write_fast(&g_cache_cs, 1);
              lcd_spi_pins_to_gpio();
              return -ETIMEDOUT;
            }
        }

      putreg32(diag_data[i], SPI_REG_DATA(BK7258_SPI1_BASE));
    }

  /* [C] After last byte written */

  st = getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE));
  syslog(LOG_INFO,
         "[spidiag] [C] after last byte: INT_STATUS=0x%08lX\n",
         (unsigned long)st);

  /* [D] Poll for tx_finish, printing status up to 5 times */

  {
    int prints = 0;
    poll_cnt = SPI_POLL_TIMEOUT;

    for (;;)
      {
        st = getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

        if (st & (SPI_INT_TX_FINISH | SPI_INT_TX_UNDERFLOW))
          {
            break;
          }

        if (prints < 5)
          {
            syslog(LOG_INFO,
                   "[spidiag] [D] polling: INT_STATUS=0x%08lX\n",
                   (unsigned long)st);
            prints++;
          }

        if (--poll_cnt == 0)
          {
            st = getreg32(SPI_REG_INT_STATUS(BK7258_SPI1_BASE));
            syslog(LOG_ERR,
                   "[spidiag] [E] TIMEOUT: INT_STATUS=0x%08lX "
                   "polls=%lu\n",
                   (unsigned long)st,
                   (unsigned long)SPI_POLL_TIMEOUT);
            gpio_write_fast(&g_cache_cs, 1);
            lcd_spi_pins_to_gpio();
            return -ETIMEDOUT;
          }
      }

    /* [E] Success */

    syslog(LOG_INFO,
           "[spidiag] [E] OK: INT_STATUS=0x%08lX polls=%lu\n",
           (unsigned long)st,
           (unsigned long)(SPI_POLL_TIMEOUT - poll_cnt));
  }

  /* Clear completion bits */

  putreg32(st & (SPI_INT_TX_FINISH | SPI_INT_TX_UNDERFLOW),
           SPI_REG_INT_STATUS(BK7258_SPI1_BASE));

  /* Deassert CS, restore pins */

  gpio_write_fast(&g_cache_cs, 1);
  lcd_spi_pins_to_gpio();

  syslog(LOG_INFO, "[spidiag] done — 4 bytes sent successfully\n");
  return 0;
}

#endif /* CONFIG_LCD_GC9D01_HW_SPI */

/****************************************************************************
 * Name: lcd_set_pins / lcd_setup_pins
 ****************************************************************************/

static void lcd_set_pins(const lcd_pins_t *pins)
{
  g_active_pins = pins;

  /* Rebuild GPIO caches so g_cache_cs/dc/sclk/mosi point to this panel's
   * GPIOs.  Without this, bit-bang writes (spi_write_byte, lcd_send_cmd,
   * lcd_send_data) target the PREVIOUS panel's pins after a switch.
   * This caused: blit using left SPI path but right CS/DC (black screen),
   * and four earlier bugs where g_active_pins was used as a guard but
   * didn't reflect the actual cache state.
   */

  gpio_set_output_cached(pins->sclk, &g_cache_sclk);
  gpio_set_output_cached(pins->mosi, &g_cache_mosi);
  gpio_set_output_cached(pins->cs,   &g_cache_cs);
  gpio_set_output_cached(pins->dc,   &g_cache_dc);
  g_cached_pins = pins;
}

static void lcd_setup_pins(const lcd_pins_t *pins)
{
  lcd_set_pins(pins);
  gpio_set_output(pins->rst);

  /* Always set up GPIO mode first — needed for command bit-bang */

  gpio_set_output_cached(pins->sclk, &g_cache_sclk);
  gpio_set_output_cached(pins->mosi, &g_cache_mosi);
  gpio_set_output_cached(pins->cs,   &g_cache_cs);
  gpio_set_output_cached(pins->dc,   &g_cache_dc);

  /* Initial idle state: CS=HIGH, DC=HIGH, SCLK=LOW, MOSI=LOW */

  gpio_write_fast(&g_cache_cs,   1);
  gpio_write_fast(&g_cache_dc,   1);
  gpio_write_fast(&g_cache_sclk, 0);
  gpio_write_fast(&g_cache_mosi, 0);

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  /* Initialize SPI1 for left screen — ONCE.  The SPI1 peripheral config
   * (clock, CTRL, reset) persists across panel switches; the right panel
   * bit-bangs its own pins (P22-24) and never touches SPI1.  Re-running
   * the full init on every panel switch (i.e. every eye redraw) was the
   * source of the per-frame "[lcd] SPI1 init OK" spam and frame drops.
   * lcd_spi_deinit() clears g_hw_spi_capable, so the next session
   * re-inits once.  Pins stay GPIO until transfer time (lcd_spi_write
   * mux's them per write).  Failure is non-fatal — falls back to bit-bang.
   */

  if (pins->sclk == 2 && !g_hw_spi_capable)
    {
      int ret = lcd_spi_init(pins);

      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "[lcd] SPI1 init failed (%d), using bit-bang\n", ret);
        }
    }
#endif
}

/****************************************************************************
 * Name: spi_write_byte
 *
 * Description:
 *   Bit-bang one byte, MSB-first, SPI mode 0.
 *   Uses cached GPIO values (no getreg32 read).
 *   spi_delay() after each SCLK edge adds timing margin that
 *   prevents window corruption when PSRAM/DVP bus traffic
 *   stalls the CPU mid-transfer.
 *
 ****************************************************************************/

static void spi_write_byte(uint8_t byte)
{
  int bit;

  for (bit = 7; bit >= 0; bit--)
    {
      gpio_write_fast(&g_cache_mosi, (byte >> bit) & 1);
      gpio_write_fast(&g_cache_sclk, 1);
      spi_delay();
      gpio_write_fast(&g_cache_sclk, 0);
      spi_delay();
    }
}

/****************************************************************************
 * Name: lcd_send_cmd / lcd_send_data / lcd_send_cmd_data
 ****************************************************************************/

static void lcd_send_cmd(uint8_t cmd)
{
#ifdef CONFIG_LCD_GC9D01_HW_SPI
  /* Commands always use bit-bang — ensure GPIO mode */

  if (lcd_hw_spi_usable())
    {
      lcd_spi_pins_to_gpio();
    }
#endif

  gpio_write_fast(&g_cache_dc, 0);
  gpio_write_fast(&g_cache_cs, 0);
  spi_write_byte(cmd);
  gpio_write_fast(&g_cache_cs, 1);
}

static void lcd_send_data(const uint8_t *data, int len)
{
#ifdef CONFIG_LCD_GC9D01_HW_SPI
  /* Use hardware SPI for data when available (left screen).
   * Switch pins to SPI mode, send, switch back to GPIO.
   * Falls back to bit-bang on timeout.
   */

  if (lcd_hw_spi_usable() && len >= SPI_HWSPI_MIN_LEN)
    {
      gpio_write_fast(&g_cache_dc, 1);

      /* Mux SCLK+MOSI to SPI1 peripheral BEFORE asserting CS.
       * Previous code asserted CS first, then muxed — the mux switch
       * can glitch SCLK while CS is low, confusing the panel.
       * After mux, SCLK is driven by SPI1 in mode 0 (CPOL=0) → idle low.
       */

      lcd_spi_pins_to_spi();
      gpio_write_fast(&g_cache_cs, 0);

      if (lcd_spi_write(data, len, NULL) == 0)
        {
          gpio_write_fast(&g_cache_cs, 1);
          lcd_spi_pins_to_gpio();
          return;
        }

      /* Timeout — fallback to bit-bang */

      gpio_write_fast(&g_cache_cs, 1);
      lcd_spi_pins_to_gpio();
    }
#endif

  /* Bit-bang fallback */

  int i;

  gpio_write_fast(&g_cache_dc, 1);
  gpio_write_fast(&g_cache_cs, 0);
  for (i = 0; i < len; i++)
    {
      spi_write_byte(data[i]);
    }

  gpio_write_fast(&g_cache_cs, 1);
}

static void lcd_send_cmd_data(uint8_t cmd, const uint8_t *data, int len)
{
  lcd_send_cmd(cmd);
  if (len > 0)
    {
      lcd_send_data(data, len);
    }
}

/****************************************************************************
 * Name: lcd_init_sequence
 *
 * Description:
 *   Full GC9D01 init sequence from ARMINO lcd_spi_gc9d01.c.
 *   If display_on is true, sends 0x29 (display on) at the end;
 *   otherwise leaves the display off so the caller can fill the
 *   framebuffer first (avoids flash of uninitialised GRAM).
 *
 *   WARNING: a caller passing display_on=false MUST issue
 *   lcd_display_on() (or lcd_send_cmd(0x29)) itself once the frame is
 *   ready.  Forgetting it yields a lit backlight over a black screen,
 *   which is easily mistaken for a data-path failure — and is masked
 *   whenever an earlier command has already turned the display on, so
 *   it only shows up on a cold boot.  Never rely on display state left
 *   behind by a previous command.
 *
 ****************************************************************************/

static void lcd_init_sequence(bool display_on)
{
  lcd_send_cmd(0xfe);
  lcd_send_cmd(0xef);

  lcd_send_cmd_data(0x80, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x81, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x82, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x83, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x84, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x85, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x86, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x87, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x88, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x89, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x8a, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x8b, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x8c, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x8d, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x8e, (const uint8_t *)"\xff", 1);
  lcd_send_cmd_data(0x8f, (const uint8_t *)"\xff", 1);

  lcd_send_cmd_data(0x3a, (const uint8_t *)"\x05", 1);
  lcd_send_cmd_data(0xec, (const uint8_t *)"\x01", 1);
  lcd_send_cmd_data(0x74,
    (const uint8_t *)"\x02\x0e\x00\x00\x00\x00\x00", 7);
  lcd_send_cmd_data(0x98, (const uint8_t *)"\x3e\x99\x3e", 3);
  lcd_send_cmd_data(0xb5, (const uint8_t *)"\x0d\x0d", 2);
  lcd_send_cmd_data(0x60,
    (const uint8_t *)"\x38\x0f\x79\x67", 4);
  lcd_send_cmd_data(0x61,
    (const uint8_t *)"\x38\x11\x79\x67", 4);
  lcd_send_cmd_data(0x64,
    (const uint8_t *)"\x38\x17\x71\x5f\x79\x67", 6);
  lcd_send_cmd_data(0x65,
    (const uint8_t *)"\x38\x13\x71\x5b\x79\x67", 6);
  lcd_send_cmd_data(0x6a, (const uint8_t *)"\x00\x00", 2);
  lcd_send_cmd_data(0x6c,
    (const uint8_t *)"\x22\x02\x22\x02\x22\x22\x50", 7);
  lcd_send_cmd_data(0x6e,
    (const uint8_t *)"\x03\x03\x01\x01\x00\x00"
                     "\x0f\x0f\x0d\x0d\x0b\x0b"
                     "\x09\x09\x00\x00\x00\x00"
                     "\x0a\x0a\x0c\x0c\x0e\x0e"
                     "\x10\x10\x00\x00\x02\x02"
                     "\x04\x04", 32);
  lcd_send_cmd_data(0xbf, (const uint8_t *)"\x01", 1);
  lcd_send_cmd_data(0xf9, (const uint8_t *)"\x40", 1);
  lcd_send_cmd_data(0x9b, (const uint8_t *)"\x3b", 1);
  lcd_send_cmd_data(0x93, (const uint8_t *)"\x33\x7f\x00", 3);
  lcd_send_cmd_data(0x7e, (const uint8_t *)"\x30", 1);
  lcd_send_cmd_data(0x70,
    (const uint8_t *)"\x0d\x02\x08\x0d\x02\x08", 6);
  lcd_send_cmd_data(0x71,
    (const uint8_t *)"\x0d\x02\x08", 3);
  lcd_send_cmd_data(0x91, (const uint8_t *)"\x0e\x09", 2);
  lcd_send_cmd_data(0xc3, (const uint8_t *)"\x18", 1);
  lcd_send_cmd_data(0xc4, (const uint8_t *)"\x18", 1);
  lcd_send_cmd_data(0xc9, (const uint8_t *)"\x3c", 1);
  lcd_send_cmd_data(0xf0,
    (const uint8_t *)"\x13\x15\x04\x05\x01\x38", 6);
  lcd_send_cmd_data(0xf2,
    (const uint8_t *)"\x13\x15\x04\x05\x01\x34", 6);
  lcd_send_cmd_data(0xf1,
    (const uint8_t *)"\x4b\xb8\x7b\x34\x35\xef", 6);
  lcd_send_cmd_data(0xf3,
    (const uint8_t *)"\x47\xb4\x72\x34\x35\xda", 6);
  lcd_send_cmd_data(0x36, (const uint8_t *)"\x00", 1);

  lcd_send_cmd(0x34);  /* tearing effect off */
  lcd_send_cmd(0x11);  /* sleep out */
  up_mdelay(120);

  if (display_on)
    {
      lcd_send_cmd(0x29);  /* display on */
    }
}

/****************************************************************************
 * Name: lcd_display_on
 *
 * Description:
 *   Send 0x29 (display on) — call after framebuffer is ready.
 *
 ****************************************************************************/

static void lcd_display_on(void)
{
  lcd_send_cmd(0x29);
}

/****************************************************************************
 * Name: lcd_fill_rect
 *
 * Description:
 *   Fill a rectangle (x0,y0)-(x1,y1) with a single RGB565 color.
 *   Much smaller than full-screen fill — avoids millions of bit-bangs.
 *
 ****************************************************************************/

static void lcd_fill_rect(uint16_t x0, uint16_t y0,
                          uint16_t x1, uint16_t y1,
                          uint16_t color)
{
  uint8_t ca[4];
  uint8_t ra[4];
  int npix = (int)(x1 - x0 + 1) * (int)(y1 - y0 + 1);

  /* CASET */

  ca[0] = (x0 >> 8) & 0xff;
  ca[1] = x0 & 0xff;
  ca[2] = (x1 >> 8) & 0xff;
  ca[3] = x1 & 0xff;
  lcd_send_cmd_data(0x2a, ca, 4);

  /* RASET */

  ra[0] = (y0 >> 8) & 0xff;
  ra[1] = y0 & 0xff;
  ra[2] = (y1 >> 8) & 0xff;
  ra[3] = y1 & 0xff;
  lcd_send_cmd_data(0x2b, ra, 4);

  /* RAMWR + pixel data — use HW SPI for bulk fill when available */

  lcd_send_cmd(0x2c);

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  if (lcd_hw_spi_usable())
    {
      /* Static tile buffer — avoids large stack allocation.
       * SPI_TILE_BYTES defaults to 1024 (configurable up to 4095).
       * At 1024: 51200 bytes / 1024 = 50 SPI transactions per frame
       * (vs 800 at the 64-byte FIFO_DEPTH).
       */

      static uint8_t tile[SPI_TILE_BYTES];
      uint8_t hi = (color >> 8) & 0xff;
      uint8_t lo = color & 0xff;
      int i;

      for (i = 0; i < SPI_TILE_BYTES; i += 2)
        {
          tile[i]     = hi;
          tile[i + 1] = lo;
        }

      gpio_write_fast(&g_cache_dc, 1);
      gpio_write_fast(&g_cache_cs, 0);
      lcd_spi_pins_to_spi();

      while (npix > 0)
        {
          int batch = npix;

          if (batch > SPI_TILE_BYTES / 2)
            {
              batch = SPI_TILE_BYTES / 2;
            }

          if (lcd_spi_write(tile, batch * 2, NULL) != 0)
            {
              goto fill_bb_fallback;
            }

          npix -= batch;
        }

      lcd_spi_pins_to_gpio();
      gpio_write_fast(&g_cache_cs, 1);
      return;
    }

fill_bb_fallback:
  /* Restore GPIO mode if HW SPI failed mid-transfer */

  if (g_pins_in_spi_mode)
    {
      lcd_spi_pins_to_gpio();
    }
#endif

  /* Bit-bang fallback */

  {
    uint8_t hi = (color >> 8) & 0xff;
    uint8_t lo = color & 0xff;
    int i;

    gpio_write_fast(&g_cache_dc, 1);
    gpio_write_fast(&g_cache_cs, 0);

    for (i = 0; i < npix; i++)
      {
        spi_write_byte(hi);
        spi_write_byte(lo);
      }

    gpio_write_fast(&g_cache_cs, 1);
  }
}

/****************************************************************************
 * Name: bk7258_lcd_blit_rgb565
 *
 * Description:
 *   Blit a full RGB565 framebuffer to the LCD.
 *   Sends CASET/RASET/RAMWR then streams pixel data in rows.
 *
 *   panel: 0=left (SPI1 HW), 1=right (bit-bang), other=left
 *
 ****************************************************************************/

void bk7258_lcd_blit_rgb565(int panel,
                             uint16_t x0, uint16_t y0,
                             uint16_t w, uint16_t h,
                             const uint8_t *rgb565)
{
  uint8_t ca[4];
  uint8_t ra[4];
  uint16_t x1 = x0 + w - 1;
  uint16_t y1 = y0 + h - 1;

  /* Auto-switch panel if different from current */

  if (panel == 1 && g_active_pins != &g_lcd_right)
    {
      lcd_set_pins(&g_lcd_right);
      lcd_setup_pins(&g_lcd_right);
    }
  else if (panel != 1 && g_active_pins != &g_lcd_left)
    {
      lcd_set_pins(&g_lcd_left);
      lcd_setup_pins(&g_lcd_left);
    }

  /* CASET */

  ca[0] = (x0 >> 8) & 0xff;
  ca[1] = x0 & 0xff;
  ca[2] = (x1 >> 8) & 0xff;
  ca[3] = x1 & 0xff;
  lcd_send_cmd_data(0x2a, ca, 4);

  /* RASET */

  ra[0] = (y0 >> 8) & 0xff;
  ra[1] = y0 & 0xff;
  ra[2] = (y1 >> 8) & 0xff;
  ra[3] = y1 & 0xff;
  lcd_send_cmd_data(0x2b, ra, 4);

  /* RAMWR — CS must stay LOW for entire pixel transfer (GC9D01 datasheet).
   * lcd_spi_write() internally chunks by SPI_MAX_CHUNK_BYTES (1024)
   * without touching CS, so a single call for the whole frame is safe.
   */

  lcd_send_cmd(0x2c);

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  /* Use HW SPI for pixel data when available (left screen only) */

  if (lcd_hw_spi_usable())
    {
      int total = (int)w * (int)h * 2;
      size_t sent = 0;

      gpio_write_fast(&g_cache_dc, 1);
      gpio_write_fast(&g_cache_cs, 0);
      lcd_spi_pins_to_spi();

      if (lcd_spi_write(rgb565, total, &sent) != 0)
        {
          /* HW SPI timeout mid-frame.
           * RAMWR pointer has advanced by 'sent' bytes — we cannot
           * reliably resume or resend.  Abort cleanly: raise CS to
           * terminate RAMWR, restore GPIO, log and return.
           * A partially-drawn frame is better than a garbled one.
           */

          gpio_write_fast(&g_cache_cs, 1);
          syslog(LOG_ERR,
                 "[blit] HW SPI timeout, abort frame "
                 "(sent=%lu/%d)\n",
                 (unsigned long)sent, total);
          return;
        }

      lcd_spi_pins_to_gpio();
      gpio_write_fast(&g_cache_cs, 1);
      return;
    }
#endif

  /* Bit-bang fallback (right screen, or HW SPI not compiled in).
   * CS stays LOW for the entire transfer — same as lcd_fill_rect() and
   * lcdtest_pat().  Each pixel is 2 bytes (RGB565 big-endian).
   */

  {
    int i;
    int total = (int)w * (int)h;

    gpio_write_fast(&g_cache_dc, 1);
    gpio_write_fast(&g_cache_cs, 0);

    for (i = 0; i < total; i++)
      {
        spi_write_byte(rgb565[0]);
        spi_write_byte(rgb565[1]);
        rgb565 += 2;
      }

    gpio_write_fast(&g_cache_cs, 1);
  }
}

/****************************************************************************
 * Name: bk7258_lcd_preview_init
 *
 * Description:
 *   Initialize the left-screen LCD for camera preview use.
 *   Enables LDO_3V3 (P52), backlight (P25), configures SPI pins,
 *   resets GC9D01, runs init sequence, turns display on, fills
 *   with black to confirm screen is alive.
 *
 *   Reuses the same static functions that lcdtest uses — no
 *   duplicated init sequence.
 *
 *   Safe to call even if lcdtest has previously run — pin setup
 *   is idempotent (gpio_set_output + cached values).
 *
 * Returns:
 *   0 on success.
 *
 ****************************************************************************/

int bk7258_lcd_preview_init(void)
{
  /* LCD 3.3V power supply — P52 enables U3 (ME6211C33 LDO) */

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);

  /* Backlight on */

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* --- Left screen (P2/P3/P4/P5/P45) --- */

  lcd_setup_pins(&g_lcd_left);

  gpio_write(g_lcd_left.rst, 0);
  up_mdelay(15);
  gpio_write(g_lcd_left.rst, 1);
  up_mdelay(120);

  lcd_init_sequence(true);
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  /* --- Right screen (P22/P23/P24/P7/P6) — bit-bang only --- */

  lcd_setup_pins(&g_lcd_right);

  gpio_write(g_lcd_right.rst, 0);
  up_mdelay(15);
  gpio_write(g_lcd_right.rst, 1);
  up_mdelay(120);

  lcd_init_sequence(true);
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  /* Switch back to left as default active panel */

  lcd_set_pins(&g_lcd_left);

  syslog(LOG_INFO,
         "[lcd] dual-screen init done, HW SPI %s\n",
         lcd_hw_spi_usable() ? "ON (left screen)" : "OFF (bit-bang)");

  return 0;
}

/****************************************************************************
 * Name: bk7258_lcd_preview_deinit
 *
 * Description:
 *   Release LCD resources after camera preview.
 *   Turns off backlight only.  P52 (LDO_3V3) stays on because
 *   other peripherals may depend on the 3.3V rail.
 *
 ****************************************************************************/

void bk7258_lcd_preview_deinit(void)
{
  /* Backlight off — leave LDO_3V3 enabled */

  gpio_write(LCD_PIN_BL, 0);

  syslog(LOG_INFO, "[lcd] preview deinit — backlight off\n");
}

/****************************************************************************
 * Name: sqrt_int — integer square root (Newton's method)
 ****************************************************************************/

static int16_t sqrt_int(int32_t val)
{
  int32_t guess;
  int32_t next;
  int i;

  if (val <= 0)
    {
      return 0;
    }

  guess = val;

  for (i = 0; i < 10; i++)
    {
      next = (guess + val / guess) / 2;

      if (next >= guess)
        {
          break;
        }

      guess = next;
    }

  return (int16_t)guess;
}

/****************************************************************************
 * Name: lcd_fill_circle
 *
 * Description:
 *   Fill a circle centred at (cx,cy) with radius r using scanline
 *   rendering.  Each scanline is a 1px-high lcd_fill_rect call.
 *
 ****************************************************************************/

static void lcd_fill_circle(int16_t cx, int16_t cy, int16_t r,
                            uint16_t color)
{
  int16_t y;
  int16_t dx;
  int16_t x0;
  int16_t x1;
  int32_t r2 = (int32_t)r * r;

  for (y = cy - r; y <= cy + r; y++)
    {
      int32_t dy  = (int32_t)(y - cy);
      int32_t rem = r2 - dy * dy;

      if (rem < 0)
        {
          continue;
        }

      dx = sqrt_int(rem);
      x0 = cx - dx;
      x1 = cx + dx;

      if (x0 < 0)
        {
          x0 = 0;
        }

      if (x1 >= LCD_WIDTH)
        {
          x1 = LCD_WIDTH - 1;
        }

      if (y < 0 || y >= LCD_HEIGHT)
        {
          continue;
        }

      lcd_fill_rect(x0, y, x1, y, color);
    }
}

/****************************************************************************
 * Name: lcd_fill_round_rect
 *
 * Description:
 *   Fill a rounded rectangle centred at (cx,cy), width w, height h,
 *   corner radius r.  Built from one inner rect + four corner circles.
 *
 ****************************************************************************/

static void lcd_fill_round_rect(int16_t cx, int16_t cy,
                                int16_t w, int16_t h,
                                int16_t r, uint16_t color)
{
  int16_t x0 = cx - w / 2;
  int16_t y0 = cy - h / 2;
  int16_t x1 = cx + w / 2;
  int16_t y1 = cy + h / 2;

  /* Horizontal centre strip (full width, reduced height) */

  lcd_fill_rect(x0, y0 + r, x1, y1 - r, color);

  /* Vertical centre strip (reduced width, full height) */

  lcd_fill_rect(x0 + r, y0, x1 - r, y1, color);

  /* Four corner circles */

  lcd_fill_circle(x0 + r, y0 + r, r, color);
  lcd_fill_circle(x1 - r, y0 + r, r, color);
  lcd_fill_circle(x0 + r, y1 - r, r, color);
  lcd_fill_circle(x1 - r, y1 - r, r, color);
}

/****************************************************************************
 * Name: draw_eye
 *
 * Description:
 *   Draw a cute robot eye on the currently active panel:
 *     black background → cyan iris → white highlight
 *
 ****************************************************************************/

static void draw_eye(void)
{
  syslog(LOG_INFO, "[eye] fill black\n");
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  syslog(LOG_INFO, "[eye] cyan iris r=60 at (80,80)\n");
  lcd_fill_circle(80, 80, 60, 0x07ff);

  syslog(LOG_INFO, "[eye] white highlight r=16 at (62,60)\n");
  lcd_fill_circle(62, 60, 16, 0xffff);
}

/****************************************************************************
 * Name: eye_draw_full
 *
 * Description:
 *   Full draw of the animated eye model on the current panel.
 *   Call once per eye before starting animation.
 *
 *   Background: black
 *   Iris:       cyan 0x07FF, centre (80,80), r=58
 *   Pupil:      black 0x0000, r=22, centre (80+gaze_dx, 80)
 *   Highlight:  white 0xFFFF, r=8, centre (80+gaze_dx-8, 80-10)
 *
 ****************************************************************************/

#define EYE_CX        80
#define EYE_CY        80
#define EYE_IRIS_R    58
#define EYE_PUPIL_R   22
#define EYE_HIGHLIGHT_R 8

static void eye_draw_full(int gaze_dx)
{
  int pcx = EYE_CX + gaze_dx;

  syslog(LOG_INFO,
         "[eye] full draw gaze_dx=%d\n", gaze_dx);

  /* Black background */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  /* Cyan iris */

  lcd_fill_circle(EYE_CX, EYE_CY, EYE_IRIS_R, 0x07ff);

  /* Black pupil */

  lcd_fill_circle(pcx, EYE_CY, EYE_PUPIL_R, 0x0000);

  /* White highlight (upper-left of pupil) */

  lcd_fill_circle(pcx - 8, EYE_CY - 10,
                  EYE_HIGHLIGHT_R, 0xffff);
}

/****************************************************************************
 * Name: eye_move_pupil
 *
 * Description:
 *   Stream-based local update: composite one bounding-box region into
 *   a RAM buffer (highlight > pupil > iris > background), then blast
 *   it to the LCD in a single CASET/RASET/RAMWR sequence.
 *   Eliminates the per-circle command storm that caused horizontal
 *   tearing when DVP interrupts preempted mid-draw.
 *
 ****************************************************************************/

static void eye_move_pupil(int old_dx, int new_dx)
{
  static uint8_t buf[128 * 52 * 2];
  int old_cx = EYE_CX + old_dx;
  int new_cx = EYE_CX + new_dx;
  int hx = new_cx - 8;
  int hy = EYE_CY - 10;
  int x0;
  int x1;
  int y0;
  int y1;
  int w;
  int h;
  int x;
  int y;
  int idx;
  uint8_t ca[4];
  uint8_t ra[4];

  /* Bounding box covering old + new pupil + highlight, with 2px pad */

  x0 = (old_cx < new_cx ? old_cx : new_cx) - EYE_PUPIL_R - 2;
  x1 = (old_cx > new_cx ? old_cx : new_cx) + EYE_PUPIL_R + 2;
  y0 = EYE_CY - EYE_PUPIL_R - 2;
  y1 = EYE_CY + EYE_PUPIL_R + 2;

  /* Clamp to iris circle and screen bounds */

  if (x0 < EYE_CX - EYE_IRIS_R) x0 = EYE_CX - EYE_IRIS_R;
  if (x1 > EYE_CX + EYE_IRIS_R) x1 = EYE_CX + EYE_IRIS_R;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > LCD_WIDTH  - 1) x1 = LCD_WIDTH  - 1;
  if (y1 > LCD_HEIGHT - 1) y1 = LCD_HEIGHT - 1;

  w = x1 - x0 + 1;
  h = y1 - y0 + 1;
  if (w <= 0 || h <= 0 || w * h * 2 > (int)sizeof(buf)) return;

  /* Per-pixel composite: highlight(white) > pupil(black) >
   * iris(cyan) > background(black)
   */

  idx = 0;
  for (y = y0; y <= y1; y++)
    {
      int dyp = y - EYE_CY;
      int dyh = y - hy;

      for (x = x0; x <= x1; x++)
        {
          int dxp = x - new_cx;
          int dxh = x - hx;
          int dxi = x - EYE_CX;
          int dyi = y - EYE_CY;
          uint16_t c;

          if (dxh * dxh + dyh * dyh <=
              EYE_HIGHLIGHT_R * EYE_HIGHLIGHT_R)
            {
              c = 0xffff;
            }
          else if (dxp * dxp + dyp * dyp <=
                   EYE_PUPIL_R * EYE_PUPIL_R)
            {
              c = 0x0000;
            }
          else if (dxi * dxi + dyi * dyi <=
                   EYE_IRIS_R * EYE_IRIS_R)
            {
              c = 0x07ff;
            }
          else
            {
              c = 0x0000;
            }

          buf[idx++] = (uint8_t)((c >> 8) & 0xff);
          buf[idx++] = (uint8_t)(c & 0xff);
        }
    }

  /* Single window open + one data stream, no per-row commands */

  ca[0] = (uint8_t)((x0 >> 8) & 0xff);
  ca[1] = (uint8_t)(x0 & 0xff);
  ca[2] = (uint8_t)((x1 >> 8) & 0xff);
  ca[3] = (uint8_t)(x1 & 0xff);
  lcd_send_cmd_data(0x2a, ca, 4);

  ra[0] = (uint8_t)((y0 >> 8) & 0xff);
  ra[1] = (uint8_t)(y0 & 0xff);
  ra[2] = (uint8_t)((y1 >> 8) & 0xff);
  ra[3] = (uint8_t)(y1 & 0xff);
  lcd_send_cmd_data(0x2b, ra, 4);

  lcd_send_cmd(0x2c);
  lcd_send_data(buf, w * h * 2);
}

/****************************************************************************
 * Name: bk7258_lcd_eye_draw
 *
 * Description:
 *   Full draw of the animated eye on the specified panel.
 *   Wrapper that handles panel switching, then calls eye_draw_full().
 *
 ****************************************************************************/

void bk7258_lcd_eye_draw(int panel, int gaze_dx)
{
  if (panel == 1 && g_active_pins != &g_lcd_right)
    {
      lcd_set_pins(&g_lcd_right);
      lcd_setup_pins(&g_lcd_right);
    }
  else if (panel != 1 && g_active_pins != &g_lcd_left)
    {
      lcd_set_pins(&g_lcd_left);
      lcd_setup_pins(&g_lcd_left);
    }

  eye_draw_full(gaze_dx);
}

/****************************************************************************
 * Name: bk7258_lcd_eye_gaze
 *
 * Description:
 *   Local refresh: move pupil from old_dx to new_dx on the specified panel.
 *   Wrapper that handles panel switching, then calls eye_move_pupil().
 *
 ****************************************************************************/

void bk7258_lcd_eye_gaze(int panel, int old_dx, int new_dx)
{
  if (panel == 1 && g_active_pins != &g_lcd_right)
    {
      lcd_set_pins(&g_lcd_right);
      lcd_setup_pins(&g_lcd_right);
    }
  else if (panel != 1 && g_active_pins != &g_lcd_left)
    {
      lcd_set_pins(&g_lcd_left);
      lcd_setup_pins(&g_lcd_left);
    }

  eye_move_pupil(old_dx, new_dx);
}

/****************************************************************************
 * Name: eye_blink
 *
 * Description:
 *   Quick blink: fill iris area black, pause, then redraw fully.
 *
 ****************************************************************************/

static void eye_blink(int gaze_dx)
{
  int pcx = EYE_CX + gaze_dx;

  syslog(LOG_INFO, "[eye] blink (gaze_dx=%d)\n", gaze_dx);

  /* Close: black over iris area */

  lcd_fill_circle(EYE_CX, EYE_CY, EYE_IRIS_R, 0x0000);
  up_mdelay(120);

  /* Open: iris -> pupil -> highlight (layered, no white dots) */

  lcd_fill_circle(EYE_CX, EYE_CY, EYE_IRIS_R, 0x07ff);
  lcd_fill_circle(pcx, EYE_CY, EYE_PUPIL_R, 0x0000);
  lcd_fill_circle(pcx - 8, EYE_CY - 10,
                  EYE_HIGHLIGHT_R, 0xffff);
}

/****************************************************************************
 * Horizontal human-like expression eyes
 *
 * Bounding box: x in [10,150], y in [22,138].
 * Each full-draw expression clears the box first (no ghosting).
 * eye_look() uses local pupil update for speed.
 ****************************************************************************/

#define EMO_X0  10
#define EMO_X1  150
#define EMO_Y0  22
#define EMO_Y1  138

#define EMO_EYE_W   124
#define EMO_EYE_H   86
#define EMO_EYE_R   42
#define EMO_PUPIL_R 26
#define EMO_HL_R    10

/* Current pupil gaze offset (tracked for local eye_look updates) */

static int g_emo_gaze = 0;

static void emo_clear(void)
{
  lcd_fill_rect(EMO_X0, EMO_Y0, EMO_X1, EMO_Y1, 0x0000);
}

/* Draw eye shape + pupil + highlight at gaze offset dx.
 * Used by both neutral (after emo_clear) and look (overdraw).
 */

static void draw_eye_shape(int dx)
{
  lcd_fill_round_rect(80, 80, EMO_EYE_W, EMO_EYE_H,
                      EMO_EYE_R, 0x07ff);
  lcd_fill_circle(80 + dx, 80, EMO_PUPIL_R, 0x0000);
  lcd_fill_circle(80 + dx - 10, 68, EMO_HL_R, 0xffff);
}

static void eye_neutral(void)
{
  emo_clear();
  draw_eye_shape(0);
  g_emo_gaze = 0;
}

static void eye_look(int dx)
{
  /* Redraw eye shape over current image — covers old pupil/highlight
   * with iris colour, preserves rounded contour perfectly.
   */

  draw_eye_shape(dx);
  g_emo_gaze = dx;
}

static void eye_happy(void)
{
  emo_clear();

  /* Cyan filled circle */

  lcd_fill_circle(80, 80, 54, 0x07ff);

  /* Black over upper half (y < 80) to leave a smile */

  lcd_fill_rect(EMO_X0, EMO_Y0, EMO_X1, 79, 0x0000);

  g_emo_gaze = 0;
}

static void eye_emo_blink(void)
{
  emo_clear();
  lcd_fill_round_rect(80, 80, EMO_EYE_W, 14, 7, 0x07ff);
  g_emo_gaze = 0;
}

/****************************************************************************
 * Official-website-style eye: white bg + blue iris + black eyelids
 *
 * Color palette (RGB565):
 *   0xFFFF  white  (background)
 *   0x02DF  sky-blue (iris) — try 0x001F for deeper blue
 *   0x0000  black  (pupil, eyelids)
 *   0xFFFF  white  (highlight)
 ****************************************************************************/

#define OEYE_BG        0xFFFF  /* white */
#define OEYE_IRIS      0x02DF  /* sky-blue */
#define OEYE_PUPIL     0x0000  /* black */
#define OEYE_HL        0xFFFF  /* white highlight */
#define OEYE_LID       0x0000  /* black eyelid */

#define OEYE_IRIS_R    56     /* blue iris with visible white border */
#define OEYE_PUPIL_R   28     /* black pupil */
#define OEYE_HL1_R     10     /* primary highlight upper-left */
#define OEYE_HL1_X     68     /* primary highlight centre */
#define OEYE_HL1_Y     68
#define OEYE_HL2_R     4      /* secondary highlight lower-right */
#define OEYE_HL2_X     92     /* secondary highlight centre */
#define OEYE_HL2_Y     92

/* Iris bounding box — the only region we redraw per frame.
 * iris at (80,80) r56 → x,y ∈ [24,136].
 */

#define OEYE_BOX_L     24
#define OEYE_BOX_R     136
#define OEYE_BOX_T     24
#define OEYE_BOX_B     136

/* Eyelid Y boundaries for each expression */

#define OEYE_NEUTRAL_TOP  16   /* neutral: upper lid covers y 0..16 */
#define OEYE_NEUTRAL_BOT  143  /* neutral: lower lid covers y 143..159 */
#define OEYE_HALF_TOP     50   /* half: upper lid covers y 0..50 */
#define OEYE_HALF_BOT     110  /* half: lower lid covers y 110..159 */
#define OEYE_BLINK_TOP    70   /* blink: black arc top edge */
#define OEYE_BLINK_BOT    90   /* blink: black arc bottom edge */
#define OEYE_HAPPY_BOT    115  /* happy: lower lid pushed up */

/****************************************************************************
 * Name: draw_oeye_bg
 *
 * Description:
 *   Full-screen white fill.  Call ONCE per panel before the
 *   expression loop.  Never called again during the loop.
 *
 ****************************************************************************/

static void draw_oeye_bg(void)
{
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, OEYE_BG);
}

/****************************************************************************
 * Name: draw_oeye_content
 *
 * Description:
 *   Redraw only the iris bounding box: clear box to white, then
 *   draw blue iris + black pupil + two highlights.  This is the
 *   fast per-frame content update — no full-screen fill.
 *
 ****************************************************************************/

static void draw_oeye_content(void)
{
  /* Clear iris box to white (erases previous pupil/highlight/lid) */

  lcd_fill_rect(OEYE_BOX_L, OEYE_BOX_T, OEYE_BOX_R, OEYE_BOX_B,
                OEYE_BG);

  /* Blue iris */

  lcd_fill_circle(80, 80, OEYE_IRIS_R, OEYE_IRIS);

  /* Black pupil */

  lcd_fill_circle(80, 80, OEYE_PUPIL_R, OEYE_PUPIL);

  /* Two white highlights */

  lcd_fill_circle(OEYE_HL1_X, OEYE_HL1_Y, OEYE_HL1_R, OEYE_HL);
  lcd_fill_circle(OEYE_HL2_X, OEYE_HL2_Y, OEYE_HL2_R, OEYE_HL);
}

/****************************************************************************
 * Name: eye_o_neutral
 *
 * Description:
 *   Neutral / fully open: redraw iris content + thin black eyelid
 *   strips top and bottom.  The strips are outside the iris box
 *   (y 0..16 and 143..159) so they overwrite the white bg there.
 *
 ****************************************************************************/

static void eye_o_neutral(void)
{
  draw_oeye_content();

  /* Upper eyelid: thin black strip at top */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, OEYE_NEUTRAL_TOP, OEYE_LID);

  /* Lower eyelid: thin black strip at bottom */

  lcd_fill_rect(0, OEYE_NEUTRAL_BOT, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                OEYE_LID);
}

/****************************************************************************
 * Name: eye_o_half
 *
 * Description:
 *   Half-closed: redraw iris content + wider black eyelid strips.
 *   Upper lid extends into the iris box (y 0..50), lower lid
 *   extends into the iris box (y 110..159).
 *
 ****************************************************************************/

static void eye_o_half(void)
{
  draw_oeye_content();

  /* Upper eyelid: covers top portion of iris (extends into box) */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, OEYE_HALF_TOP, OEYE_LID);

  /* Lower eyelid: covers bottom portion (extends into box) */

  lcd_fill_rect(0, OEYE_HALF_BOT, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                OEYE_LID);
}

/****************************************************************************
 * Name: eye_o_blink
 *
 * Description:
 *   Fully closed: clear iris box to white + draw thick black band
 *   y70..90.  Also restore any previous eyelid strips outside the
 *   box (top/bottom) back to white so only the black arc remains.
 *
 ****************************************************************************/

static void eye_o_blink(void)
{
  /* Clear iris box to white */

  lcd_fill_rect(OEYE_BOX_L, OEYE_BOX_T, OEYE_BOX_R, OEYE_BOX_B,
                OEYE_BG);

  /* Restore top/bottom strips outside box to white
   * (removes leftover black eyelid from previous frame)
   */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, OEYE_BOX_T - 1, OEYE_BG);
  lcd_fill_rect(0, OEYE_BOX_B + 1, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                OEYE_BG);

  /* Black arc: thick band across the middle of the iris box */

  lcd_fill_rect(OEYE_BOX_L, OEYE_BLINK_TOP, OEYE_BOX_R, OEYE_BLINK_BOT,
                OEYE_LID);
}

/****************************************************************************
 * Name: eye_o_happy
 *
 * Description:
 *   Happy: redraw iris content + thin top lid + lower lid pushed
 *   up to y=115 (smile arc).  Lower lid extends into the iris box.
 *
 ****************************************************************************/

static void eye_o_happy(void)
{
  draw_oeye_content();

  /* Upper eyelid: thin strip (same as neutral) */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, OEYE_NEUTRAL_TOP, OEYE_LID);

  /* Lower eyelid: push up to carve a smile arc */

  lcd_fill_rect(0, OEYE_HAPPY_BOT, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                OEYE_LID);
}

/****************************************************************************
 * Smooth blink animation — incremental eyelid sweep
 *
 * Instead of jumping neutral → half → blink in 3 big frames, the eyelids
 * sweep in ~10-12 small steps.  Each step only draws a narrow black band
 * (closing) or redraws the revealed eye content (opening), so the per-step
 * pixel count is tiny — even bit-bang can do it smoothly.
 *
 *   Closing: top lid descends from OEYE_NEUTRAL_TOP → OEYE_BLINK_TOP,
 *            bottom lid ascends from OEYE_NEUTRAL_BOT → OEYE_BLINK_BOT.
 *   Opening: reverse.
 ****************************************************************************/

#define OEYE_BLINK_STEPS  10  /* number of animation sub-frames */

/****************************************************************************
 * Name: oeye_pixel_color
 *
 * Description:
 *   Return the RGB565 color for pixel (px, py) in the eye content.
 *   Checks highlight, pupil, iris, then falls back to background.
 *
 ****************************************************************************/

static inline uint16_t oeye_pixel_color(int16_t px, int16_t py)
{
  int16_t ddx;
  int16_t ddy;
  int32_t d2;

  /* Highlight 1 */

  ddx = px - OEYE_HL1_X;
  ddy = py - OEYE_HL1_Y;

  if ((int32_t)ddx * ddx + (int32_t)ddy * ddy <=
      (int32_t)OEYE_HL1_R * OEYE_HL1_R)
    {
      return OEYE_HL;
    }

  /* Highlight 2 */

  ddx = px - OEYE_HL2_X;
  ddy = py - OEYE_HL2_Y;

  if ((int32_t)ddx * ddx + (int32_t)ddy * ddy <=
      (int32_t)OEYE_HL2_R * OEYE_HL2_R)
    {
      return OEYE_HL;
    }

  /* Pupil */

  ddx = px - 80;
  ddy = py - 80;
  d2 = (int32_t)ddx * ddx + (int32_t)ddy * ddy;

  if (d2 <= (int32_t)OEYE_PUPIL_R * OEYE_PUPIL_R)
    {
      return OEYE_PUPIL;
    }

  /* Iris */

  if (d2 <= (int32_t)OEYE_IRIS_R * OEYE_IRIS_R)
    {
      return OEYE_IRIS;
    }

  return OEYE_BG;
}

/****************************************************************************
 * Name: oeye_draw_row_band
 *
 * Description:
 *   Draw eye content (iris/pupil/highlights/bg) for rows y0..y1,
 *   clipped to the iris bounding box X range.  Uses CASET/RASET once
 *   and streams pixels row by row — much faster than lcd_fill_circle
 *   per row.
 *
 ****************************************************************************/

static void oeye_draw_row_band(int16_t y0, int16_t y1)
{
  uint8_t ca[4];
  uint8_t ra[4];
  uint8_t cmd[2];
  int16_t x;
  int16_t y;
  int16_t x0 = OEYE_BOX_L;
  int16_t x1 = OEYE_BOX_R;

  /* CASET + RASET for the band */

  ca[0] = (x0 >> 8) & 0xff;
  ca[1] = x0 & 0xff;
  ca[2] = (x1 >> 8) & 0xff;
  ca[3] = x1 & 0xff;
  lcd_send_cmd_data(0x2a, ca, 4);

  ra[0] = (y0 >> 8) & 0xff;
  ra[1] = y0 & 0xff;
  ra[2] = (y1 >> 8) & 0xff;
  ra[3] = y1 & 0xff;
  lcd_send_cmd_data(0x2b, ra, 4);

  /* RAMWR — stream pixels row by row */

  lcd_send_cmd(0x2c);
  gpio_write_fast(&g_cache_dc, 1);
  gpio_write_fast(&g_cache_cs, 0);

  for (y = y0; y <= y1; y++)
    {
      for (x = x0; x <= x1; x++)
        {
          uint16_t c = oeye_pixel_color(x, y);

          cmd[0] = (c >> 8) & 0xff;
          cmd[1] = c & 0xff;
          spi_write_byte(cmd[0]);
          spi_write_byte(cmd[1]);
        }
    }

  gpio_write_fast(&g_cache_cs, 1);
}

/****************************************************************************
 * Name: eye_o_blink_anim
 *
 * Description:
 *   Smooth eyelid animation: close → brief hold → open.
 *   Each step draws only the narrow newly-occluded/revealed strip,
 *   so the per-step pixel count is small (160×~6px for 10 steps).
 *
 ****************************************************************************/

static void eye_o_blink_anim(void)
{
  int step;
  int16_t top_prev;
  int16_t top_cur;
  int16_t bot_prev;
  int16_t bot_cur;

  /* --- Phase 1: close (top lid descends, bottom lid ascends) --- */

  top_prev = OEYE_NEUTRAL_TOP;
  bot_prev = OEYE_NEUTRAL_BOT;

  for (step = 1; step <= OEYE_BLINK_STEPS; step++)
    {
      /* Interpolate lid positions */

      top_cur = OEYE_NEUTRAL_TOP +
                (int16_t)((int32_t)(OEYE_BLINK_TOP - OEYE_NEUTRAL_TOP) *
                          step / OEYE_BLINK_STEPS);
      bot_cur = OEYE_NEUTRAL_BOT +
                (int16_t)((int32_t)(OEYE_BLINK_BOT - OEYE_NEUTRAL_BOT) *
                          step / OEYE_BLINK_STEPS);

      /* Draw newly covered rows as black (only the delta strip) */

      if (top_cur > top_prev)
        {
          lcd_fill_rect(OEYE_BOX_L, top_prev + 1,
                        OEYE_BOX_R, top_cur, OEYE_LID);
        }

      if (bot_cur < bot_prev)
        {
          lcd_fill_rect(OEYE_BOX_L, bot_cur,
                        OEYE_BOX_R, bot_prev - 1, OEYE_LID);
        }

      top_prev = top_cur;
      bot_prev = bot_cur;
    }

  /* --- Phase 2: brief hold at fully closed --- */

  up_mdelay(60);

  /* --- Phase 3: open (top lid ascends, bottom lid descends) --- */

  for (step = OEYE_BLINK_STEPS - 1; step >= 0; step--)
    {
      top_cur = OEYE_NEUTRAL_TOP +
                (int16_t)((int32_t)(OEYE_BLINK_TOP - OEYE_NEUTRAL_TOP) *
                          step / OEYE_BLINK_STEPS);
      bot_cur = OEYE_NEUTRAL_BOT +
                (int16_t)((int32_t)(OEYE_BLINK_BOT - OEYE_NEUTRAL_BOT) *
                          step / OEYE_BLINK_STEPS);

      /* Redraw revealed rows with eye content (only the delta strip) */

      if (top_cur < top_prev)
        {
          oeye_draw_row_band(top_cur + 1, top_prev);
        }

      if (bot_cur > bot_prev)
        {
          oeye_draw_row_band(bot_prev, bot_cur - 1);
        }

      top_prev = top_cur;
      bot_prev = bot_cur;
    }

  /* Restore the thin neutral eyelid strips (outside iris box) */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, OEYE_NEUTRAL_TOP, OEYE_LID);
  lcd_fill_rect(0, OEYE_NEUTRAL_BOT, LCD_WIDTH - 1, LCD_HEIGHT - 1,
                OEYE_LID);
}

/****************************************************************************
 * Name: lcdtest_is_dvp_reserved_pin
 *
 * Description:
 *   Check if a GPIO pin is in the DVP camera pin range (P29–P39).
 *   These pins must never be driven by lcdtest GPIO loops because
 *   they are routed to the GC2145 camera connector on the contest
 *   board.  Driving them could damage the sensor or cause bus
 *   contention.
 *
 *   Also protects SWD pins (P20, P21) and UART0 (P10, P11) as a
 *   convenience — callers still check these explicitly for clarity.
 *
 ****************************************************************************/

static bool lcdtest_is_dvp_reserved_pin(int pin)
{
  /* DVP camera connector: P29 through P39 inclusive.
   * This range covers DVP_VS, DVP_HS, DVP_PCLK, DVP_MCLK,
   * DVP_D[0:7], DVP_PWDNB, DVP_RST (depending on board routing).
   */

  if (pin >= 29 && pin <= 39)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: lcdtest_stages
 *
 * Description:
 *   Staged LCD bring-up: A=backlight, B=init, C=40x40 red square.
 *
 ****************************************************************************/

static int lcdtest_stages(void)
{
  /* --- Stage A: power + backlight --- */

  syslog(LOG_INFO,
         "[lcdtest] Stage A: LDO_3V3 ON (P52) + backlight ON (P25)\n");

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);   /* wait for LDO output to stabilize */

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);
  up_mdelay(1000);

  syslog(LOG_INFO,
         "[lcdtest] Stage A done - screen should be lit white\n");

  /* --- Stage B: reset + init --- */

  syslog(LOG_INFO,
         "[lcdtest] Stage B: RST(P45) reset + GC9D01 init\n");

  /* --- Left screen --- */

  lcd_setup_pins(&g_lcd_left);

  gpio_write(g_active_pins->rst, 0);
  up_mdelay(10);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);

  lcd_init_sequence(true);
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  syslog(LOG_INFO,
         "[lcdtest] Stage B done - left panel initialized + cleared\n");

  /* --- Right screen --- */

  lcd_setup_pins(&g_lcd_right);

  gpio_write(g_active_pins->rst, 0);
  up_mdelay(10);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);

  lcd_init_sequence(true);
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  syslog(LOG_INFO,
         "[lcdtest] Stage B done - right panel initialized + cleared\n");

  /* --- Stage C: 40x40 red square on both screens --- */

  syslog(LOG_INFO,
         "[lcdtest] Stage C: fill 40x40 red at (60,60) on both screens\n");

  /* Left screen — explicit setup, don't rely on leftover g_active_pins */

  lcd_setup_pins(&g_lcd_left);
  lcd_fill_rect(60, 60,
                60 + SQ_SIZE - 1, 60 + SQ_SIZE - 1,
                0xf800);

  /* Right screen — switch SPI to right panel, draw, switch back */

  lcd_setup_pins(&g_lcd_right);
  lcd_fill_rect(60, 60,
                60 + SQ_SIZE - 1, 60 + SQ_SIZE - 1,
                0xf800);
  lcd_setup_pins(&g_lcd_left);

  syslog(LOG_INFO,
         "[lcdtest] Stage C done - look for red square on both screens\n");

  return OK;
}

/****************************************************************************
 * Name: lcdtest_scan
 *
 * Description:
 *   Scan GPIO 0-47 for the LCD power enable pin.
 *   Strategy: cumulative — pull each pin high and leave it high, so that
 *   if multiple pins need to be high simultaneously the condition is met.
 *   P25 (backlight) is held high throughout.  UART console pins (10, 11)
 *   and LCD signal pins (2-5, 29) are skipped to keep the console and
 *   SPI bus alive.
 *
 ****************************************************************************/

static int lcdtest_scan(void)
{
  int pin;

#ifdef CONFIG_EXAMPLES_GC2145_ID
  if (bk7258_camera_dvp_active())
    {
      syslog(LOG_ERR,
             "[lcdtest] refused: DVP pins P29-P39 are in use, "
             "run 'camera stop' first\n");
      return -EBUSY;
    }
#endif

  /* Backlight on first */

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  syslog(LOG_INFO,
         "[scan] backlight P25 high, scanning GPIO 0-47 ...\n");

  for (pin = 0; pin <= 47; pin++)
    {
      /* Skip UART0 console: RX=10, TX=11 */

      if (pin == 10 || pin == 11)
        {
          syslog(LOG_INFO,
                 "[scan] pin=%d skipped (UART0 console)\n", pin);
          continue;
        }

      /* Skip LCD signal pins (both eyes + backlight) */

      if (pin == g_lcd_left.sclk  || pin == g_lcd_left.cs  ||
          pin == g_lcd_left.mosi  || pin == g_lcd_left.dc  ||
          pin == g_lcd_left.rst   ||
          pin == g_lcd_right.sclk || pin == g_lcd_right.cs ||
          pin == g_lcd_right.mosi || pin == g_lcd_right.dc ||
          pin == g_lcd_right.rst  ||
          pin == LCD_PIN_BL)
        {
          syslog(LOG_INFO,
                 "[scan] pin=%d skipped (LCD signal)\n", pin);
          continue;
        }

      /* Skip SWD debug port: SWCLK=20, SWDIO=21 */

      if (pin == 20 || pin == 21)
        {
          syslog(LOG_INFO,
                 "[scan] pin=%d skipped (SWD debug)\n", pin);
          continue;
        }

      /* Skip DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          syslog(LOG_INFO,
                 "[scan] pin=%d skipped (DVP camera reserved)\n", pin);
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
      up_mdelay(800);

      syslog(LOG_INFO, "[scan] pin=%d high\n", pin);
    }

  syslog(LOG_INFO,
         "[scan] done - check screen for any change\n");

  return OK;
}

/****************************************************************************
 * Name: lcdtest_pwr
 *
 * Description:
 *   Full power-on sequence with configurable GPIO range for binary-search
 *   identification of the LCD power enable pin.
 *
 *   Usage: lcdtest pwr [lo] [hi]
 *     lo..hi  — decimal GPIO range to drive high (default 0..52)
 *
 *   Always skips: 2,3,4,5,25,29 (LCD)  10,11 (UART)  7,8 (motor)
 *                 20,21 (SWD)  38,39 (LEDs)
 *
 ****************************************************************************/

static int lcdtest_pwr(int lo, int hi)
{
  int pin;
  int count = 0;
  int off = 0;
  char pinbuf[256];

#ifdef CONFIG_EXAMPLES_GC2145_ID
  if (bk7258_camera_dvp_active())
    {
      syslog(LOG_ERR,
             "[lcdtest] refused: DVP pins P29-P39 are in use, "
             "run 'camera stop' first\n");
      return -EBUSY;
    }
#endif

  /* Step 1: drive GPIO lo..hi high, skip reserved pins.
   * Collect the actual pin list for the syslog dump.
   */

  syslog(LOG_INFO,
         "[pwr] step1: GPIO %d-%d high (skip reserved)\n",
         lo, hi);

  for (pin = lo; pin <= hi; pin++)
    {
      /* LCD signals (both eyes) */

      if (pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
          pin == 6 || pin == 7 || pin == 22 || pin == 23 ||
          pin == 24 || pin == 25 || pin == 45)
        {
          continue;
        }

      /* UART0 console */

      if (pin == 10 || pin == 11)
        {
          continue;
        }

      /* Vibration motor (P7 is right-eye DC, only skip P8) */

      if (pin == 8)
        {
          continue;
        }

      /* Indicator LEDs */

      if (pin == 38 || pin == 39)
        {
          continue;
        }

      /* SWD debug port: SWCLK=20, SWDIO=21 */

      if (pin == 20 || pin == 21)
        {
          continue;
        }

      /* DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
      count++;

      if (off < (int)sizeof(pinbuf) - 8)
        {
          off += snprintf(pinbuf + off, sizeof(pinbuf) - off,
                          " %d", pin);
        }
    }

  pinbuf[off] = '\0';

  /* Print which pins were actually driven high */

  syslog(LOG_INFO,
         "[pwr] driven high (%d pins):%s\n", count, pinbuf);

  /* Step 2: wait for power rails to settle */

  syslog(LOG_INFO, "[pwr] step2: wait 50 ms ...\n");
  up_mdelay(50);

  /* Step 3: backlight on */

  syslog(LOG_INFO, "[pwr] step3: backlight P25 high\n");
  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* Step 4: setup left-eye SPI pins + hardware reset */

  syslog(LOG_INFO,
         "[pwr] step4: left SPI setup + RST low 15 ms then high\n");
  lcd_setup_pins(&g_lcd_left);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);

  /* Step 5: GC9D01 init sequence */

  syslog(LOG_INFO, "[pwr] step5: GC9D01 init sequence\n");
  lcd_init_sequence(true);

  /* Step 6: clear screen to black */

  syslog(LOG_INFO, "[pwr] step6: fill screen black\n");
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  /* Step 7: draw 40x40 red square at (60,60) */

  syslog(LOG_INFO,
         "[pwr] step7: fill 40x40 red at (60,60)\n");
  lcd_fill_rect(60, 60,
                60 + SQ_SIZE - 1, 60 + SQ_SIZE - 1,
                0xf800);

  syslog(LOG_INFO,
         "[pwr] done (%d pins driven) - "
         "screen should show red on black\n",
         count);
  return OK;
}

/****************************************************************************
 * Name: lcdtest_go
 *
 * Description:
 *   One-step dual-eye LCD bring-up: left (red) + right (blue).
 *
 ****************************************************************************/

static int lcdtest_go(void)
{
#ifdef CONFIG_EXAMPLES_GC2145_ID
  if (bk7258_camera_dvp_active())
    {
      syslog(LOG_ERR,
             "[lcdtest] refused: DVP pins P29-P39 are in use, "
             "run 'camera stop' first\n");
      return -EBUSY;
    }
#endif

  /* Step 1: LDO_3V3 enable (P52).
   * Early development blindly drove GPIO 0-52 high to find the power
   * enable pin.  Binary search confirmed LDO33_EN=P52 alone is
   * sufficient.  The blind scan also drove P44(LCD_TE), P27/P28(DVP)
   * and other unrelated pins — leaving it active is a hazard.
   */

  syslog(LOG_INFO, "[go] step1: LDO_3V3 ON (P52)\n");

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);   /* wait for LDO output to stabilize */

  syslog(LOG_INFO, "[go] step1 done\n");

  /* Step 3: backlight on (shared) */

  syslog(LOG_INFO, "[go] step3: backlight P25 high\n");
  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* ---- Left eye ---- */

  syslog(LOG_INFO, "[go] step4 [left]: SPI pins setup\n");
  lcd_setup_pins(&g_lcd_left);

  syslog(LOG_INFO, "[go] step5 [left]: RST + init (display off)\n");
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);

  syslog(LOG_INFO, "[go] step6 [left]: draw eye\n");
  draw_eye();

  syslog(LOG_INFO, "[go] step7 [left]: display on\n");
  lcd_display_on();

  /* ---- Right eye ---- */

  syslog(LOG_INFO, "[go] step9 [right]: SPI pins setup\n");
  lcd_setup_pins(&g_lcd_right);

  syslog(LOG_INFO, "[go] step10 [right]: RST + init (display off)\n");
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);

  syslog(LOG_INFO, "[go] step11 [right]: draw eye\n");
  draw_eye();

  syslog(LOG_INFO, "[go] step12 [right]: display on\n");
  lcd_display_on();

  syslog(LOG_INFO,
         "[go] done - both eyes show robot eye\n");
  return OK;
}

/****************************************************************************
 * Name: lcdtest_anim
 *
 * Description:
 *   Eye animation demo: pupils track left/centre/right, with blinking.
 *   Reuses go's power-on + init sequence, then runs 3 animation rounds.
 *
 ****************************************************************************/

static int lcdtest_anim(void)
{
  int pin;
  int count = 0;
  int round;
  int gaze = 0;

  /* --- Power-on (same as go step1-3) --- */

  syslog(LOG_INFO,
         "[anim] step1: GPIO 0-52 high (skip reserved)\n");

  for (pin = 0; pin <= 52; pin++)
    {
      if (pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
          pin == 6 || pin == 7 || pin == 22 || pin == 23 ||
          pin == 24 || pin == 25 || pin == 45)
        {
          continue;
        }

      if (pin == 10 || pin == 11 || pin == 8 ||
          pin == 20 || pin == 21 ||
          pin == 38 || pin == 39)
        {
          continue;
        }

      /* DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
      count++;
    }

  syslog(LOG_INFO, "[anim] step1 done: %d pins\n", count);

  syslog(LOG_INFO, "[anim] step2: wait 500 ms\n");
  up_mdelay(500);

  syslog(LOG_INFO, "[anim] step3: backlight P25 high\n");
  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* --- Init left eye --- */

  syslog(LOG_INFO, "[anim] init left eye\n");
  lcd_setup_pins(&g_lcd_left);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  eye_draw_full(0);
  lcd_display_on();

  /* --- Init right eye --- */

  syslog(LOG_INFO, "[anim] init right eye\n");
  lcd_setup_pins(&g_lcd_right);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  eye_draw_full(0);
  lcd_display_on();

  /* --- Animation loop: 3 rounds --- */

  for (round = 0; round < 3; round++)
    {
      syslog(LOG_INFO, "[anim] round %d/3\n", round + 1);

      /* Look left: 0 -> -24 */

      syslog(LOG_INFO, "[anim] look left\n");
      lcd_set_pins(&g_lcd_left);
      eye_move_pupil(gaze, -24);
      lcd_set_pins(&g_lcd_right);
      eye_move_pupil(gaze, -24);
      gaze = -24;
      up_mdelay(500);

      /* Look centre: -24 -> 0 */

      syslog(LOG_INFO, "[anim] look centre\n");
      lcd_set_pins(&g_lcd_left);
      eye_move_pupil(gaze, 0);
      lcd_set_pins(&g_lcd_right);
      eye_move_pupil(gaze, 0);
      gaze = 0;
      up_mdelay(500);

      /* Look right: 0 -> +24 */

      syslog(LOG_INFO, "[anim] look right\n");
      lcd_set_pins(&g_lcd_left);
      eye_move_pupil(gaze, 24);
      lcd_set_pins(&g_lcd_right);
      eye_move_pupil(gaze, 24);
      gaze = 24;
      up_mdelay(500);

      /* Look centre: +24 -> 0 */

      syslog(LOG_INFO, "[anim] look centre\n");
      lcd_set_pins(&g_lcd_left);
      eye_move_pupil(gaze, 0);
      lcd_set_pins(&g_lcd_right);
      eye_move_pupil(gaze, 0);
      gaze = 0;
      up_mdelay(500);

      /* TODO: eye_blink disabled — bit-bang redraw too slow,
       * enable after switching to QSPI with smooth refresh.
       */
    }

  syslog(LOG_INFO, "[anim] done\n");
  return OK;
}

/****************************************************************************
 * Name: lcdtest_emo
 *
 * Description:
 *   Cozmo/Vector-style expression eye animation.
 *   Two rounds of: neutral -> look L/C/R/C -> happy -> blink -> neutral.
 *
 ****************************************************************************/

static int lcdtest_emo(void)
{
  int pin;
  int count = 0;
  int round;

  /* --- Power-on (same as go) --- */

  syslog(LOG_INFO,
         "[emo] step1: GPIO 0-52 high (skip reserved)\n");

  for (pin = 0; pin <= 52; pin++)
    {
      if (pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
          pin == 6 || pin == 7 || pin == 22 || pin == 23 ||
          pin == 24 || pin == 25 || pin == 45)
        {
          continue;
        }

      if (pin == 10 || pin == 11 || pin == 8 ||
          pin == 20 || pin == 21 ||
          pin == 38 || pin == 39)
        {
          continue;
        }

      /* DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
      count++;
    }

  syslog(LOG_INFO, "[emo] step1 done: %d pins\n", count);

  syslog(LOG_INFO, "[emo] step2: wait 500 ms\n");
  up_mdelay(500);

  syslog(LOG_INFO, "[emo] step3: backlight P25 high\n");
  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* --- Init left eye --- */

  syslog(LOG_INFO, "[emo] init left eye\n");
  lcd_setup_pins(&g_lcd_left);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);
  eye_neutral();
  lcd_display_on();

  /* --- Init right eye --- */

  syslog(LOG_INFO, "[emo] init right eye\n");
  lcd_setup_pins(&g_lcd_right);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);
  eye_neutral();
  lcd_display_on();

  /* --- Expression loop: 2 rounds --- */

  for (round = 0; round < 2; round++)
    {
      syslog(LOG_INFO, "[emo] round %d/2\n", round + 1);

      /* neutral */

      syslog(LOG_INFO, "[emo] neutral\n");
      lcd_set_pins(&g_lcd_left);
      eye_neutral();
      lcd_set_pins(&g_lcd_right);
      eye_neutral();
      up_mdelay(800);

      /* look left */

      syslog(LOG_INFO, "[emo] look left\n");
      lcd_set_pins(&g_lcd_left);
      eye_look(-22);
      lcd_set_pins(&g_lcd_right);
      eye_look(-22);
      up_mdelay(600);

      /* look centre */

      syslog(LOG_INFO, "[emo] look centre\n");
      lcd_set_pins(&g_lcd_left);
      eye_look(0);
      lcd_set_pins(&g_lcd_right);
      eye_look(0);
      up_mdelay(600);

      /* look right */

      syslog(LOG_INFO, "[emo] look right\n");
      lcd_set_pins(&g_lcd_left);
      eye_look(22);
      lcd_set_pins(&g_lcd_right);
      eye_look(22);
      up_mdelay(600);

      /* happy */

      syslog(LOG_INFO, "[emo] happy\n");
      lcd_set_pins(&g_lcd_left);
      eye_happy();
      lcd_set_pins(&g_lcd_right);
      eye_happy();
      up_mdelay(1000);

      /* TODO: eye_emo_blink disabled — bit-bang scanline blink
       * looks blocky; enable after switching to QSPI.
       */

      /* neutral */

      syslog(LOG_INFO, "[emo] neutral\n");
      lcd_set_pins(&g_lcd_left);
      eye_neutral();
      lcd_set_pins(&g_lcd_right);
      eye_neutral();
      up_mdelay(600);
    }

  syslog(LOG_INFO, "[emo] done\n");
  return OK;
}

/****************************************************************************
 * Name: lcdtest_hear
 *
 * Description:
 *   Continuous sound localization loop.
 *   LCD + audio_init once at start.  Then loop up to 300 iterations:
 *     capture N=2048 → onset gate → locate → eye_look → print.
 *   Exits on iteration cap or serial keypress.
 *
 ****************************************************************************/

#define HEAR_LOOP_MAX    80    /* ~4-5 s at ~60 ms per iteration */
#define HEAR_N_SAMPLES   2048
#define HEAR_ONSET_GATE  80   /* RMS threshold to trigger locate */

static int lcdtest_hear(void)
{
  int tau_q8;
  int dx;
  int iter;
  int n;
  int energy;
  int pin;
  int count = 0;

  /* ---- Hardware bring-up (same as lcdtest_go steps 1-5,7,9-10,12
   *       but WITHOUT draw_eye — we paint emo eyes directly).
   * ---- Step 1: GPIO 0-52 high, skip reserved pins ---- */

  for (pin = 0; pin <= 52; pin++)
    {
      if (pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
          pin == 6 || pin == 7 || pin == 22 || pin == 23 ||
          pin == 24 || pin == 25 || pin == 45)
        {
          continue;
        }

      if (pin == 10 || pin == 11 || pin == 8 ||
          pin == 20 || pin == 21 ||
          pin == 38 || pin == 39)
        {
          continue;
        }

      /* DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
      count++;
    }

  /* Step 2: wait for power rails */

  up_mdelay(500);

  /* Step 3: backlight on */

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* ---- Left eye: SPI pins + RST + init + display on ---- */

  lcd_setup_pins(&g_lcd_left);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  lcd_display_on();

  /* ---- Right eye: SPI pins + RST + init + display on ---- */

  lcd_setup_pins(&g_lcd_right);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  lcd_display_on();

  /* ---- Draw emo eyes (no round-eye flash) ---- */

  lcd_set_pins(&g_lcd_left);
  eye_neutral();
  lcd_set_pins(&g_lcd_right);
  eye_neutral();
  lcd_set_pins(&g_lcd_left);

  /* Initialize audio ADC (once) */

  audio_init();

  /* Suppress per-capture syslog during tight loop */

  bk7258_mic_set_quiet(true);

  syslog(LOG_INFO,
         "[hear] === sound localization loop ===\n"
         "[hear] loop_max=%d  N=%d  onset_gate=%d\n",
         HEAR_LOOP_MAX, HEAR_N_SAMPLES, HEAR_ONSET_GATE);

  for (iter = 0; iter < HEAR_LOOP_MAX; iter++)
    {
      /* Capture a chunk */

      n = audio_capture(HEAR_N_SAMPLES);
      if (n < 64)
        {
          continue;
        }

      /* Onset gate: check energy before running full locate */

      energy = bk7258_mic_energy(n);
      if (energy < HEAR_ONSET_GATE)
        {
          continue;
        }

      /* Sound detected — run full locate on this capture */

      dx = mic_locate_process(n, &tau_q8);

      syslog(LOG_INFO,
             "[hear] iter=%d  energy=%d  dx=%d  "
             "tau_q8=%d (%.2f samp)\n",
             iter, energy, dx,
             tau_q8, (double)tau_q8 / 256.0);

      /* Drive eye toward detected direction (both panels) */

      lcd_set_pins(&g_lcd_left);
      eye_look(dx);
      lcd_set_pins(&g_lcd_right);
      eye_look(dx);
      lcd_set_pins(&g_lcd_left);
    }

  /* Cleanup */

  bk7258_mic_set_quiet(false);
  audio_deinit();

  syslog(LOG_INFO,
         "[hear] loop ended after %d iterations\n", iter);

  return OK;
}

/****************************************************************************
 * Name: lcdtest_oeye
 *
 * Description:
 *   Preview official-website-style eyes on both panels.
 *   Cycles: neutral → half → blink → neutral → happy → neutral.
 *
 ****************************************************************************/

static int lcdtest_oeye(void)
{
  /* Hardware bring-up (same as lcdtest_go but skip draw_eye) */

  int pin;
  int count = 0;

  for (pin = 0; pin <= 52; pin++)
    {
      if (pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
          pin == 6 || pin == 7 || pin == 22 || pin == 23 ||
          pin == 24 || pin == 25 || pin == 45)
        {
          continue;
        }

      if (pin == 10 || pin == 11 || pin == 8 ||
          pin == 20 || pin == 21 ||
          pin == 38 || pin == 39)
        {
          continue;
        }

      /* DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
      count++;
    }

  up_mdelay(500);

  /* Left panel init (no display-on yet — wait until white is ready) */

  lcd_setup_pins(&g_lcd_left);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);

  /* Right panel init (no display-on yet) */

  lcd_setup_pins(&g_lcd_right);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);

  /* Fill both panels white BEFORE display-on and backlight.
   * This eliminates the flash of random framebuffer content
   * that occurs when backlight and display-on happen first.
   */

  lcd_set_pins(&g_lcd_left);
  draw_oeye_bg();
  lcd_set_pins(&g_lcd_right);
  draw_oeye_bg();

  /* White is ready — now enable display on both panels */

  lcd_set_pins(&g_lcd_left);
  lcd_display_on();
  lcd_set_pins(&g_lcd_right);
  lcd_display_on();

  /* Backlight ON last — screen shows white, not random garbage */

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  lcd_set_pins(&g_lcd_left);

  syslog(LOG_INFO,
         "[oeye] === official eye preview (partial redraw) ===\n"
         "[oeye] iris_r=%d pupil_r=%d hl1_r=%d hl2_r=%d\n",
         OEYE_IRIS_R, OEYE_PUPIL_R, OEYE_HL1_R, OEYE_HL2_R);

  /* --- Frame 1: neutral (fully open) --- */

  syslog(LOG_INFO, "[oeye] frame: NEUTRAL\n");

  lcd_set_pins(&g_lcd_left);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_right);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_left);

  up_mdelay(2000);

  /* --- Frame 2: half-closed --- */

  syslog(LOG_INFO, "[oeye] frame: HALF\n");

  lcd_set_pins(&g_lcd_left);
  eye_o_half();
  lcd_set_pins(&g_lcd_right);
  eye_o_half();
  lcd_set_pins(&g_lcd_left);

  up_mdelay(1200);

  /* --- Frame 3: smooth blink (incremental eyelid sweep) --- */

  syslog(LOG_INFO, "[oeye] frame: SMOOTH BLINK\n");

  lcd_set_pins(&g_lcd_left);
  eye_o_blink_anim();
  lcd_set_pins(&g_lcd_right);
  eye_o_blink_anim();
  lcd_set_pins(&g_lcd_left);

  /* --- Frame 4: back to neutral --- */

  syslog(LOG_INFO, "[oeye] frame: NEUTRAL\n");

  lcd_set_pins(&g_lcd_left);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_right);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_left);

  up_mdelay(2000);

  /* --- Frame 5: happy --- */

  syslog(LOG_INFO, "[oeye] frame: HAPPY\n");

  lcd_set_pins(&g_lcd_left);
  eye_o_happy();
  lcd_set_pins(&g_lcd_right);
  eye_o_happy();
  lcd_set_pins(&g_lcd_left);

  up_mdelay(2000);

  /* --- Frame 6: back to neutral --- */

  syslog(LOG_INFO, "[oeye] frame: NEUTRAL\n");

  lcd_set_pins(&g_lcd_left);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_right);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_left);

  up_mdelay(2000);

  syslog(LOG_INFO, "[oeye] done\n");

  return OK;
}

/****************************************************************************
 * Name: lcdtest_blink
 *
 * Description:
 *   Preview smooth blink animation on both panels.
 *   Usage: lcdtest blink [count]   (default 5 blinks)
 *
 ****************************************************************************/

static int lcdtest_blink(int count)
{
  int pin;
  int i;

  /* Power-on non-LCD GPIOs */

  for (pin = 0; pin <= 52; pin++)
    {
      if (pin == 2 || pin == 3 || pin == 4 || pin == 5 ||
          pin == 6 || pin == 7 || pin == 22 || pin == 23 ||
          pin == 24 || pin == 25 || pin == 45)
        {
          continue;
        }

      if (pin == 10 || pin == 11 || pin == 8 ||
          pin == 20 || pin == 21 ||
          pin == 38 || pin == 39)
        {
          continue;
        }

      /* DVP camera connector: P29–P39 */

      if (lcdtest_is_dvp_reserved_pin(pin))
        {
          continue;
        }

      gpio_set_output(pin);
      gpio_write(pin, 1);
    }

  up_mdelay(500);

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  /* Left panel init */

  lcd_setup_pins(&g_lcd_left);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  lcd_display_on();

  /* Right panel init */

  lcd_setup_pins(&g_lcd_right);
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(false);
  lcd_display_on();

  /* White background */

  lcd_set_pins(&g_lcd_left);
  draw_oeye_bg();
  lcd_set_pins(&g_lcd_right);
  draw_oeye_bg();

  syslog(LOG_INFO,
         "[blink] === smooth blink preview: %d blinks ===\n", count);

  /* Initial neutral on both panels */

  lcd_set_pins(&g_lcd_left);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_right);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_left);

  up_mdelay(1000);

  /* Blink loop */

  for (i = 0; i < count; i++)
    {
      syslog(LOG_INFO, "[blink] blink %d/%d\n", i + 1, count);

      lcd_set_pins(&g_lcd_left);
      eye_o_blink_anim();
      lcd_set_pins(&g_lcd_right);
      eye_o_blink_anim();
      lcd_set_pins(&g_lcd_left);

      /* Brief pause between blinks */

      up_mdelay(800 + (i % 3) * 400); /* vary rhythm slightly */
    }

  /* End neutral */

  lcd_set_pins(&g_lcd_left);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_right);
  eye_o_neutral();
  lcd_set_pins(&g_lcd_left);

  syslog(LOG_INFO, "[blink] done\n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_lcdtest_main
 *
 * Description:
 *   NSH command "lcdtest":
 *     lcdtest          — staged bring-up (A/B/C, diagnostics)
 *     lcdtest go       — one-step dual-eye LCD bring-up
 *     lcdtest anim     — pupil tracking animation (3 rounds)
 *     lcdtest emo      — Cozmo/Vector expression eyes (2 rounds)
 *     lcdtest oeye     — official-website-style eye preview
 *     lcdtest blink [N] — smooth blink preview (default 5 blinks)
 *     lcdtest hear     — sound localization → eye_look
 *     lcdtest mic      — raw ADC capture + RMS dump
 *     lcdtest pwr lo hi — power-on GPIO range for binary-search
 *     lcdtest scan     — GPIO pin scan for LCD power enable
 *     lcdtest one left|right — single-panel isolation test (red fill)
 *     lcdtest pat [left|right|both] — quantifiable test pattern
 *
 ****************************************************************************/

/****************************************************************************
 * Name: lcdtest_pat
 *
 * Description:
 *   Quantifiable test pattern for diagnosing byte-level SPI artifacts.
 *   Draws a single continuous frame (one CASET/RASET + one 0x2C + entire
 *   pixel data in a single CS-low session) to eliminate multi-transfer
 *   boundary interference.
 *
 *   Pattern (160×160 RGB565):
 *     a) 1px alternating vertical stripes: even col white(0xFFFF),
 *        odd col black(0x0000).  Any byte offset flips entire regions.
 *     b) 1px red(0xF800) border on all 4 edges.
 *     c) 8×8 solid squares at four corners: top-left green(0x07E0),
 *        top-right blue(0x001F), bottom-left yellow(0xFFE0),
 *        bottom-right magenta(0xF81F).  Detects shift/mirror.
 *     d) 1px horizontal white line at y=80.  Phase reference for
 *        vertical stripes.
 *
 *   Usage: lcdtest pat [left|right|both]  (default both)
 *
 ****************************************************************************/

static int lcdtest_pat_draw(const lcd_pins_t *pins, const char *label)
{
  /* Static line buffer — 160 pixels × 2 bytes = 320 bytes.
   * Must be static: lcdtest stack is only 2048 bytes.
   */

  static uint8_t linebuf[LCD_WIDTH * 2];
  int x;
  int y;
  size_t total_bytes = 0;

  /* Power on + init */

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  lcd_setup_pins(pins);

  gpio_write(pins->rst, 0);
  up_mdelay(15);
  gpio_write(pins->rst, 1);
  up_mdelay(120);

  lcd_init_sequence(false);

  syslog(LOG_INFO,
         "[pat] %s: init done (SCLK=P%d CS=P%d MOSI=P%d)\n",
         label, pins->sclk, pins->cs, pins->mosi);

  /* CASET + RASET for full screen */

  {
    uint8_t ca[4] = {0, 0, 0, LCD_WIDTH - 1};
    uint8_t ra[4] = {0, 0, 0, LCD_HEIGHT - 1};

    lcd_send_cmd_data(0x2a, ca, 4);
    lcd_send_cmd_data(0x2b, ra, 4);
  }

  /* RAMWR — send pixel data in one continuous CS-low session */

  lcd_send_cmd(0x2c);

  /* Switch to SPI mode if using HW path, assert CS once */

  gpio_write_fast(&g_cache_dc, 1);

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  if (lcd_hw_spi_usable())
    {
      lcd_spi_pins_to_spi();
    }
#endif

  gpio_write_fast(&g_cache_cs, 0);

  for (y = 0; y < LCD_HEIGHT; y++)
    {
      for (x = 0; x < LCD_WIDTH; x++)
        {
          uint16_t color;
          int off = x * 2;

          /* (a) Default: alternating vertical stripes */

          if ((x & 1) == 0)
            {
              color = 0xffff;  /* even column: white */
            }
          else
            {
              color = 0x0000;  /* odd column: black */
            }

          /* (d) Horizontal white line at y=80 */

          if (y == 80)
            {
              color = 0xffff;
            }

          /* (b) 1px red border */

          if (y == 0 || y == LCD_HEIGHT - 1 ||
              x == 0 || x == LCD_WIDTH - 1)
            {
              color = 0xf800;
            }

          /* (c) Corner 8×8 squares (override stripes + border) */

          if (y < 8 && x < 8)
            {
              color = 0x07e0;  /* top-left: green */
            }
          else if (y < 8 && x >= LCD_WIDTH - 8)
            {
              color = 0x001f;  /* top-right: blue */
            }
          else if (y >= LCD_HEIGHT - 8 && x < 8)
            {
              color = 0xffe0;  /* bottom-left: yellow */
            }
          else if (y >= LCD_HEIGHT - 8 && x >= LCD_WIDTH - 8)
            {
              color = 0xf81f;  /* bottom-right: magenta */
            }

          linebuf[off]     = (uint8_t)(color >> 8);
          linebuf[off + 1] = (uint8_t)(color & 0xff);
        }

      /* Send this line (320 bytes) — CS stays low across all lines */

      if (lcd_hw_spi_usable())
        {
          if (lcd_spi_write(linebuf, sizeof(linebuf), NULL) != 0)
            {
              goto pat_bb_fallback;
            }
        }
      else
        {
          /* Bit-bang path */

          int i;

          for (i = 0; i < (int)sizeof(linebuf); i++)
            {
              spi_write_byte(linebuf[i]);
            }
        }

      total_bytes += sizeof(linebuf);
    }

  /* Deassert CS + display on */

  gpio_write_fast(&g_cache_cs, 1);

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  if (g_pins_in_spi_mode)
    {
      lcd_spi_pins_to_gpio();
    }
#endif

  lcd_send_cmd(0x29);  /* display on */

  syslog(LOG_INFO,
         "[pat] %s: done — sent %zu bytes (expected %d)\n",
         label, total_bytes, LCD_WIDTH * LCD_HEIGHT * 2);

  if (total_bytes != (size_t)(LCD_WIDTH * LCD_HEIGHT * 2))
    {
      syslog(LOG_ERR,
             "[pat] %s: BYTE COUNT MISMATCH! sent=%zu expected=%d\n",
             label, total_bytes, LCD_WIDTH * LCD_HEIGHT * 2);
    }

  return 0;

pat_bb_fallback:
  /* HW SPI failed mid-frame — finish with bit-bang */

  {
    int y2;
    int i;

    for (y2 = y + 1; y2 < LCD_HEIGHT; y2++)
      {
        /* Regenerate line (we need to, linebuf was for line y) */

        for (x = 0; x < LCD_WIDTH; x++)
          {
            uint16_t color;
            int off = x * 2;

            if ((x & 1) == 0)
              {
                color = 0xffff;
              }
            else
              {
                color = 0x0000;
              }

            if (y2 == 80)
              {
                color = 0xffff;
              }

            if (y2 == 0 || y2 == LCD_HEIGHT - 1 ||
                x == 0 || x == LCD_WIDTH - 1)
              {
                color = 0xf800;
              }

            if (y2 < 8 && x < 8)
              {
                color = 0x07e0;
              }
            else if (y2 < 8 && x >= LCD_WIDTH - 8)
              {
                color = 0x001f;
              }
            else if (y2 >= LCD_HEIGHT - 8 && x < 8)
              {
                color = 0xffe0;
              }
            else if (y2 >= LCD_HEIGHT - 8 && x >= LCD_WIDTH - 8)
              {
                color = 0xf81f;
              }

            linebuf[off]     = (uint8_t)(color >> 8);
            linebuf[off + 1] = (uint8_t)(color & 0xff);
          }

        for (i = 0; i < (int)sizeof(linebuf); i++)
          {
            spi_write_byte(linebuf[i]);
          }

        total_bytes += sizeof(linebuf);
      }
  }

  gpio_write_fast(&g_cache_cs, 1);
  lcd_spi_pins_to_gpio();
  lcd_send_cmd(0x29);

  syslog(LOG_INFO,
         "[pat] %s: HW SPI failed, finished with bit-bang — "
         "sent %zu bytes\n",
         label, total_bytes);
  return 0;
}

/****************************************************************************
 * Name: lcdtest_cross_draw / lcdtest_cross
 *
 * Description:
 *   Enclosure-alignment pattern.  The two GC9D01 round panels are glued
 *   to the PCB and cannot be moved, so the eye-window positions of the
 *   3D-printed front plate must be derived from the panels — not the
 *   other way round.
 *
 *   The module outline is 20.12 x 22.3 mm: the extra 2.18 mm of height
 *   is the FPC / driver-IC bonding tab at the BOTTOM, which means the
 *   circular 18 mm active area (AA) is NOT concentric with the glass
 *   outline — the AA centre sits roughly 1.1 mm above the geometric
 *   centre of the glass bounding box.  Centring the front-plate window
 *   on the glass therefore clips the image (this is exactly how the
 *   first printed plate failed).
 *
 *   This pattern turns the panel itself into a measuring instrument:
 *   the red mark is drawn at pixel (80,80), i.e. the true AA centre,
 *   so its physical position can be read straight off a face-on photo.
 *
 *   Photograph face-on with a steel rule in frame and the four M3
 *   mounting holes visible (57.00 x 47.00 mm, D3.0 — the only exactly
 *   known datum on the board), then measure:
 *     1. distance between the two red marks  = true eye spacing
 *     2. offset of each red mark from the mounting holes
 *
 *   Usage: lcdtest cross [left|right|both]   (default both)
 *
 ****************************************************************************/

static void lcdtest_cross_draw(const lcd_pins_t *pins, const char *label)
{
  /* Power on + init.  lcd_init_sequence(true) leaves the display ON, so
   * the pattern is visible without a separate lcd_display_on() call.
   */

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  lcd_setup_pins(pins);

  gpio_write(pins->rst, 0);
  up_mdelay(15);
  gpio_write(pins->rst, 1);
  up_mdelay(120);

  lcd_init_sequence(true);

  /* Black background */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0x0000);

  /* Outer ring r=79 — the AA boundary.  Drawn as a filled disc then
   * hollowed out, reusing lcd_fill_circle rather than a pixel loop.
   */

  lcd_fill_circle(80, 80, 79, 0xffff);
  lcd_fill_circle(80, 80, 78, 0x0000);

  /* Intermediate reference ring r=40 */

  lcd_fill_circle(80, 80, 40, 0xffff);
  lcd_fill_circle(80, 80, 39, 0x0000);

  /* Full-width crosshair through the AA centre */

  lcd_fill_rect(0, 80, LCD_WIDTH - 1, 80, 0xffff);
  lcd_fill_rect(80, 0, 80, LCD_HEIGHT - 1, 0xffff);

  /* 5x5 red aiming mark at the AA centre — high contrast against the
   * white crosshair so it stays identifiable in a photograph.
   */

  lcd_fill_rect(78, 78, 82, 82, 0xf800);

  syslog(LOG_INFO,
         "[cross] %s: AA-centre=(80,80) r_outer=79 r_inner=40 "
         "(SCLK=P%d CS=P%d MOSI=P%d)\n",
         label, pins->sclk, pins->cs, pins->mosi);
}

static int lcdtest_cross(int argc, char *argv[])
{
  const char *target = "both";

  if (argc > 2)
    {
      target = argv[2];
    }

  if (strcmp(target, "left") != 0 &&
      strcmp(target, "right") != 0 &&
      strcmp(target, "both") != 0)
    {
      syslog(LOG_ERR,
             "[cross] usage: lcdtest cross [left|right|both]\n");
      return -EINVAL;
    }

  if (strcmp(target, "left") == 0 || strcmp(target, "both") == 0)
    {
      lcdtest_cross_draw(&g_lcd_left, "LEFT");
    }

  if (strcmp(target, "right") == 0 || strcmp(target, "both") == 0)
    {
      lcdtest_cross_draw(&g_lcd_right, "RIGHT");
    }

  syslog(LOG_INFO,
         "[cross] enclosure alignment: photograph face-on, steel rule in\n"
         "[cross]   frame, all four M3 holes visible.  The RED mark is the\n"
         "[cross]   true AA centre of each panel.  Measure (1) red-to-red\n"
         "[cross]   distance = eye spacing, (2) red mark offset from the\n"
         "[cross]   mounting holes (57.00x47.00 mm, D3.0).\n"
         "[cross]   Module bbox 20.12x22.3 mm; AA centre is ~1.1 mm ABOVE\n"
         "[cross]   the glass bounding-box centre — do not centre the\n"
         "[cross]   window on the glass.\n");

  return 0;
}

static int lcdtest_pat(int argc, char *argv[])
{
  const char *target = "both";

  if (argc > 2)
    {
      target = argv[2];
    }

  if (strcmp(target, "left") == 0 || strcmp(target, "both") == 0)
    {
      lcdtest_pat_draw(&g_lcd_left, "LEFT");
    }

  if (strcmp(target, "right") == 0 || strcmp(target, "both") == 0)
    {
      lcdtest_pat_draw(&g_lcd_right, "RIGHT");
    }

  if (strcmp(target, "left") != 0 &&
      strcmp(target, "right") != 0 &&
      strcmp(target, "both") != 0)
    {
      syslog(LOG_ERR, "[pat] usage: lcdtest pat [left|right|both]\n");
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: lcdtest_one
 *
 * Description:
 *   Isolation test for a single panel.  Powers on LDO, initializes one
 *   screen, fills it with solid red, and prints pin diagnostics.
 *   Usage: lcdtest one left | lcdtest one right
 *
 ****************************************************************************/

static int lcdtest_one(int argc, char *argv[])
{
  const lcd_pins_t *pins;
  const char *label;

  if (argc < 3)
    {
      syslog(LOG_ERR, "[one] usage: lcdtest one left|right\n");
      return -EINVAL;
    }

  if (strcmp(argv[2], "left") == 0)
    {
      pins = &g_lcd_left;
      label = "LEFT";
    }
  else if (strcmp(argv[2], "right") == 0)
    {
      pins = &g_lcd_right;
      label = "RIGHT";
    }
  else
    {
      syslog(LOG_ERR, "[one] unknown panel: %s\n", argv[2]);
      return -EINVAL;
    }

  /* Step 1: LDO + backlight */

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);
  syslog(LOG_INFO, "[one] %s: LDO_3V3 ON (P%d)\n", label, LCD_PIN_LDO33_EN);

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);
  syslog(LOG_INFO, "[one] %s: backlight ON (P%d)\n", label, LCD_PIN_BL);

  /* Step 2: pin setup + reset */

  lcd_setup_pins(pins);
  syslog(LOG_INFO, "[one] %s: setup_pins done "
         "(SCLK=P%d CS=P%d MOSI=P%d DC=P%d RST=P%d)\n",
         label, pins->sclk, pins->cs, pins->mosi, pins->dc, pins->rst);

  gpio_write(pins->rst, 0);
  up_mdelay(15);
  gpio_write(pins->rst, 1);
  up_mdelay(120);
  syslog(LOG_INFO, "[one] %s: RST pulse done (0→15ms→1→120ms)\n", label);

  /* Step 3: init sequence (no display-on) */

  lcd_init_sequence(false);
  syslog(LOG_INFO, "[one] %s: init_sequence done\n", label);

  /* Print SPI path used */

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  if (pins == &g_lcd_left && g_hw_spi_capable)
    {
      syslog(LOG_INFO, "[one] %s: data path = HW SPI1\n", label);
    }
  else
#endif
    {
      syslog(LOG_INFO, "[one] %s: data path = bit-bang\n", label);
    }

  /* Step 4: fill entire screen red.
   * Use lcd_fill_rect which correctly holds CS low across the entire
   * RAMWR transfer.  The previous manual loop called lcd_send_data()
   * per 256-pixel chunk, and each call raised CS — terminating the
   * RAMWR prematurely so only the first chunk landed.
   */

  lcd_fill_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, 0xf800);
  syslog(LOG_INFO, "[one] %s: filled %d px RED (0xF800)\n",
         label, LCD_WIDTH * LCD_HEIGHT);

  /* Step 5: display on */

  lcd_send_cmd(0x29);  /* display on */
  syslog(LOG_INFO, "[one] %s: display ON (0x29)\n", label);

  /* Step 6: dump GPIO CFG for all 5 pins */

  syslog(LOG_INFO,
         "[one] %s: GPIO CFG dump:\n"
         "[one]   P%d(SCLK) cfg=0x%05lX  P%d(CS)  cfg=0x%05lX\n"
         "[one]   P%d(MOSI) cfg=0x%05lX  P%d(DC)  cfg=0x%05lX\n"
         "[one]   P%d(RST)  cfg=0x%05lX\n",
         label,
         pins->sclk, (unsigned long)getreg32(BK7258_GPIO_CFG(pins->sclk)),
         pins->cs,   (unsigned long)getreg32(BK7258_GPIO_CFG(pins->cs)),
         pins->mosi, (unsigned long)getreg32(BK7258_GPIO_CFG(pins->mosi)),
         pins->dc,   (unsigned long)getreg32(BK7258_GPIO_CFG(pins->dc)),
         pins->rst,  (unsigned long)getreg32(BK7258_GPIO_CFG(pins->rst)));

  syslog(LOG_INFO, "[one] %s: done — screen should be solid RED\n", label);
  return 0;
}

/****************************************************************************
 * Name: lcdtest_chunk
 *
 * Description:
 *   Probe SPI chunk sizes with full LCD power-on and init.
 *   Usage: lcdtest chunk <bytes> [sram|psram] [one|multi]
 *
 *   bytes   — chunk size in bytes (2..4095, even only, default 1024)
 *   sram    — source buffer in SRAM static array (default)
 *   psram   — source buffer in PSRAM at 0x60200000
 *   one     — one lcd_spi_write per chunk_bytes, loop until screen full (default)
 *   multi   — one lcd_spi_write for entire 51200-byte frame, SPI chunks internally
 *
 *   Four combinations isolate (a) PSRAM vs SRAM source and
 *   (b) single-chunk vs multi-chunk SPI path.
 *
 ****************************************************************************/

/* PSRAM test buffer address — past camera buffers (0x60000000–0x601C2000) */

#define CHUNK_PSRAM_TEST_ADDR  0x60200000u

static int lcdtest_chunk(int argc, char *argv[])
{
  static uint8_t sram_tile[4096];
  int chunk_bytes = SPI_MAX_CHUNK_BYTES;
  int total = 160 * 160 * 2;  /* full screen RGB565 */
  uint32_t saved_max = g_spi_max_chunk;
  const uint8_t *src;
  uint8_t *psram_buf = NULL;
  bool use_psram = false;
  bool use_multi = false;
  size_t sent = 0;
  int ret;
  int i;
  clock_t t0;
  uint32_t elapsed_ms;
  uint32_t throughput_kbps;

  /* Parse arguments */

  if (argc > 2)
    {
      chunk_bytes = atoi(argv[2]);
    }

  for (i = 3; i < argc; i++)
    {
      if (strcmp(argv[i], "psram") == 0)
        {
          use_psram = true;
        }
      else if (strcmp(argv[i], "sram") == 0)
        {
          use_psram = false;
        }
      else if (strcmp(argv[i], "multi") == 0)
        {
          use_multi = true;
        }
      else if (strcmp(argv[i], "one") == 0)
        {
          use_multi = false;
        }
      else
        {
          syslog(LOG_ERR,
                 "[chunk] unknown option: %s\n"
                 "  usage: lcdtest chunk <bytes> [sram|psram] [one|multi]\n",
                 argv[i]);
          return -EINVAL;
        }
    }

  /* Validate chunk_bytes: 2..4095, even (RGB565 = 2 bytes/pixel) */

  if (chunk_bytes < 2 || chunk_bytes > 4095)
    {
      syslog(LOG_ERR,
             "[chunk] error: chunk_bytes=%d out of range (2..4095)\n",
             chunk_bytes);
      return -EINVAL;
    }

  if (chunk_bytes & 1)
    {
      syslog(LOG_ERR,
             "[chunk] error: chunk_bytes=%d must be even "
             "(RGB565 = 2 bytes/pixel)\n",
             chunk_bytes);
      return -EINVAL;
    }

  /* Full power-on + init sequence (same as lcdtest_go steps 1-5).
   * Must run BEFORE lcd_hw_spi_usable() — g_hw_spi_capable is set
   * inside lcd_setup_pins() → lcd_spi_init(), so checking it before
   * init always returns false on cold boot.
   */

  syslog(LOG_INFO,
         "[chunk] init: LDO_3V3 P52 ON, backlight P25 ON\n");

  gpio_set_output(LCD_PIN_LDO33_EN);
  gpio_write(LCD_PIN_LDO33_EN, 1);
  up_mdelay(50);

  gpio_set_output(LCD_PIN_BL);
  gpio_write(LCD_PIN_BL, 1);

  syslog(LOG_INFO, "[chunk] init: SPI pins setup\n");
  lcd_set_pins(&g_lcd_left);
  lcd_setup_pins(&g_lcd_left);

  syslog(LOG_INFO, "[chunk] init: RST pulse + lcd_init_sequence\n");
  gpio_write(g_active_pins->rst, 0);
  up_mdelay(15);
  gpio_write(g_active_pins->rst, 1);
  up_mdelay(120);
  lcd_init_sequence(true);

  /* HW SPI gate — checked AFTER init (g_hw_spi_capable is now set) */

  syslog(LOG_INFO,
         "[chunk] hw_spi_usable=%d\n",
         (int)lcd_hw_spi_usable());

  if (!lcd_hw_spi_usable())
    {
      syslog(LOG_ERR,
             "[chunk] error: HW SPI not usable after init, cannot test\n");
      return -ENODEV;
    }

  syslog(LOG_INFO,
         "[chunk] init done. params: chunk_bytes=%d src=%s mode=%s\n",
         chunk_bytes,
         use_psram ? "psram" : "sram",
         use_multi ? "multi" : "one");

  /* Prepare source buffer */

  if (use_psram)
    {
      ret = bk7258_psram_init();
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "[chunk] PSRAM init failed: %d\n", ret);
          return ret;
        }

      psram_buf = (uint8_t *)CHUNK_PSRAM_TEST_ADDR;
      src = psram_buf;

      /* Fill PSRAM buffer with blue 0x001F */

      for (i = 0; i < total; i += 2)
        {
          psram_buf[i]     = 0x00;
          psram_buf[i + 1] = 0x1f;
        }
    }
  else
    {
      /* Fill SRAM tile with blue — entire tile, not just chunk_bytes */

      for (i = 0; i < (int)sizeof(sram_tile); i += 2)
        {
          sram_tile[i]     = 0x00;
          sram_tile[i + 1] = 0x1f;
        }

      src = sram_tile;
    }

  /* CASET + RASET for full screen */

  {
    uint8_t ca[4] = {0, 0, 0, 159};
    uint8_t ra[4] = {0, 0, 0, 159};

    lcd_send_cmd_data(0x2a, ca, 4);
    lcd_send_cmd_data(0x2b, ra, 4);
  }

  /* Override runtime chunk limit for this test */

  g_spi_max_chunk = (uint32_t)chunk_bytes;

  /* RAMWR — single CS-low session */

  lcd_send_cmd(0x2c);
  gpio_write_fast(&g_cache_dc, 1);
  gpio_write_fast(&g_cache_cs, 0);
  lcd_spi_pins_to_spi();

  t0 = clock_systime_ticks();

  if (use_multi)
    {
      /* MULTI: one lcd_spi_write for the entire frame.
       * If src is SRAM tile (4096 bytes), lcd_spi_write will reuse
       * the same buffer for each internal chunk — this is fine because
       * we're sending constant blue.  For PSRAM, the full 51200-byte
       * buffer is contiguous.
       */

      if (!use_psram)
        {
          /* SRAM tile is only 4096 bytes but we need to send 51200.
           * lcd_spi_write reads from the pointer as it chunks, so
           * we need a full-size SRAM buffer.  Use a second static.
           */

          static uint8_t sram_full[51200];

          for (i = 0; i < (int)sizeof(sram_full); i += 2)
            {
              sram_full[i]     = 0x00;
              sram_full[i + 1] = 0x1f;
            }

          src = sram_full;
        }

      sent = 0;
      ret = lcd_spi_write(src, (size_t)total, &sent);
    }
  else
    {
      /* ONE: loop lcd_spi_write with chunk_bytes-sized batches */

      int remaining = total;

      ret = 0;

      while (remaining > 0)
        {
          int batch = remaining;

          if (batch > chunk_bytes)
            {
              batch = chunk_bytes;
            }

          sent = 0;
          ret = lcd_spi_write(src, (size_t)batch, &sent);

          if (ret != 0)
            {
              break;
            }

          remaining -= batch;
        }
    }

  elapsed_ms = (uint32_t)TICK2MSEC(clock_systime_ticks() - t0);
  lcd_spi_pins_to_gpio();
  gpio_write_fast(&g_cache_cs, 1);
  g_spi_max_chunk = saved_max;

  if (ret != 0)
    {
      syslog(LOG_ERR,
             "[chunk] FAIL: chunk_bytes=%d src=%s mode=%s "
             "sent=%lu/%d elapsed=%lums\n",
             chunk_bytes,
             use_psram ? "psram" : "sram",
             use_multi ? "multi" : "one",
             (unsigned long)sent, total,
             (unsigned long)elapsed_ms);
      return ret;
    }

  /* Avoid division by zero */

  if (elapsed_ms == 0)
    {
      elapsed_ms = 1;
    }

  throughput_kbps = (uint32_t)total * 1000u / elapsed_ms / 1024u;

  syslog(LOG_INFO,
         "[chunk] PASS: chunk_bytes=%d src=%s mode=%s "
         "filled %d bytes elapsed=%lums throughput=%lu KB/s\n",
         chunk_bytes,
         use_psram ? "psram" : "sram",
         use_multi ? "multi" : "one",
         total,
         (unsigned long)elapsed_ms,
         (unsigned long)throughput_kbps);
  return 0;
}

/****************************************************************************
 * Name: lcdtest_clk
 *
 * Description:
 *   Set SPI1 clock divider at runtime.  Actual SPI clock = 26/(div+1) MHz.
 *   Re-writes the CTRL register's clk_rate field and readback-verifies.
 *   Range: 0..255.
 *
 * Usage: lcdtest clk <div>
 *
 ****************************************************************************/

static int lcdtest_clk(int argc, char *argv[])
{
  uint32_t div;
  uint32_t ctrl;
  uint32_t readback;
  int mhz_int;
  int mhz_frac;

  if (argc < 3)
    {
      syslog(LOG_INFO,
             "[clk] current: div=%lu (~%d.%d MHz)\n",
             (unsigned long)g_spi_clk_div,
             26 / (int)(g_spi_clk_div + 1),
             (26 * 10 / (int)(g_spi_clk_div + 1)) % 10);
      syslog(LOG_INFO, "[clk] usage: lcdtest clk <div>  (0..255)\n");
      return 0;
    }

  div = (uint32_t)atoi(argv[2]);
  if (div > 255)
    {
      div = 255;
    }

  g_spi_clk_div = div;

  /* Re-write CTRL register with new clk_rate */

  ctrl = getreg32(SPI_REG_CTRL(BK7258_SPI1_BASE));
  ctrl &= ~(0xFFu << SPI_CTRL_CLK_RATE_SHIFT);  /* clear old clk_rate */
  ctrl |= (div << SPI_CTRL_CLK_RATE_SHIFT);      /* set new clk_rate */
  putreg32(ctrl, SPI_REG_CTRL(BK7258_SPI1_BASE));

  /* Readback verify */

  readback = getreg32(SPI_REG_CTRL(BK7258_SPI1_BASE));

  mhz_int  = 26 / (int)(div + 1);
  mhz_frac = (26 * 10 / (int)(div + 1)) % 10;

  syslog(LOG_INFO,
         "[clk] set div=%lu → ~%d.%d MHz, "
         "CTRL wrote=0x%08lX read=0x%08lX\n",
         (unsigned long)div, mhz_int, mhz_frac,
         (unsigned long)ctrl, (unsigned long)readback);

  if (((readback >> SPI_CTRL_CLK_RATE_SHIFT) & 0xFF) != div)
    {
      syslog(LOG_ERR,
             "[clk] WARNING: clk_rate readback mismatch "
             "(wrote %lu, read %lu)\n",
             (unsigned long)div,
             (unsigned long)((readback >> SPI_CTRL_CLK_RATE_SHIFT) & 0xFF));
    }

  /* Also report tx_fifo_int_level (CTRL bits[1:0]) for reference */

  syslog(LOG_INFO,
         "[clk] tx_fifo_int_level=%lu  (burst=%lu)\n",
         (unsigned long)((readback >> SPI_CTRL_TX_FIFO_INT_LVL_SHIFT) &
                          SPI_CTRL_TX_FIFO_INT_LVL_MASK),
         (unsigned long)g_spi_burst);

  return 0;
}

/****************************************************************************
 * Name: lcdtest_burst
 *
 * Description:
 *   Set SPI FIFO burst write count at runtime.
 *   Accepted values: 1, 16, 32.  (48 is NOT offered — see below.)
 *
 *   burst=1: per-byte tx_fifo_wr_ready polling (safe, 67 KB/s).
 *   burst=16/32: tx_fifo_int level polling, then batch write (416 KB/s).
 *
 *   Updates CTRL bits[1:0] (tx_fifo_int_level) and bit6 (tx_fifo_int_en)
 *   at runtime.  Readback verifies the write.
 *
 *   FIFO depth is 64.  tx_fifo_int_level is an *occupancy* threshold:
 *   interrupt fires when FIFO occupancy <= level, i.e. empty >= 64 - level.
 *   Safe write count = 64 - level.  burst=48 requires level=3 (empty >= 16),
 *   but writes 48 bytes → overflow.  Not offered.
 *
 * Usage: lcdtest burst <1|16|32>
 *
 ****************************************************************************/

static int lcdtest_burst(int argc, char *argv[])
{
  uint32_t n;
  uint32_t level;
  uint32_t ctrl;

  if (argc < 3)
    {
      const char *mode = (g_spi_burst <= 1) ? "per-byte tx_fifo_wr_ready" :
                          "tx_fifo_int level polling";
      uint32_t safe = SPI_FIFO_DEPTH -
                       fifo_level_to_bytes(burst_to_fifo_level(g_spi_burst));

      syslog(LOG_INFO,
             "[burst] current: %lu  (%s, safe_write=%lu)\n",
             (unsigned long)g_spi_burst, mode, (unsigned long)safe);
      syslog(LOG_INFO,
             "[burst] usage: lcdtest burst <1|16|32>\n");
      syslog(LOG_INFO,
             "[burst]    1  = per-byte polling (safe, 67 KB/s)\n");
      syslog(LOG_INFO,
             "[burst]   16  = tx_fifo_int level 1, write 16/poll "
             "(416 KB/s, safe_write=48)\n");
      syslog(LOG_INFO,
             "[burst]   32  = tx_fifo_int level 2, write 32/poll "
             "(416 KB/s, safe_write=32, default)\n");
      syslog(LOG_INFO,
             "[burst]  NOT 48 — level=3 only guarantees 16 empty slots, "
             "writing 48 overflows\n");
      return 0;
    }

  n = (uint32_t)atoi(argv[2]);

  /* Accept only 1, 16, 32.  48 is intentionally rejected. */

  if (n != 1 && n != 16 && n != 32)
    {
      syslog(LOG_ERR,
             "[burst] invalid %lu.  Accepted: 1, 16, 32.  "
             "48 overflows (FIFO_DEPTH=64, level=3 → safe=16).\n",
             (unsigned long)n);
      return -EINVAL;
    }

  g_spi_burst = n;

  /* Update CTRL tx_fifo_int_level (bits[1:0]) and tx_fifo_int_en (bit6) */

  level = burst_to_fifo_level(n);

  ctrl = getreg32(SPI_REG_CTRL(BK7258_SPI1_BASE));
  ctrl &= ~(SPI_CTRL_TX_FIFO_INT_LVL_MASK << SPI_CTRL_TX_FIFO_INT_LVL_SHIFT);
  ctrl &= ~(1u << 6);  /* clear tx_fifo_int_en */

  if (n > 1)
    {
      ctrl |= (level << SPI_CTRL_TX_FIFO_INT_LVL_SHIFT);
      ctrl |= (1u << 6);  /* tx_fifo_int_en */
    }

  putreg32(ctrl, SPI_REG_CTRL(BK7258_SPI1_BASE));

  /* Readback verify */

  uint32_t readback = getreg32(SPI_REG_CTRL(BK7258_SPI1_BASE));
  uint32_t rb_level = (readback >> SPI_CTRL_TX_FIFO_INT_LVL_SHIFT) &
                       SPI_CTRL_TX_FIFO_INT_LVL_MASK;
  uint32_t rb_int_en = (readback >> 6) & 1;

  syslog(LOG_INFO,
         "[burst] set to %lu  level=%lu  safe_write=%lu  "
         "CTRL=0x%08lx  readback level=%lu int_en=%lu %s\n",
         (unsigned long)n,
         (unsigned long)level,
         (unsigned long)(SPI_FIFO_DEPTH - fifo_level_to_bytes(level)),
         (unsigned long)readback,
         (unsigned long)rb_level,
         (unsigned long)rb_int_en,
         (rb_level == level && rb_int_en == (n > 1 ? 1u : 0u)) ?
           "OK" : "MISMATCH!");

  return 0;
}

int bk7258_lcdtest_main(int argc, char *argv[])
{
  if (argc > 1 && strcmp(argv[1], "go") == 0)
    {
      return lcdtest_go();
    }

  if (argc > 1 && strcmp(argv[1], "anim") == 0)
    {
      return lcdtest_anim();
    }

  if (argc > 1 && strcmp(argv[1], "emo") == 0)
    {
      return lcdtest_emo();
    }

  if (argc > 1 && strcmp(argv[1], "oeye") == 0)
    {
      return lcdtest_oeye();
    }

  if (argc > 1 && strcmp(argv[1], "blink") == 0)
    {
      int count = 5;

      if (argc > 2)
        {
          count = atoi(argv[2]);
        }

      if (count < 1)
        {
          count = 1;
        }

      return lcdtest_blink(count);
    }

  if (argc > 1 && strcmp(argv[1], "scan") == 0)
    {
      return lcdtest_scan();
    }

  if (argc > 1 && strcmp(argv[1], "pwr") == 0)
    {
      int lo = 0;
      int hi = 52;

      if (argc > 2)
        {
          lo = atoi(argv[2]);
        }

      if (argc > 3)
        {
          hi = atoi(argv[3]);
        }

      if (lo < 0)
        {
          lo = 0;
        }

      if (hi > 52)
        {
          hi = 52;
        }

      if (lo > hi)
        {
          syslog(LOG_ERR,
                 "[pwr] error: lo(%d) > hi(%d)\n", lo, hi);
          return -EINVAL;
        }

      return lcdtest_pwr(lo, hi);
    }

  if (argc > 1 && strcmp(argv[1], "hear") == 0)
    {
      return lcdtest_hear();
    }

  if (argc > 1 && strcmp(argv[1], "mic") == 0)
    {
      return bk7258_mic_main(argc - 1, &argv[1]);
    }

  if (argc > 1 && strcmp(argv[1], "one") == 0)
    {
      return lcdtest_one(argc, argv);
    }

  if (argc > 1 && strcmp(argv[1], "pat") == 0)
    {
      return lcdtest_pat(argc, argv);
    }

  if (argc > 1 && strcmp(argv[1], "cross") == 0)
    {
      return lcdtest_cross(argc, argv);
    }

#ifdef CONFIG_LCD_GC9D01_HW_SPI
  if (argc > 1 && strcmp(argv[1], "spidiag") == 0)
    {
      return lcd_spidiag();
    }

  if (argc > 1 && strcmp(argv[1], "chunk") == 0)
    {
      return lcdtest_chunk(argc, argv);
    }

  if (argc > 1 && strcmp(argv[1], "clk") == 0)
    {
      return lcdtest_clk(argc, argv);
    }

  if (argc > 1 && strcmp(argv[1], "burst") == 0)
    {
      return lcdtest_burst(argc, argv);
    }
#endif

  return lcdtest_stages();
}

/****************************************************************************
 * DEBUG_JOURNAL — LCD/SPI Performance & Hardware Notes
 ****************************************************************************
 *
 * ========================================================================
 * SPI Performance Measurements (final, 2026-08-15)
 * ========================================================================
 *
 * Full-screen 160×160 = 51200 bytes.
 * Command: lcdtest chunk 4094 sram multi
 *
 *   SPI Clock   Burst   Time     Throughput   Notes
 *   ─────────   ─────   ──────   ──────────   ─────────────────────────
 *   2.0 MHz     1       740 ms    67 KB/s     per-byte polling (old default)
 *   6.5 MHz     1       390 ms   128 KB/s     per-byte polling
 *   6.5 MHz     16      120 ms   416 KB/s     tx_fifo_int level 1
 *   6.5 MHz     32      120 ms   416 KB/s     tx_fifo_int level 2
 *   6.5 MHz     48      FAIL     —            FIFO overflow (see below)
 *   13  MHz     32       50 ms  1000 KB/s     ← FINAL DEFAULT
 *   26  MHz     32       50 ms  1000 KB/s     no further gain
 *
 * Total speedup: 740ms → 50ms ≈ 15×.
 * 26 MHz offers no gain — polling-loop ceiling reached.
 * Timer resolution is 10ms; 50ms actual range is 45-55ms.
 * 1px vertical pattern (lcdtest pat) clean at both 13 and 26 MHz.
 * 13 MHz chosen to preserve timing margin.
 *
 * Final defaults (Kconfig + defconfig):
 *   CONFIG_LCD_GC9D01_SPI_CLK_DIV       = 1   (13 MHz)
 *   CONFIG_LCD_GC9D01_SPI_BURST         = 32
 *   CONFIG_LCD_GC9D01_SPI_BYTE_INTERVAL = 0
 *
 * ========================================================================
 * FIFO Depth: 64, NOT 48
 * ========================================================================
 *
 * SPI_FIFO_DEPTH was originally set to 48, copied from ARMino's
 * SPI_FIFO_INT_LEVEL_48 enum.  That enum defines interrupt *thresholds*,
 * not FIFO capacity.  ARMino's spi_ll.h says "≥48 bytes" — consistent
 * with 64, but was read as "exactly 48".
 *
 * Empirical proof (the ONLY model that explains all three burst levels):
 *
 *   tx_fifo_int_level is an OCCUPANCY threshold:
 *     interrupt fires when FIFO_occupancy <= level
 *     → empty_slots >= FIFO_DEPTH - level
 *
 *   Safe write count = FIFO_DEPTH - level:
 *     level=16 → empty >= 48, write 16 → safe ✓
 *     level=32 → empty >= 32, write 32 → safe ✓ (exact fit)
 *     level=48 → empty >= 16, write 48 → overflow by 32 bytes ✗
 *
 *   Only FIFO_DEPTH=64 makes all three observations consistent.
 *
 * ========================================================================
 * Camera Preview Performance (post-fix, ~7 fps)
 * ========================================================================
 *
 * camera preview 30 left: ~127.6 ms/frame -> ~7 fps  (measured at -O0)
 *   blit               ~50 ms   (SPI transfer)
 *   downsample+convert ~78 ms   <- dominant cost
 *
 * SUPERSEDED 2026-08-20.  This block used to blame the PSRAM access
 * pattern (~76800 scattered reads behind QSPI) and proposed SRAM staging
 * as the fix.  "camera bench" disproved both.  Full analysis lives in
 * bk7258_camera.c's DEBUG_JOURNAL and DEBUG_JOURNAL_zh-cn.md section 25;
 * the short version:
 *
 *   Three paths with identical output pixel count and identical
 *   instruction path, differing only in source location and access order:
 *              -O0                    -Os
 *     A  PSRAM scattered   55 ms       13 ms   (204800 B touched)
 *     C  PSRAM sequential  54 ms       13 ms   ( 51200 B touched)
 *     B  SRAM  sequential  54 ms       12 ms   ( 51200 B touched)
 *
 *   Access pattern is worth 1% (-O0) / 3% (-Os); memory type shows no
 *   measurable difference.  The real cause is -O0 code size: 177
 *   instructions per output pixel, 66 of them (37%) stack accesses.  At
 *   -Os that becomes 43 instructions / 3 stack accesses and the loop runs
 *   4.2x faster.  Measured CPI 1.45 (-O0) / 1.40 (-Os), so XIP flash
 *   instruction fetch is not stalling either.
 *
 * Therefore:
 *   a) SRAM staging: DEAD.  Moving the 204800 B a real downscale needs
 *      costs 44 ms (-O0) / 12 ms (-Os) to recover ~0.4 ms.
 *   b) DVP hardware crop: still useful, but because it cuts the pixel
 *      count, not because it removes scatter.
 *
 * Still NOT optimising the preview path further:
 *   - Full-screen preview is a demo, not the production path.
 *   - Production path: camera -> PSRAM detect -> direction -> screen
 *     draws eyes (partial refresh only).
 *   - A 60x60 iris region = 7200 bytes.  At 1666 KB/s (-Os) -> ~4 ms
 *     blit.  More than sufficient for smooth eye animation.
 *
 * Note on this file's own numbers: the SPI figures above were taken at
 * -O0.  At -Os the same "lcdtest chunk 4094 sram multi" reports 30 ms /
 * 1666 KB/s instead of 50 ms / 1000 KB/s.  The 15x figure in the
 * measurement table below is a -O0-to--O0 comparison and remains valid as
 * such, but absolute timings shift with the optimisation level.
 *
 * ========================================================================
 * Hardware Behavior Gotchas (HIGH RISK — repeated pitfall source)
 * ========================================================================
 *
 * a) SPI trans_len latches on tx_en RISING EDGE (0→1).
 *    Multi-chunk transfer: each chunk must clear tx_en first, then set
 *    it.  Writing trans_len+tx_en=1 in the same store has no rising edge
 *    for the second chunk → FIFO stalls at ~70 bytes, INT_STATUS=0x00000000.
 *
 * b) CFG (REG_0x05) MUST be read-modify-write.
 *    Blank overwrite clears tx_finish_int_en (bit2), which prevents
 *    tx_finish (bit13) from ever asserting → apparent SPI stall.
 *
 * c) trans_len=1 does NOT trigger tx_finish_int.
 *    Single-byte transfers must use bit-bang.  Workaround: LCD driver
 *    uses LCD_GC9D01_HWSPI_MIN_LEN threshold to avoid single-byte HW SPI.
 *
 * d) tx_fifo_wr_ready (INT_STATUS bit1) means "≥1 empty slot", NOT
 *    "48/64 empty slots".  Batch writes must use tx_fifo_int_level
 *    (CTRL bits[1:0]) + tx_fifo_int (INT_STATUS bit8) as the gate.
 *
 * e) GC9D01: CS must stay LOW for the entire pixel transfer after 0x2C
 *    (RAMWR).  Raising CS mid-transfer terminates the RAMWR command.
 *
 * f) GC2145 P0:0x84=0x02 is UYVY, NOT YUYV.
 *    ARMino dvp_gc2145.c's "yuyv" annotation is wrong.
 *    Actual PSRAM layout: U Y0 V Y1 (confirmed by memory dump).
 *
 * g) LDO33_EN=P52 alone is sufficient to power the LCD panel.
 *    lcdtest_go's historical 26-pin blind scan was legacy; removed.
 *
 * h) Registers 0x44010028 / 0x44010030 are SHARED between LCD and DVP
 *    clock configuration.  Must use read-modify-write; blank overwrite
 *    corrupts the other subsystem's clock settings.
 *
 * ========================================================================
 * Methodology Lessons
 * ========================================================================
 *
 * 1. Do not use variables that don't reflect real state as guards.
 *    This same anti-pattern caused four bugs in this sprint:
 *
 *    - lcd_spi_pins_to_gpio() hardcoded g_lcd_left, polluting GPIO cache
 *    - lcdtest_chunk() used g_active_pins != &g_lcd_left as guard (static
 *      initial value equals it → condition always false → init skipped)
 *    - lcd_hw_spi_usable() checked before lcd_setup_pins() ran
 *    - preview_init used lcd_set_pins to switch panel, but GPIO cache
 *      didn't follow
 *
 *    Fix: g_cached_pins + lcd_set_pins() now rebuilds the cache from the
 *    root, eliminating stale-guard bugs at the source.
 *
 * 2. Distinguish "level/index number" from "the physical quantity it
 *    represents".  They are NOT interchangeable.
 *
 *    This sprint hit the same mistake twice:
 *    - SPI_FIFO_DEPTH was set to 48 (the largest tx_fifo_int_level enum
 *      value), when the actual FIFO depth is 64 bytes.
 *    - safe_write was computed as SPI_FIFO_DEPTH - level (e.g. 64-2=62),
 *      subtracting the level INDEX (2) instead of the level's BYTE COUNT
 *      (32).  Correct: 64 - fifo_level_to_bytes(2) = 64 - 32 = 32.
 *
 *    Rule: when a register field encodes a physical quantity via a lookup
 *    table (level→bytes, enum→frequency, code→voltage), always convert
 *    through the table.  Never use the raw field value in arithmetic that
 *    expects the physical quantity.
 *
 ****************************************************************************/

/* Resolved: commit granularity for the contest submission
 *
 * The two large commits here (HW SPI trans_len fix, +1041 lines; and the
 * 15x throughput work, +1644 lines) each bundle a bug fix with the
 * diagnostic commands written to find it (spidiag, chunk, pat, still,
 * flat, clk, burst).  Splitting them into separate "fix" and "tool"
 * commits was considered and deliberately NOT done:
 *
 *   - Fix and diagnostic code are interleaved within the same functions,
 *     so a split means picking apart individual hunks by hand.
 *   - Estimated 1-3 hours with a real risk of introducing errors into
 *     code that is already tested on hardware.
 *   - The only benefit is reviewer ergonomics upstream; this is a contest
 *     submission repository, not an openvela upstream PR.
 *
 * If any of this is ever proposed to openvela upstream, do the split at
 * that point.  The split boundary would be:
 *   fix   - trans_len latching, pin cache desync, FIFO-depth correction,
 *           safe_write bug, multi-chunk transfer
 *   tool  - lcd_spidiag, lcdtest subcommands, measurement infrastructure
 */
