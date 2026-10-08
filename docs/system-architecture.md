# System Architecture

## High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                      STM32C031K6T6 (LQFP32)                         │
│                                                                       │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │           ARM Cortex-M0+ Core (12 MHz SYSCLK)               │  │
│  │  Flash: 32 KB  |  RAM: 12 KB  |  Stack: 0x400  |  Heap: 0x200  │
│  └───────────────────────────────────────────────────────────────┘  │
│                                                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                   Peripheral Interfaces                     │   │
│  │                                                              │   │
│  │  I2C1 (100 kHz, PA9/PB9)      ─→  ADS1015 ADC @ 0x48       │   │
│  │                                    (4 diff channels, 12-bit)  │   │
│  │  SPI1 (750 kHz, PA1/PA2)      ─→  ST7567 LCD 128×64 pixels  │   │
│  │                                    (A0=PA0, /RST=PA4, CS=GND)│   │
│  │  USART1 (9600 8E1, RS-485)    ─→  Modbus RTU slave (DE: PA12)│  │
│  │  USART2 (115200, RX-only PA3) ─→  UART2 config commands     │   │
│  │                                                              │   │
│  │  GPIO (9 outputs)             ─→  4× LED pairs + D0         │   │
│  │                                                              │   │
│  │  WWDG (~128–175 ms window)    ─→  Watchdog (active, guarded)│   │
│  │                                                              │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Pinout Summary

### Power & Debug

| Pin | Type | Function | Notes |
|-----|------|----------|-------|
| VSS | GND | Ground (multiple pins) | - |
| VDD | +3.3V | Supply (multiple pins) | - |
| VDDA | +3.3V | Analog supply (ADC reference) | - |
| VREF+ | +3.3V | Internal reference (ADC) | - |
| PA13 | SWD | J-Link SWDIO (debug) | SWD @ 3.2 MHz confirmed working (HW v2, 2026-09-11). If attach fails, try 400 kHz. |
| PA14 | SWD | J-Link SWCLK (debug) | SWD @ 3.2 MHz confirmed working (HW v2, 2026-09-11). If attach fails, try 400 kHz. |

### Sensor & Measurement

| Pin | Port | Peripheral | Signal | Direction | Role | Notes |
|-----|------|-----------|--------|-----------|------|-------|
| PA9 | GPIOA | I2C1 | SCL | Bidirectional | ADS1015 ADC clock | 100 kHz, open-drain, 4.7 kΩ pull-up |
| PB9 | GPIOB | I2C1 | SDA | Bidirectional | ADS1015 ADC data | 100 kHz, open-drain, 4.7 kΩ pull-up |
| PA1 | GPIOA | SPI1 | SCLK | Output | ST7567 LCD clock | currently 750 kHz (lcd_init lowers the prescaler to /16) |
| PA2 | GPIOA | SPI1 | MOSI | Output | ST7567 LCD data | currently 750 kHz (lcd_init lowers the prescaler to /16) |
| PA0 | GPIOA | GPIO | A0 | Output | ST7567 LCD command/data | High=data, Low=command |
| PA4 | GPIOA | GPIO | LCD_RST | Output | ST7567 LCD hardware reset | lcd_init() configures PA4 once SPI is stable, hard-resets the LCD before the soft reset |

### Communication (RS-485)

| Pin | Port | Peripheral | Signal | Direction | Role |
|-----|------|-----------|--------|-----------|------|
| PC14 | GPIOC | USART1 | TX | Output | RS-485 transmit (via PA12 DE) |
| PB2 | GPIOB | USART1 | RX | Input | RS-485 receive |
| PA12 | GPIOA | USART1 | DE | Output | Driver Enable (high = TX, low = RX) |

### USART2 configuration (RX-only)

| Pin | Port | Peripheral | Signal | Direction | Role |
|-----|------|-----------|--------|-----------|------|
| PA3 | GPIOA | USART2 | RX | Input | Config commands (`id=`/`addr=`/`thr=`/`wire=`/`qty=`/`v=`/`kN=`/`config`), 115200 8N1, ISR ring. Feedback on LCD |

PA4 has been moved to GPIO for LCD /RST (see the Sensor & Measurement table).

### Status LEDs (4 green/red pairs, active-LOW, common anode)

| Pin | Port | Name | GPIO | Function | Status |
|-----|------|------|------|----------|--------|
| PB7 | GPIOB | L_GREEN1 | GPIO Output | Channel 1 green | Active |
| PB8 | GPIOB | L_RED1 | GPIO Output | Channel 1 red | Active |
| PB5 | GPIOB | L_GREEN2 | GPIO Output | Channel 2 green | Active |
| PB6 | GPIOB | L_RED2 | GPIO Output | Channel 2 red | Active |
| PB3 | GPIOB | L_GREEN3 | GPIO Output | Channel 3 green | Active |
| PB4 | GPIOB | L_RED3 | GPIO Output | Channel 3 red | Active |
| PA10 | GPIOA | L_GREEN4 | GPIO Output | Channel 4 green | Active |
| PA11 | GPIOA | L_RED4 | GPIO Output | Channel 4 red | Active |

Active-low LEDs (common anode), current-limited by external resistors. OK = green, leakage = red, wire broken = red heartbeat blink, ERR/INIT = off.

### Wire Check

Pins PA5, PA7, PB0, PB1 (ADC1), PA8 (TIM1_CH1) and PA15, PC6, PC15, PA6 (GPIO) are used by the
wire-break check. The implementation ships as a prebuilt library `Core/Lib/libwireprobe.a`;
see `Core/Inc/wire_probe.h` for the interface.

**TIM3**: free-running 1 µs timebase (SYSCLK 12 MHz, PSC=11, ARR=65535), replaces DWT→CYCCNT (not available on Cortex-M0+).

### Control Outputs

| Pin | Port | Name | GPIO | Notes |
|-----|------|------|------|-------|
| PA0 | GPIOA | D0 | GPIO Output | ST7567 LCD A0 (command/data select) |

---

## Clock Tree

### System Clock Configuration

```
HSI (High Speed Internal Oscillator)
  │
  ├─ Frequency: 48 MHz (default, calibrated)
  │
  └─ RCC_HSI_DIV4 (prescaler /4)
       │
       └─ SYSCLK = 12 MHz
            │
            ├─ HCLK (AHB) = 12 MHz  (no divider)
            │
            ├─ PCLK1 (APB1) = 12 MHz (no divider)
            │
            └─ Peripherals:
                 I2C1: 12 MHz kernel clock (APB1)
                 SPI1: 12 MHz kernel clock (APB1)  → CubeMX prescaler /2 (6 Mbit/s); lcd_init lowers to /16 → 750 kHz
                 USART1: 12 MHz kernel clock (APB1) → 9600 bps
                 USART2: 12 MHz kernel clock (APB1) → 115200 bps
                 WWDG: 12 MHz kernel clock (APB1)  → PCLK1 / 4096 / 8 ≈ 366 Hz (~2.73 ms/count)
```

### Flash Latency

**FLASH_LATENCY_0** (0 wait cycles) — sufficient for 12 MHz operation.

### Recommended Frequency

At 12 MHz:
- Power consumption: Very low (~2–5 mA active, depending on peripherals)
- Timing budget: Adequate for sensor polling and RS-485 messaging
- Headroom: Available for future CPU-intensive tasks (DSP, FFT)

---

## Peripheral Details

### I2C1 Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Mode | Standard (~100 kHz) | Timing: 0x00402D41 (optimized for 12 MHz) |
| Addressing | 7-bit | 8-bit addresses not used |
| Filters | Analog (enabled), Digital (0) | Noise filtering on SCL/SDA |
| Own Address | 0x00 | Slave mode not used (master only) |
| Pins | PA9 (SCL), PB9 (SDA) | Open-drain, pull-up required |

**Use case**: Read current-sensing transducers, ADCs, or other I2C sensors.

### SPI1 Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Mode | Master | Slave mode not used |
| Direction | 2-line (MOSI+MISO) | TX-only intent in .ioc, but direction set to 2-line (conservative) |
| Data Size | 8-bit | Single byte transactions |
| Clock Polarity | 0 (CPOL=0) | Clock idles low |
| Clock Phase | 0 (CPHA=0) | Sampling on first edge |
| NSS | Soft (GPIO controlled) | No hardware NSS; PA15 or other GPIO pin managed in code |
| Baud Rate | CubeMX: PCLK1/2 = 6 Mbit/s; runtime: lcd_init sets /16 = 750 kHz | Slowed down during LCD debugging |
| Bit Order | MSB First | Standard SPI |

**Use case**: ST7567 LCD 128×64 (write-only; A0=PA0, /RST=PA4, CS tied to GND).

### USART1 (RS-485) Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Baud Rate | 9600 bps | Industrial RS-485 |
| Data Bits | 8 | |
| Parity | **Even (8E1)** | Standard Modbus RTU |
| Stop Bits | 1 | |
| Mode | Asynchronous | Half-duplex, DE=PA12 |
| RS-485 Mode | Enabled | Hardware DEM, compatible with TX DMA |
| FIFO | Disabled | RX 1 byte RDR → ISR bare-register |
| Pins | PC14 (TX), PB2 (RX) | |

**Use case**: Modbus RTU slave (`modbus_rtu.c` + `mb_port_usart1.c`). FC03/04/06, map 1-based — [rs485-modbus-protocol.md](./rs485-modbus-protocol.md).

After `mb_init`: **no HAL UART TX API on USART1**. TX = DMA1 ch1 (DMAMUX req 51), polls TCIF. The RX ISR marks gaps ≥ 5 ms. NVIC USART1 priority 1.

### USART2 (Config Commands RX-only) Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Baud Rate | 115200 bps | Configuration command interface |
| Data Bits | 8 | Standard byte format |
| Parity | None | No error detection |
| Stop Bits | 1 | Standard frame format |
| Mode | Receive only | PA3 RX; PA4 GPIO freed for LCD /RST |
| FIFO | Disabled | CubeMX default |
| Pins | PA3 (RX) | RX-only, no TX |

**Use case**: RX-only technician commands (`id=`/`addr=`/`thr=`/`wire=`/`qty=`/`v=`/`kN=`/`config`). `USART2_IRQHandler` → 32-byte ring (NVIC priority 2, below Modbus). `poll_serial()` only drains the ring + parses. LCD feedback for 3 s. Details: [rs485-modbus-protocol.md](./rs485-modbus-protocol.md).

### Window Watchdog (WWDG) Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Prescaler | 8 (PCLK1 / 4096 / 8) | Watchdog clock = 12 MHz / 4096 / 8 ≈ 366 Hz (~2.73 ms/count) |
| Window | 80 (0x50) | Refresh allowed only when counter < 80 |
| Counter | 127 (0x7F) | Initial/reload value; reset fires when counter drops below 64 (0x40) |
| EWI (Early Wakeup) | Disabled | No interrupt before reset |
| Timeout | ~175 ms | (127 − 63) × 2.73 ms |
| Refresh window | ~131–175 ms after reload | Counter 79→64: window is open for ~44 ms |

**Critical caveats**:
- Code calls `app_wwdg_kick()` (main.c) everywhere: it only refreshes when the counter < Window and ignores early calls. Since the valid window is only ~44 ms wide, **two consecutive kick calls must be less than ~44 ms apart**, otherwise the first may be ignored (too early) and the next may already be past 175 ms → reset.
- Lesson learned: a blocking UART transmit of ~50 ms without kicking caused a reset on every measurement loop. Every long blocking operation (slow UART, LCD, delays) must be split into chunks with kicks in between; Modbus TX now uses background DMA.
- The RTT boot log prints `rst=... WWDG` when `RCC_FLAG_WWDGRST` is set, so watchdog resets are visible in the log.

---

## Memory Layout

### Flash (32 KB, 0x08000000–0x0800_7FFF)

```
0x08000000  ┌─────────────────────┐
            │  Vector Table       │  0x0C0 bytes (48 vectors)
            │  (.isr_vector)      │
0x080000C0  ├─────────────────────┤
            │                     │
            │  Code + Constants   │
            │  (.text)            │
            │  (CubeMX HAL ~28 KB)│
            │  (User code ~2–4 KB)│
            │                     │
0x0800_7FFF └─────────────────────┘  End of Flash
```

**Available for code**: 30 KB (page 15 = config_store). Production build (DEBUG=0) text+data **~28.1 KB**, RAM bss ~5.5 KB. DEBUG=1 (RTT, built with `-flto`) ~26.8 KB. Tight headroom (~1.9 KB in production).

### RAM (12 KB, 0x20000000–0x2000_2FFF)

```
0x20000000  ┌─────────────────────┐
            │  Initialized Data   │
            │  (.data)            │
            │  (HAL + User ~0.5 KB)
            ├─────────────────────┤  data_end
            │  Uninitialized Data │
            │  (.bss)             │
            │  (Global buffers, ~1 KB)
            ├─────────────────────┤  bss_end
            │  Heap (bottom-up)   │
            │  (Reserved 0x200 B) │  ← Growth direction
            ├─────────────────────┤  heap_base
            │  ...free...         │
            │                     │
            ├─────────────────────┤  stack_base
            │  Stack (top-down)   │
            │  (Reserved 0x400 B) │  ← Growth direction
0x2000_2FFF └─────────────────────┘  End of RAM (12 KB)
```

**Safe allocation**:
- `.bss` + `.data`: ~1.5 KB
- Stack: 0x400 bytes (1 KB)
- Heap: 0x200 bytes (0.5 KB)
- **Free for user buffers**: ~10 KB

---

## Interrupt Priority Model

All interrupts use **Cortex-M0+ NVIC** (16 priority levels, no sub-priorities in M0+).

### Default ISR Priorities (HAL configured)

| Handler | Vector | Priority | Type |
|---------|--------|----------|------|
| Non-maskable Interrupt (NMI) | -14 | Highest | Fault |
| Hard Fault | -13 | Very High | Fault |
| System Tick (SysTick) | 15 | 0 (default) | Periodic |
| USART1 | 6 | 1 | Modbus RX ISR (mb_port_usart1) |
| USART2 | 7 | 2 | Config RX ISR (ring, below Modbus) |
| I2C1 (Events + Errors) | 23–24 | 0 (default) | Peripheral |
| SPI1 | 25 | 0 (default) | Peripheral |

USART1 (priority 1) preempts USART2 (priority 2) so config commands never cause lost Modbus bytes.

---

## Power Considerations

### Operating Supply

- **VDD**: +3.3 V (single supply)
- **VDDA**: +3.3 V (same as VDD, can be shared with filter capacitor)
- **Decoupling**: 100 nF (typical for 32-pin QFP)

### Current Consumption (Estimate)

At 12 MHz, running all peripherals:

| Component | Current (mA) |
|-----------|--------------|
| Core (M0+, 12 MHz) | ~2–3 |
| I2C (idle, pull-ups) | ~0.1–0.5 |
| SPI1 (idle) | <0.1 |
| USART1 (idle) | <0.1 |
| USART2 (idle) | <0.1 |
| GPIO (all outputs low) | <0.1 |
| **Total (idle)** | **~2–4 mA** |

**Note**: Actual power depends on:
- Sensor power draw (external)
- LED brightness (if enabled)
- Bus activity (RS-485 driver)
- Flash accesses

---

## System Initialization Sequence

```
Power-on Reset
    ↓
startup_stm32c031xx.s (Reset Handler)
    ↓
SystemInit() [system_stm32c0xx.c]
    └─ Set flash latency
    └─ Configure clock (HSI → 12 MHz)
    └─ Set NVIC priority defaults
    ↓
main()
    └─ HAL_Init()
         └─ Configure SysTick (1 ms tick)
         └─ Set default NVIC priorities
    └─ SystemClock_Config()  [already done, safety check]
    └─ MX_GPIO_Init()
    └─ MX_I2C1_Init()
    └─ MX_SPI1_Init()
    └─ MX_USART1_UART_Init()
    └─ MX_USART2_UART_Init()  [RXNEIE, NVIC prio 2]
    └─ MX_ADC1_Init()  [ADC1 sequencer, SamplingTimeCommon1 79.5 cycles]
    └─ MX_TIM1_Init()  [TIM1 PWM at PA8 for wire check]
    └─ MX_TIM3_Init()  [TIM3 1 µs timebase]
    └─ MX_WWDG_Init()
    └─ APP_LOG_INIT()  [SEGGER RTT init if DEBUG=1]
    └─ config_load()  [layout v4: id/addr/qty/thr/wire/K]
    └─ probe_set_open_mvpp(g_cfg.wire_mvpp)
    └─ mb_init()  [Modbus RTU, no HAL TX on USART1]
    └─ ads_probe() × 2  [probe ADS1015 @ 0x48, 0x49]
    └─ probe_init()  [wire check]
    └─ lcd_init()  [SPI + PA4 hard reset]
    ↓
Main Loop (~1 Hz per group)
    └─ for each channel 0..3: if i ≥ ch_count → LED off, clear data, skip
    └─ probe_measure() + probe_update()
    └─ ads_measure_channel(); vac = calibration_table × K[i]
    └─ mb_poll() between channels (hooks in ads1015 + main)
    └─ lcd_show_channels()  [disabled channels blank; 5 s config page after the config command]
    └─ poll_serial()  [drain the USART2 ring]
    └─ app_wwdg_kick()
    ↓
(Continues at ~1 Hz; WWDG fires reset if not refreshed ~131–175 ms)
```

---

## Flashing & Debugging

### J-Link Programming with FLASH_ACR.EMPTY Workaround

**Issue**: When programming a blank STM32C031 chip via J-Link, the MCU may fail to boot firmware after reset, instead jumping to the system bootloader (PC = 0x1FFF0CDE). Cause: FLASH_ACR.EMPTY flag is latched at power-on when flash is blank; soft reset (SYSRESETREQ) does not reload option bytes, so every reset—including HAL_Delay softresets—stays in bootloader.

**Solution**: After loading firmware via `loadfile`, issue J-Link command `w4 0x40022000 0x00040600` to clear the EMPTY flag. This writes the correct FLASH_ACR value (bits EMPTY=0, PRFTEN=0, DUF=1) before disconnecting.

```bash
JLinkExe -nogui <<EOF
si 1
speed 3200
device STM32C031K6
connect
halt
loadfile build/FrameGuard.bin 0x08000000
w4 0x40022000 0x00040600    # ← Clear FLASH_ACR.EMPTY before reset
r
g
exit
EOF
```

**Note**: ST-Link and CubeProgrammer do not exhibit this issue because they use hardware reset (NRST) instead of SYSRESETREQ, which automatically handles the flag reload.

**SWD Speed**: `speed 3200` (3.2 MHz) is stable for flashing and RTT on this board; if attach fails, drop to 400 kHz.

---

## External Connections

### ADS1015 ADC (I2C @ 0x48)

```
STM32C031K6T6 (PA9=SCL, PB9=SDA)
         │
    [I2C @ 100 kHz]
         │
   ┌─────┴─────┐
   │ ADS1015   │
   │ (ADDR=GND)│
   │ 12-bit    │
   │ 4-channel │
   └─────┬─────┘
         │
    [AIN0–AIN3: 2 differential channels from ZMPT107 transformers; second IC @ 0x49 (ADDR=VDD)]
```

**Note**: 4.7 kΩ pull-ups on SCL/SDA to 3.3V required.

### ST7567 LCD (SPI, currently 750 kHz)

```
STM32C031K6T6
  ├─ PA1 (SCLK) ──────────┐
  ├─ PA2 (MOSI) ──────────┤
  ├─ PA0 (A0)   ──────────├─ ST7567 LCD
  ├─ PA4 (/RST) ──────────┤  128×64 pixels
  └─ CS (GND)   ──────────┤  COG12864-14-1
                          │
                [V0–XV0, VG–VSS:]
                 0.1–1 µF X7R caps
                 (booster supply)
```

**Hardware changes (HW v2)**:
- LCD /RST pin now connected to GPIO PA4
- `lcd_init()` configures PA4 as output and hard-resets the LCD (~10 µs pulse) **after SPI is stable** (SCK/MOSI at defined levels)
- Why not NRST: if the LCD powers up before the MCU it sees a floating (undefined) SCK → byte slip. A self-configured PA4 GPIO reset guarantees the timing.
- The firmware also sends a soft reset (0xE2 command) in the init routine

**Hardware note (resolved)**: LCD V0 supply (booster capacitor voltage) was collapsing due to undersized booster caps. Fixed with 0.1–1 µF X7R C0G capacitors at V0–XV0 and VG–VSS.

### RS-485 Network (USART1)

```
┌───────────────────────────────────────┐
│  Central Monitoring System            │
│  (RS-485 Master)                      │
└───────────────────┬───────────────────┘
                    │
        ┌───────────┴──────────┐
        │   Twisted Pair (A, B)│
        │   (120 Ω terminator) │
        └───────────┬──────────┘
                    │
     [PA12 DE]  [PC14 TX/PB2 RX]
         │           │
    ┌────┴───────────┘
    │
STM32C031K6T6 (RS-485 Slave)
```

USART1 is **in use**: Modbus RTU slave. The master polls FC03 addr 1 qty 17; recommended timeout 500 ms over USB-RS485.

---

**For register-level details, consult the [STM32C031 Datasheet](https://www.st.com/en/microcontrollers/stm32c031k6.html).**  
**For AC measurement algorithm (Goertzel, calib, RMS), see [AC Measurement Algorithm](./ac-measurement-algorithm.md).**
