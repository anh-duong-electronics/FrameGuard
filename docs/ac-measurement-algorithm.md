# AC Measurement and the Goertzel Algorithm

A short description of how the device measures AC leakage voltage. Source code: `Core/Src/ads1015.c`.

## Measurement chain

```
AC leakage voltage
    │
    ▼
ZMPT107 (voltage transformer)  ← step-down, isolation
    │
    ▼
ADS1015 (ADC, I2C)             ← differential measurement, digitising
    │
    ▼
STM32 (software)               ← RMS, extract the 50 Hz component
    │
    ▼
Calibration table              ← convert to the actual voltage
    │
    ▼
LCD / LED / RS-485
```

The analog front end only steps down and digitises. All frequency filtering and RMS computation is done in
software, so no analog filter is needed, and moving to a 60 Hz grid only means changing one constant.

## Sampling

Each channel is sampled continuously over a short window containing several whole mains cycles.
The mean of the samples (the DC component, from ADC offset) is subtracted before processing.

## Two ways to compute RMS

| Value | Computed by | Meaning |
|-------|-------------|---------|
| Full-band RMS | square root of the mean of squared samples | 50 Hz + harmonics + noise, like a true-RMS meter |
| 50 Hz RMS | Goertzel algorithm | Only the component at the mains frequency |

The device reports leakage using **full-band RMS**. Real leakage voltages (through EMI filter capacitors,
inverters) are often distorted and rich in harmonics; using only 50 Hz would read lower than a true-RMS meter.
The 50 Hz RMS is still computed for reference.

## The Goertzel algorithm

Goertzel answers the question: "how large is the component at **one specific frequency** in this signal?"
without computing the whole spectrum like an FFT.

Idea: compare the signal with a reference 50 Hz sine wave. The 50 Hz component is in step with it and
accumulates into a large value; harmonics and noise are out of step and cancel themselves out.

Goertzel does this with a recursive loop using only a few state variables and no sine table, which suits
a RAM-constrained microcontroller such as the Cortex-M0+. The target frequency is recomputed from the actual
sample rate of each measurement, so the result does not drift when the I2C speed varies.

## Calibration

The ZMPT107 response is not linear at low voltage, so the firmware converts readings through a **multi-point
calibration table** (piecewise-linear interpolation) rather than a single scale factor.

Small differences between transformers are compensated by a **per-channel factor K**, stored in flash.
Set K by applying a reference voltage to the channels and sending `v=<mV>` (or `k1=`…`k4=`) over UART2;
the firmware computes and saves it.
