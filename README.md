# CMSIS-DSP FIR Filtering on the B-L475E-IOT01A

Real-time FIR low-pass filtering of 3-axis accelerometer data on an STM32L475,
using the ARM CMSIS-DSP library. Built for the NTU Electrical Engineering Lab
(Embedded Systems) course, Lab 7.

The firmware does two things in sequence:

1. **Validates the filter against a known signal.** It runs the CMSIS-DSP
   reference input (a 1 kHz + 15 kHz sine sum, sampled at 48 kHz) through the
   filter and compares the result to a MATLAB-computed reference using
   `arm_snr_f32()`. This proves the DSP path is correct before any real sensor
   data is involved.
2. **Filters live accelerometer data.** It samples one axis of the on-board
   LSM6DSL at a timer-locked 100 Hz, then runs the same capture through four
   FIR low-pass filters with different cutoff frequencies (2 / 5 / 10 / 20 Hz)
   and dumps everything as CSV over the ST-LINK virtual COM port.

Because all four filters see the *identical* input, the only difference between
their outputs is the cutoff frequency — which makes the effect of the cutoff
directly measurable rather than merely plausible.

## Hardware and toolchain

| | |
|---|---|
| Board | ST B-L475E-IOT01A (STM32L475VGT6, Cortex-M4F @ 80 MHz) |
| Sensor | LSM6DSL 6-axis IMU, on I2C2 (PB10 = SCL, PB11 = SDA), address `0xD4` |
| Console | USART1 @ 115200 8N1, via the on-board ST-LINK VCP |
| Sample clock | TIM2, PSC = 79, ARR = 9999 → exactly 100 Hz |
| IDE | STM32CubeIDE 2.2.0 (arm-none-eabi-gcc 14.3.1) |
| Firmware pack | STM32Cube_FW_L4 V1.18.2 |

## Signal chain

```
LSM6DSL  --I2C2-->  TIM2 ISR sets flag  -->  main loop reads sample
   104 Hz ODR          100 Hz                 accRaw[512]  (5.12 s)
                                                    |
                                   arm_fir_f32(), 4x in parallel
                                   2 Hz / 5 Hz / 10 Hz / 20 Hz
                                                    |
                                           CSV over USART1
```

The TIM2 interrupt handler only sets a flag; the blocking I2C read happens in
the main loop. `HAL_I2C_Mem_Read()` uses `HAL_GetTick()` for its timeout, and
`HAL_GetTick()` is driven by the SysTick interrupt — calling it from a
higher-priority ISR can deadlock if the transfer ever needs to time out. A
dropped-sample counter is incremented whenever the flag is still set when the
next tick arrives, so a distorted time axis cannot go unnoticed.

## Filter design

29-tap linear-phase FIR, Hamming window, designed in Python:

```python
from scipy.signal import firwin
h = firwin(29, cutoff_hz, fs=100.0)    # DC gain = 1.0
```

29 taps give a transition width of roughly `3.3 / N * fs` ≈ 11 Hz and a group
delay of `(N - 1) / 2` = 14 samples = 140 ms, identical at every frequency
because the coefficients are symmetric. Every coefficient set sums to 1.0, so
the DC component (gravity) passes through untouched and only the AC component
is attenuated — this is visible in the output as four filtered columns that
share the same mean but have progressively smaller standard deviations.

The state buffer is sized `blockSize + numTaps - 1` = 60 floats per filter.
After `arm_fir_init_f32()` zeroes it, the first 28 outputs would otherwise be a
startup transient (the filter averaging real samples against zeros). The code
pre-loads the first `numTaps - 1` entries with the first captured sample so the
filter starts in steady state and no data has to be discarded.

## Repository layout

```
Core/Src/main.c       capture loop, FIR setup, CSV output, printf retarget
Core/Src/imu.c        LSM6DSL driver (direct register access over HAL I2C)
Core/Src/arm_fir_data.c   CMSIS-DSP reference input + MATLAB reference output
Core/Src/math_helper.c    arm_snr_f32() and friends, from CMSIS-DSP examples
Drivers/CMSIS_DSP/    CMSIS-DSP headers + the two FIR sources actually used
tools/analyze_acc.py  spectrum / band-energy / cutoff-comparison analysis
captures/             CSV captures referenced by the report
docs/                 figures
```

Only `arm_fir_f32.c` and `arm_fir_init_f32.c` are compiled from CMSIS-DSP, so
no prebuilt `.a` and no linker configuration are required.

## Building

1. Open STM32CubeIDE, `File > Open Projects from File System`, select this folder.
2. Build. No extra include paths or linker flags need to be set — the project
   file already contains `../Drivers/CMSIS_DSP/Include`.
3. Flash and open a serial terminal on the ST-LINK VCP at **115200 8N1**.
4. Press any key when prompted, then move the board during the 5.12 s capture.

## CMSIS-DSP integration notes

These cost real time, so they are worth writing down.

**The CMSIS-DSP shipped with STM32Cube_FW_L4 V1.18.2 is the pre-5.8 monolithic
version.** `Drivers/CMSIS/DSP/Include/` contains exactly three files:
`arm_math.h` (a single 264 KB header), `arm_common_tables.h` and
`arm_const_structs.h`. There is no `PrivateInclude/`, no `dsp/` subdirectory,
and no `arm_math_memory.h`.

Downloading `arm_math.h` from the CMSIS-DSP GitHub repository and dropping it
into the project therefore does not work: the modern header is a 2 KB umbrella
that includes `arm_math_types.h`, `arm_math_memory.h` and a dozen `dsp/*.h`
files that this pack does not contain. The failure mode is
`fatal error: arm_math_memory.h: No such file or directory`.

Worse, if that file is placed in `Drivers/CMSIS/Include/` it *shadows* the
correct one, because CubeIDE emits `-I../Drivers/CMSIS/Include` before any
include path you add yourself. Keep the DSP headers in their own directory.

**Each `Source/<Name>Functions/<Name>Functions.c` is a unity-build file** that
`#include`s every individual `.c` in the same folder — `FilteringFunctions.c`
has 99 of them. CubeIDE compiles every `.c` it finds, so adding the whole
`Source/` tree produces multiple-definition errors at link time. Either exclude
the unity files from the build or exclude the individual ones; do not keep both.

**`--specs=nano.specs` drops `%f` support from `printf`.** Rather than adding
`-u _printf_float` to the linker flags, floats are formatted by splitting them
into integer and fractional parts (`print_f32()` in `main.c`).

## Analysis

`tools/analyze_acc.py` takes a CSV capture and reports clipping, the dominant
shake frequency, the energy distribution across frequency bands, and the
standard-deviation reduction for each cutoff:

```
python tools/analyze_acc.py captures/shake.csv --plot
```

`--plot` additionally writes a time-domain overlay and a spectrum with the
cutoffs marked. Requires numpy, and matplotlib for the plots.

## Results

Known-signal validation (fill in from your own run):

```
SNR vs MATLAB reference = ___ dB   (threshold 140 dB)
```

A representative hand-shake capture, 512 samples at 100 Hz:

| | std (mg) | shake removed | DC mean (mg) |
|---|---|---|---|
| raw | 1629.5 | — | 373.7 |
| 2 Hz | 832.6 | 48.9 % | 372.6 |
| 5 Hz | 1179.6 | 27.6 % | 373.0 |
| 10 Hz | 1569.8 | 3.7 % | 373.5 |
| 20 Hz | 1628.1 | 0.1 % | 373.4 |

The DC mean is essentially unchanged across all four filters, confirming unity
DC gain. The attenuation figures are only meaningful relative to where the
signal's energy actually sits: in this capture the shake was centred at 3.7 Hz,
so a 10 or 20 Hz cutoff has almost nothing to remove. Comparing cutoffs without
also looking at the input spectrum invites the wrong conclusion.

## Known limitations

- The accelerometer runs at ±2 g full scale. Vigorous shaking clips at
  ±1998 mg, which distorts the waveform and injects odd harmonics. The firmware
  counts and reports clipped samples; a clean capture should report zero. For
  larger motion, change `CTRL1_XL` to `0x4C` (±8 g) in `imu.c` and scale by
  0.244 mg/LSB instead of 0.061.
- Capture is batch-mode: 512 samples are collected, then filtered, then
  printed. This avoids any interaction between the blocking UART output and the
  sampling loop, at the cost of not being a streaming implementation.
- Only the Z axis is captured (`ACC_AXIS` in `main.c`).

## License

Course work. CMSIS-DSP sources and ST HAL drivers retain their original
Apache-2.0 / ST licenses.
