# FPGA Controller for Crazyflie - Interrupt-Driven

This application implements an out-of-tree controller that communicates with an external FPGA over SPI. The communication is **fully asynchronous and non-blocking** - the control loop never waits for the FPGA.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           CRAZYFLIE (STM32)                             │
│                                                                         │
│  ┌─────────────────┐                      ┌─────────────────────────┐  │
│  │ Control Loop    │                      │ FPGA SPI Task           │  │
│  │ (1kHz)          │                      │ (Highest Priority)      │  │
│  │                 │                      │                         │  │
│  │ 1. Update state │──write──▶ [State    │                         │  │
│  │    buffer       │           Buffer]   │                         │  │
│  │    (~1 µs)      │                      │                         │  │
│  │                 │                      │  On EXTI5 interrupt:    │  │
│  │ 2. Read control │◀──read─── [Control  │  - SPI exchange         │  │
│  │    buffer       │           Buffer]◀──│  - ~20 µs total         │  │
│  │    (~0.1 µs)    │                      │                         │  │
│  │                 │                      │                         │  │
│  │ 3. Apply motors │                      │                         │  │
│  └─────────────────┘                      └────────────▲────────────┘  │
│                                                        │               │
└────────────────────────────────────────────────────────┼───────────────┘
                                                         │ IRQ (IO2)
                                            ┌────────────┴────────────┐
                                            │         FPGA            │
                                            │                         │
                                            │  1. Receive state       │
                                            │  2. Compute control     │
                                            │  3. Assert IRQ when done│
                                            │  4. Output motor cmds   │
                                            └─────────────────────────┘
```

## Timing

- **Control loop overhead**: ~1 µs (just buffer reads/writes)
- **SPI transaction**: ~20 µs at 21 MHz (48 bytes total)
- **Latency**: 1 control cycle (motor commands reflect previous state)

The control loop **never blocks**. The SPI transfer happens asynchronously in a high-priority task triggered by the FPGA interrupt.

## Hardware Setup

### Pin Connections

| Signal | Crazyflie Pin | STM32 Pin | Description |
|--------|---------------|-----------|-------------|
| CS     | IO1           | PB8       | Chip Select (directly controlled) |
| IRQ    | IO2           | PB5       | FPGA "done" signal (rising edge interrupt) |
| SCK    | SCK           | PA5       | SPI Clock |
| MOSI   | MOSI          | PA7       | Master Out, Slave In |
| MISO   | MISO          | PA6       | Master In, Slave Out |
| GND    | GND           | GND       | Ground |

### FPGA Requirements

Your FPGA must:
1. **Receive state** when CS goes low (36 bytes, full-duplex with result output)
2. **Compute control** based on received state
3. **Assert IRQ high** when computation is complete
4. **Output results** during the next SPI transaction (while receiving new state)

### Recommended FPGA State Machine

```
                    ┌──────────┐
                    │   IDLE   │◀─────────────────┐
                    └────┬─────┘                  │
                         │ CS low                 │
                         ▼                        │
                    ┌──────────┐                  │
                    │ RECEIVE  │ (receive state,  │
                    │ + SEND   │  send last result)│
                    └────┬─────┘                  │
                         │ CS high                │
                         ▼                        │
                    ┌──────────┐                  │
                    │ COMPUTE  │                  │
                    └────┬─────┘                  │
                         │ done                   │
                         ▼                        │
                    ┌──────────┐                  │
                    │  READY   │──────────────────┘
                    │ (IRQ=1)  │
                    └──────────┘
```

## SPI Protocol

### SPI Configuration
- Mode: SPI Mode 0 (CPOL=0, CPHA=0)
- Speed: 21 MHz (adjustable in `fpga_spi.h`)
- Bit Order: MSB First

### Full-Duplex Transaction

During each SPI transaction, data flows **both directions simultaneously**:

| Direction | Bytes | Content |
|-----------|-------|---------|
| MOSI (CF→FPGA) | 36 | New state data (12 × 24-bit) |
| MISO (FPGA→CF) | 36 | Previous motor commands (only first 12 bytes used) |

**Note**: The result is only 12 bytes (4 motors), but we clock 36 bytes for the state. Your FPGA can output zeros after the motor data, or you can pad/repeat.

### State Data Layout (MOSI)

| Index | Value | Unit | Byte Offset |
|-------|-------|------|-------------|
| 0 | x | m | 0-2 |
| 1 | y | m | 3-5 |
| 2 | z | m | 6-8 |
| 3 | vx | m/s | 9-11 |
| 4 | vy | m/s | 12-14 |
| 5 | vz | m/s | 15-17 |
| 6 | roll | deg | 18-20 |
| 7 | pitch | deg | 21-23 |
| 8 | yaw | deg | 24-26 |
| 9 | p | deg/s | 27-29 |
| 10 | q | deg/s | 30-32 |
| 11 | r | deg/s | 33-35 |

Each value is 24-bit Q16.8 fixed-point, transmitted **LSB first**.

### Motor Data Layout (MISO)

| Index | Value | Byte Offset |
|-------|-------|-------------|
| 0 | motor1 | 0-2 |
| 1 | motor2 | 3-5 |
| 2 | motor3 | 6-8 |
| 3 | motor4 | 9-11 |

Each value is 24-bit Q16.8 fixed-point (0.0 to 1.0 range = 0 to 256), transmitted **MSB first**.

### Motor Layout

```
     Front
   M1     M2
     \   /
      \ /
      / \
     /   \
   M4     M3
     Back
```

## Fixed-Point Format (Q16.8)

### Conversion (FPGA side)

**Float to Q16.8:**
```verilog
// For value in range -32768 to +32767.996
wire signed [23:0] fixed = $signed(float_value * 256.0);
```

**Q16.8 to Float:**
```verilog
// For motor values (0.0 to 1.0)
wire [31:0] float_value = fixed / 256.0;
```

### Examples

| Float Value | Q16.8 (decimal) | Q16.8 (hex) |
|-------------|-----------------|-------------|
| 0.0 | 0 | 0x000000 |
| 0.5 | 128 | 0x000080 |
| 1.0 | 256 | 0x000100 |
| -1.0 | -256 | 0xFFFF00 |
| 45.0 (degrees) | 11520 | 0x002D00 |

## Building and Flashing

```bash
cd examples/app_fpga_controller
make clean && make
make cload   # Flash via radio
```

## Runtime Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `fpga.enable` | uint8 | 1 | Enable FPGA controller (0=PID fallback) |
| `fpga.fallback` | uint8 | 1 | Use PID if no FPGA data |
| `fpga.motorMin` | float | 0.0 | Minimum motor clamp |
| `fpga.motorMax` | float | 1.0 | Maximum motor clamp |

### Log Variables

| Variable | Type | Description |
|----------|------|-------------|
| `fpga.init` | uint8 | FPGA initialized |
| `fpga.enabled` | uint8 | Controller enabled |
| `fpga.valid` | uint8 | Have valid motor data |
| `fpga.success` | uint32 | Successful transactions |
| `fpga.fail` | uint32 | Failed transactions |
| `fpga.irqCnt` | uint32 | Interrupt count |
| `fpga.txTimeUs` | uint32 | Avg transaction time (µs) |
| `fpga.txMaxUs` | uint32 | Max transaction time (µs) |
| `fpga.m1-m4` | float | Motor commands |

## Troubleshooting

### No Interrupts (`fpga.irqCnt` stays at 0)
- Check IO2 wiring to FPGA IRQ output
- Verify FPGA asserts IRQ as rising edge
- Check FPGA is powered and running

### Transactions Failing
- Check SPI wiring (SCK, MOSI, MISO, CS)
- Reduce SPI speed in `fpga_spi.h` if signal integrity issues
- Verify FPGA SPI mode matches (Mode 0)

### High Transaction Time
- Expected: ~20 µs at 21 MHz
- If higher: check for SPI bus contention with other decks

### Motors Not Responding
- Check `fpga.valid` is 1
- Check `fpga.m1` through `fpga.m4` show expected values
- Verify motor value range (0.0 to 1.0)

## Comparison: Blocking vs Interrupt-Driven

| Aspect | Blocking (old) | Interrupt-Driven (new) |
|--------|---------------|------------------------|
| Control loop time | 100-800 µs | ~1 µs |
| CPU usage | High (polling) | Low (event-driven) |
| Latency | 0 cycles | 1 cycle |
| Jitter | High | Low |
| Suitable for flight | Risky | Yes |

## License

Same license as Crazyflie firmware (GPL v3).
