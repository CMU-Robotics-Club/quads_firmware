# STM32H723 FDCAN notes (clock + CubeMX parameters)

2026-10-22 · branch `fdcan-loopback`, commit `a6a8ae5` · status: **internal loopback 3/3 PASS on the Nucleo; not yet tested on a real bus**

Sources (not in this repo; kept in the club's shared `ref/` folder):
- RM = `STM32H723_reference_manual_RM0468r3.pdf` (Rev 3)
- UM2407 = Nucleo-H723ZG user manual
- ODrive docs, CAN Protocol page, v0.6.12

## Overview

```
                           ┌── FDCAN1 (PD0/PD1)   → CAN1, U3, J6
8 MHz HSE → PLL2Q 80 MHz ──┼── FDCAN2 (PB12/PB13) → CAN2, U6, J5
                           └── FDCAN3 (PF6/PF7)   → CAN3, U7, J7
MCP2518FD (SPI1) ─────────────────────────────────→ CAN4, U9, J4
```

All buses: classic CAN, 1 Mbps, sample point 87.5% (matches ODrive).

## 1. Kernel clock

### Limit

FDCAN kernel clock max (RM p.336, Table 56):

| Voltage scale | Max |
|---|---|
| VOS0 (ours, `PWR_REGULATOR_VOLTAGE_SCALE0`) | 125 MHz |
| VOS1 / VOS2 / VOS3 | 100 / 75 / 50 MHz |

The default source PLL1Q was 275 MHz. That is over the limit, so CubeMX showed it in red.

The RM line "at least 80 MHz" applies only to the clock calibration unit. We don't use it.

### Source mux (FDCANSEL)

| Value | Source |
|---|---|
| 0 | HSE |
| 1 | PLL1Q (also the CPU PLL, don't touch) |
| 2 | **PLL2Q ← chosen** |

### PLL2 setup

```
HSE 8 MHz ─÷M=4→ 2 MHz ─×N=160→ VCO 320 MHz ─┬─÷P=2 → 160 MHz (unused)
                                            ├─÷Q=4 → 80 MHz → FDCAN
                                            └─÷R=2 → 160 MHz (unused)
```

- **M** divides first. The PLL input must be in a fixed range; 2 MHz is in range 1 (`RCC_PLL2VCIRANGE_1`).
- **N** multiplies. VCO = the PLL's internal frequency, not an output (`RCC_PLL2VCOWIDE`).
- **P / Q / R** are three independent output dividers. Only Q is used.

### HSE feeds everything, including the CPU

PLL1/2/3 share one input source, and ours is HSE. So the CPU clock comes from the same 8 MHz (`main.c` `SystemClock_Config`):

```
HSE 8 MHz ─÷4→ 2 MHz ─×275→ 550 MHz ─┬─÷P=1 → 550 MHz → CPU (SYSCLK)
                                     └─÷Q=2 → 275 MHz (old FDCAN source, over limit)
```

- The PLLs share only the input. Their M/N/P/Q/R are independent, so changing PLL2 left the CPU at 550 MHz.
- At reset the CPU runs on the internal HSI (64 MHz). `SystemClock_Config()` starts HSE, locks PLL1, then switches SYSCLK to it.
- If HSE is wrong or unstable, both the CPU and the CAN bit rate go wrong (the CPU hangs waiting for HSE, the CAN baud drifts).

### What HSE is on the Nucleo

Not a crystal on the H723 side. The on-board ST-Link is itself an STM32 with its own crystal, and it drives an 8 MHz square wave out of its MCO pin into our OSC_IN (PH0) (UM2407: "MCO from ST-LINK (Default)… fixed at 8 MHz"). The H723's own crystal footprint (X3) is not fitted by default.

```
Nucleo-H723ZG
┌──────────────────────────────────────────────────────────────┐
│  ST-Link side                  │  Target side                │
│  ┌─────────┐   ┌──────────┐    │      ┌────────────────────┐ │
│  │ crystal │──▶│ ST-Link  │ MCO│ 8 MHz│ STM32H723          │ │
│  └─────────┘   │ (STM32)  │────┼─────▶│ PH0 / OSC_IN (HSE) │ │
│                └──────────┘    │square└────────────────────┘ │
│                                │wave   X3 crystal: not fitted│
└──────────────────────────────────────────────────────────────┘
```

| Setting | Internal oscillator | Use when OSC_IN has |
|---|---|---|
| `RCC_HSE_ON` | on, drives a crystal | a passive crystal |
| `RCC_HSE_BYPASS` | off, takes the clock as-is | an external clock signal (Nucleo default) |

Our code uses `HSE_ON`. Strictly the Nucleo wants BYPASS. It works in practice because the square wave is strong enough. On our PCB, choose by the schematic: crystal → ON, active oscillator → BYPASS.

### Why 80 MHz

Worked backwards from the bus requirements:

1. ODrive needs 1 Mbps with an 87.5% sample point.
2. The FDCAN has no stopwatch. It measures time only by counting kernel clock ticks. Each tick (with prescaler 1) is one time quantum (TQ).
   At 80 MHz it counts 80 ticks per µs, and 1 bit at 1 Mbps is 1 µs, so **one bit = 80 TQ**.

   ```
   1 bit = 1 µs
   ├────────────────────────────────────────────────────────────────────────────────┤
   │▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯▯│
    1                                                                              80
   each ▯ = one tick = 1 TQ = 12.5 ns
   ```
3. The sample point can only be "after tick N", never between ticks. So 87.5% of the TQ count must be a whole number:

   | Clock | TQ per bit | 87.5% falls at | Usable? |
   |---|---|---|---|
   | **80 MHz** | 80 | 70 | yes |
   | 40 MHz | 40 | 35 | yes |
   | 60 MHz | 60 | 52.5 | no: nearest is 52 (86.7%) or 53 (88.3%) |
   | 20 MHz | 20 | 17.5 | no (same reason the MCP on a 20 MHz crystal can't hit 87.5%) |

   Zoomed in near the sample point (│ = a tick boundary, the only places a read can happen):

   ```
   80 MHz   … │ 69 │ 70 │ 71 │ …
                       ↑
                 after TQ 70: 70/80 = 87.5%   ✓ lands on a boundary

   60 MHz   … │ 52 │ 53 │ 54 │ …
                   ↑  ↑  ↑
                   │  │  └ after TQ 53: 88.3%
                   │  └─── 87.5% = 52.5: middle of a tick, impossible  ✗
                   └────── after TQ 52: 86.7%
   ```

   87.5% = 7/8, so any TQ count that is a multiple of 8 works.
4. Must stay under the 125 MHz limit.

80 MHz is the most TQ (finest resolution) under the limit that PLL2 makes easily. 40 MHz would also work.
A higher clock isn't "better" beyond this: the bus is fixed at 1 Mbps, and the clock only needs to divide into it exactly.

## 2. Pins

| Controller | Pins | AF | Nucleo header |
|---|---|---|---|
| FDCAN1 | PD0 RX / PD1 TX | AF9 | CN9-25 / CN9-27 (labelled CAN_RX/TX) |
| FDCAN2 | PB12 RX / PB13 TX | AF9 | CN7-7 / — |
| FDCAN3 | PF6 RX / PF7 TX | AF2 | CN9-11 / CN9-26 |

- Chosen to match the PCB (quads_PCBs `update-architecture-doc`, 0c355be).
- PB13 is also wired to the Ethernet PHY RMII_TXD1 through JP6. That is a PHY input, so it's harmless.
- Internal loopback doesn't need any wires.

## 3. Per-instance parameters (same for all three except offset)

### Frame and mode

| Parameter | Value | Why |
|---|---|---|
| Frame Format | Classic | ODrive uses classic CAN, max 8 data bytes |
| Mode | Normal | Real bus mode. The loopback test switches to `FDCAN_MODE_INTERNAL_LOOPBACK` at runtime, so regenerating CubeMX won't erase the test |
| Auto Retransmission | Enable | Standard CAN: resend if no ACK |
| Transmit Pause / Protocol Exception | Disable | Defaults |

### Bit timing (1 Mbps, 87.5%)

| Parameter | Value |
|---|---|
| Nominal Prescaler | 1 |
| Nominal Time Seg1 | 69 |
| Nominal Time Seg2 | 10 |
| Nominal SJW | 10 |

```
80 MHz ÷ prescaler 1 → 1 TQ = 12.5 ns
1 bit = Sync 1 + Seg1 69 + Seg2 10 = 80 TQ = 1 µs → 1 Mbps
read once at the end of TQ 70 → (1 + 69) / 80 = 87.5%
on each edge, shift by at most 10 TQ (SJW)
```

**Prescaler = how fine the ruler is.** It sets how many kernel clock cycles make one time quantum (TQ), the unit everything else is counted in. Baud = clock ÷ prescaler ÷ TQ per bit.

| Prescaler | 1 TQ | TQ per bit at 1 Mbps |
|---|---|---|
| **1 (ours)** | 12.5 ns | 80 |
| 2 | 25 ns | 40 |
| 4 | 50 ns | 20 |

More TQ per bit means the sample point can be placed more precisely. With 80, 87.5% lands exactly on TQ 70 (see "Why 80 MHz" above).

- It is an integer divider (1, 2, 3, …), divide only. 1 = no division: 1 TQ = one kernel clock period.
- PLL vs prescaler: the PLL (in RCC) multiplies and divides to make 80 MHz; the prescaler (inside FDCAN) can only divide that further.
- Register value = prescaler − 1. CubeMX/HAL take the real value and subtract 1 (`stm32h7xx_hal_fdcan.c:534`, `NominalPrescaler - 1U`). Matters only when reading `NBTP.NBRP` in a debugger: 0 there means ÷1.

**Segments = where in the bit we read.**

```
|Sync|──────────── Seg1 = 69 ────────────|── Seg2 = 10 ──|
  1                                       ↑
                              read here, once (sample point)
```

| Segment | Job |
|---|---|
| Sync (always 1 TQ) | Start of the bit. A level change is expected to land here |
| Seg1 | Wait for the line to settle: propagation delay, ringing |
| Seg1/Seg2 boundary | The single read that decides 0 or 1 |
| Seg2 | Time left after the read: processing, and room to resync (below) |

Seg2 is not "where reading happens". The read is the instant Seg1 ends.

**Why read late (87.5%)?**

```
level
  1 ‾‾‾‾‾‾╲    ╱╲
           ╲__╱  ╲_╱╲______________________________________   ← next bit
  0
         │S│ ringing:    │        stable              ↑│ Seg2 │
         0%  early read             safe to read     87.5%  100%
             may be wrong
```

- Right after an edge the line still rings and reflects. Later = more stable.
- Arbitration and ACK need every node to see the same bit within one bit time, so Seg1 must cover the round-trip down the cable.
- But not too late: Seg2 is the resync margin. Too small and clock drift between nodes causes misreads.

**The real reason for 87.5% here: match the ODrive.** ODrive's timing is fixed at 8 TQ = 1 + 6 + 1, i.e. 87.5%, not configurable. Every node resyncs on the same edges; if sample points differ a lot (e.g. 80% vs 87.5%), an edge with jitter can be read as 0 by one node and 1 by another. Short cables may hide this; longer cables or more nodes show stray error frames.
(CiA publishes recommended sample points per bit rate; not checked here, so not cited.)

The MCP2518FD is still at 80% (`0x001E0707`). Change it to `0x00210404` before the real bus.

**SJW = the largest timing correction per edge.** It is clock alignment, not error correction.

*Who sends, who receives.* CAN is a shared broadcast bus. Any node may transmit; at any moment the one transmitting is the sender, and **every other node is a receiver**. There is no master clock line: receivers time each bit by their own clock and align to the sender's edges.

```
                     CAN_H / CAN_L (one shared pair)
      ═══════╤══════════════════╤══════════════════╤═══════
             │                  │                  │
        ┌────┴────┐        ┌────┴────┐        ┌────┴────┐
        │ STM32   │        │ ODrive  │        │ ODrive  │
        │ FDCAN1  │        │ node 1  │        │ node 2  │
        └─────────┘        └─────────┘        └─────────┘

 Command:   STM32 sends Set_Input_Vel  → node 1 and node 2 both receive
                                          (node 2 ignores it: ID not its own)
 Heartbeat: node 1 sends              → STM32 and node 2 both receive
 ACK:       every receiver that got the frame right drives one ACK bit
            dominant; that is how the sender knows someone heard it
```

So the role changes frame by frame. Whoever is receiving is the one that resyncs to the sender's edges.

*Why the receiver drifts.* It assumes each bit is 80 of its own TQ. The sender's clock is slightly different, so the error builds up bit after bit:

```
sender's bits:    |←──── 1 µs ────→|←──── 1 µs ────→|←──── 1 µs ────→|
receiver counts:  |←── 80 TQ ──→|←── 80 TQ ──→|←── 80 TQ ──→|     (its clock a bit fast)
                                ↑ further off every bit until an edge lets it re-align
```

The only moment the receiver can "see" the sender's timing is an edge (a level change on the bus).

*Hard sync, once per frame.* The first edge of a frame (Start Of Frame) resets the bit counter to Sync outright, however far off it was. No SJW limit. Every frame starts perfectly aligned.

*Resync, on later edges.* On each recessive→dominant edge inside the frame, the receiver checks where it landed:

| Edge lands in | Meaning | Correction |
|---|---|---|
| Sync | already aligned | **none** |
| Seg1 (late) | sender's clock slower | lengthen Seg1 of this bit, sample point moves later |
| previous Seg2 (early) | sender's clock faster | shorten Seg2, next bit starts sooner |

Only the current bit changes length; the next bit is back to 80 TQ until the next edge is checked.

```
Edge ON TIME → nothing to do
  planned:  │S│────── Seg1 69 ──────↑── Seg2 10 ──│
  edge:      ↓

Edge LATE by 3 TQ (sender's clock slower) → stretch this bit
  planned:  │S│────── Seg1 69 ──────↑── Seg2 10 ──│
  edge:        ···↓ (3 TQ late)
  fixed:    │S│────── Seg1 69+3 ───────↑── Seg2 10 ──│   read moves 3 TQ later

Edge EARLY by 3 TQ (sender's clock faster) → shrink this bit
  planned:  ── Seg1 ──↑── Seg2 10 ──│S│ next bit
  edge:                      ↓ (3 TQ early)
  fixed:    ── Seg1 ──↑─ Seg2 10−3 ─│S│ next bit starts 3 TQ sooner

Either way the shift is capped at SJW (10 TQ).
```

SJW caps that shift (ours: 10 TQ = 125 ns).

Bigger is not always better:
- **Hard limit:** SJW ≤ min(Seg1, Seg2), since Seg2 can't shrink below 0. Ours is already at the max (Seg2 = 10). Raising it means raising Seg2, which moves the sample point earlier than 87.5% and breaks the match with ODrive.
- **Trade-off:** the controller can't tell a real edge from a noise glitch. Larger SJW tolerates more clock error, but one glitch can drag the sample point further into the unstable zone.
- **Ours matches ODrive in time:** ODrive 1 of 8 TQ, ours 10 of 80 TQ, both 12.5% of a bit = 125 ns. Plenty for crystal error of tens of ppm.

*Edges are guaranteed.* Bit stuffing: after 5 identical bits the sender inserts one opposite bit (receivers remove it). So there is an edge at least every 6 bits. Crystal error is small (typically tens of ppm; check the part's datasheet), so drift over 6 bits is far below 1 TQ, and SJW = 10 is a big margin.

*If it still goes wrong.* SJW can't fix a bit already misread. That is handled by error detection:

```
drift too large / bad wiring / wrong bit timing
        │
        ▼
a bit is misread ──▶ CRC (or stuff/form/ACK check) fails
        │
        ▼
the node that noticed sends an ERROR FRAME ──▶ every node discards the frame
        │
        ▼
sender retransmits automatically (Auto Retransmission = Enable)
        │
        ▼  if errors keep happening, error counters climb (TEC = transmit, REC = receive)
Error Active ──▶ Error Passive ──▶ Bus-Off (TEC > 255)
(normal)         (quieter errors)   node leaves the bus; software must restart it
```

In practice the cause is rarely the clock. It is usually a bit-timing mismatch (e.g. us 80% vs ODrive 87.5%), no termination resistors, long or loose wiring. SJW can't save those; they show up as constant error frames or Bus-Off. Real-bus code should read the error counters and Bus-Off status.

Think of musicians counting beats: the drummer is the sender; on each drum hit the others nudge toward it, but by at most SJW at a time, and the first hit of each song (hard sync) resets everyone.

Data-phase settings (Data Prescaler, Data Seg1/2) only matter for CAN FD bit rate switching. Ignored in Classic.

### Message RAM

All three FDCANs share one 10 KB Message RAM (2560 words of 32 bits). Each instance gets its own region via `MessageRAMOffset` (in words).

| Element | Count | Size each | Words |
|---|---|---|---|
| Std ID filter | 1 | 1 word | 1 |
| RX FIFO0 element | 8 | 2-word header + 8 B data = 4 words | 32 |
| TX FIFO element | 8 | 4 words | 32 |
| **Total** | | | **65** |

| Instance | MessageRAMOffset | Uses words |
|---|---|---|
| FDCAN1 | 0 | 0–64 |
| FDCAN2 | 128 | 128–192 |
| FDCAN3 | 256 | 256–320 |

```
Message RAM (words, 1 word = 4 bytes)
0        65       128      193      256      321                         2560
├─FDCAN1─┤  free  ├─FDCAN2─┤  free  ├─FDCAN3─┤           free              ┤

inside each 65-word block:
├F┤──── RX FIFO0: 8 × 4 ────┤──── TX FIFO: 8 × 4 ────┤
 1           32                        32
F = std filter
```

128-word spacing leaves room to grow (e.g. more filters or a bigger FIFO) without overlap.
If you change element counts, recompute so the regions don't overlap.

| Parameter | Value | Why |
|---|---|---|
| Std Filters Nbr | 1 | One filter, set to accept all in the test |
| Ext Filters Nbr | 0 | ODrive uses 11-bit standard IDs only |
| Rx Fifo0 Elmts Nbr | 8 | Buffer up to 8 received frames |
| Rx Fifo0 Elmt Size | 8 bytes | Classic CAN |
| Rx Fifo1 / Rx Buffers | 0 | Not needed yet |
| Tx Events Nbr | 0 | No TX timestamp log needed |
| Tx Buffers Nbr | 0 | Use the FIFO only |
| Tx Fifo Queue Elmts Nbr | 8 | Queue up to 8 frames |
| Tx Fifo Queue Mode | FIFO | Send in the order queued (Queue mode would send lowest ID first) |
| Tx Elmt Size | 8 bytes | Classic CAN |

## 4. Loopback test (`Core/Src/fdcan_test.c`)

Runs once from `StartDefaultTask`, results over RTT. Per instance:

1. Re-run `HAL_FDCAN_Init` with `FDCAN_MODE_INTERNAL_LOOPBACK`: the controller receives its own frames inside the chip. TX pin stays recessive, RX pin ignored, so no transceiver or wiring.
2. Filter 0 = mask 0 (accept every standard ID into RX FIFO0). Global filter set to **reject** non-matching frames, so a frame only arrives if filter 0 matched it. (The reset default accepts non-matching frames into FIFO0 and would hide a broken filter.)
3. Start, send ID `0x123` with data `01..08`, poll FIFO0 for up to 10 ms, compare ID, DLC and data.
4. Restore: Stop (only if Start succeeded; `HAL_FDCAN_Stop` errors on a controller that never started), Init back to Normal, global filter back to the reset default (Init doesn't touch `GFC`). A failed restore counts as FAIL.

| A PASS proves | A PASS does not prove |
|---|---|
| Kernel clock on, Init/Start work | Pins, transceivers, wiring |
| Filter 0 matches | Bit timing against another node |
| TX → RX path through this instance's Message RAM | Message RAM regions don't overlap (instances run one at a time, each Init clears its own region) |

## 5. Generated code to check after every CubeMX regenerate

- `main.c` `PeriphCommonClock_Config`: `PLL2M=4, PLL2N=160, PLL2Q=4`, `FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2`
- `fdcan.c`: prescaler / Seg1 / Seg2 / SJW / offsets as above
- `stm32h7xx_hal_conf.h`: `HAL_FDCAN_MODULE_ENABLED`
- `cmake/stm32cubemx/CMakeLists.txt`: CubeMX tends to drop the `\r` on the `stm32h7xx_hal_exti.c` line (files are CRLF). Fix it, or git shows a fake change.

## To confirm

- [x] FDCAN internal loopback passes on the Nucleo (3/3 PASS, 2026-10-22)
- [ ] HSE_ON vs HSE_BYPASS on the Nucleo and on our PCB
- [ ] Motor split across CAN1–4 (Advaith, TBD)
- [ ] Bus and transceivers (ADM3057) work at 87.5% on the real bus
