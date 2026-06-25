# canbus-diag

Minimal pure-ESP-IDF TWAI diagnostic tool for the ESP32-C3, built with PlatformIO.

Default Pins (can be modified in `platformio.ini`:

| Signal   | GPIO |
|--------- |------|
| CAN TX   | 21   |
| CAN RX   | 20   |
| SEND BTN |  9   |

Default bitrate on boot: **1 mbps**.

No frame is sent until you press `s`, `S` when connected via serial, or the
`BTN_SEND` button (GPIO9, active-low / short-to-GND, internal pull-up enabled).

A short press of `BTN_SEND` sends one `s`-equivalent frame (edge-triggered,
debounced). Holding `BTN_SEND` for more than 500 ms enters throughput-flood
mode (id=0x7E0, 8-byte payload: little-endian 32-bit sequence counter followed
by the fixed pattern `55 AA 55 AA`); release the button to stop.

## Build / flash / monitor

```sh
cd canbus-diag
pio run -t upload
pio device monitor
```

## CLI

Console runs over USB-Serial-JTAG (single USB cable, see `sdkconfig.defaults`).

| Key | Action |
|-----|--------|
| `b` | cycle bitrate  125 -> 250 -> 500 -> 1000 kbps (re-creates the node) |
| `m` | cycle mode     normal -> listen-only -> loopback -> self-test (re-creates) |
| `s` | send one frame: id=0x00F, data=AB CD (DLC=2) |
| `S` | send one frame: id=0x00F, data= (DLC=0, empty data frame) |
| `t` | throughput test: flood id=0x7E0 (8-byte, seq+pattern) for 2s |
| `i` | print status snapshot (error state, TEC, REC, bus_err_num) |
| `?` | help |

All TWAI events arrive via ISR callbacks and are printed by a monitor task, e.g.:

```
E (1234) cantest: CB TX done: FAILED (no ACK / bus-off)
E (1234) cantest: CB error flags=0x10
E (1234) cantest:   ack_err   -> NO ACK (other node didn't hear/ACK)
```

## Throughput test

`id=0x7E0` frames are recognised by every node automatically — no handshake.
On receiving the first `0x7E0` frame a listener starts counting frames,
tracking sequence gaps (lost frames) and pattern mismatches (corrupt frames);
after 500 ms of silence it prints a one-line summary:

```
TP RX: frames=1234/1240 (0.5% lost/0 corrupt) time=2.10s rate=588fps throughput=76/1000kbps
```

This scales to N listeners with no extra coordination: each prints its own
summary, so cross-checking them reveals whether loss is link-specific or
bus-wide. Only one sender should flood at a time. 

## How to use it to find faults

Flash this onto all nodes. After boot both should print `state=ACTIVE` and (with `i`) `TEC=0 REC=0`.

1. **Self-test on one node, no bus needed.** Press `m` until it says
   `mode=self-test`, then `s`. You should get `CB TX done: SUCCESS` because
   the controller ACKs itself in loopback+self-test. If even this fails, the
   TWAI peripheral / GPIO config is broken on that board.
2. **Listen-only probe.** Put node A in `listen-only`, node B in `normal`.
   Press `s` on B. If A prints `CB RX id=0x00F` but B still gets
   `ack_err`, the wiring is fine but B gets no ACK because A is forbidden to
   ACK — this is the cleanest way to confirm the bus carries data.
3. **Both normal, one sends.** Press `s` on one. Expect `CB TX done: SUCCESS`
   on the sender and `CB RX id=0x00F` on the other. If you instead get
   `ack_err` / `bit_err`, the problem is electrical (transceiver, termination
   actually present at both physical ends, common ground, TX/RX swap, stuck
   transceiver holding the line dominant).

### Reading the error flags

| flag        | meaning |
|-------------|---------|
| `ack_err`   | no other node acknowledged the frame |
| `bit_err`   | TX line monitored value != what was sent (stuck-dominant, TX/RX swap, dead transceiver) |
| `form_err`  | fixed-form bits corrupted on the wire |
| `stuff_err` | bit-stuffing rule violated on the wire |
| `arb_lost`  | lost arbitration (benign on a busy bus) |

`fail_retry_cnt` is set to 0, so a no-ACK reports `TX done: FAILED` immediately instead of retransmitting forever.

