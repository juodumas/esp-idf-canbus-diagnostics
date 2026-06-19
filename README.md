# canbus-diag

Minimal pure-ESP-IDF TWAI diagnostic tool for the ESP32-C3, built with
PlatformIO. Used to hunt down the

```
[canbus:054]: send to standard id=0x00f failed with error 2!
```

failure on a 2-node CAN bus where the CAN LED stays on and no ACK seems to
come back.

Pins (taken from `worm-relay.yaml`; this is an independent app, none of the
worm-relay code is reused):

| Signal | GPIO |
|--------|------|
| CAN TX | 21   |
| CAN RX | 20   |

Default bitrate on boot: **125 kbps** (matches the YAML). No frame is sent
until you press `s`.

## Build / flash / monitor

```sh
cd canbus-diag
pio run -t upload
pio device monitor
```

Console runs over USB-Serial-JTAG (single USB cable, see `sdkconfig.defaults`).

## CLI

| Key | Action |
|-----|--------|
| `b` | cycle bitrate  125 -> 250 -> 500 -> 1000 kbps (re-creates the node) |
| `m` | cycle mode     normal -> listen-only -> loopback -> self-test (re-creates) |
| `s` | send one frame: id=0x00F, data=AB CD |
| `i` | print status snapshot (error state, TEC, REC, bus_err_num) |
| `?` | help |

All TWAI events arrive via ISR callbacks and are printed by a monitor task,
e.g.:

```
E (1234) cantest: CB TX done: FAILED (no ACK / bus-off)
E (1234) cantest: CB error flags=0x10
E (1234) cantest:   ack_err   -> NO ACK (other node didn't hear/ACK)
```

## How to use it to find the fault

Flash this onto **both** nodes. After boot both should print
`state=ACTIVE` and (with `i`) `TEC=0 REC=0`.

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

The `on_error` callback surfaces the actual cause ESPHome's `esp32_can`
collapses into "error 2":

| flag        | meaning |
|-------------|---------|
| `ack_err`   | no other node acknowledged the frame |
| `bit_err`   | TX line monitored value != what was sent (stuck-dominant, TX/RX swap, dead transceiver) |
| `form_err`  | fixed-form bits corrupted on the wire |
| `stuff_err` | bit-stuffing rule violated on the wire |
| `arb_lost`  | lost arbitration (benign on a busy bus) |

`fail_retry_cnt` is set to 0, so a no-ACK reports `TX done: FAILED`
immediately instead of retransmitting forever — that infinite retransmit is
exactly what pins your CAN LED on under ESPHome.

## Why not just keep using ESPHome's esp32_can

ESPHome's `esp32_can` wraps the legacy `driver/twai.h` and only logs
`send ... failed with error N`. It discards the alert/flag detail that tells
ack_err from bit_err from bus-off, and it retransmits forever so a single
wiring fault looks like a constant error storm with the LED stuck on. This
tool gives you the raw flags and counters so you can tell them apart in one
key press.
