# poom_ble_remote

Poom side of the hexapod remote. The app turns the Poom into a BLE peripheral
that streams its six buttons to the Pi 5, which runs
[`poom-remote`](../../../poom-remote) and translates them into a motion intent
for the hexapod.

Menu location: **THE MAKER → BLE REMOTE**.

## Protocol

The wire format is defined by the Pi side (`poom-remote/src/proto.rs`) and
mirrored in [`include/poom_ble_remote_proto.h`](include/poom_ble_remote_proto.h).

| UUID | Name | Direction | Payload |
|------|------|-----------|---------|
| `7917cbf0-fa7b-4d8e-8aa3-f1f8f2cd4052` | service | | |
| `7917cbf1-fa7b-4d8e-8aa3-f1f8f2cd4052` | `buttons` | Poom → Pi, notify | `version u8 = 1`, `seq u8`, `buttons u16 LE`, `battery u8` |
| `7917cbf2-fa7b-4d8e-8aa3-f1f8f2cd4052` | `status`  | Pi → Poom, write  | reserved (robot state), logged only |

Button bits: 0 up, 1 down, 2 left, 3 right, 4 A, 5 B. These are *not* the
`button_layout_t` numbers used by `button_driver`; `poom_ble_remote.c` maps
between the two.

A report goes out on every button change and as a heartbeat every 100 ms. The Pi
stops the robot if nothing arrives for 300 ms, so the heartbeat is a hard
requirement, not a convenience.

The Poom has no battery sensing, so the battery byte is always `0xFF`
("unknown"), which the Pi reports as no battery reading.

### Advertising

The 128-bit service UUID (18 bytes) and the complete local name `poom-hexapod`
(14 bytes) do not fit together in one 31-byte legacy advertisement. The UUID
goes in the advertisement, the name in the scan response. The Pi matches on
either, or on a pinned address.

No pairing or bonding: the Pi connects to the first device that matches.

## Controls

| Held | Effect |
|------|--------|
| any button | transmitted to the Pi; it decides what the robot does |
| A + B for 1 s | leaves the app and returns to the menu |

B on its own stays a protocol button (the Pi maps it to `toggle_stand`), which
is why exit is the A+B combo rather than the usual B press. On the way out the
app sends one final "nothing pressed" report so the robot stops immediately
instead of waiting out the Pi's heartbeat timeout.

## Screen and LEDs

```
┌─────────────────────┐
│     BLE REMOTE      │  inverted header
│ LINK: CONNECTED     │
│ 10:FE:ED:00:11:22   │  peer, or "AS poom-hexapod"
│ SEQ:143  NTF:ON     │  report counter, subscription state
│ BTN: U..R.B         │  live readout, U D L R A B
│   A+B HOLD: EXIT    │
└─────────────────────┘
```

The LED strip is taken over from the rainbow animation while the app runs and
handed back on exit: blinking amber while advertising, solid amber once
connected but before the Pi subscribes, green while streaming. The first LED
flashes white on each report that carries a button change (not on heartbeats).

## Structure

```text
applications/poom_ble_remote
├── CMakeLists.txt
├── component.mk
├── poom_ble_remote.c            app task, button state, screen, LEDs
├── poom_ble_remote_gatt.c       Bluedroid GATT peripheral
├── README.md
└── include/
    ├── poom_ble_remote.h        menu entrypoint
    ├── poom_ble_remote_gatt.h
    └── poom_ble_remote_proto.h  wire format
```

The app carries its own GATT server rather than using
`modules/poom_ble_gatt_server` (one characteristic, and it sends indications,
which cannot sustain a 100 ms heartbeat) or `modules/poom_ble_gatt_dynamic`
(several characteristics but no client characteristic configuration descriptor,
so no notifications at all). `poom_ble_hid` owns its profile for the same
reason.

Note for anyone reading the Pi side: its README says the firmware will use
NimBLE. The Poom is built against Bluedroid (`CONFIG_BT_BLUEDROID_ENABLED`), so
this app is Bluedroid. The difference is invisible over the air.

## Status

Not yet run against a real `poom-remote`. The encoder matches
`ButtonReport::parse` field for field and the advertising split matches what
`is_poom()` in `poom-remote/src/source/ble.rs` looks for, but the link has only
been checked on paper.
