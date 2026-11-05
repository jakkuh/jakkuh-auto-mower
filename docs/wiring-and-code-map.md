# Auto-mower: wiring and code map

Status: draft, nothing here has been bench-tested. `TBD` items depend on photos of the mower electronics.
Design rule: parts on hand plus one ESP32, used only as the WiFi bridge.

## Architecture

```text
                      WiFi (MAVLink)
Mudi 7 <------------------------------- ESP32 <----- TELEM1 --+
  ^ cellular / Tailscale                                      |
  v                                                           |
Phone / laptop (Mission Planner)                              |
                                                              |
UM982 + 2 antennas ------------------------------- GPS2 ---> PIXHAWK 6C MINI
                                                              |  |  |
HOTRC SBUS-A -- SBUS ------------------------- RC IN ---------+  |  |
                                                                 |  |
KEYIUAV box <-- fore-aft, differential PWM, MAIN1, MAIN3 --------+  |
   --> speed + reverse lines --> 2 BLDC track controllers           |
                                                                    |
8-channel relay board <-- AUX1-2 (GPIO) ----------------------------+
   R1 tracks, R2 engine
```

## Wire map

Pixhawk connectors are JST-GH. Pin 1 is the red wire. All signals are 3.3 V unless noted.

### Power

| From | To | Notes |
|---|---|---|
| Key-switched 24 V, ahead of R1, fused | Holybro PM02 V3 XT60 input, then its 6-pin cable to POWER1 | Listed at 7-51 V input, 5.2 V 3 A output, so it covers 24 V and charging voltage (about 29 V). Leave its XT60 output unused, no motor current goes through it. The Pixhawk itself is 6 V max, so never put 24 V on it or its rails |
| Pixhawk MAIN/AUX rail | Nothing | Leave the servo rail unpowered. Only signal and GND are used |
| Mudi 7 | Internal battery | Charge between sessions, or use a 24 V-capable USB-C car charger if you have one (power-only port, 5-12 V PD) |
| Pixhawk TELEM2 pin 1 (5 V), pin 6 (GND) | Relay board VCC and GND | Keep the VCC/JD-VCC jumper on. Powering the board from the Pixhawk is deliberate, see "Relay board" |

All 5 V loads run from the PM02 V3 through Pixhawk ports, so no separate 5 V adapter is needed. Port limits: TELEM1 + GPS2 (ESP32, 982) share 1.5 A, and other ports including TELEM2 (relay board) share 1.5 A. Measure 982 current on the bench to confirm.

Tie all grounds together at one point.

### Signals

| Pixhawk port | Pin | Goes to | Purpose |
|---|---|---|---|
| GPS2 (SERIAL4) | 1 VCC, 2 TX, 3 RX, 6 GND | WTRTK-982DT main group: VCC, RXD, TXD, GND | Position and heading. Pixhawk TX to RXD, Pixhawk RX to TXD. It ran directly on an ESP32 in earlier tests, so it is not RS-232, but meter TXD idle to GND first: it should be about 3.3 V. Leave PPS unconnected, and EN as it was in that test |
| TELEM1 (SERIAL1) | 1 VCC, 2 TX, 3 RX, 6 GND | ESP32-S3 terminal board: 5V, GPIO41 (RX), GPIO42 (TX), GND | MAVLink to the Mudi over WiFi. Set these pins in DroneBridge. Avoid GPIO 0, 3, 19, 20, 43-46. Unplug USB when powered from TELEM1. The module appears to have a U.FL connector, so it needs an antenna fitted, routed outside the steel bay |
| RC IN | S, +, - | HOTRC SBUS-A: SBUS, 5 V, GND | Manual driving. Manual: DC 4-9 V, 35 mA, 1.25 mm plug. Confirm plug pin order and SBUS polarity. Do not wire its voltage feedback to 24 V, it reads 0-27 V max. Route the antenna away from the steel body and the 2.4 GHz WiFi gear |
| MAIN1 | S, - | Box fore-aft signal (yellow wire, confirmed by test), box GND | Do not connect `+`, the box has its own 5 V |
| MAIN3 | S, - | Box differential signal (white wire, confirmed by test) | Mixing confirmed by test |
| AUX1-2 | S, - | Relay board IN1 (blue wire), IN2 (purple wire), GND | User's colors. Label the IN1 wire, because the main relay coil wire is also blue |

Unplug the receiver from the box and label the connector so stock wiring can be restored.

### Drive box (from photos)

Label: KEYIUAV dual-channel speed controller, "PWM 1000-2000us Convert 5V-0-5V", input DC 15-80 V, BEC 5 V at 500 mA.

- **Inputs:** a 3-pin lead marked fore-aft (`前后`) and a 1-pin lead marked differential (`差速`), both 1-2 ms. So the box very likely does the mixing.
- **Outputs:** speed voltage and a reverse line for each track. This explains "5V-0-5V": speed is zero at center stick and rises in both directions, with the reverse line setting direction.
- **Power:** it feeds the receiver through its BEC today. Do not connect that 5 V to anything else on the Pixhawk side. Grounds still need to be common.
- **Terminals (7):** DC+, DC-, GND, speed L, speed R, reverse L, reverse R. Two trim pots set the speed voltage range. The cable from the receiver has 4 wires: 5 V, GND, and 2 signals.
- **Stock radio loss:** turning the transmitter off while rolling stops the mower (user test with the F-06A). This does not show whether the F-06A or the box causes the stop, and it says nothing about the SBUS-A.

**Bench test, tracks off the ground, with a meter on the box outputs and the stock radio:**
1. Fore-aft only: PASSED (user test). Both tracks turn the same direction at about the same speed.
2. Differential only: PASSED (user test). The tracks counter-rotate, so the mower does a tank turn. Check the scaling during Pixhawk bring-up, since the formula below assumes a simple mix.
3. Lost signal: PASSED (user test). With the tracks turning, unplugging the receiver cable stopped them, and turning the transmitter off with the F-06A did too. So the box stops by itself when pulses vanish, which covers Pixhawk power loss or a dead output. It does not cover a Pixhawk that freezes while still sending a steady signal.

**If the box mixes:** ArduPilot's skid-steer mode and pivot turns use left/right outputs (73/74). A Lua script on the Pixhawk reads them and converts, assuming a simple mix:

```text
fore-aft     = 1500 + ((L - 1500) + (R - 1500)) / 2
differential = 1500 + ((L - 1500) - (R - 1500)) / 2
```

The script sends these with a short timeout and also calls `set_output_scaled(94/95, 0)`. That call is required: without it, a dead script holds the last command forever. In simulation the outputs reached 1500 about 0.24 s after the script stopped. Correct the formula if motor tests show a different mix.

**Script failure test:** with tracks off the ground, stop the script while commanding motion. The box outputs must go to zero. If they do not, this design needs rethinking.

### Relay map

Everything must fail safe: a de-energized relay means stopped.

| Relay | Driven by | Contact wiring | De-energized result |
|---|---|---|---|
| R1 tracks | AUX1, pin 50 | NO, in series with the main relay coil. See "Track power cut" | Tracks off |
| R2 engine | AUX2, pin 51 | COM to kill wire, NC to GND | Engine killed |
| R3-R8 | Spare | | |

- Throttle and blade clutch are manual (user confirmed), so neither is on the board. Set throttle and engage the clutch by hand before arming.
- The blade can only be stopped by killing the engine, so R2 is the blade stop too. It will coast down after a kill.
- Do not switch track motor current through this board. Contacts are rated 10 A at 30 VDC.
- Kill wires can spike to several hundred volts, so check the relay rating below before wiring R2.
- Kill wire: the gray wire that runs from the starter relay area up under the gas tank. Key off stops the engine, and gray to ground reads 0-1 ohm with the key off and about 10 ohms with the key on (user tests, engine off for the valid readings). The key-off short identifies it as the ignition kill.
- R2 goes in parallel with the key switch's kill contact: COM to gray, NC to the same ground the key uses. Nothing gets cut. The key keeps its stock kill function. Remove the R2 tap to return to stock.
- Latch side effects: a routine disarm, motor test, or lidar false trip needs a key cycle to restart the engine. A script restart clears the latch and re-energizes R2 without a key cycle.
- Dead script gap: if the script dies, R1 and R2 hold their state and the vehicle stays armed. A radio kill then disarms but opens neither relay, so the engine and blade keep running until the key is turned off. Decide before running the blade whether that is acceptable.
- `RCx_OPTION` 153 acts on switch movement, so it is not a latch. A ground station can still arm with the switch low.
- Tap point: the white connector behind the Start Switch, in the battery bay. Gray on the harness side mates with black on the switch side (user check), so the tap can stay away from the engine. The key's kill contact is switch-side black to switch-side yellow, closed only with the key off (user test). So R2 COM taps switch-side black and R2 NC taps switch-side yellow. Switch-side yellow is confirmed as ground (user test). Colors change across this connector, so tap by the switch-side colors.
- R2 policy: energized after boot, so the engine can start and idle before arming. Any disarm after arming, failsafe, kill switch, or tilt drops R2 and latches it off until the key is cycled. The engine cannot start until the Pixhawk has booted, and it dies if the Pixhawk or relay board loses power.
- Keep the gray tap short and away from signal wiring. It carries ignition spikes. The relays are rated 10 A at 250 VAC (from the photo). Never connect its contacts to the low-voltage side.
- Do not measure the gray wire with the engine running. Ohm readings there are not valid and the spikes can damage a meter.
- Leave the starter off the board. Start the engine by hand at the mower.

### Relay board

8-channel opto board, SRD-05VDC-SL-C relays (5 V coils), header GND, IN1-8, VCC, and a VCC/JD-VCC jumper. There is no trigger jumper. Boards of this type are active-low (not yet tested on this one): a low pin energizes the relay, and a high or floating pin leaves it off.

**Why the Pixhawk powers it:** with an active-low board on its own 5 V supply, a dead Pixhawk can pull its pins low and energize R1 and R2, which is the unsafe direction. Fed from the Pixhawk's 5 V rail, the board loses power along with the Pixhawk.

Bench result so far: R1 and R2 switch on command from AUX1 and AUX2 (user test, ArduRover flashed, board powered from TELEM2 with the PM02). The tests below are still open unless noted.

Boot behavior: both relays energize for a few seconds while the Pixhawk boots, then drop (user test on USB power). This fits an active-low board with its pins held low before ArduPilot sets them.

- On the mower, that window matches stock key-on: main relay closed, kill wire released. Expected effect is no motion, because the box stops without pulses. Confirm with tracks off the ground: watch through several key cycles, and meter MAIN1/MAIN3 for no pulses during boot.
- A Pixhawk reset while mowing would energize both relays again during its reboot. If it hangs in that state, only the key stops the engine.

Bench tests with no mower wiring attached:
1. Key off, then power the Pixhawk. Listen and meter both relays through boot. Record any energizing.
2. Command each relay on and off. With the pin high at 3.3 V, the relay must fully drop. If it does not, the outputs need 5 V signal mode.
3. Unplug the Pixhawk's power while a relay is on. Both must drop.
4. Turn the key off with the mower wired. The main relay must open and the engine must stop.
5. Meter the NO, COM, and NC screw terminals instead of trusting their positions.

### Track power cut

The 24 V 120 A relay (MG686 RL/280-24) is the main power relay. Confirmed: its right lug is dead with the key off and at 24 V with the key on.

```text
Key-switched 24 V --+--> R1 (NO) --> main relay coil --> GND
                    |
                    +--> fuse --> PM02 V3 --> Pixhawk (runs all 5 V loads)
```

- Put R1 in series with the coil's key-side wire: the blue wire, which reads 24 V with the key on (user test).
- Take autopilot power from the key side, ahead of R1. If it comes from the right lug, opening R1 powers off the Pixhawk, and the Pixhawk can then never reclose R1.
- Coil current is 0.09 A (user test), far below a typical relay board rating. A flyback diode is optional at this level, so watch R1's contacts and add one if they show wear.
- The bus bar feeding the controllers is dead with the key off and live with the key on (user test), so this relay does cut track power.
- The black block has metal stamped 40A inside (user check), so it is very likely a 40 A fuse link shared by both controllers, not a plain bus bar.
- Check the key switch can carry the coil plus the autopilot load.
- Each reclose restarts the controllers, so Lua must wait for them to boot, then command neutral first. For a normal disarm, Lua commands neutral before opening R1.
- The starter is wired directly to the battery with its own relay (user check), so R1 does not affect cranking.

### Hard stop: the key

The key is the hard stop (user choice). No extra switch is wired. Key off, with no software involved:

- Removes 24 V from the blue wire, so the main relay opens and the tracks lose power.
- Shorts the kill wire to ground, so the engine stops. The blade coasts down.
- Powers off the Pixhawk and relay board.

Limit: it only works from the mower. If the Pixhawk or script fails with the mower moving, the radio kill may not work, and the only sure stop is reaching the key. Keep the mower slow and stay close.

### Optional: forward lidar (TFmini Plus)

| Pixhawk port | Pin | Goes to | Notes |
|---|---|---|---|
| TELEM2 (SERIAL2) | 1 VCC, 2 TX, 3 RX, 6 GND | TFmini: 5 V, RX, TX, GND | Shares pins 1 and 6 with the relay board. Confirm supply and logic levels on the sensor's label or manual. The Pixhawk UART is 3.3 V |

- Mount it level on the front center, as high as practical, clear of grass and thrown debris.
- At 2 m the 3.6° beam is only about 13 cm wide, so it misses anything outside that spot.
- More sensors can use the GPS1 port as a UART.

### Mounting

- Keep the Pixhawk, ESP32, and relay board in a sealed box. This bay collects grass and debris. Foam-mount the Pixhawk against engine vibration.

### Antennas

- The two vertical roll bar uprights look like good antenna mounts: wide baseline, clear sky, above the engine. Confirm clearance from the engine and any steel above them.
- Mount at least 30 cm apart (farther is better), level, with a clear sky view, away from the engine and ignition.
- The WTRTK-982DT ports are labeled M (master) and S (slave). Measure the offset for `GPS1_MB_OFS_*` between those antennas.

## Code map

| Item | Status | What it does |
|---|---|---|
| ArduPilot Rover 4.6+ | Stock firmware | Navigation, modes, failsafes, missions. A forum report says 4.6 fixed relays defaulting ON at power-up |
| `tools/um982_monitor.py` | Exists | Bench-test heading and RTK from the laptop first |
| DroneBridge on ESP32 | Stock firmware | MAVLink WiFi bridge in client mode, joined to the Mudi's WiFi. TCP port 5760 or UDP 14550 |
| `ardupilot/mower.param` | Draft, simulation only | Parameters below |
| `ardupilot/scripts/mower_safety.lua` | Draft, simulation only | R1 follows arm state, R2 latching engine kill per the R2 policy, tilt cutoff, left/right to fore-aft/differential conversion, lidar stop (under 2 m, debounced, Hold and engine kill) |
| Mudi setup | Config only | WiFi for the ESP32, Tailscale for remote access |
| Mowing paths | Mission Planner | Survey grid plus fence, no code |

The WitMotion inclinometer is not needed. The Pixhawk IMU supplies tilt.

## Starter parameters

Check names against your firmware's parameter list on first connect.

```text
# Drive. MAIN5/6 carry the left/right values for Lua and connect to nothing
SERVO1_FUNCTION      94      # script out 1, to box fore-aft
SERVO3_FUNCTION      95      # script out 2, to box differential
SERVO1_TRIM          1500
SERVO3_TRIM          1500
SERVO5_FUNCTION      73      # left
SERVO6_FUNCTION      74      # right
SERVO5_TRIM          1500
SERVO6_TRIM          1500
MOT_PWM_TYPE         0       # normal RC PWM
BRD_SAFETY_DEFLT     0       # no safety button fitted

# GPS
SERIAL3_PROTOCOL     -1
SERIAL4_PROTOCOL     5       # 982 on GPS2 port
GPS1_TYPE            25      # Unicore moving baseline
GPS1_MB_TYPE         1
GPS1_MB_OFS_X/Y/Z    measure
AHRS_EKF_TYPE        3
EK3_SRC1_YAW         2       # GPS yaw, compass is unreliable near motors

# Telemetry
SERIAL1_PROTOCOL     2
SERIAL1_BAUD         115     # match DroneBridge
BRD_SER1_RTSCTS      0

# Relays: repeat for relay 2 on pin 51
SERVO9_FUNCTION      -1      # AUX1 as GPIO, repeat for SERVO10
RELAY1_FUNCTION      1
RELAY1_PIN           50
RELAY1_DEFAULT       0
RELAY1_INVERTED      1       # active-low board, confirm on bench

# Failsafes
FS_THR_ENABLE        1
FS_ACTION            5       # disarm, Lua then drops the relays, only while the script is alive
RC6_OPTION           153     # kill switch: low disarms (channel from radio test)
FS_GCS_ENABLE        1       # tune FS_GCS_TIMEOUT if cellular or WiFi drops trip it
FS_OPTIONS           1       # failsafes also act in Hold
ARMING_RUDDER        0       # steering cannot arm
FS_CRASH_CHECK       2       # hold and disarm

# Optional lidar
SERIAL2_PROTOCOL     9       # lidar
SERIAL2_BAUD         115
BRD_SER2_RTSCTS      0
RNGFND1_TYPE         20      # Benewake serial
# Rover 4.6 uses RNGFND1_MIN_CM 10 and RNGFND1_MAX_CM 600 instead
RNGFND1_MIN          0.1
RNGFND1_MAX          6       # ArduPilot's outdoor example
RNGFND1_ORIENT       0       # forward

# Battery monitor (PM02 V3). Voltage only is useful, current excludes the motors
BATT_MONITOR         3       # analog voltage only
BATT_VOLT_MULT       18.182

# Scripting
SCR_ENABLE           1
```

## Radio channel plan

Transmitter: HOTRC CT-6A (user believes, check the label), which the SBUS-A manual lists as compatible. Only 6 channels are available, even though the SBUS-A supports 16.

| Channel | Use | Requirement |
|---|---|---|
| Steering control | Steering, `RCMAP_ROLL` | Centers |
| Throttle control | Throttle, `RCMAP_THROTTLE` | Centers |
| Spare 1 | Kill and arm, `RCx_OPTION` 153 | Must latch, and low must be the kill position |
| Spare 2 | Mode: Manual, Hold, Auto, `MODE_CH` | Latching, ideally 3 positions |
| Spare 3-4 | Unused | |

Assign channel numbers after the test. Binding the SBUS-A may mean rebinding the F-06A to return to stock.

## Bring-up order

1. Run the UM982 alone on the laptop and confirm heading and RTK.
2. Bench the Pixhawk with no mower power: RC, GPS, WiFi link, relay clicks. Watch for relays clicking at boot.
3. Tracks off the ground, engine off: motor test, every failsafe, hard stop, radio kill switch, script failure test.
4. Manual driving through the Pixhawk, then tune.
5. Auto with the blade off, then attended with blade on and fence enabled.

Without the lidar, this plan has no obstacle sensing. Even with it, a single narrow beam will miss most people, pets, and objects, so stay at the mower with the radio kill in hand and keep the area clear.

## Open items

| Need | Decides |
|---|---|
| Bench test: drive box | Mix formula, pivot ability, behavior on signal loss |
| Continuity test: start switch and its white connector, battery disconnected | Which wires are power, start, and engine kill. May give a safe tap point for R2 |
| Bench test: relay board behavior | Boot state, 3.3 V drive, power-loss drop |
| Meter: WTRTK-982DT TXD idle voltage | Should be about 3.3 V before connecting to GPS2 |
| Bench test: SBUS-A radio loss, transmitter off | Whether it signals failsafe or holds last values. If it holds, ArduPilot may never see the loss. Must pass before any driving |
| Measure: box signal input | Whether it accepts 3.3 V from MAIN, or needs the Pixhawk's 5 V output mode |
| Bench test: CT-6A channels in Mission Planner | What each control sends on CH1-6 and whether it latches. Sets the radio channel plan |

## RTK corrections from the base station

Base project: `../rtk-base-station`. It serves RTCM3 (1006, 1033, MSM4 1074/1084/1094/1124, 1230) over NTRIP on the home LAN, port 2101, mountpoint `BASE`. No new hardware or base firmware changes are planned.

```text
Base :2101 --NTRIP--> Mission Planner --MAVLink--> ESP32 --> TELEM1 --> Pixhawk --> GPS2 --> 982
```

Mission Planner stays connected while mowing, and it is already needed for the ground station failsafe. Enter the base in its RTK/GPS Inject screen.

Network, whichever applies:
- **Home WiFi reaches the mowing area:** set the Mudi as a repeater on home WiFi, with the laptop on the Mudi. Then the base's LAN IP is reachable. No relay needed.
- **Otherwise:** Mudi on cellular, with Tailscale on the Mudi and a home device routing to the base LAN. The base relay and port forward are the fallback.

Before saving any fence or mission, fix the base position (`rtk-base-station/docs/rtcm_to_rinex.md`). Each survey-in can shift every coordinate by a metre or two.

Not tested: the base with this 982 as a rover, or ArduPilot passing injected RTCM to a `GPS1_TYPE` 25 receiver. Test order: base to 982 on the laptop first, then through the Pixhawk.
