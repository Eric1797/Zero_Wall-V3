# line_follower_v4: DIP switch modes and start/stop

Board: JSumo XMotion V3 (upload as Arduino Leonardo). DIP switches DIP1 = D5, DIP2 = D6, DIP3 = D7.
A switch that is ON connects its pin to GND (reads LOW).

## Modes

The switches are read once at power-on. Change a switch, then switch the robot off and on.
After power-on, both LEDs blink the mode number.

| DIP1 | DIP2 | DIP3 | LED blinks | Mode | What it does |
|---|---|---|---|---|---|
| OFF | OFF | OFF | 1 | **COMPETITION** | Full speed, the tuned settings. Use this for the race. |
| ON | OFF | OFF | 2 | **TEST** | 10% speed (motors never go below PWM 45). First try on a new track, or checking that it follows the right way. |
| OFF | ON | OFF | 3 | **SAFE** | 70% speed. Dusty or slippery track, worn tyres, first real runs, or when COMPETITION flies off. |
| OFF | OFF | ON | 4 | **SENSOR CHECK** | Motors never run. LEDs show whether the line is on the left or right. The Serial Monitor (115200 baud) prints all 16 sensors and the MicroStart signal on A0. Put the robot on the line and check every sensor. |
| ON | OFF | ON | 6 | **TECHNICAL** | For a hard track: 90 degree corners, tight turns. Brakes earlier, lower corner speed, pivots as soon as the line disappears. |
| OFF | ON | ON | 5 | **FAST** | 110% speed. Only with a fresh battery, clean tyres and the fan running. |
| ON | ON | OFF | 7 | **COMP, NO MICROSTART** | Full competition speed, but starts by itself about 1 s after the 7 blinks. No remote needed. Use it when the MicroStart is missing or not working. To stop: lift the robot and switch it off. |
| any other mix | | | 2 | TEST | Safest fallback. |

Steering strength scales with the mode's speed, so the slow modes do not jerk from side to side.

Sim results on the competition track (1.5 cm tape):

| Mode | Lap |
|---|---|
| COMPETITION | about 9.3 s |
| SAFE | about 11.0 s |
| FAST | about 9.5 s (faster only with a stronger fan) |
| TEST | about 19.8 s |

## MicroStart start module (A0)

Setting: `START_WITH_MODULE = 1` (default).

1. Switch the robot on and check the mode blinks.
2. Place it on the start line.
3. Press **START** on the remote. A0 goes LOW to HIGH and the run starts.
4. Press **STOP** at any time. This is an **emergency stop**: the motors switch off at once, both LEDs blink, and the robot stays stopped until you **cut the power** (battery **and** USB). The module also needs a power cycle after STOP.

If the module is not fitted, or you want the robot to start by itself, set `START_WITH_MODULE = 0`. The robot then starts 1 s after power-on, and STOP still works once START has been pressed.

The remote's buttons must be recorded into the module once. See `../microstart/startstop_instructions.md`.

## Before the race

1. Set DIP3 only (SENSOR CHECK). Power on: you should see 4 blinks. Move the line under the bar and watch the `#` move on the Serial Monitor. Press START: `start A0:HIGH` should appear.
   - If you see **1 blink**, the switches work the other way round (ON = HIGH). Tell the code author to flip `== LOW` to `== HIGH` in `setup()`.
2. Set all switches OFF (COMPETITION) and power-cycle.
3. Clean the tyres. Use a charged battery and switch the fan on.
4. If COMPETITION goes off the track, use SAFE (DIP2) and lower `MAX_SPEED` / `BOOST_SPEED` in the code.
