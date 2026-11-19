# MicroStart start/stop: instructions for agents

Applies to the JSumo XPulse line follower on the **XMotion V3** board (ATmega32U4, upload as `arduino:avr:leonardo`).

## Hardware facts (verified on this robot, 2026-11-19)

- Start module: **JSumo MicroStart** (datasheet: `microstart/microstart-eng.pdf`).
- MicroStart signal pin on this board: **A0**. This was found by a pin scan and confirmed on serial: pressing Stop changed A0 from HIGH to LOW.
- **Do not trust JSumo's published XMotion pin table for this board.** It says start = D10, motor PWM = D3/D11, and A0 = battery voltage sensor. On this V3 board, **D10 is the left motor PWM** (proven by the working `line_follower.ino`), and A0 is the start input.
- Pins used by the robot (do not reuse):

  | Pins | Used for |
  |---|---|
  | D10, D11 | Motor PWM |
  | D12, D13 | Motor direction |
  | A1, A2, A4, A5, D4 | Xline mux |
  | D8, D9 | LEDs |

## MicroStart signal behaviour (from the datasheet)

| Event | A0 level | Module LEDs |
|---|---|---|
| Power-up / standby | LOW | both off |
| Remote Release pressed | LOW | red on |
| Remote Start pressed | **HIGH** (stays HIGH) | blue on |
| Remote Stop pressed | **LOW** (latched) | both blink |

- **After Stop, the module stays latched LOW until its power is cut and reapplied.** Pressing the Leonardo reset button or uploading code does NOT reset it, because the 5V rail stays up. If USB is plugged in, unplug USB as well as turning off the battery.
- The module **cannot be forced HIGH** by code. It only goes HIGH after it receives the remote's Start signal.
- Remote buttons must be recorded once. Hold the white button for 1 s (LEDs blink 3 times), then press the Release, Start and Stop buttons, waiting 1 s between each. The LEDs blink fast 4 times when recording is done. The recording is saved in the module.

## Required code logic (emergency stop)

- Read A0 with `pinMode(A0, INPUT_PULLUP)`. The module drives the pin through a 1k resistor, which overrides the pull-up.
- **Do NOT treat A0 == LOW as stop.** LOW is also the normal standby level, so the robot would never move.
- **Emergency stop = a debounced HIGH to LOW edge on A0** (A0 was HIGH, now LOW). On that edge:
  - immediately set all motor PWM pins (D10, D11) to 0;
  - latch the robot in a stopped state until it is power-cycled;
  - print a serial message (for example `EMERGENCY STOP`).
- Check A0 every loop iteration, before motor output. The robot is very fast, so do not check it only occasionally.
- The robot still starts by itself through the existing `AUTO_START` logic in `xpulse_advanced.ino`. The MicroStart Start signal only **arms** the emergency stop.

## Operator routine (competition)

1. Power on the robot.
2. Press **Start** on the remote once (blue LED on, A0 goes HIGH). This arms the emergency stop. If you skip it, Stop does nothing that run.
3. Place the robot on the line. It auto-starts.
4. Press **Stop** at any time to kill the motors.
5. To run again: cut power (battery and USB), then repeat from step 1.

If the competition rules require the start module to start the run, change the logic so that the A0 LOW to HIGH edge starts the robot instead of `AUTO_START`.

## Test sketch

`microstart/microstart_test/microstart_test.ino` prints `A0=HIGH/LOW` every second and `>>> TRIGGERED: ...` on each change. It holds the motor pins LOW during the test.

```
arduino-cli compile --upload -p COM21 --fqbn arduino:avr:leonardo microstart/microstart_test
arduino-cli monitor -p COM21 -c baudrate=115200
```

COM21 is the Leonardo's port on this PC. Close any open serial monitor before uploading, or you get `unable to open port COM21`.
