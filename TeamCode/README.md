# BIOBUZZ robot code

For the 2026-27 REV DUO FTC Starter Bot, on FTC SDK v12.0. Needs Android Studio Narwhal 3 Feature
Drop or later. There are no third-party libraries. Pedro Pathing has no tank drivetrain, so
`core/DriveController` does that job.

**This code has never run on a robot.** It compiles against the SDK 12 jars and passes its tests in
simulation. Every value marked ESTIMATE in `Constants.java` needs measuring, in the order below.

## How it fits together

```
opmodes/      MainTeleOp, MainAuto, Diagnostics    what the drivers pick
Robot, Snap                                        owns everything; update() once per loop
subsystems/   Drive Shooter Intake Poker Vision    the only code that touches hardware
core/         geometry, planning, control          plain Java, tested on a laptop
Constants                                          every tunable number
```

Keep FTC imports out of `core/` and `Constants`. That is what lets the tests run without a robot.

Each loop: read the encoders and IMU, update the pose, read any new camera frame, then act.

## What makes this season different

* **The tags move.** They are on the underside of each Cell and each Hive tips. The SDK therefore
  leaves every cluster's field position at (0, 0, 0), so `detection.robotPose` is not a field pose.
  `core/TagLocalizer` uses `ftcPose` (camera to Cell opening) with our own map in `core/Field`.
* **SDK 12 detections are clusters.** One `AprilTagClusterDetection` per Cell, named e.g.
  `RED AUDIENCE`. Older AprilTag code will not compile.
* **Only a raised Cell scores.** Its cluster reads `|roll| < 90`. `core/HiveTracker` follows that.
* **A Tip moves the target.** Openings face the audience and rear walls, so after a Tip the robot
  must cross to the other side. It can drive under the Hive, between the frame legs.
* **The camera must look up.** Tilt it about 45 degrees, facing the same way as the launcher.

## Controls

Same as REV's starter code, plus the items marked NEW.

| Input | Action |
|---|---|
| Left stick | Arcade drive |
| Left trigger (hold) | NEW: self-drive to the best shot spot, aim, spin up. Rumbles when ready |
| Dpad left (hold) | NEW: self-drive beside the nearest Flower |
| Dpad right (hold) | NEW: self-drive into our Loading Zone |
| Left bumper | Launch. With the left trigger held, it waits until arrived and aimed |
| Right bumper / trigger | Intake / outtake |
| Y / X | Flywheel only / reverse flywheel |
| Dpad up / down | Feeder forward / reverse |
| A / B | Poker down / up |
| Back | NEW: Hive tipped but the camera missed it, swap target Cell |
| Right stick button | NEW: reset heading. First square the robot so it faces the audience or rear wall (either way), then show the camera a raised Cell |

Letting go of a self-drive button hands the wheels straight back to the driver. Holding the left
trigger and left bumper together drives, aims and fires by itself.

If telemetry says `NO POSE`, self-driving is refusing to run because the robot does not know where it
is. Point the camera at a raised Cell. This happens when teleop starts without auto having run.

## Bring-up, in order

Use the **Diagnostics** OpMode. Up on blocks first.

1. **Configuration.** Load REV's config, then add the webcam as `Webcam 1` and confirm the IMU is
   named `imu`.
2. **Directions.** Stick forward drives the intake end first, and both wheels' encoders count up.
3. **IMU.** Set the hub orientation at the top of `subsystems/Drive.java`. Turning the robot
   counter-clockwise (seen from above) must make `Raw yaw` go up.
4. **Launcher direction.** Check `LAUNCHER_FACES_REAR`. It was inferred from REV's auto code.
5. **Poker side.** Check `POKER_ON_LEFT` and measure `POKER_FORWARD_IN`.
6. **Distance.** Press Y, push the robot exactly 72 in along the tiles, and scale
   `DRIVE_TICKS_PER_IN` by reading / 72.
7. **Robot size.** Measure `ROBOT_LENGTH_IN`, `ROBOT_WIDTH_IN`, and confirm the robot fits under the
   Hive (30.6 in clearance). If not, set `CAN_DRIVE_UNDER_HIVE` to false.
8. **Camera.** Measure its position and tilt. With a real Cell in view, `height` should read about
   58.4. If not, the tilt is usually wrong. Then put the robot on a known spot and compare
   `robot would be at` with where it really is. An error that grows with distance means the tilt or
   calibration is off. A non-C270 camera also needs lens calibration.
9. **Field numbers.** Check `OPENING_X`, `OPENING_Z` and the Flower and Loading Zone positions in
   `core/Field.java` against a real field or the CAD. They came from manual figures.
10. **Shooter.** Every shot's flywheel speed comes from `SHOT_TABLE`, so fill it in from real shots:
    1. Park with the launcher straight on to a raised Cell and the camera seeing it.
    2. Read `shot distance` from telemetry. Press X to start from what the table says now.
    3. Hold the left bumper to fire. Dpad up/down 10 at a time until 4 in a row score cleanly.
       Low means faster. If it is still rising as it reaches the opening, back up instead.
    4. Write down {distance, speed}. Repeat at least 3 distances: as close as you would shoot, the
       shot spot, and as far as you would shoot.
    5. Put the rows in `SHOT_TABLE`, shortest first. Set `SHOT_FRONT_TO_WALL_IN` to the spot that
       scored most reliably.
11. **Self-driving.** On the field, hold the left trigger with a finger ready to let go. Tune
    `AUTO_MIN_*` until the robot finishes moves without stalling, then `AUTO_*_KP` until it stops
    without wobbling. Raise `AUTO_MAX_DRIVE_POWER` last.
12. **Auto.** Start with the Garden run off. Measure `FEED_SECONDS_PER_POLLEN`. Turn the Garden run on
    once a run along the wall collects reliably.

## Tests

```
./gradlew :TeamCode:testDebugUnitTest
```

`SnapSimulationTest` drives a simulated robot to every Cell from a grid of starting poses, checking
that it arrives and never hits the Hive frame. It proves the logic works. It does not prove the
gains suit a real robot.

## Rules the code enforces

* G304, G403: nothing moves during INIT. The poker is set after START.
* G402: auto only picks shot spots on our half of the field.
* G410: rumbles at 60 s left, when Nectar may go in Flowers, and 20 s left, to go and park.

Drivers still own G407 (max 4 scoring elements): the robot cannot count what it holds.
