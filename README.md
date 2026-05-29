# FTC Team 24804 — DECODE

> Inspire here we come 😈

Robot code for FIRST Tech Challenge team **24804**, **DECODE** season (2025–2026).
The robot shoots artifacts at an elevated goal using predictive ballistic aiming,
a mecanum drivetrain, and GoBilda Pinpoint odometry.

---

## Contents

- [Robot Capabilities](#robot-capabilities)
- [Project Structure](#project-structure)
- [Hardware Map](#hardware-map)
- [OpModes](#opmodes)
- [Architecture](#architecture)
- [Tuning](#tuning)
- [Build & Deploy](#build--deploy)
- [Dependencies](#dependencies)

---

## Robot Capabilities

- **Mecanum drive** with field-centric control and heading-hold PID.
- **Dual-motor flywheel** (`leftFlyWheel` + `rightFlyWheel`) with custom voltage-compensated PIDF velocity control.
- **Turret + tangage (pitch)** on servos for horizontal and vertical aim.
- **Predictive aiming**: iterative lead-time computation accounting for robot velocity (`Aim.aimOld`).
- **Gate + transfer**: artifacts only feed the flywheels once velocity is within `FW_error`.
- **GoBilda Pinpoint** odometry (with OTOS, 2- and 3-dead-wheel fallbacks in `roadrunner/`).
- **Limelight 3A** vision for autonomous targeting.
- **Dual path planners**: both [RoadRunner 1.0](roadrunner/) and [PedroPathing](pedroPathing/) coexist.

---

## Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Main.java                  # Primary TeleOp
├── TestMotors.java            # Bench drivetrain test
├── OuttakeTest.java           # Shooter test rig
│
├── controlllers/              # High-level subsystems
│   ├── Aim.java               # Ballistics + turret aiming
│   ├── Intake.java            # Roller intake
│   ├── Outtake.java           # Flywheels, gate, transfer, tangage
│   ├── Movement.java          # Field-centric / robot-centric drive
│   ├── RRPoint.java
│   └── VoltageProcessor.java
│
├── roadrunner/                # RoadRunner 1.0 drive
│   ├── MecanumDrive.java
│   ├── PinpointLocalizer.java
│   ├── OTOSLocalizer.java
│   ├── TwoDeadWheelLocalizer.java
│   ├── ThreeDeadWheelLocalizer.java
│   ├── TankDrive.java
│   ├── messages/              # FtcDashboard logging
│   └── tuning/                # LocalizationTest, SplineTest, etc.
│
├── pedroPathing/              # PedroPathing autonomous routines
│   ├── Constants.java         # Follower / mecanum / pinpoint config
│   ├── Tuning.java
│   ├── Camera.java            # Limelight-driven auto
│   ├── Shoot.java
│   ├── SmallFar.java
│   ├── AutoFarFromGoalWithGate.java
│   ├── AutoFarFromGoalWithGateMinus1cycle.java
│   └── AutoCloseToGoalWithGateWithoutTimer.java
│
└── Utilities/                 # Utilities
    ├── GamepadEx.java
    ├── GamepadDeadZoned.java
    ├── LerpController.java
    ├── Storage.java           # Shared state between OpModes
    ├── xCusedPIDController.java
    ├── xCusedPIDFController.java
    ├── xCusedServo.java
    └── xCusedTimer.java
```

> The `controlllers` typo (three `l`s) is historical — renaming it cascades through
> every autonomous routine. Don't touch without a full migration.

---

## Hardware Map

| Config name | Type | Purpose |
|---|---|---|
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | DcMotor | Mecanum wheels |
| `leftFlyWheel`, `rightFlyWheel` | DcMotorEx | Shooter flywheels (left is REVERSE) |
| `transfer` | DcMotorEx | Feeds artifacts into the flywheels |
| `intake` | DcMotorEx | Roller intake (REVERSE) |
| `gate` | Servo | Gate in front of the flywheels |
| `tangash` | Servo | Shooter pitch (tangage) |
| `turret`, `RightTurret` | Servo | Turret (two servos) |
| `pinpoint` | GoBildaPinpointDriver | Odometry |
| voltage sensor | VoltageSensor | Battery sag compensation for flywheels |
| Limelight 3A | — | See `pedroPathing/Camera.java` |

Odometry parameters (`pedroPathing/Constants.java`):
```
forwardPodY = 3.54  in
strafePodX  = -4.5  in
encoder     = goBILDA 4-Bar Pod
```

---

## OpModes

### TeleOp

**`Main`** — primary competition TeleOp.

Driver controls (gamepad1):

| Input | Action |
|---|---|
| Left stick | Drive (field-centric) |
| Right stick X | Rotate |
| RT > 0.5 | Shoot (spin up → gate auto-opens once velocity is in tolerance) |
| LT > 0.3 | Slow transfer + intake |
| RB | Reverse-eject from shooter |
| X (during init) | Toggle RED / BLUE alliance |

Operator controls (gamepad2) — goal coordinate trim:

| Input | Action |
|---|---|
| D-pad Left / Right | ±2" goal X |
| D-pad Up / Down | ∓2" goal Y |

### Autonomous (`pedroPathing/`)

- **`AutoFarFromGoalWithGate`** — far-zone start, cycle: human-zone → score → gate-zone → score → leave.
- **`AutoFarFromGoalWithGateMinus1cycle`** — same, minus one cycle (fallback for time-critical runs).
- **`AutoCloseToGoalWithGateWithoutTimer`** — close-to-goal start.
- **`Shoot`** (`@Autonomous(name = "ashhot")`) — shooter debug routine.
- **`Camera`** — Limelight-based autonomous targeting.
- **`SmallFar`** — short variant of the far autonomous.

### Utility

- **`TestMotors`** — drivetrain diagnostics.
- **`OuttakeTest`** — bench for tuning FW velocity / tangage.
- **`roadrunner/tuning/*`** — standard RoadRunner tuners (`LocalizationTest`, `SplineTest`, `ManualFeedbackTuner`).
- **`pedroPathing/Tuning.java`** — PedroPathing tuner.

---

## Architecture

### Aiming loop

```
poseEstimate ──┐
velocity     ──┼──► Aim.aimOld() ──► aim_pos (with lead)
goal         ──┘                ├──► flyWheelVel = 6.75·d + 666.87
                                ├──► tangageAngle = atan(2h / (d − peakDistance))
                                └──► targetAngle (for turret + chassis yaw)

aim_pos is refined for 5 iterations — leadTime is recomputed from
v·cos(angle), which compensates for both shot angle and current linear velocity.
```

Ballistic constants (`Aim.java`):

```
LEAD_TIME_CEF   = 480
h               = 33.6   // goal height relative to shooter exit
PEAK_DISTANCE   = 5      // close-range compensation
```

### Flywheels (voltage-compensated PIDF)

`Outtake.UpdateFlyWheelPIDF()`:

```
kF         = targetVelocity / ticksPerVolt / voltage
correction = kP · (target − actual) + kF
```

`ticksPerVolt = 201.5625`, `kP = 0.004`. When `target == 0` the output is forced to
zero (so the motor isn't held by PIDF at idle).

### Field-centric drive

`Movement.FieldCentricDrive()` rotates the stick vector by `−heading`,
uses a heading-hold PID with integral reset on `|error| > 10°`, and pulls the
derivative term from `vH_robot` (Pinpoint) rather than numeric differentiation.

`autoPricel = true` swaps `targetHeading` for `angleToLook` (target-locked yaw
overrides the right stick).

### Shared state

`Utilities/Storage.java` persists poses and zeroes between OpModes (Auto → TeleOp):

```java
CurrentPosePedro     // last pose from autonomous
turretEncoderZero    // turret encoder zero
allianceBlue
```

---

## Tuning

All key parameters are marked `@Config` and are live-editable through
**FTC Dashboard** (http://192.168.43.1:8080/dash while connected to the RC).

| Class | Knobs |
|---|---|
| `Main` | `FlyWheelVel`, `robotkP/I/D/F`, `leadTimeCef`, `h`, `peakDistance`, `FW_error` |
| `Aim` | `LEAD_TIME_CEF`, `h`, `PEAK_DISTANCE`, `turretKp`, `minDegree`/`maxDegree`, `servoAtZeroDeg`, `servoPerDegree` |
| `Outtake` | `ticksPerVolt`, `CloseGatePos`, `OpenGatePos`, `minAngle`, `FarTan`, `CloseTan`, `goalYRed`, `goalYBlue` |
| `Movement` | `maxSpeedChangePerSec`, `ShapeExpo` |
| `Storage` | autonomous start poses |

Flywheel-velocity regression vs. distance (baked into `Aim` and `Outtake`):

```
FlyWheelVel = 6.75 · distance + 666.87
```

Re-calibrate via `OuttakeTest` whenever the artifact compound or wheel wear changes.

---

## Build & Deploy

```powershell
# from the repo root
.\gradlew :TeamCode:assembleDebug
```

Deploy via Android Studio (module `TeamCode`) to the Control Hub.

Minimum requirements:
- Android Studio Koala+
- FTC SDK 11.0+ (see `build.dependencies.gradle`)
- Control Hub on current firmware

---

## Dependencies

- **FTC SDK** (`FtcRobotController`)
- **RoadRunner 1.0** (`com.acmerobotics.roadrunner`)
- **FTC Dashboard** (`com.acmerobotics.dashboard`)
- **PedroPathing** (`com.pedropathing`)
- **GoBilda Pinpoint Driver** (`com.qualcomm.hardware.gobilda`)
- **Limelight 3A driver** (`com.qualcomm.hardware.limelightvision`)

---

## Team

**24804** — FIRST Tech Challenge. **DECODE** season, 2025–2026.

> Inspire here we come 😈
