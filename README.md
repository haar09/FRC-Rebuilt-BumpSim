# RobotBumpSim

**Standalone robot-bump physics simulation for MapleSim (FRC)**

Simulates how a robot behaves when driving over field bumps, including realistic sliding when it lacks enough speed to climb.

---

## Overview

`RobotBumpSim` models the robot’s **3D pose** while interacting with raised field elements:

* Z height (vertical position)
* Pitch (forward/back tilt)
* Roll (side tilt)

It also prevents unrealistic behavior like clipping through bumps by simulating **gravity-driven sliding** on inclines.

---

## Core Concept

The robot is treated as four independent contact points (swerve modules). When interacting with a bump:

* The system switches into a **ramp simulation mode**
* Motion along the bump becomes **frictionless**
* Only gravity affects forward/backward movement
* The robot either:

  * Successfully crests the bump, or
  * Slides back down if speed is insufficient

---

## Physics Behavior

### Ramp Mode

Triggered when a module touches the **ascending face** of a bump.

During ramp mode:

* X velocity and position are **captured and controlled internally**
* Motion is governed only by gravity
* No drivetrain acceleration is applied

If velocity reaches zero before the peak:

→ The robot slides backward

---

### Exit Conditions

Ramp mode ends when:

* All modules return to flat ground (robot backs off), or
* The robot crosses the bump and reaches flat ground on the other side

---

### Diagonal Crossing Advantage

Approaching at an angle reduces effective deceleration:

```
contactFactor = |cos(2 * robotYaw)|
```

* Head-on (0°) → hardest
* 45° → easiest

---

### Minimum Required Speed

To fully crest a bump:

```
v = sqrt(2 * g * h)
≈ sqrt(2 * 9.81 * 0.165)
≈ 1.80 m/s
```

---

## Integration (MapleSim)

### 1. Initialize

```java
RobotBumpSim robotBumpSim = new RobotBumpSim(drivetrain.getModuleLocations());
```

---

### 2. Update Each Simulation Step

Call after MapleSim updates:

```java
Pose2d simPose = mapleSimDrive.getSimulatedDriveTrainPose();

ChassisSpeeds robotRelSpeeds =
    mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative();

ChassisSpeeds fieldSpeeds =
    ChassisSpeeds.fromRobotRelativeSpeeds(robotRelSpeeds, simPose.getRotation());

Pose3d simPose3d = robotBumpSim.update(simPose, fieldSpeeds, subticks);
```

Recommended:

* `subticks = 5` for a 20 ms loop (~4 ms physics steps)

---

### 3. Override Physics When on Ramp

```java
if (robotBumpSim.isOnRamp()) {
    mapleSimDrive.setSimulationWorldPose(
        robotBumpSim.getSimWorldPose(simPose)
    );
}
```

This ensures the robot physically slides instead of just visually correcting.

---

### 4. Log / Visualize

```java
Logger.recordOutput("Drive/Pose3d", simPose3d);
```

---

## Tunable Constants

### `WHEEL_RADIUS`

Effective wheel contact radius (meters)

* Higher → robot appears to float above bumps
* Lower → tighter contact with surface

---

### `CHASSIS_HEIGHT`

Offset from module contact plane to robot body origin

* Use real chassis clearance for accurate visuals

---

### `BUMP_COR`

Coefficient of restitution (vertical bounce)

* `0` → no bounce
* `1` → perfectly elastic

---

## Field Geometry

* Bumps are defined as **XZ line segments**
* Each includes:

  * Ascending face
  * Descending face
* Guarded by Y-range checks

All geometry is:

* In **meters**
* Origin at **Blue Alliance driver station corner**

See:

```
BUMP_LINE_STARTS
BUMP_LINE_ENDS
```

for raw coordinates.

---

## Why This Matters

Without this model:

* Robots unrealistically drive through obstacles
* Simulation doesn’t match real-world traversal
* Autonomous tuning becomes unreliable

With `RobotBumpSim`:

* Crossing behavior becomes physically accurate
* Speed requirements are realistic
* Driver strategies (like angled approach) matter
