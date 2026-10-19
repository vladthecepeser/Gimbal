"# Gimbal" 
# Keeping a Camera Level During Sustained Linear Acceleration

To keep a camera perfectly level with the horizon during sustained linear acceleration, such as an aircraft accelerating down a runway or holding a prolonged bank, a gyroscope alone is fundamentally insufficient.

Because of Einstein's equivalence principle, no filter, whether a complementary filter, EKF, or ESKF, can differentiate between the acceleration of gravity and the sustained linear acceleration of the aircraft using only an IMU. To solve this, you must introduce external physical references or additional sensors.

The industry-standard solutions, ranging from hardware designs to sensor fusion architectures, are outlined below.

## 1. Hardware Architecture: Active Gimbal with Sensor Fusion

To physically maintain the camera position relative to the horizon, you need a 3-axis active brushless gimbal paired with an Attitude and Heading Reference System (AHRS) or an Inertial Navigation System (INS).

To overcome the sustained acceleration problem, your flight controller or gimbal controller must use one of the following sensor fusion approaches.

### Option A: GPS/GNSS-Aided INS (The Professional Standard)

**Hardware**

- A standard 6-DOF IMU (Gyroscope + Accelerometer)
- GPS/GNSS receiver

**How It Works**

The filter (typically an EKF) uses GPS to track the aircraft's true ground velocity. By taking the derivative of this velocity, the filter calculates the true kinematic linear acceleration (\(\mathbf{a}\)). It then mathematically subtracts this from the accelerometer's total reading (\(\mathbf{f}\)), isolating the gravity vector:

\[
\mathbf{g} = \mathbf{a} - \mathbf{f}
\]

**Result**

- Roll and pitch remain stable.
- Performance is unaffected by sustained acceleration.
- Long turns and airspeed changes do not corrupt attitude estimates.

### Option B: Airspeed Sensor Aiding (The Fixed-Wing Approach)

**Hardware**

- 6-DOF IMU
- Pitot tube (airspeed sensor)

**How It Works**

In a sustained turn, an aircraft experiences continuous centripetal acceleration. By fusing the gyroscope's yaw rate (\(\omega_z\)) with the forward airspeed (\(v\)), the filter calculates the expected centripetal acceleration:

\[
a_c = \omega_z \times v
\]

The filter subtracts this effective acceleration from the lateral accelerometer measurement, preventing the gimbal from leaning into the turn.

**Result**

- Improved horizon stability during coordinated turns.
- Reduced attitude distortion from sustained lateral acceleration.

## 2. Software Filter Modification: Adaptive Covariance Gating

If you cannot add a GPS or Pitot tube and must rely only on an IMU, you cannot completely eliminate drift during sustained acceleration. However, you can significantly mitigate it using **Adaptive Measurement Covariance**.

In a standard filter, the accelerometer continuously corrects gyro pitch and roll drift under the assumption that the accelerometer primarily measures gravity. During sustained acceleration, this assumption becomes invalid.

### The Fix

Program the filter to monitor the accelerometer magnitude:

\[
\lVert A \rVert
=
\sqrt{
a_x^2 +
a_y^2 +
a_z^2
}
\]

When either of the following occurs:

- \(\lVert A \rVert\) deviates significantly from \(1\,g\) (approximately \(9.81 \,\text{m/s}^2\))
- The gyroscope indicates a sustained turn

the filter dynamically increases the accelerometer measurement noise covariance \(R\).

### Result

The filter temporarily "blindfolds" the accelerometer and relies primarily on gyroscope integration during the acceleration event.

When the aircraft returns to steady flight:

- Accelerometer measurements regain weight.
- Pitch and roll estimates are re-corrected.
- Minor gyro drift is removed.

## Summary of System Recommendations

| Expected Duration of Acceleration | Recommended Setup | Complexity |
|----------------------------------|-------------------|------------|
| Short bursts (< 5-10 s) | Standard IMU + Gimbal with Adaptive Covariance Gating | Low (software tuning only) |
| Sustained turns / orbiting | IMU + Gimbal + Airspeed Sensor (centripetal compensation) | Medium |
| Heavy or effectively unlimited sustained acceleration | IMU + Gimbal + GPS-Aided EKF/INS (full kinematic isolation) | High (requires GPS lock) |

## Key Takeaway

A gyroscope by itself cannot distinguish gravity from sustained linear acceleration. As a result, a purely IMU-based horizon estimate will inevitably become biased during prolonged acceleration.

The available solutions form a hierarchy:

1. **Adaptive covariance gating** reduces the error but cannot eliminate it.
2. **Airspeed aiding** compensates for known aircraft dynamics, especially coordinated turns.
3. **GPS/GNSS-aided INS** provides true kinematic acceleration, allowing gravity to be isolated and delivering a horizon estimate that remains accurate even under prolonged acceleration.

For applications where horizon lock must remain accurate indefinitely, a GPS-aided INS is generally the most robust solution.




---------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------

# Inertial Navigation System (INS) Alignment with GPS-Aided EKF/ESKF

You have hit upon the exact crux of Inertial Navigation System (INS) alignment. You are completely correct: knowing the true kinematic acceleration vector from the GPS is useless unless you know the spatial rotation (\(\mathbf{R}_{b}^{n}\)) between the IMU's body frame and the GPS's navigation frame (usually North-East-Down).

In a GPS-aided Extended Kalman Filter (EKF) or Error-State Kalman Filter (ESKF), this frame matching is solved through mathematical observability during dynamic motion. The filter does not measure the relative direction directly; instead, it dynamically estimates the 3D orientation matrix by continuously checking for mathematical consistency between the two sensor systems.

Here is exactly how the filter resolves this alignment and extracts the gravity direction.

## 1. The Mathematical Mechanism: Tracking the Residuals

The filter maintains an ongoing estimate of the aircraft's orientation matrix:

\[
\hat{\mathbf{R}}_{b}^{n}
\]

(Body to Navigation frame).

Every time a GPS measurement arrives, the filter performs a "What if?" prediction step:

### 1. Predict Velocity Change

The filter takes the IMU's proper acceleration reading (\(\mathbf{f}\)), rotates it into the navigation frame using its current guess of orientation (\(\hat{\mathbf{R}}_{b}^{n}\)), and adds gravity to predict what the new GPS velocity should be:

\[
\hat{\mathbf{v}}_{\text{new}}
=
\mathbf{v}_{\text{old}}
+
\left(
\hat{\mathbf{R}}_{b}^{n}\mathbf{f}
+
\mathbf{g}
\right)\Delta t
\]

### 2. Calculate the Error (Innovation)

The filter compares this predicted velocity to the actual velocity reported by the GPS. The difference is called the innovation residual:

\[
\tilde{\mathbf{v}}
=
\mathbf{v}_{GPS}
-
\hat{\mathbf{v}}_{\text{new}}
\]

If the aircraft accelerates forward along its body X-axis, but the filter's orientation matrix \(\hat{\mathbf{R}}_{b}^{n}\) is slightly misaligned, the predicted velocity vector will point in the wrong direction compared to the true GPS velocity vector.

The Kalman Gain matrix detects this directional mismatch. Because the cross-product relationships between acceleration, velocity errors, and attitude errors are hardcoded into the filter's system dynamics matrix, the velocity error mathematically forces a correction directly into the attitude (roll, pitch, and yaw) states.

## 2. The Requirement of Dynamic Motion (Observability)

Because the system relies on correlating IMU acceleration with GPS velocity changes, heading/yaw cannot be fully resolved if the aircraft is sitting perfectly still or moving in a perfectly straight line at a constant speed.

To lock down the exact 3D orientation matrix, and thus know exactly how GPS acceleration maps to the IMU axes, the aircraft must undergo dynamic maneuvers.

### Takeoff Run / Straight-Line Acceleration

When the aircraft accelerates down the runway, the sudden change in GPS velocity magnitude aligns the IMU's forward axis with the ground track. This immediately locks down the pitch and narrows down the heading.

### First Turn

Making a coordinated turn introduces lateral centripetal acceleration. Combining this cross-axis acceleration with the GPS track allows the filter's covariance to collapse, rendering the roll and heading (yaw) completely observable and highly accurate.

Once these maneuvers occur, the filter successfully "locks" the IMU's 3D orientation frame to the global GPS frame.

## 3. Lever-Arm Calibration (Physical Offset)

There is one final physical detail: the IMU and the GPS antenna are rarely in the exact same physical spot on the aircraft.

If the aircraft rolls or pitches rapidly, the GPS antenna will experience a high velocity purely because it is swinging around the aircraft's center of mass. This is called the **lever-arm effect**.

To prevent this from breaking the filter alignment, you must input the physical X, Y, and Z distances from the IMU center to the GPS antenna phase center into your EKF configuration.

The filter uses the gyroscope's angular velocity:

\[
\boldsymbol{\omega}
\]

to mathematically subtract this rotational velocity:

\[
\boldsymbol{\omega}
\times
\mathbf{r}_{\text{lever}}
\]

from the GPS readings, ensuring the velocity profiles match perfectly.

## Summary of the Data Flow

```text
 IMU Accelerometer (f)
          │
          ▼
  Rotate via R̂_bⁿ
          │
          ▼
  Navigation Acceleration
          │
      + Gravity
          │
          ▼
 Predicted Velocity (v̂)
          │
          ▼
 Compare with GPS Velocity
          │
          ▼
 Innovation Residual (ṽ)
          │
          ▼
    Kalman Gain
          │
          ▼
 Attitude Correction
 (Roll, Pitch, Yaw)
          │
          ▼
 Updated R̂_bⁿ
```

## Key Insight

In short, you do not need a hardware sensor to tell you the direction of the GPS velocity relative to the gyro.

The Kalman filter treats the relative angle as an unknown variable and forces it to converge on the truth by minimizing the difference between:

- Where the IMU says the aircraft is accelerating, and
- Where the GPS proves it is moving.

The orientation becomes observable through motion, and the filter continuously refines its estimate until both sensor systems become mathematically consistent.

If you are writing the EKF/ESKF from scratch, the next step is understanding the specific State Transition Matrix (\(F\)) blocks that couple velocity errors to attitude errors.

---------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------
A high-performance camera gimbal should absolutely use both gyroscopes. This dual-gyro architecture is the industry standard for modern professional drones and handheld stabilizers.

### ⚖️ Why One Gimbal Gyro Isn't Enough

* **The Frame Gyro (Drone):** Measures the aggressive disturbances of the drone (wind buffeting, rapid banking).
* **The IMU Gyro (Stabilized Platform/Camera):** Measures the residual error of the camera itself.

If you only use the camera gyro, the system is purely **reactive**. The camera has to actually start moving out of position before the motors realize they need to correct it, leading to a laggy, "spongy" stabilization. By using both, the system becomes **proactive**.

---

### ⚙️ How the Two Readings Interact (The Dual-Loop Control Strategy)

To control the motors in a positive, stable way, you don't just throw both readings into a single Mahony filter. Instead, you feed them into a **Cascaded (Dual-Loop) Control System**.

The two gyros feed into two different layers of control: an **Outer Position Loop** and an **Inner Velocity Loop**.

```text
[Target Angle] (e.g., Hold 0° Pitch)
       │
       ▼
┌──────────────┐
│ Outer Loop   │ ◄── [Platform Position] (From Platform Gyro + Accel run through a Mahony Filter)
│ (Position)   │
└──────┬───────┘
       │  Outputs: "Target Angular Velocity"
       ▼
┌──────────────┐
│ Feed-Forward │ ◄── [Drone Angular Velocity] (Direct from Drone Gyro)
└──────┬───────┘
       │  Combines them to anticipate the drone's harsh movements
       ▼
┌──────────────┐
│ Inner Loop   │ ◄── [Platform Angular Velocity] (Direct raw Gyro from Platform)
│ (Velocity)   │
└──────┬───────┘
       │  Outputs: Motor Torque / Current
       ▼
 [Gimbal Motor]
```

#### 1. The Inner Loop: Velocity Control (High Speed)

* **What it uses:** Raw angular velocity from the Platform Gyro.
* **How it works:** This loop runs incredibly fast (usually 2,000Hz to 4,000Hz). If a sudden gust of wind hits the drone, the platform gyro instantly feels a tiny rotational velocity. The inner PID loop immediately commands the motor to apply an opposing torque to kill that velocity.
* **Goal:** Keep the camera's angular velocity at exactly zero.

#### 2. The Feed-Forward Mix: Drone Gyro Injection (The Secret Sauce)

* **What it uses:** Raw angular velocity from the Drone Gyro.
* **How it works:** Instead of waiting for the platform to move, the gimbal controller looks directly at the drone's gyro. If the drone suddenly rolls right at 50 degrees per second, the gimbal controller instantly commands the gimbal motor to turn left at 50 degrees per second.
* **Goal:** "Anticipate" the drone's movements and cancel them out before they can even shake the camera.

#### 3. The Outer Loop: Position Control (Slower Speed)

* **What it uses:** Absolute angular position derived from the Platform Gyro + Platform Accelerometer fused together using a Mahony filter (or Madgwick filter).
* **How it works:** This loop runs slightly slower (around 500Hz). It calculates the camera's actual tilt relative to the earth. If the camera has drifted 1 degree off-horizon due to gyro drift, this loop outputs a small "target velocity" command to gently nudge the camera back to a perfect 0-degree horizon.
* **Goal:** Keep the camera perfectly level over long periods of time.

---

### 🛠️ Summary Matrix for Implementation

If you are writing firmware or configuring an open-source gimbal controller (like BaseCam SimpleBGC), map the sensors like this:

| Sensor Data | Loop Layer | What it controls | Why it's critical |
| :--- | :--- | :--- | :--- |
| **Drone Gyro** *(Velocity)* | **Feed-Forward Link** | Bypasses filters; directly scales motor output. | **Anticipation:** Cancels out 90% of the drone's violent frame movements instantly. |
| **Platform Gyro** *(Velocity)* | **Inner PID Loop** | High-frequency motor adjustments. | **Damping:** Fights micro-vibrations and aerodynamic drag on the camera itself. |
| **Platform Gyro + Accel** *(Fused Position via Mahony)* | **Outer PID Loop** | Long-term tracking and stabilization target. | **Horizon Locking:** Ensures the camera stays perfectly level and doesn't drift over time. |
----------------------------------------------
---------------
# Main Programming Architecure
Yes, use both gyro readings in the fast control task. But with encoders, the platform gyro is usually the inertial rate feedback loop; the innermost motor loop uses encoder rate and produces torque. The base gyro supplies feedforward. They can all be updated each cycle even though they serve different loops.

For one axis, a practical arrangement is:

            ωP,ref = Kθ(θref − θP)

This attitude controller requests a platform rate. Then form the relative motor-rate reference:

            ωm,ref = (ωP,ref − ωB) + Cω(ωP,ref − ωP)

Finally, the encoder-rate PID uses

            em = ωm,ref − ωm,encoder

to command motor torque or current.

The base-rate term anticipates base motion; the platform-rate term corrects any remaining inertial motion. If the platform is level and tracking well, ωP,ref and ωP are near zero, so the motor-rate reference is approximately

            ωm,ref ≈ −ωB

Cω is the gyro-rate controller. In this arrangement, it takes platform rate error and outputs an additional motor-rate correction. It could be a tuned P or PI controller; it is not another gyro reading or a universal scale factor. The encoder-rate PID then turns the resulting rate command into torque.

The quoted description gets the sensor roles broadly right, but its “platform gyro inner loop” label is misleading for your encoder-equipped gimbal: the encoder rate loop is the motor’s inner loop. Also, base-gyro feedforward does not guarantee a fixed amount of disturbance cancellation; its performance depends on the mechanics and tuning. This is consistent with the paper’s encoder inner-rate and gyro outer-rate structure.