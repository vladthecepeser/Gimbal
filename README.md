"# Gimbal" 
To keep a camera perfectly level with the horizon during sustained linear acceleration (like an aircraft accelerating down a runway or holding a prolonged bank), a gyroscope alone is fundamentally insufficient.
Because of Einstein's equivalence principle, no filter—whether a complementary filter, EKF, or ESKF—can differentiate between the acceleration of gravity and the sustained linear acceleration of the aircraft using only an IMU. To solve this, you must introduce external physical references or additional sensors.
The industry-standard solutions, ranging from hardware designs to sensor fusion architectures, are outlined below.

1. Hardware Architecture: Active Gimbal with Sensor Fusion

To physically maintain the camera position relative to the horizon, you need a 3-axis active brushless gimbal paired with an Attitude and Heading Reference System (AHRS) or an Inertial Navigation System (INS).
To overcome the sustained acceleration problem, your flight controller or gimbal controller must use one of the following sensor fusion approaches:

Option A: GPS/GNSS-Aided INS (The Professional Standard)

• The Hardware: A standard 6-DOF IMU (Gyro + Accel) paired with a GPS/GNSS receiver.
• How it Works: The filter (typically an EKF) uses GPS to track the aircraft's true ground velocity. By taking the derivative of this velocity, the filter calculates the true kinematic linear acceleration (\(\mathbf{a}\)). It then mathematically subtracts this from the accelerometer's total reading (\(\mathbf{f}\)), perfectly isolating the gravity vector (\(\mathbf{g} = \mathbf{a} - \mathbf{f}\)).
• Result: Roll and pitch remain perfectly stable, completely unaffected by sustained acceleration, long turns, or changes in airspeed.

Option B: Airspeed Sensor Aiding (The Fixed-Wing Approach)

• The Hardware: A 6-DOF IMU paired with a Pitot tube (airspeed sensor).
• How it Works: In a sustained turn, an aircraft experiences continuous centripetal acceleration. By fusing the gyroscope's yaw rate (\(\omega _{z}\)) with the forward airspeed (\(v\)), the filter calculates the expected centripetal acceleration (\(a_c = \omega_z \times v\)). The filter subtracts this fictitious force from the lateral accelerometer, preventing the gimbal from leaning into the turn.

2. Software Filter Modification: Adaptive Covariance Gating

If you cannot add a GPS or Pitot tube and must rely only on an IMU, you cannot completely eliminate drift during sustained acceleration, but you can heavily mitigate it using Adaptive Measurement Covariance.
In a standard filter, the accelerometer is used to constantly "correct" the gyro's pitch and roll drift by assuming the accelerometer points straight down. During sustained acceleration, this assumption breaks.
• The Fix: Program your filter to monitor the magnitude of the gravity vector: \(\vert{}\vert{}A\vert{}\vert{} = \sqrt{a_x^2 + a_y^2 + a_z^2}\).
• If \(\vert{}\vert{}A\vert{}\vert{}\) deviates from \(1.0\text{g}\) (approx. \(9.81\text{ m/s}^2\)), or if the gyroscope registers a sustained turn, the filter dynamically increases the accelerometer measurement noise covariance (\(R\)).
• Result: The filter temporarily "blindfolds" the accelerometer and trusts only the gyroscope during the acceleration phase. Once the aircraft returns to steady flight, the filter trusts the accelerometer again to clean up any minor gyro drift.

Summary of System Recommendations


Expected Duration of Acceleration	Recommended Setup	Complexity
Short Bursts (< 5–10 seconds)	Standard IMU + Gimbal with Adaptive Covariance Gating.	Low (Software tuning only)
Sustained Turns / Orbiting	IMU + Gimbal + Airspeed Sensor (Centripetal compensation).	Medium
Heavy/Infinite Sustained Acceleration	IMU + Gimbal + GPS-Aided EKF/INS (Full kinematic isolation).	High (Requires GPS lock)

To narrow down the exact implementation, let me know:
1. What is the maximum duration of the sustained acceleration (e.g., a 10-second takeoff run vs. a 2-minute continuous banking turn)?
2. Do you have access to the aircraft's GPS data or flight controller telemetry on your camera/gimbal system?


You have hit upon the exact crux of Inertial Navigation System (INS) alignment. You are completely correct: knowing the true kinematic acceleration vector from the GPS is useless unless you know the spatial rotation (\(\mathbf{R}_{b}^{n}\)) between the IMU's body frame and the GPS's navigation frame (usually North-East-Down).
In a GPS-aided Extended Kalman Filter (EKF) or Error-State Kalman Filter (ESKF), this frame-matching is solved through mathematical observability during dynamic motion. The filter does not measure the relative direction directly; instead, it dynamically estimates the 3D orientation matrix by continuously checking for mathematical consistency between the two sensor systems.
Here is exactly how the filter resolves this alignment and extracts the gravity direction.

1. The Mathematical Mechanism: Tracking the Residuals

The filter maintains an ongoing estimate of the aircraft's orientation matrix, \(\^{\mathbf{R}}_{b}^{n}\) (Body to Navigation frame).
Every time a GPS measurement arrives, the filter performs a "What if?" prediction step:
1. Predict Velocity change: The filter takes the IMU's proper acceleration reading (\(\mathbf{f}\)), rotates it into the navigation frame using its current guess of orientation (\(\^{\mathbf{R}}_{b}^{n}\)), and adds gravity to predict what the new GPS velocity should be:
\(\^{\mathbf{v}}_{new}=\mathbf{v}_{old}+\left(\^{\mathbf{R}}_{b}^{n}\mathbf{f}+\mathbf{g}\right)\Delta t\)
2. Calculate the Error (Innovation): The filter compares this predicted velocity to the actual velocity reported by the GPS. The difference is called the innovation residual:
\(\~{\mathbf{v}}=\mathbf{v}_{GPS}-\^{\mathbf{v}}_{new}\)
If the aircraft accelerates forward along its body X-axis, but the filter's orientation matrix \(\^{\mathbf{R}}_{b}^{n}\) is slightly misaligned, the predicted velocity vector will point in the wrong direction compared to the true GPS velocity vector.
The Kalman Gain matrix detects this directional mismatch. Because the cross-product relationships between acceleration, velocity errors, and attitude errors are hardcoded into the filter's system dynamics matrix, the velocity error mathematically forces a correction directly into the attitude (roll, pitch, and yaw) states.

2. The Requirement of Dynamic Motion (Observability)

Because the system relies on correlating IMU acceleration with GPS velocity changes, heading/yaw cannot be fully resolved if the aircraft is sitting perfectly still or moving in a perfectly straight line at a constant speed.
To lock down the exact 3D orientation matrix—and thus know exactly how GPS acceleration maps to the IMU axes—the aircraft must undergo dynamic maneuvers.
• The Takeoff Run / Straight-Line Acceleration: When the aircraft accelerates down the runway, the sudden change in GPS velocity magnitude aligns the IMU's forward axis with the ground track. This immediately locks down the pitch and narrows down the heading.
• The First Turn: Making a coordinated turn introduces lateral centripetal acceleration. Combining this cross-axis acceleration with the GPS track allows the filter's covariance to collapse, rendering the roll and heading (yaw) completely observable and highly accurate.
Once these maneuvers occur, the filter successfully "locks" the IMU's 3D orientation frame to the global GPS frame.

3. Lever-Arm Calibration (Physical Offset)

There is one final physical detail: the IMU and the GPS antenna are rarely in the exact same physical spot on the aircraft.
If the aircraft rolls or pitches rapidly, the GPS antenna will experience a high velocity purely because it is swinging around the aircraft's center of mass (called the lever-arm effect).
To prevent this from breaking the filter alignment, you must input the physical X, Y, and Z distances from the IMU center to the GPS antenna phase center into your EKF configuration. The filter uses the gyroscope's angular velocity (\(\mathbfit{\omega }\)) to mathematically subtract this rotational velocity (\(\boldsymbol{\omega} \times \mathbf{r}_{\text{lever}}\)) from the GPS readings, ensuring the velocity profiles match perfectly.

Summary of the Data Flow

Прикажи кôд
Преузми SVG дијаграма
In short, you don't need a hardware sensor to tell you the direction of the GPS velocity relative to the gyro. The Kalman filter treats the relative angle as an unknown variable and forces it to converge on the truth by minimizing the difference between where the IMU says the aircraft is accelerating and where the GPS proves it is moving.
Are you planning on using an off-the-shelf autopilot (like Pixhawk/ArduPilot or PX4) which has this GPS-aided EKF entirely built-in, or are you looking to write this EKF/ESKF code from scratch?
If you are writing it from scratch, I can outline the specific State Transition Matrix (\(F\)) blocks that couple the velocity errors to the attitude errors.