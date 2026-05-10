FreeRTOS IMU-to-Motor Architecture Integration (V3)
This updated plan honors the requirement to do everything strictly in quaternion space. We will not convert back to degrees or use trigonometric functions (atan2, asin, etc.) during the runtime control loop.

Instead, we use the properties of the quaternion vector components!

The Quaternion Trick
A quaternion representing a rotation is defined as $q = [\cos(\theta/2), v_x \sin(\theta/2), v_y \sin(\theta/2), v_z \sin(\theta/2)]$. This means the raw $X$ and $Y$ components of $q_{delta}$ scale directly with the angle of tilt around the X (roll) and Y (pitch) axes. We can compare them directly to precomputed $\sin(\theta/2)$ threshold constants.

5° Deadzone Constant: $\sin(5^\circ / 2) \approx 0.0436$
10° Roll Steering Constant: $\sin(10^\circ / 2) \approx 0.0872$
45° Max Speed Constant: $\sin(45^\circ / 2) \approx 0.3827$
Proposed Changes
1. Core/Inc/motor_test.h
[MODIFY] Add MotorDir_t enum.
[MODIFY] Add MotorCmd_t struct (contains direction and speed for both motors).
[MODIFY] Declare extern osMessageQueueId_t motorCmdQueueHandle;.
2. Core/Src/main.c
Create the message queue and update the IMU task.

[MODIFY] Declare osMessageQueueId_t motorCmdQueueHandle;.

[MODIFY] In main(), instantiate osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);.

[MODIFY] imu_task_fn:

Startup Sequence (Before while(1)):

Initialize GPIOA PIN_5 (LED). Turn LED ON.
imu_app_stream_set(false);
imu_app_init(&hi2c1);
imu_app_ekf_reset();
imu_app_madgwick_reset();
imu_app_cal_gyro(3000);
imu_app_stream_set(true);
Capture q_ref (Neutral Pose) directly from the EKF quaternion output.
Turn LED OFF.
Math & Mapping (Inside while(1)):

Get current EKF quaternion q_now.
Compute q_delta = q_ref* ⊗ q_now.
Pitch/Speed Logic:
Read the raw $Y$ component (q_delta.y).
If abs(q_delta.y) < 0.0436: Speed is 0 (Deadzone).
If abs(q_delta.y) > 0.3827: Speed is 1000 (Max).
Else: Interpolate speed linearly between 0.0436 and 0.3827.
Direction: If q_delta.y > 0 = DIR_FORWARD, else = DIR_BACKWARD.
Roll/Steering Logic:
Read the raw $X$ component (q_delta.x).
If q_delta.x > 0.0872 (tilted right): Disable Right motor.
If q_delta.x < -0.0872 (tilted left): Disable Left motor.
Else (between -10° and 10° equivalent): Both motors run.
Push MotorCmd_t struct to the motorCmdQueueHandle.
3. Core/Src/motor_test.c
Update the motor task.

[MODIFY] motor_test_task_fn:
Block on osMessageQueueGet(motorCmdQueueHandle, &currentCmd, NULL, osWaitForever);.
Apply currentCmd.dirA and currentCmd.dirB to the IN1-IN4 GPIOs.
Apply currentCmd.speedA and currentCmd.speedB to TIM3_CH1 and TIM3_CH2 using __HAL_TIM_SET_COMPARE.