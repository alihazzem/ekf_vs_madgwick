#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "utils/timebase.h"
#include "filters/madgwick.h"
#include "filters/ekf.h"
#include "utils/math3d.h"
#include "app/imu_types.h"
#include "app/app_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"

#if SENSOR_GY91
#include "drivers/mpu9255.h"
#include "drivers/ak8963.h"
#else
#include "drivers/mpu6050.h"
#endif

#include <string.h>
#include <math.h>   /* sqrtf for mag norm */
#include <stdio.h>  /* snprintf for log queue entries */
#include <stdlib.h> /* abs() for stillness detection */

static I2C_HandleTypeDef *s_hi2c = NULL;

SemaphoreHandle_t g_i2c_mutex = NULL;

/* Log queue — defined here, created in main.c before the scheduler starts */
osMessageQueueId_t logQueueHandle = NULL;

static volatile bool s_stream_en = false;
static uint32_t s_print_div = 1; // 200Hz default

static uint32_t s_tick_count = 0;
static uint32_t s_sample_count = 0;
static uint32_t s_missed = 0;

#if SENSOR_GY91
static mpu9255_raw_t s_last[NUM_IMUS];
static ak8963_raw_t s_last_mag[NUM_IMUS];
static ak8963_cfg_t s_ak_cfg[NUM_IMUS];
static float s_mx_ut[NUM_IMUS] = {0};
static float s_my_ut[NUM_IMUS] = {0};
static float s_mz_ut[NUM_IMUS] = {0};
static float s_m_norm[NUM_IMUS] = {0};
static uint8_t s_mag_body_valid[NUM_IMUS] = {0};
static float s_mx_cal_ut[NUM_IMUS] = {0};
static float s_my_cal_ut[NUM_IMUS] = {0};
static float s_mz_cal_ut[NUM_IMUS] = {0};
static float s_m_cal_norm[NUM_IMUS] = {0};
static uint8_t s_mag_cal_valid[NUM_IMUS] = {0};
static volatile bool s_mag_stream_en = false;
#else
static mpu6050_raw_t s_last[NUM_IMUS];
#endif

// Raw accel (body frame, in g)
static float s_ax_g[NUM_IMUS] = {0};
static float s_ay_g[NUM_IMUS] = {0};
static float s_az_g[NUM_IMUS] = {0};

// time + dt stats
static uint32_t s_reset_ms = 0;
static uint32_t s_last_sample_cyc = 0; // store cycles (wrap-safe)
static uint32_t s_dt_min_us = 0xFFFFFFFFu;
static uint32_t s_dt_max_us = 0;
static uint64_t s_dt_sum_us = 0;
static uint32_t s_svc_last_us = 0;
static uint32_t s_svc_max_us = 0;
static uint32_t s_last_miss_tick = 0;

// rate window (~1s) using cycles
static uint32_t s_win_start_cyc = 0;
static uint32_t s_win_samples = 0;

// avoid float printf issues: store milli-Hz (100000 = 100.000 Hz)
static uint32_t s_last_rate_mhz = 0;

// Madgwick state/output
static madgwick_t s_mad[NUM_IMUS];
static Attitude_t s_mad_att[NUM_IMUS];
static uint8_t s_mad_valid[NUM_IMUS] = {0};
static uint8_t s_mad_aligned[NUM_IMUS] = {0};  // set to 1 after first-sample gravity alignment
static uint32_t s_mad_last_us[NUM_IMUS] = {0}; // CPU time of last Madgwick step

// EKF state/output
static ekf7_t s_ekf[NUM_IMUS];
static Attitude_t s_ekf_att[NUM_IMUS];
static uint8_t s_ekf_valid[NUM_IMUS] = {0};
static uint8_t s_ekf_aligned[NUM_IMUS] = {0};  // set to 1 after first-sample gravity alignment
static uint32_t s_ekf_last_us[NUM_IMUS] = {0}; // CPU time of last EKF step

// --- Gyro raw offsets ---
static int16_t s_gx_off[NUM_IMUS] = {0};
static int16_t s_gy_off[NUM_IMUS] = {0};
static int16_t s_gz_off[NUM_IMUS] = {0};

// --- Per-IMU sensor scale factors (set at init, different per IMU in LEG mode) ---
static float s_gyro_lsb_per_dps[NUM_IMUS];
static float s_acc_lsb_per_g[NUM_IMUS];

// --- Gait Analysis Variables ---
static int s_gait_phase = 0; // 0 = STANCE, 1 = SWING
static float s_last_hs_s = 0.0f;
static float s_last_to_s = 0.0f;
static float s_gyro_sma = 0.0f;
static float s_current_knee_angle = 0.0f;
static float s_current_thigh_pitch = 0.0f;
static float s_current_shin_pitch = 0.0f;
static float s_swing_max_knee = 0.0f;
static uint8_t s_mid_swing_armed = 0;
static float s_stride_time = 0.0f;
static int s_event_flag = 0;

static float s_tare_thigh_offset = 0.0f;
static float s_tare_shin_offset = 0.0f;
static float s_tare_knee_offset = 0.0f;

/* Sensor-to-Bone Alignment Quaternions */
static float s_q_ref[NUM_IMUS][4] = { {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f} };

// Helper to extract Pitch purely from the Gravity Vector (Immune to Yaw!)
static float imu_app_get_gravity_pitch(const float q[4]) {
    // Extract the Earth's vertical Z-axis (Gravity) mapped into the Sensor frame
    float gx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    // Pitch is the tilt of the gravity vector in the X-Z plane
    return atan2f(-gx, gz) * (180.0f / (float)M_PI);
}

void imu_app_gait_tare(void)
{
    // Capture the exact pitch of the gravity vector at this millisecond
    // to compensate for physical velcro strap misalignments.
    s_tare_thigh_offset = imu_app_get_gravity_pitch(s_ekf[0].q);
    s_tare_shin_offset  = imu_app_get_gravity_pitch(s_ekf[1].q);
    s_tare_knee_offset  = 0.0f;

    /* Capture the Bone Frame alignment quaternions */
    for (int i = 0; i < NUM_IMUS; i++) {
        s_q_ref[i][0] = s_ekf[i].q[0];
        s_q_ref[i][1] = s_ekf[i].q[1];
        s_q_ref[i][2] = s_ekf[i].q[2];
        s_q_ref[i][3] = s_ekf[i].q[3];
    }
}

void imu_app_get_leg_angles(float *thigh_pitch, float *shin_pitch, float *knee_angle)
{
    if (thigh_pitch) *thigh_pitch = s_current_thigh_pitch;
    if (shin_pitch) *shin_pitch = s_current_shin_pitch;
    if (knee_angle) *knee_angle = s_current_knee_angle;
}



// --- Accel raw offsets (per-IMU, initialised from app_config.h defaults) ---
static int16_t s_ax_off[NUM_IMUS] = {0};
static int16_t s_ay_off[NUM_IMUS] = {0};
static int16_t s_az_off[NUM_IMUS] = {0};

static const float DEG2RAD = 0.017453292519943295f;

#if ROBOT_MODE == ROBOT_MODE_LEG
UART_HandleTypeDef huart6_esp;

#pragma pack(push, 1)
typedef struct {
    uint8_t header1;           // 0xAA
    uint8_t header2;           // 0x55
    int16_t thigh_angle_deg100;
    int16_t shin_angle_deg100;
    int16_t knee_angle_deg100;
    uint8_t checksum;
} EspTelemetry_t;
#pragma pack(pop)

static void imu_app_usart6_init(void)
{
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA11 -> USART6_TX
    // (RX Disabled to prevent floating pin noise)
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart6_esp.Instance = USART6;
    huart6_esp.Init.BaudRate = 115200;
    huart6_esp.Init.WordLength = UART_WORDLENGTH_8B;
    huart6_esp.Init.StopBits = UART_STOPBITS_1;
    huart6_esp.Init.Parity = UART_PARITY_NONE;
    huart6_esp.Init.Mode = UART_MODE_TX;
    huart6_esp.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6_esp.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6_esp);
}
#endif

void imu_app_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  imu_app_stats_reset();

#if ROBOT_MODE == ROBOT_MODE_LEG
  imu_app_usart6_init();
#endif

  if (g_i2c_mutex == NULL)
  {
    g_i2c_mutex = xSemaphoreCreateMutex();
  }

  for (int i = 0; i < NUM_IMUS; i++)
  {
#if SENSOR_GY91
    /* ---- GY-91: MPU-9255 + AK8963 bring-up ---- */
    {
      mpu9255_cfg_t mpu_cfg;
      uint8_t addr = (i == 0) ? MPU6050_ADDR_0 : MPU6050_ADDR_1;
      mpu9255_status_t st = mpu9255_init_200hz(hi2c, addr, &mpu_cfg);
      if (st == MPU9255_OK)
      {
        /* Enable bypass so STM32 can reach AK8963 on the same bus */
        mpu9255_enable_bypass(hi2c, addr);

        /* Read sensitivity adjustment from AK8963 fuse ROM */
        if (ak8963_read_asa(hi2c, &s_ak_cfg[i]) == AK8963_OK)
        {
          /* Start continuous 100 Hz 16-bit measurement */
          ak8963_init_continuous_100hz(hi2c);
        }
      }
      /* Zero-initialise last mag sample — valid=0 until first read */
      s_last_mag[i].mx = 0;
      s_last_mag[i].my = 0;
      s_last_mag[i].mz = 0;
      s_last_mag[i].valid = 0;
    }
#else
    {
      mpu6050_cfg_t mpu_cfg;
      uint8_t addr = (i == 0) ? MPU6050_ADDR_0 : MPU6050_ADDR_1;

#if ROBOT_MODE == ROBOT_MODE_LEG
      /* LEG MODE: configure each IMU independently for its body segment */
      static const mpu6050_hw_cfg_t s_leg_hw[2] = {
          /* IMU 0 — Thigh: ±500dps, ±4g, 42Hz DLPF (slow segment, high precision) */
          { LEG_IMU0_SMPLRT_DIV, LEG_IMU0_DLPF_CFG, LEG_IMU0_GYRO_FS, LEG_IMU0_ACCEL_FS,
            LEG_IMU0_GYRO_LSB_PER_DPS, LEG_IMU0_ACC_LSB_PER_G },
          /* IMU 1 — Shin:  ±500dps, ±8g, 98Hz DLPF (faster, survives impacts) */
          { LEG_IMU1_SMPLRT_DIV, LEG_IMU1_DLPF_CFG, LEG_IMU1_GYRO_FS, LEG_IMU1_ACCEL_FS,
            LEG_IMU1_GYRO_LSB_PER_DPS, LEG_IMU1_ACC_LSB_PER_G },
      };
      s_gyro_lsb_per_dps[i] = s_leg_hw[i].gyro_lsb_per_dps;
      s_acc_lsb_per_g[i]    = s_leg_hw[i].acc_lsb_per_g;
      mpu6050_status_t st = mpu6050_init_custom(hi2c, addr, &s_leg_hw[i], &mpu_cfg);
#else
      /* ARM MODE: both IMUs use the original shared config — unchanged */
      s_gyro_lsb_per_dps[i] = 32.8f;   /* ±1000 dps */
      s_acc_lsb_per_g[i]    = 4096.0f; /* ±8 g       */
      mpu6050_status_t st = mpu6050_init_200hz(hi2c, addr, &mpu_cfg);
#endif

      if (st != MPU6050_OK)
      {
        uart_cli_sendf("Failed to init MPU6050 at 0x%02X (err=%d)\r\n", addr, (int)st);
      }
    }
#endif /* SENSOR_GY91 */

    // Init per-IMU accel bias from compile-time defaults
    if (i == 0)
    {
      s_ax_off[i] = ACCEL_BIAS_X_0;
      s_ay_off[i] = ACCEL_BIAS_Y_0;
      s_az_off[i] = ACCEL_BIAS_Z_0;
    }
    else
    {
      s_ax_off[i] = ACCEL_BIAS_X_1;
      s_ay_off[i] = ACCEL_BIAS_Y_1;
      s_az_off[i] = ACCEL_BIAS_Z_1;
    }

    // Madgwick init — all parameters sourced from app_config.h inside madgwick_init().
    // The individual setter calls that used to follow here are now redundant.
    madgwick_init(&s_mad[i], MADGWICK_BETA);

    s_mad_valid[i]   = 0;
    s_mad_aligned[i] = 0;

#if RUN_EKF
    // EKF init — all parameters (including accel-reject window) sourced from
    // app_config.h inside ekf7_init().  The ekf7_set_accel_reject() call that
    // used to follow here is now redundant.
    ekf7_init(&s_ekf[i], EKF_SIGMA_GYRO, EKF_SIGMA_BIAS, EKF_SIGMA_ACCEL, EKF_SIGMA_MAG,
              EKF_R_ADAPT_K, EKF_P0);
    s_ekf_valid[i]   = 0;
    s_ekf_aligned[i] = 0;
#endif
  }
}

void imu_app_add_missed(uint32_t count)
{
  s_missed += count;
  s_last_miss_tick = s_tick_count; // Approximation
}

void imu_app_step(void)
{
    // Smart Auto-Tare: Wait for the EKF to fully converge, then count down 5 seconds.
    // This guarantees you always have exactly 5 seconds to get into position AFTER the EKF stabilizes.
    static bool  s_auto_tare_done    = false;
    static float s_last_thigh_sample = -9999.0f;
    static float s_last_shin_sample  = -9999.0f;
    static uint32_t s_stable_start_ms = 0;
    
    if (!s_auto_tare_done && s_ekf_valid[0] && s_ekf_valid[1]) {
        float cur_thigh = imu_app_get_gravity_pitch(s_ekf[0].q);
        float cur_shin  = imu_app_get_gravity_pitch(s_ekf[1].q);
        
        float thigh_delta = cur_thigh - s_last_thigh_sample;
        float shin_delta  = cur_shin  - s_last_shin_sample;
        if (thigh_delta < 0.0f) thigh_delta = -thigh_delta;
        if (shin_delta  < 0.0f) shin_delta  = -shin_delta;
        
        s_last_thigh_sample = cur_thigh;
        s_last_shin_sample  = cur_shin;
        
        // EKF is considered converged when angles are stable (changing < 0.5 deg/tick)
        bool is_stable = (thigh_delta < 0.5f) && (shin_delta < 0.5f) && (s_last_thigh_sample != -9999.0f);
        
        uint32_t now_ms = HAL_GetTick();
        if (!is_stable) {
            s_stable_start_ms = now_ms; // Reset timer every time the EKF is still bouncing
        } else if ((now_ms - s_stable_start_ms) > 5000) { // 5 seconds of stable = ready to tare
            imu_app_gait_tare();
            s_auto_tare_done = true;
        }
    }

  s_tick_count++;

  if (!s_hi2c)
    return;

  // capture timestamp for this sample (used for dt + rate window)
  uint32_t now_cyc = timebase_cycles();
  uint32_t prev_cyc = s_last_sample_cyc;

  // measure service time around the I2C read
  uint32_t t0 = now_cyc;

  // read IMU (accel + gyro)
  xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  bool imu_ok[NUM_IMUS];
  for (int i = 0; i < NUM_IMUS; i++)
  {
    uint8_t addr = (i == 0) ? MPU6050_ADDR_0 : MPU6050_ADDR_1;
    imu_ok[i] = true;
#if SENSOR_GY91
    if (mpu9255_read_raw(s_hi2c, addr, &s_last[i]) == MPU9255_OK)
    {
      /* Best-effort AK8963 read — if DRDY=0, s_last_mag.valid stays 0
       * and the previous sample is preserved unchanged.
       *
       * NOTE: Two GY-91 modules on the same I2C bus will have AK8963
       *       address collision at 0x0C.  For dual-GY91 operation the
       *       second AK8963 must either use the MPU's aux I2C master
       *       or an external I2C mux / address translator.           */
      ak8963_read_raw(s_hi2c, &s_last_mag[i]);
    }
    else
    {
      imu_ok[i] = false;
    }
#else
    if (mpu6050_read_raw(s_hi2c, addr, &s_last[i]) != MPU6050_OK)
    {
      imu_ok[i] = false;
    }
#endif /* SENSOR_GY91 */
  }
  xSemaphoreGive(g_i2c_mutex);

  bool any_ok = false;
  for (int i = 0; i < NUM_IMUS; i++)
    if (imu_ok[i])
      any_ok = true;

  if (any_ok)
  {

    // ---- compute dt (seconds) from last successful sample ----
    float dt_s = 0.0f;
    if (prev_cyc != 0u)
    {
      uint32_t dt_cyc = now_cyc - prev_cyc;

      /* Use full 84MHz resolution (11.9ns) for the filter integration time.
       * This avoids the ~1us rounding error inherent in timebase_cycles_to_us(). */
      dt_s = (float)dt_cyc / 84000000.0f;

      // Keep integer dt_us for stats and display
      uint32_t dt_us = timebase_cycles_to_us(dt_cyc);

      // dt stats
      if (dt_us < s_dt_min_us)
        s_dt_min_us = dt_us;
      if (dt_us > s_dt_max_us)
        s_dt_max_us = dt_us;
      s_dt_sum_us += dt_us;
    }

    // update last sample timestamp (cycles)
    s_last_sample_cyc = now_cyc;

    // sample counter
    s_sample_count++;

    // ---- rate window (~1 second) ----
    if (s_win_start_cyc == 0)
      s_win_start_cyc = now_cyc;
    s_win_samples++;

    uint32_t win_cyc = now_cyc - s_win_start_cyc;
    uint32_t win_us = timebase_cycles_to_us(win_cyc);

    if (win_us >= 1000000u)
    {
      s_last_rate_mhz = (uint32_t)(((uint64_t)s_win_samples * 1000000000ull) / (uint64_t)win_us);
      s_win_start_cyc = now_cyc;
      s_win_samples = 0;
    }

    for (int i = 0; i < NUM_IMUS; i++)
    {
      if (!imu_ok[i])
        continue;

      // Sensor-frame accel (g) — bias-corrected then scaled with per-IMU range
      float ax_s = (float)(s_last[i].ax - s_ax_off[i]) / s_acc_lsb_per_g[i];
      float ay_s = (float)(s_last[i].ay - s_ay_off[i]) / s_acc_lsb_per_g[i];
      float az_s = (float)(s_last[i].az - s_az_off[i]) / s_acc_lsb_per_g[i];

      // Sensor-frame gyro (rad/s) — scaled with per-IMU range
      int16_t gx_corr = s_last[i].gx - s_gx_off[i];
      int16_t gy_corr = s_last[i].gy - s_gy_off[i];
      int16_t gz_corr = s_last[i].gz - s_gz_off[i];

      float wx_s = ((float)gx_corr / s_gyro_lsb_per_dps[i]) * DEG2RAD;
      float wy_s = ((float)gy_corr / s_gyro_lsb_per_dps[i]) * DEG2RAD;
      float wz_s = ((float)gz_corr / s_gyro_lsb_per_dps[i]) * DEG2RAD;

      // ---- REMAP sensor -> body (defined in app_config.h) ----
      float ax_g = REMAP_AX_G(ax_s, ay_s, az_s);
      float ay_g = REMAP_AY_G(ax_s, ay_s, az_s);
      float az_g = REMAP_AZ_G(ax_s, ay_s, az_s);

      taskENTER_CRITICAL();
      s_ax_g[i] = ax_g;
      s_ay_g[i] = ay_g;
      s_az_g[i] = az_g;
      taskEXIT_CRITICAL();

      float wx = REMAP_WX(wx_s, wy_s, wz_s);
      float wy = REMAP_WY(wx_s, wy_s, wz_s);
      float wz = REMAP_WZ(wx_s, wy_s, wz_s);

#if SENSOR_GY91
      // ---- AK8963 → body-frame µT (remap + ASA + scale) ----
      if (s_last_mag[i].valid)
      {
        float mx_s = (float)REMAP_MX(s_last_mag[i].mx, s_last_mag[i].my, s_last_mag[i].mz);
        float my_s = (float)REMAP_MY(s_last_mag[i].mx, s_last_mag[i].my, s_last_mag[i].mz);
        float mz_s = (float)REMAP_MZ(s_last_mag[i].mx, s_last_mag[i].my, s_last_mag[i].mz);

        s_mx_ut[i] = mx_s * s_ak_cfg[i].asa_x * AK8963_UT_PER_LSB;
        s_my_ut[i] = my_s * s_ak_cfg[i].asa_y * AK8963_UT_PER_LSB;
        s_mz_ut[i] = mz_s * s_ak_cfg[i].asa_z * AK8963_UT_PER_LSB;

        float n2 = s_mx_ut[i] * s_mx_ut[i] + s_my_ut[i] * s_my_ut[i] + s_mz_ut[i] * s_mz_ut[i];
        s_m_norm[i] = (n2 > 0.0f) ? sqrtf(n2) : 0.0f;
        s_mag_body_valid[i] = (s_m_norm[i] >= 5.0f && s_m_norm[i] <= 100.0f) ? 1u : 0u;

        // ---- Apply hard/soft-iron calibration (from app_config.h) ----
        // TODO: When SENSOR_GY91 && NUM_IMUS > 1, each AK8963 needs independent
        //       hard/soft-iron calibration (MAG_CAL_* per IMU).  Currently the
        //       same calibration is applied to all IMUs.
        float bx = s_mx_ut[i] - MAG_CAL_BX;
        float by = s_my_ut[i] - MAG_CAL_BY;
        float bz = s_mz_ut[i] - MAG_CAL_BZ;
        s_mx_cal_ut[i] = MAG_CAL_S11 * bx + MAG_CAL_S12 * by + MAG_CAL_S13 * bz;
        s_my_cal_ut[i] = MAG_CAL_S21 * bx + MAG_CAL_S22 * by + MAG_CAL_S23 * bz;
        s_mz_cal_ut[i] = MAG_CAL_S31 * bx + MAG_CAL_S32 * by + MAG_CAL_S33 * bz;
        float cn2 = s_mx_cal_ut[i] * s_mx_cal_ut[i] + s_my_cal_ut[i] * s_my_cal_ut[i] + s_mz_cal_ut[i] * s_mz_cal_ut[i];
        s_m_cal_norm[i] = (cn2 > 0.0f) ? sqrtf(cn2) : 0.0f;
        s_mag_cal_valid[i] = (s_m_cal_norm[i] >= 5.0f && s_m_cal_norm[i] <= 100.0f) ? 1u : 0u;

        // ---- Emit M, line for offline calibration capture ----
        if (s_mag_stream_en && i == 0) // Only stream first IMU for calibration
        {
          /* Emit body-frame (uncalibrated) µT × 100 as integers:
           * the Python script divides by 100 to recover float µT. */
          int32_t mx_100 = (int32_t)(s_mx_ut[i] * 100.0f);
          int32_t my_100 = (int32_t)(s_my_ut[i] * 100.0f);
          int32_t mz_100 = (int32_t)(s_mz_ut[i] * 100.0f);
          uart_cli_sendf("M,%ld,%ld,%ld\r\n",
                         (long)mx_100, (long)my_100, (long)mz_100);
        }
      }
#endif /* SENSOR_GY91 */

      // On the very first valid sample initialise each filter's quaternion from
      // the accelerometer so roll/pitch are correct immediately.
#if FAULT_INJECT_IDENTITY_INIT
      if (!s_mad_aligned[i])
      {
        madgwick_reset(&s_mad[i]);
        s_mad_aligned[i] = 1;
      }
#else
      if (!s_mad_aligned[i])
      {
#if SENSOR_GY91
        if (s_mag_cal_valid[i])
        {
          madgwick_init_from_marg(&s_mad[i], ax_g, ay_g, az_g, s_mx_cal_ut[i], s_my_cal_ut[i], s_mz_cal_ut[i]);
        }
        else
        {
          madgwick_init_from_accel(&s_mad[i], ax_g, ay_g, az_g);
        }
#else
        madgwick_init_from_accel(&s_mad[i], ax_g, ay_g, az_g);
#endif
        s_mad_aligned[i] = 1;
      }
#endif
#if RUN_EKF
#if FAULT_INJECT_IDENTITY_INIT
      if (!s_ekf_aligned[i])
      {
        ekf7_reset(&s_ekf[i]);
        s_ekf_aligned[i] = 1;
      }
#else
      if (!s_ekf_aligned[i])
      {
#if SENSOR_GY91
        if (s_mag_cal_valid[i])
        {
          ekf7_init_from_marg(&s_ekf[i], ax_g, ay_g, az_g, s_mx_cal_ut[i], s_my_cal_ut[i], s_mz_cal_ut[i]);
        }
        else
        {
          ekf7_init_from_accel(&s_ekf[i], ax_g, ay_g, az_g);
        }
#else
        ekf7_init_from_accel(&s_ekf[i], ax_g, ay_g, az_g);
#endif
        s_ekf_aligned[i] = 1;
      }
#endif
#endif

      // ---- Madgwick update (body-frame inputs, measured dt) ----
#if RUN_MADGWICK
      if (dt_s > 0.0f)
      {
        uint32_t t_mad0 = timebase_cycles();
#if SENSOR_GY91
        // Safety threshold: Only use mag if DRDY is set AND the calibrated norm
        // is roughly Earth-like (5 to 100 uT).
        // Checking s_last_mag[i].valid (DRDY flag) prevents feeding stale samples
        // from a previous cycle when the magnetometer hasn't refreshed yet.
        if (s_last_mag[i].valid && s_m_cal_norm[i] > 5.0f && s_m_cal_norm[i] < 100.0f)
        {
          madgwick_update_marg(&s_mad[i], wx, wy, wz, ax_g, ay_g, az_g, s_mx_cal_ut[i], s_my_cal_ut[i], s_mz_cal_ut[i], dt_s);
        }
        else
        {
          madgwick_update_imu(&s_mad[i], wx, wy, wz, ax_g, ay_g, az_g, dt_s);
        }
#else
        madgwick_update_imu(&s_mad[i], wx, wy, wz, ax_g, ay_g, az_g, dt_s);
#endif
        s_mad_last_us[i] = timebase_cycles_to_us(timebase_cycles() - t_mad0);

        s_mad_att[i].q0 = s_mad[i].q0;
        s_mad_att[i].q1 = s_mad[i].q1;
        s_mad_att[i].q2 = s_mad[i].q2;
        s_mad_att[i].q3 = s_mad[i].q3;

        math3d_quat_to_euler_deg(s_mad[i].q0, s_mad[i].q1, s_mad[i].q2, s_mad[i].q3,
                                 &s_mad_att[i].roll_deg, &s_mad_att[i].pitch_deg, &s_mad_att[i].yaw_deg);
        s_mad_valid[i] = 1;
      }
#else
      (void)dt_s; // avoid unused warning when compiling without Madgwick
#endif

      // ---- EKF update (body-frame inputs, measured dt) ----
#if RUN_EKF
      if (dt_s > 0.0f)
      {
        uint32_t t_ekf0 = timebase_cycles();
#if SENSOR_GY91
        // Safety threshold: Only use mag if the calibrated norm is roughly Earth-like (5 to 100 uT).
        // This protects the robot from spinning out if it drives over a strong magnet.
        if (s_last_mag[i].valid && s_mag_cal_valid[i])
        {
          ekf7_step_marg(&s_ekf[i], wx, wy, wz, ax_g, ay_g, az_g, s_mx_cal_ut[i], s_my_cal_ut[i], s_mz_cal_ut[i], dt_s);
        }
        else
        {
          ekf7_step(&s_ekf[i], wx, wy, wz, ax_g, ay_g, az_g, dt_s);
        }
#else
        ekf7_step(&s_ekf[i], wx, wy, wz, ax_g, ay_g, az_g, dt_s);
#endif
        s_ekf_last_us[i] = timebase_cycles_to_us(timebase_cycles() - t_ekf0);

        float q_tmp[4];
        ekf7_get_quat(&s_ekf[i], q_tmp);
        s_ekf_att[i].q0 = q_tmp[0];
        s_ekf_att[i].q1 = q_tmp[1];
        s_ekf_att[i].q2 = q_tmp[2];
        s_ekf_att[i].q3 = q_tmp[3];
        math3d_quat_to_euler_deg(s_ekf_att[i].q0, s_ekf_att[i].q1, s_ekf_att[i].q2, s_ekf_att[i].q3,
                                 &s_ekf_att[i].roll_deg, &s_ekf_att[i].pitch_deg, &s_ekf_att[i].yaw_deg);
        s_ekf_valid[i] = 1;
      }
#endif
    } // End of IMU loop

    // ---- Gait Analysis & Unified Streaming ----
    if (s_print_div != 0 && (s_sample_count % s_print_div) == 0)
    {
      uint32_t t_now = timebase_cycles_to_us(timebase_cycles());
      float t_sec = (float)t_now / 1000000.0f;

      // Extract mapped Euler angles
      if (NUM_IMUS >= 2 && s_ekf_valid[0] && s_ekf_valid[1]) {
          // 1. Calculate Signed Pitch Angles purely from Gravity Vector
          // Because gravity always points down, this is 100% immune to 180-degree Yaw spins!
          float raw_thigh_pitch = imu_app_get_gravity_pitch(s_ekf[0].q);
          float raw_shin_pitch  = imu_app_get_gravity_pitch(s_ekf[1].q);
          
          float t_pitch = -(raw_thigh_pitch - s_tare_thigh_offset);
          float s_pitch = -(raw_shin_pitch - s_tare_shin_offset);
          
          // Apply an Exponential Moving Average (Low-Pass Filter) to eliminate physical jitter
          static float s_smooth_thigh = 0.0f;
          static float s_smooth_shin = 0.0f;
          static bool s_filter_init = false;
          
          if (!s_filter_init) {
              s_smooth_thigh = t_pitch;
              s_smooth_shin  = s_pitch;
              s_filter_init  = true;
          } else {
              float alpha = 0.08f; // Lowered for maximum smoothness (was 0.15)
              s_smooth_thigh = (s_smooth_thigh * (1.0f - alpha)) + (t_pitch * alpha);
              s_smooth_shin  = (s_smooth_shin * (1.0f - alpha))  + (s_pitch * alpha);
          }
          
          // Quantize to nearest 0.1 degrees, then apply deadband:
          // Only push the new value to output if it has moved >= 0.1 deg.
          // This locks the output solid when holding still, eliminating micro-flicker.
          float q_thigh = roundf(s_smooth_thigh * 10.0f) / 10.0f;
          float q_shin  = roundf(s_smooth_shin  * 10.0f) / 10.0f;
          if (fabsf(q_thigh - s_current_thigh_pitch) >= 0.1f)
              s_current_thigh_pitch = q_thigh;
          if (fabsf(q_shin - s_current_shin_pitch) >= 0.1f)
              s_current_shin_pitch = q_shin;
          
          // 2. Knee Angle is exactly Thigh - Shin
          s_current_knee_angle = s_current_thigh_pitch - s_current_shin_pitch;
          
          if (s_current_knee_angle < 0.0f) s_current_knee_angle = 0.0f;

          // Run State Machine
          float gyroY = (float)s_last[1].gy; // Shin Gyro Y
          s_gyro_sma = s_gyro_sma * 0.8f + gyroY * 0.2f;
          s_event_flag = 0;

          if (s_gait_phase == 0) { // STANCE
              if ((t_sec - s_last_hs_s) > 0.300f) {
                  if (s_current_knee_angle > 15.0f && s_gyro_sma > 1000.0f) {
                      s_gait_phase = 1; // SWING
                      s_last_to_s = t_sec;
                      s_event_flag = 2; // TO event
                      s_swing_max_knee = s_current_knee_angle;
                  }
              }
          } else { // SWING
              if (s_current_knee_angle > s_swing_max_knee) s_swing_max_knee = s_current_knee_angle;

              if (!s_mid_swing_armed && (t_sec - s_last_to_s) > 0.300f && s_current_knee_angle < (s_swing_max_knee * 0.8f)) {
                  s_mid_swing_armed = 1;
                  s_event_flag = 3; // MidSwing Armed
              }

              if (s_mid_swing_armed && s_current_knee_angle < 25.0f && s_gyro_sma < 0.0f) {
                  s_gait_phase = 0; // STANCE
                  s_mid_swing_armed = 0;
                  s_event_flag = 1; // HS event
                  s_stride_time = t_sec - s_last_hs_s;
                  s_last_hs_s = t_sec;
              }
          }
      }

      if (s_stream_en && logQueueHandle != NULL)
      {
          LogLine_t entry;
#if NUM_IMUS >= 2
          snprintf(entry.line, sizeof(entry.line),
                   "%lu,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d\r\n",
                   (unsigned long)t_now,
                   (int)s_last[0].ax, (int)s_last[0].ay, (int)s_last[0].az,
                   (int)s_last[0].gx, (int)s_last[0].gy, (int)s_last[0].gz,
                   (long)(s_ekf[0].q[0]*10000), (long)(s_ekf[0].q[1]*10000), (long)(s_ekf[0].q[2]*10000), (long)(s_ekf[0].q[3]*10000),
                   (int)s_last[1].ax, (int)s_last[1].ay, (int)s_last[1].az,
                   (int)s_last[1].gx, (int)s_last[1].gy, (int)s_last[1].gz,
                   (long)(s_ekf[1].q[0]*10000), (long)(s_ekf[1].q[1]*10000), (long)(s_ekf[1].q[2]*10000), (long)(s_ekf[1].q[3]*10000),
                   (int)(s_current_thigh_pitch * 100.0f), (int)(s_current_shin_pitch * 100.0f), (int)(s_current_knee_angle * 100.0f),
                   s_gait_phase, s_event_flag, (int)(s_stride_time * 1000.0f));
          osMessageQueuePut(logQueueHandle, &entry, 0U, 0U);
#else
          for (int i = 0; i < NUM_IMUS; i++) {
              snprintf(entry.line, sizeof(entry.line),
                       "%lu,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%lu\r\n",
                       (unsigned long)t_now, i,
                       (int)s_last[i].ax, (int)s_last[i].ay, (int)s_last[i].az,
                       (int)s_last[i].gx, (int)s_last[i].gy, (int)s_last[i].gz,
                       (long)(s_ekf[i].q[0]*10000), (long)(s_ekf[i].q[1]*10000), (long)(s_ekf[i].q[2]*10000), (long)(s_ekf[i].q[3]*10000),
                       (unsigned long)timebase_cycles_to_us(s_svc_last_us));
              osMessageQueuePut(logQueueHandle, &entry, 0U, 0U);
          }
#endif
      }
      
#if ROBOT_MODE == ROBOT_MODE_LEG
      EspTelemetry_t pkt;
      pkt.header1 = 0xAA;
      pkt.header2 = 0x55;
      pkt.thigh_angle_deg100 = (int16_t)(s_current_thigh_pitch * 100.0f);
      pkt.shin_angle_deg100 = (int16_t)(s_current_shin_pitch * 100.0f);
      pkt.knee_angle_deg100 = (int16_t)(s_current_knee_angle * 100.0f);
      
      uint8_t chk = pkt.header1 + pkt.header2;
      chk += (uint8_t)(pkt.thigh_angle_deg100 & 0xFF) + (uint8_t)(pkt.thigh_angle_deg100 >> 8);
      chk += (uint8_t)(pkt.shin_angle_deg100 & 0xFF) + (uint8_t)(pkt.shin_angle_deg100 >> 8);
      chk += (uint8_t)(pkt.knee_angle_deg100 & 0xFF) + (uint8_t)(pkt.knee_angle_deg100 >> 8);
      pkt.checksum = chk;
      
      // UART Transmit to ESP32
      HAL_UART_Transmit(&huart6_esp, (uint8_t*)&pkt, sizeof(pkt), 5);
#endif

    }
  }
  else
  {
#if SENSOR_GY91
    uart_cli_send("mpu9255 read error\r\n");
#else
    uart_cli_send("mpu read error\r\n");
#endif
  }

  uint32_t t1 = timebase_cycles();
  uint32_t svc_us = timebase_cycles_to_us(t1 - t0);
  s_svc_last_us = svc_us;
  if (svc_us > s_svc_max_us)
    s_svc_max_us = svc_us;
}

void imu_app_stream_set(bool en)
{
  s_stream_en = en;
}

bool imu_app_stream_get(void)
{
  return s_stream_en;
}

void imu_app_set_print_div(uint32_t n)
{
  s_print_div = n;
}

uint32_t imu_app_get_print_div(void)
{
  return s_print_div;
}

void imu_app_stats_reset(void)
{
  s_tick_count = 0;
  s_sample_count = 0;
  s_missed = 0;

  memset(s_last, 0, sizeof(s_last));
#if SENSOR_GY91
  memset(s_last_mag, 0, sizeof(s_last_mag));
  for (int i = 0; i < NUM_IMUS; i++)
  {
    s_mx_ut[i] = 0.0f;
    s_my_ut[i] = 0.0f;
    s_mz_ut[i] = 0.0f;
    s_m_norm[i] = 0.0f;
    s_mx_cal_ut[i] = 0.0f;
    s_my_cal_ut[i] = 0.0f;
    s_mz_cal_ut[i] = 0.0f;
    s_m_cal_norm[i] = 0.0f;
    s_mag_body_valid[i] = 0;
    s_mag_cal_valid[i] = 0;
  }
#endif

  s_reset_ms = HAL_GetTick();
  s_last_sample_cyc = 0;
  s_dt_min_us = 0xFFFFFFFFu;
  s_dt_max_us = 0;
  s_dt_sum_us = 0;
  s_svc_last_us = 0;
  s_svc_max_us = 0;
  s_last_miss_tick = 0;

  s_win_start_cyc = timebase_cycles();
  s_win_samples = 0;
  s_last_rate_mhz = 0;

  // reset Madgwick output validity and alignment flag
  for (int i = 0; i < NUM_IMUS; i++)
  {
    s_mad_valid[i] = 0;
    s_mad_aligned[i] = 0;
    s_ekf_valid[i] = 0;
    s_ekf_aligned[i] = 0;
  }
}

// integer rate getter (recommended for printing)
uint32_t imu_app_get_rate_mhz(void)
{
  return s_last_rate_mhz;
}

void imu_app_get_stats(imu_stats_t *out)
{
  if (!out)
    return;

  // Snapshot volatile-ish state
  taskENTER_CRITICAL();
  uint32_t ticks = s_tick_count;
  uint32_t samples = s_sample_count;
  uint32_t missed = s_missed;
  bool stream_en = s_stream_en;
  taskEXIT_CRITICAL();

  uint32_t elapsed_ms = HAL_GetTick() - s_reset_ms;

  uint32_t dt_avg = 0;
  if (samples > 1)
  {
    dt_avg = (uint32_t)(s_dt_sum_us / (uint64_t)(samples - 1));
  }

  out->ticks = ticks;
  out->samples = samples;
  out->missed = missed;

  out->stream_en = (uint8_t)(stream_en ? 1 : 0);
  out->tick_due = 0; // Not used in RTOS mode

  out->dt_min_us = (s_dt_min_us == 0xFFFFFFFFu) ? 0 : s_dt_min_us;
  out->dt_avg_us = dt_avg;
  out->dt_max_us = s_dt_max_us;

  out->svc_last_us = s_svc_last_us;
  out->svc_max_us = s_svc_max_us;
  out->last_miss_tick = s_last_miss_tick;

  out->rate_hz = (float)s_last_rate_mhz / 1000.0f;

  out->elapsed_ms = elapsed_ms;
}

float imu_app_get_rate_hz(void)
{
  return (float)imu_app_get_rate_mhz() / 1000.0f;
}

bool imu_app_get_madgwick(uint8_t imu_idx, Attitude_t *out)
{
  if (!out || imu_idx >= NUM_IMUS)
    return false;

  // Snapshot to avoid tearing if called during updates
  taskENTER_CRITICAL();
  uint8_t valid = s_mad_valid[imu_idx];
  Attitude_t tmp = s_mad_att[imu_idx];
  taskEXIT_CRITICAL();

  if (!valid)
    return false;
  *out = tmp;
  return true;
}

void imu_app_madgwick_reset(void)
{
  taskENTER_CRITICAL();
  for (int i = 0; i < NUM_IMUS; i++)
  {
    madgwick_reset(&s_mad[i]);
    s_mad_valid[i] = 0;
    s_mad_aligned[i] = 0; // will re-align on next sample
  }
  taskEXIT_CRITICAL();
}

void imu_app_madgwick_set_beta(float beta)
{
  taskENTER_CRITICAL();
  for (int i = 0; i < NUM_IMUS; i++)
  {
    madgwick_set_beta(&s_mad[i], beta);
  }
  taskEXIT_CRITICAL();
}

float imu_app_madgwick_get_beta(uint8_t imu_idx)
{
  if (imu_idx >= NUM_IMUS)
    return 0.0f;
  float b;
  taskENTER_CRITICAL();
  b = madgwick_get_beta(&s_mad[imu_idx]);
  taskEXIT_CRITICAL();
  return b;
}

// ------------------------------------------------------------
// Gyro calibration (blocking, uses raw LSB averaging)
// ------------------------------------------------------------

void imu_app_cal_clear(void)
{
  for (int i = 0; i < NUM_IMUS; i++)
  {
    s_gx_off[i] = 0;
    s_gy_off[i] = 0;
    s_gz_off[i] = 0;
  }
}

bool imu_app_cal_get(uint8_t imu_idx, int16_t *gx_off, int16_t *gy_off, int16_t *gz_off)
{
  if (!gx_off || !gy_off || !gz_off || imu_idx >= NUM_IMUS)
    return false;
  *gx_off = s_gx_off[imu_idx];
  *gy_off = s_gy_off[imu_idx];
  *gz_off = s_gz_off[imu_idx];
  return true;
}

bool imu_app_cal_gyro(uint32_t duration_ms)
{
  if (!s_hi2c)
    return false;
  if (duration_ms < 200)
    return false;

  uint32_t start = HAL_GetTick();
  uint32_t count[NUM_IMUS] = {0};

  int64_t sum_x[NUM_IMUS] = {0};
  int64_t sum_y[NUM_IMUS] = {0};
  int64_t sum_z[NUM_IMUS] = {0};

  while ((HAL_GetTick() - start) < duration_ms)
  {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    for (int i = 0; i < NUM_IMUS; i++)
    {
      uint8_t addr = (i == 0) ? MPU6050_ADDR_0 : MPU6050_ADDR_1;
#if SENSOR_GY91
      mpu9255_raw_t r;
      if (mpu9255_read_raw(s_hi2c, addr, &r) == MPU9255_OK)
      {
        sum_x[i] += r.gx;
        sum_y[i] += r.gy;
        sum_z[i] += r.gz;
        count[i]++;
      }
#else
      mpu6050_raw_t r;
      if (mpu6050_read_raw(s_hi2c, addr, &r) == MPU6050_OK)
      {
        sum_x[i] += r.gx;
        sum_y[i] += r.gy;
        sum_z[i] += r.gz;
        count[i]++;
      }
#endif /* SENSOR_GY91 */
    }
    xSemaphoreGive(g_i2c_mutex);
    HAL_Delay(2);
  }

  bool all_ok = true;
  for (int i = 0; i < NUM_IMUS; i++)
  {
    if (count[i] == 0)
    {
      all_ok = false;
      continue;
    }
    s_gx_off[i] = (int16_t)(sum_x[i] / (int64_t)count[i]);
    s_gy_off[i] = (int16_t)(sum_y[i] / (int64_t)count[i]);
    s_gz_off[i] = (int16_t)(sum_z[i] / (int64_t)count[i]);
  }

  return all_ok;
}

// ------------------------------------------------------------
// Accel bias getter / setter (per-IMU)
// ------------------------------------------------------------

void imu_app_get_accel_bias(uint8_t imu_idx, int16_t *ax, int16_t *ay, int16_t *az)
{
  if (imu_idx >= NUM_IMUS)
    return;
  taskENTER_CRITICAL();
  if (ax)
    *ax = s_ax_off[imu_idx];
  if (ay)
    *ay = s_ay_off[imu_idx];
  if (az)
    *az = s_az_off[imu_idx];
  taskEXIT_CRITICAL();
}

void imu_app_set_accel_bias(uint8_t imu_idx, int16_t ax, int16_t ay, int16_t az)
{
  if (imu_idx >= NUM_IMUS)
    return;
  taskENTER_CRITICAL();
  s_ax_off[imu_idx] = ax;
  s_ay_off[imu_idx] = ay;
  s_az_off[imu_idx] = az;
  taskEXIT_CRITICAL();
}

bool imu_app_cal_accel(uint32_t duration_ms)
{
  if (!s_hi2c)
    return false;
  if (duration_ms < 200)
    return false;

  uint32_t start = HAL_GetTick();
  uint32_t count[NUM_IMUS] = {0};

  int64_t sum_x[NUM_IMUS] = {0};
  int64_t sum_y[NUM_IMUS] = {0};
  int64_t sum_z[NUM_IMUS] = {0};

  while ((HAL_GetTick() - start) < duration_ms)
  {
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    for (int i = 0; i < NUM_IMUS; i++)
    {
      uint8_t addr = (i == 0) ? MPU6050_ADDR_0 : MPU6050_ADDR_1;
#if SENSOR_GY91
      mpu9255_raw_t r;
      if (mpu9255_read_raw(s_hi2c, addr, &r) == MPU9255_OK)
      {
        sum_x[i] += r.ax;
        sum_y[i] += r.ay;
        sum_z[i] += r.az;
        count[i]++;
      }
#else
      mpu6050_raw_t r;
      if (mpu6050_read_raw(s_hi2c, addr, &r) == MPU6050_OK)
      {
        sum_x[i] += r.ax;
        sum_y[i] += r.ay;
        sum_z[i] += r.az;
        count[i]++;
      }
#endif /* SENSOR_GY91 */
    }
    xSemaphoreGive(g_i2c_mutex);
    HAL_Delay(2);
  }

  bool all_ok = true;
  for (int i = 0; i < NUM_IMUS; i++)
  {
    if (count[i] == 0)
    {
      all_ok = false;
      continue;
    }
    s_ax_off[i] = (int16_t)(sum_x[i] / (int64_t)count[i]);
    s_ay_off[i] = (int16_t)(sum_y[i] / (int64_t)count[i]);
    
    /* Subtract +1g from the Z axis reading to find the actual zero-g bias.
     * We use the per-IMU scale factor since IMUs might be running at different ranges. */
    s_az_off[i] = (int16_t)((sum_z[i] / (int64_t)count[i]) - (int64_t)s_acc_lsb_per_g[i]);
  }

  return all_ok;
}

// ------------------------------------------------------------
// Madgwick timing
// ------------------------------------------------------------

uint32_t imu_app_mad_last_us(uint8_t imu_idx)
{
  if (imu_idx >= NUM_IMUS)
    return 0;
  return s_mad_last_us[imu_idx];
}

// ------------------------------------------------------------
// EKF getters / setters
// ------------------------------------------------------------

bool imu_app_get_ekf(uint8_t imu_idx, Attitude_t *out)
{
  if (!out || imu_idx >= NUM_IMUS)
    return false;

  taskENTER_CRITICAL();
  uint8_t valid = s_ekf_valid[imu_idx];
  Attitude_t tmp = s_ekf_att[imu_idx];
  taskEXIT_CRITICAL();

  if (!valid)
    return false;
  *out = tmp;
  return true;
}

void imu_app_ekf_reset(void)
{
  taskENTER_CRITICAL();
  for (int i = 0; i < NUM_IMUS; i++)
  {
    ekf7_reset(&s_ekf[i]);
    s_ekf_valid[i] = 0;
    s_ekf_aligned[i] = 0; // will re-align from accel on next sample
  }
  taskEXIT_CRITICAL();
}

void imu_app_ekf_set_noise(float sigma_gyro, float sigma_bias,
                           float sigma_accel, float sigma_mag, float r_adapt_k)
{
  taskENTER_CRITICAL();
  for (int i = 0; i < NUM_IMUS; i++)
  {
    ekf7_set_noise(&s_ekf[i], sigma_gyro, sigma_bias, sigma_accel, sigma_mag, r_adapt_k);
  }
  taskEXIT_CRITICAL();
}

float imu_app_ekf_trace_p(uint8_t imu_idx)
{
  if (imu_idx >= NUM_IMUS)
    return 0.0f;
  float tr;
  taskENTER_CRITICAL();
  tr = ekf7_trace_P(&s_ekf[imu_idx]);
  taskEXIT_CRITICAL();
  return tr;
}

void imu_app_ekf_get_bias(uint8_t imu_idx, float *bx, float *by, float *bz)
{
  if (!bx || !by || !bz || imu_idx >= NUM_IMUS)
    return;
  float b[3];
  taskENTER_CRITICAL();
  ekf7_get_bias(&s_ekf[imu_idx], b);
  taskEXIT_CRITICAL();
  *bx = b[0];
  *by = b[1];
  *bz = b[2];
}

uint32_t imu_app_ekf_last_us(uint8_t imu_idx)
{
  if (imu_idx >= NUM_IMUS)
    return 0;
  return s_ekf_last_us[imu_idx];
}

void imu_app_get_accel_g(uint8_t imu_idx, float *ax, float *ay, float *az)
{
  if (imu_idx >= NUM_IMUS)
    return;
  taskENTER_CRITICAL();
  if (ax)
    *ax = s_ax_g[imu_idx];
  if (ay)
    *ay = s_ay_g[imu_idx];
  if (az)
    *az = s_az_g[imu_idx];
  taskEXIT_CRITICAL();
}

#if SENSOR_GY91
/* ----------------------------------------------------------------
 * Magnetometer raw getters (CLI diagnostic use only in Phase 1).
 * ---------------------------------------------------------------- */
void imu_app_get_mag_raw(uint8_t imu_idx, ak8963_raw_t *out)
{
  if (!out || imu_idx >= NUM_IMUS)
    return;
  taskENTER_CRITICAL();
  *out = s_last_mag[imu_idx];
  taskEXIT_CRITICAL();
}

void imu_app_get_ak_cfg(uint8_t imu_idx, ak8963_cfg_t *out)
{
  if (!out || imu_idx >= NUM_IMUS)
    return;
  *out = s_ak_cfg[imu_idx]; /* written once at init, never changes */
}

/* Body-frame magnetometer (uncalibrated) getter. */
void imu_app_get_mag_body(uint8_t imu_idx, float *mx_ut, float *my_ut, float *mz_ut,
                          float *norm, uint8_t *valid)
{
  if (imu_idx >= NUM_IMUS)
    return;
  taskENTER_CRITICAL();
  if (mx_ut)
    *mx_ut = s_mx_ut[imu_idx];
  if (my_ut)
    *my_ut = s_my_ut[imu_idx];
  if (mz_ut)
    *mz_ut = s_mz_ut[imu_idx];
  if (norm)
    *norm = s_m_norm[imu_idx];
  if (valid)
    *valid = s_mag_body_valid[imu_idx];
  taskEXIT_CRITICAL();
}

/* Calibrated body-frame magnetometer getter (hard + soft iron applied). */
void imu_app_get_mag_cal(uint8_t imu_idx, float *mx_ut, float *my_ut, float *mz_ut,
                         float *norm, uint8_t *valid)
{
  if (imu_idx >= NUM_IMUS)
    return;
  taskENTER_CRITICAL();
  if (mx_ut)
    *mx_ut = s_mx_cal_ut[imu_idx];
  if (my_ut)
    *my_ut = s_my_cal_ut[imu_idx];
  if (mz_ut)
    *mz_ut = s_mz_cal_ut[imu_idx];
  if (norm)
    *norm = s_m_cal_norm[imu_idx];
  if (valid)
    *valid = s_mag_cal_valid[imu_idx];
  taskEXIT_CRITICAL();
}

/* Enable/disable M, line emission for offline calibration capture.
 * Automatically starts the main poll loop if not already running.  */
void imu_app_mag_stream_set(bool en)
{
  s_mag_stream_en = en;
  if (en)
    imu_app_stream_set(true); /* poll loop must be running */
}

bool imu_app_mag_stream_get(void)
{
  return s_mag_stream_en;
}
#endif /* SENSOR_GY91 */
