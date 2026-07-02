#ifndef INC_FILTERS_EKF_H_
#define INC_FILTERS_EKF_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* -----------------------------------------------------------------------
     * Main EKF state structure
     * ----------------------------------------------------------------------- */
    typedef struct
    {

        /* State ---------------------------------------------------------------- */
        float q[4]; /* unit quaternion [w, x, y, z]          */
        float b[3]; /* gyro bias estimate [bx, by, bz] rad/s */

        /* Error covariance (7x7, stored row-major) --------------------------- */
        float P[7][7];

        /* Initial P diagonal (kept for reset) -------------------------------- */
        float P0;

        /* Noise source parameters -------------------------------------------- */
        float sigma_gyro;  /* gyro noise density  (rad/s/sqrt(Hz))  */
        float sigma_bias;  /* bias random-walk    (rad/s^2/sqrt(Hz)) */
        float sigma_accel; /* accel noise density (g/sqrt(Hz))       */
        float sigma_mag;   /* mag noise density   (uT/sqrt(Hz))      */

        /* Magnetometer reference / gating ----------------------------------- */
        float mag_ref_n[3]; /* earth-frame magnetic direction (unit)       */
        bool mag_ref_valid; /* set after ekf7_init_from_marg or first mag  */
        float mag_nis_gate; /* reject when NIS = y^T S^-1 y exceeds this   */

        /* Adaptive R tuning -------------------------------------------------- */
        float r_adapt_k; /* R_eff = sigma_accel^2 * (1 + r_adapt_k * dev^2)
                            dev = ||a_norm| - 1|; larger k -> faster trust drop */

        /* Last computed adaptive R value (diagonal scalar, updated each accel step) */
        float last_r_eff;

        /* Hard-reject safety fallback (before adaptive R) -------------------- */
        bool accel_reject_en;
        float accel_reject_min_g; /* below this -> skip update entirely */
        float accel_reject_max_g; /* above this -> skip update entirely */
        float accel_sane_timer_s;
        float accel_sane_timeout_s;

        /* Per-instance symmetry enforcement counters (not static — each EKF
         * instance must track its own counter to avoid cross-IMU coupling). */
        uint32_t sym_ctr;
        uint32_t sym_ctr_mag;

        /* Convergence / health flag ------------------------------------------ */
        bool     healthy;     /* true once trace(P) < EKF_CONVERGENCE_TRACE        */
        uint32_t healthy_ctr; /* consecutive steps below threshold (for hysteresis) */

        /* -----------------------------------------------------------------------
         * Per-instance scratch buffers.
         *
         * Previously these were module-level statics, which meant all EKF
         * instances shared the same intermediate storage. That was safe only
         * under a single-threaded super-loop calling each instance sequentially.
         *
         * Moving them here gives every ekf7_t instance its own scratch space so
         * that two IMUs can be updated from different RTOS tasks (or interrupts)
         * without corrupting each other's intermediate state.
         *
         * Memory cost: +1428 bytes per instance (predictable BSS, not stack).
         * With NUM_IMUS=2 this adds ~2.9 KB — well within the STM32F4's 192 KB.
         *
         * Fields are prefixed with '_' to signal they are internal; do not access
         * them directly from application code.
         * ----------------------------------------------------------------------- */
        float _F[7][7];    /* state-transition Jacobian                  */
        float _FP[7][7];   /* F * P  (predict intermediate)              */
        float _HP[3][7];   /* H * P  (update intermediate)               */
        float _PHt[7][3];  /* P * H^T                                    */
        float _K[7][3];    /* Kalman gain                                 */
        float _A[7][7];    /* I - K*H  (Joseph form)                     */
        float _AP[7][7];   /* A * P_prior  (Joseph form)                 */
        float _Pnew[7][7]; /* updated covariance  (Joseph form output)   */
        float _Pold[7][7]; /* copy of P_prior  (Joseph form input)       */

    } ekf7_t;

    /* -----------------------------------------------------------------------
     * API
     * ----------------------------------------------------------------------- */

    /**
     * @brief  Initialise the EKF with noise parameters. Must be called before use.
     *
     * @param e           Pointer to ekf7_t instance
     * @param sigma_gyro  Gyro noise density  (rad/s/sqrt(Hz))  e.g. 0.01
     * @param sigma_bias  Bias random-walk    (rad/s^2/sqrt(Hz)) e.g. 1e-5
     * @param sigma_accel Accel noise density (g/sqrt(Hz))       e.g. 0.05
     * @param sigma_mag   Mag noise density   (uT/sqrt(Hz))      e.g. 0.5
     * @param r_adapt_k   Adaptive-R steepness coefficient       e.g. 20.0
     * @param P0          Initial diagonal value for P           e.g. 1.0
     */
    void ekf7_init(ekf7_t *e,
                   float sigma_gyro,
                   float sigma_bias,
                   float sigma_accel,
                   float sigma_mag,
                   float r_adapt_k,
                   float P0);

    /**
     * @brief  Reset state to identity quaternion, zero bias, and P = P0*I.
     *         Noise parameters and accel-reject settings are preserved.
     */
    void ekf7_reset(ekf7_t *e);

    /**
     * @brief  Set or update accel magnitude hard-reject window.
     *         When enabled, if |a| is outside [min_g, max_g] the measurement
     *         update is skipped entirely (before adaptive R even applies).
     */
    void ekf7_set_accel_reject(ekf7_t *e, bool en, float min_g, float max_g, float timeout_s);

    /**
     * @brief  Update noise parameters at runtime (re-buildable via CLI).
     *         Takes effect on the next ekf7_predict / ekf7_update_accel call.
     */
    void ekf7_set_noise(ekf7_t *e,
                        float sigma_gyro,
                        float sigma_bias,
                        float sigma_accel,
                        float sigma_mag,
                        float r_adapt_k);

    /**
     * @brief  Align initial roll/pitch from a static gravity reading.
     *         Avoids slow initial convergence if the board is tilted at startup.
     *         Yaw remains 0 (unobservable without a magnetometer).
     *         Bias estimate is preserved.
     */
    void ekf7_init_from_accel(ekf7_t *e, float ax_g, float ay_g, float az_g);

    /**
     * @brief  Align initial roll/pitch from accel, and yaw from mag.
     *         Prevents EKF from diverging out of bounds due to extreme initial
     *         yaw errors during the first measurement update.
     */
    void ekf7_init_from_marg(ekf7_t *e, float ax_g, float ay_g, float az_g, float mx, float my, float mz);

    /* ----------------------------------------------------------------------- *
     * Core filter steps                                                        *
     * ----------------------------------------------------------------------- */

    /**
     * @brief  Predict step: gyro-driven time update.
     *         Subtracts current bias estimate, propagates quaternion via
     *         first-order kinematics, builds 7x7 F Jacobian, and updates P.
     *
     * @param wx/wy/wz  Raw gyro (rad/s, body frame, BEFORE bias subtraction)
     * @param dt_s      Measured time-step in seconds
     */
    void ekf7_predict(ekf7_t *e, float wx, float wy, float wz, float dt_s);

    /**
     * @brief  Update step: accelerometer measurement correction.
     *         Innovation y = normalize(a) - R(q)^T*[0,0,1].
     *         Adaptive R scales measurement noise with ||a|-1| deviation.
     *
     * @param ax_g/ay_g/az_g  Accel in g (body frame)
     * @param dt_s            Measured time-step in seconds
     */
    void ekf7_update_accel(ekf7_t *e, float ax_g, float ay_g, float az_g, float dt_s);

    /**
     * @brief  Convenience: calls ekf7_predict then ekf7_update_accel.
     */
    void ekf7_step(ekf7_t *e,
                   float wx, float wy, float wz,
                   float ax_g, float ay_g, float az_g,
                   float dt_s);

    /**
     * @brief  Update step: magnetometer measurement correction.
     *         Observation h(q) = R(q)^T * m_ref where m_ref is a fixed
     *         earth-frame magnetic direction set at initial alignment.
     *         Uses NIS gating for magnetic outlier rejection.
     *
     * @param mx/my/mz  Calibrated Mag in uT (body frame)
     */
    void ekf7_update_mag(ekf7_t *e, float mx, float my, float mz);

    /**
     * @brief  Convenience MARG wrapper: Predict -> Update Accel -> Update Mag.
     *         Provides full 9-axis yaw drift elimination.
     */
    void ekf7_step_marg(ekf7_t *e,
                        float wx, float wy, float wz,
                        float ax_g, float ay_g, float az_g,
                        float mx, float my, float mz,
                        float dt_s);

    /* ----------------------------------------------------------------------- *
     * Accessors / diagnostics                                                  *
     * ----------------------------------------------------------------------- */

    /**
     * @brief  Return trace(P) — sum of diagonal elements.
     *         Decreases as the filter converges.
     */
    float ekf7_trace_P(const ekf7_t *e);

    /**
     * @brief  Copy current quaternion estimate into q_out[4] as [w, x, y, z].
     */
    void ekf7_get_quat(const ekf7_t *e, float q_out[4]);

    /**
     * @brief  Copy current gyro bias estimate into b_out[3] as [bx, by, bz] rad/s.
     */
    void ekf7_get_bias(const ekf7_t *e, float b_out[3]);

    /**
     * @brief  Return the most recently computed effective measurement noise R_eff
     *         (diagonal scalar): sigma_accel^2 * (1 + k * dev^2).
     *         Returns 0 before the first ekf7_update_accel() call.
     */
    float ekf7_get_r_eff(const ekf7_t *e);

    /**
     * @brief  Returns true once trace(P) has stayed below EKF_CONVERGENCE_TRACE
     *         for at least 10 consecutive update steps, indicating the filter
     *         has converged and the quaternion estimate is trustworthy.
     */
    bool ekf7_is_healthy(const ekf7_t *e);

#ifdef __cplusplus
}
#endif

#endif /* INC_FILTERS_EKF_H_ */
