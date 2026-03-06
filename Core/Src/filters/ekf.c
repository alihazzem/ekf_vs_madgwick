/*
 * ekf.c — 7-State Extended Kalman Filter implementation
 *
 * State:    x = [q0, q1, q2, q3, bx, by, bz]
 *           q : unit quaternion (w, x, y, z)
 *           b : gyro bias estimate (rad/s, body frame)
 *
 * Predict:  Quaternion kinematic model driven by bias-corrected gyro.
 *           7×7 Jacobian F propagates covariance.
 *
 * Update:   Gravity direction from accelerometer corrects q and b.
 *           Adaptive R: measurement noise inflates continuously with
 *           ||a| − 1| deviation, so the EKF never hard-rejects accel —
 *           it gracefully reduces trust during dynamic motion.
 *
 * Stack safety: large temporaries are declared static (file scope) because
 *           the STM32F411 default stack is 1 KB.  The EKF runs in a single
 *           cooperative loop — no reentrancy is needed.
 */

#include "filters/ekf.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Static scratch buffers (shared across predict / update — never called
 * concurrently in the cooperative super-loop).
 * ----------------------------------------------------------------------- */
static float s_F[7][7];   /* state-transition Jacobian              */
static float s_FP[7][7];  /* F * P intermediate                     */
static float s_HP[3][7];  /* H * P  (3×7) for S and PHt computation */
static float s_PHt[7][3]; /* P * H^T  (7×3) used to build K         */
static float s_K[7][3];   /* Kalman gain  (7×3)                     */

/* -----------------------------------------------------------------------
 * Local helpers
 * ----------------------------------------------------------------------- */

/*
 * C = A * B   (7×7 × 7×7 → 7×7)
 */
static void m77_mult(const float A[7][7], const float B[7][7], float C[7][7])
{
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

/*
 * Analytic 3×3 matrix inverse (Cramer's rule).
 * Returns 1 on success, 0 if the matrix is singular.
 * S = H*P*H^T + R is always positive-definite, so this never fails in
 * practice.
 */
static int m33_inv(const float A[3][3], float Ainv[3][3])
{
    float d = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (d > -1e-12f && d < 1e-12f)
        return 0;

    float id = 1.0f / d;

    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * id;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * id;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * id;

    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * id;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * id;
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * id;

    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * id;
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * id;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * id;

    return 1;
}

/* -----------------------------------------------------------------------
 * API — Init / Reset / Config
 * ----------------------------------------------------------------------- */

void ekf7_init(ekf7_t *e,
               float sigma_gyro,
               float sigma_bias,
               float sigma_accel,
               float r_adapt_k,
               float P0)
{
    if (!e)
        return;

    e->sigma_gyro = sigma_gyro;
    e->sigma_bias = sigma_bias;
    e->sigma_accel = sigma_accel;
    e->r_adapt_k = r_adapt_k;
    e->P0 = P0;

    /* Accel hard-reject disabled by default — rely on adaptive R */
    e->accel_reject_en = false;
    e->accel_reject_min_g = 0.0f;
    e->accel_reject_max_g = 1e9f;

    ekf7_reset(e);
}

void ekf7_reset(ekf7_t *e)
{
    if (!e)
        return;

    /* Identity quaternion */
    e->q[0] = 1.0f;
    e->q[1] = 0.0f;
    e->q[2] = 0.0f;
    e->q[3] = 0.0f;

    /* Zero bias */
    e->b[0] = 0.0f;
    e->b[1] = 0.0f;
    e->b[2] = 0.0f;

    /* P = P0 * I_7 */
    memset(e->P, 0, sizeof(e->P));
    for (int i = 0; i < 7; i++)
        e->P[i][i] = e->P0;
}

void ekf7_set_accel_reject(ekf7_t *e, bool en, float min_g, float max_g)
{
    if (!e)
        return;
    e->accel_reject_en = en;
    e->accel_reject_min_g = min_g;
    e->accel_reject_max_g = max_g;
}

void ekf7_set_noise(ekf7_t *e,
                    float sigma_gyro,
                    float sigma_bias,
                    float sigma_accel,
                    float r_adapt_k)
{
    if (!e)
        return;
    e->sigma_gyro = sigma_gyro;
    e->sigma_bias = sigma_bias;
    e->sigma_accel = sigma_accel;
    e->r_adapt_k = r_adapt_k;
}

void ekf7_init_from_accel(ekf7_t *e, float ax_g, float ay_g, float az_g)
{
    if (!e)
        return;

    float a2 = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    if (a2 < 1e-9f)
        return;

    float inv = 1.0f / sqrtf(a2);
    float ax = ax_g * inv;
    float ay = ay_g * inv;
    float az = az_g * inv;

    /* Roll / pitch from gravity direction; yaw = 0 (unobservable) */
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    /* ZYX Euler → quaternion (yaw = 0 → cy = 1, sy = 0) */
    e->q[0] = cr * cp;
    e->q[1] = sr * cp;
    e->q[2] = cr * sp;
    e->q[3] = -sr * sp;

    /* Preserve existing bias estimate — don't throw it away on re-align */
}

/* -----------------------------------------------------------------------
 * Predict step — gyro-driven time update
 *
 * State transition Jacobian F (7×7):
 *
 *   F = [ F_qq  |  F_qb ]    row 0..3, col 0..3 | row 0..3, col 4..6
 *       [ 0_3x4 |  I_3  ]    row 4..6, col 0..3 | row 4..6, col 4..6
 *
 * F_qq = I_4 + 0.5*dt * Omega(w_c)
 *
 *   Omega(w) = [[  0, -wx, -wy, -wz ],
 *               [ wx,   0,  wz, -wy ],
 *               [ wy, -wz,   0,  wx ],
 *               [ wz,  wy, -wx,   0 ]]
 *
 * F_qb = -0.5*dt * Xi(q)
 *
 *   Xi(q) = [[ -q1, -q2, -q3 ],
 *             [  q0, -q3,  q2 ],
 *             [  q3,  q0, -q1 ],
 *             [ -q2,  q1,  q0 ]]
 *
 *   (Xi is d(Omega_L(w)*q)/dw — how quaternion rate changes with gyro)
 *
 * Process noise Q (diagonal):
 *   Q[0..3][0..3] = sigma_gyro^2 * dt * I_4
 *   Q[4..6][4..6] = sigma_bias^2 * dt * I_3
 * ----------------------------------------------------------------------- */
void ekf7_predict(ekf7_t *e, float wx, float wy, float wz, float dt_s)
{
    if (!e || dt_s <= 0.0f)
        return;

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];

    /* 1. Bias-corrected gyro */
    float wx_c = wx - e->b[0];
    float wy_c = wy - e->b[1];
    float wz_c = wz - e->b[2];

    /* 2. Build F in static scratch s_F */
    float h = 0.5f * dt_s;

    memset(s_F, 0, sizeof(s_F));

    /* F_qq block (rows 0..3, cols 0..3) */
    s_F[0][0] = 1.0f;
    s_F[0][1] = -h * wx_c;
    s_F[0][2] = -h * wy_c;
    s_F[0][3] = -h * wz_c;
    s_F[1][0] = h * wx_c;
    s_F[1][1] = 1.0f;
    s_F[1][2] = h * wz_c;
    s_F[1][3] = -h * wy_c;
    s_F[2][0] = h * wy_c;
    s_F[2][1] = -h * wz_c;
    s_F[2][2] = 1.0f;
    s_F[2][3] = h * wx_c;
    s_F[3][0] = h * wz_c;
    s_F[3][1] = h * wy_c;
    s_F[3][2] = -h * wx_c;
    s_F[3][3] = 1.0f;

    /* F_qb block (rows 0..3, cols 4..6) = -h * Xi(q)
     * Xi[0] = [-q1, -q2, -q3]  ->  -h*Xi[0] = [ h*q1,  h*q2,  h*q3]
     * Xi[1] = [ q0, -q3,  q2]  ->  -h*Xi[1] = [-h*q0,  h*q3, -h*q2]
     * Xi[2] = [ q3,  q0, -q1]  ->  -h*Xi[2] = [-h*q3, -h*q0,  h*q1]
     * Xi[3] = [-q2,  q1,  q0]  ->  -h*Xi[3] = [ h*q2, -h*q1, -h*q0]
     */
    s_F[0][4] = h * q1;
    s_F[0][5] = h * q2;
    s_F[0][6] = h * q3;
    s_F[1][4] = -h * q0;
    s_F[1][5] = h * q3;
    s_F[1][6] = -h * q2;
    s_F[2][4] = -h * q3;
    s_F[2][5] = -h * q0;
    s_F[2][6] = h * q1;
    s_F[3][4] = h * q2;
    s_F[3][5] = -h * q1;
    s_F[3][6] = -h * q0;

    /* F_bb = I_3 (rows 4..6, cols 4..6) */
    s_F[4][4] = 1.0f;
    s_F[5][5] = 1.0f;
    s_F[6][6] = 1.0f;

    /* 3. Propagate quaternion: q_new = F_qq * q */
    float q_new[4] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            q_new[i] += s_F[i][j] * e->q[j];

    float qnorm = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] +
                        q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    if (qnorm > 1e-9f)
    {
        float inv = 1.0f / qnorm;
        e->q[0] = q_new[0] * inv;
        e->q[1] = q_new[1] * inv;
        e->q[2] = q_new[2] * inv;
        e->q[3] = q_new[3] * inv;
    }
    /* Bias propagates as identity (random walk) — no change to e->b */

    /* 4. P = F * P * F^T + Q */

    /* s_FP = F * P */
    m77_mult(s_F, e->P, s_FP);

    /* e->P = s_FP * F^T  (write directly into e->P) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += s_FP[i][k] * s_F[j][k]; /* F^T[k][j] = F[j][k] */
            e->P[i][j] = s;
        }

    /* Add process noise Q on the diagonal */
    float q_var = e->sigma_gyro * e->sigma_gyro * dt_s;
    float b_var = e->sigma_bias * e->sigma_bias * dt_s;
    for (int i = 0; i < 4; i++)
        e->P[i][i] += q_var;
    for (int i = 4; i < 7; i++)
        e->P[i][i] += b_var;
}

/* -----------------------------------------------------------------------
 * Update step — accelerometer measurement correction
 *
 * Measurement model  h(q) = R(q)^T * [0, 0, 1]^T  (gravity in body frame):
 *   hx = 2*(q1*q3 - q0*q2)
 *   hy = 2*(q2*q3 + q0*q1)
 *   hz = q0^2 - q1^2 - q2^2 + q3^2
 *
 * Jacobian H (3×7) = [ dh/dq | 0_{3×3} ]:
 *   row 0 (dhx): [-2q2,  2q3, -2q0,  2q1,  0, 0, 0]
 *   row 1 (dhy): [ 2q1,  2q0,  2q3,  2q2,  0, 0, 0]
 *   row 2 (dhz): [ 2q0, -2q1, -2q2,  2q3,  0, 0, 0]
 *
 * Adaptive R:
 *   dev    = |a_raw_mag| - 1              (deviation from 1 g)
 *   R_eff  = sigma_accel^2 * (1 + k*dev^2) * I_3
 *   -> trust drops smoothly as |a| deviates from 1 g
 *   -> no hard binary threshold needed (but hard-reject still acts as safety)
 * ----------------------------------------------------------------------- */
void ekf7_update_accel(ekf7_t *e, float ax_g, float ay_g, float az_g)
{
    if (!e)
        return;

    /* 1. Accel magnitude — used for hard-reject and adaptive R */
    float a_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    /* 2. Hard-reject safety (last resort — adaptive R already handles dynamics) */
    if (e->accel_reject_en)
    {
        if (a_mag < e->accel_reject_min_g || a_mag > e->accel_reject_max_g)
            return;
    }

    /* 3. Normalize accel */
    if (a_mag < 1e-9f)
        return;
    float inv_a = 1.0f / a_mag;
    float ax = ax_g * inv_a;
    float ay = ay_g * inv_a;
    float az = az_g * inv_a;

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];

    /* 4. Predicted gravity in body frame: h(q) = R(q)^T * [0,0,1] */
    float hx = 2.0f * (q1 * q3 - q0 * q2);
    float hy = 2.0f * (q2 * q3 + q0 * q1);
    float hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 5. Innovation  y = z − h(q)  (3×1) */
    float y[3] = {ax - hx, ay - hy, az - hz};

    /* 6. Jacobian H (3×7) on the stack — only 84 bytes */
    float H[3][7] = {
        {-2.0f * q2, 2.0f * q3, -2.0f * q0, 2.0f * q1, 0, 0, 0},
        {2.0f * q1, 2.0f * q0, 2.0f * q3, 2.0f * q2, 0, 0, 0},
        {2.0f * q0, -2.0f * q1, -2.0f * q2, 2.0f * q3, 0, 0, 0}};

    /* 7. Adaptive R scalar — inflates with |a_mag − 1| deviation
     *    R_eff = sigma_accel^2 * (1 + r_adapt_k * dev^2)
     *    All three measurement axes share the same scalar (isotropic noise). */
    float dev = a_mag - 1.0f;
    float r_val = e->sigma_accel * e->sigma_accel *
                  (1.0f + e->r_adapt_k * dev * dev);

    /* 8. s_HP = H * P   (3×7) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += H[i][k] * e->P[k][j];
            s_HP[i][j] = s;
        }

    /* 9. S = s_HP * H^T + R_eff * I_3   (3×3, on the stack) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += s_HP[i][k] * H[j][k];
            S[i][j] = s + (i == j ? r_val : 0.0f);
        }

    /* 10. Invert S → Sinv (3×3, on the stack) */
    float Sinv[3][3];
    if (!m33_inv(S, Sinv))
        return; /* singular — skip update (should never happen) */

    /* 11. s_PHt = P * H^T   (7×3)
     *     PHt[i][m] = sum_j P[i][j] * H[m][j]
     *     H[m][j] = 0 for j >= 4, so only cols 0..3 contribute. */
    for (int i = 0; i < 7; i++)
        for (int m = 0; m < 3; m++)
        {
            float s = 0.0f;
            for (int j = 0; j < 4; j++)
                s += e->P[i][j] * H[m][j]; /* exploit H zeros */
            s_PHt[i][m] = s;
        }

    /* 12. s_K = s_PHt * Sinv   (7×3) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += s_PHt[i][k] * Sinv[k][j];
            s_K[i][j] = s;
        }

    /* 13. State update: x += K * y */
    for (int i = 0; i < 4; i++)
    {
        float dq = 0.0f;
        for (int m = 0; m < 3; m++)
            dq += s_K[i][m] * y[m];
        e->q[i] += dq;
    }
    for (int i = 0; i < 3; i++)
    {
        float db = 0.0f;
        for (int m = 0; m < 3; m++)
            db += s_K[i + 4][m] * y[m];
        e->b[i] += db;
    }

    /* Renormalize quaternion after update */
    float qnorm = sqrtf(e->q[0] * e->q[0] + e->q[1] * e->q[1] +
                        e->q[2] * e->q[2] + e->q[3] * e->q[3]);
    if (qnorm > 1e-9f)
    {
        float inv = 1.0f / qnorm;
        e->q[0] *= inv;
        e->q[1] *= inv;
        e->q[2] *= inv;
        e->q[3] *= inv;
    }

    /* 14. P update: P = (I − K*H) * P = P − K * (H * P) = P − K * s_HP
     *     In-place: P[i][j] -= sum_m K[i][m] * HP[m][j]
     *     No aliasing: K and HP are in separate static buffers. */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int m = 0; m < 3; m++)
                s += s_K[i][m] * s_HP[m][j];
            e->P[i][j] -= s;
        }

    /* 15. Periodic P symmetry enforcement — P = (P + P^T) / 2.
     *     The simplified P update (I − KH)P is not the Joseph form and
     *     accumulates floating-point asymmetry over many iterations.
     *     Correcting every 64 updates (~0.64 s at 100 Hz) is negligible CPU
     *     cost and prevents the covariance from becoming indefinite. */
    static uint32_t s_sym_ctr = 0;
    if ((++s_sym_ctr & 0x3Fu) == 0u)
    {
        for (int i = 0; i < 7; i++)
            for (int j = i + 1; j < 7; j++)
            {
                float avg = 0.5f * (e->P[i][j] + e->P[j][i]);
                e->P[i][j] = avg;
                e->P[j][i] = avg;
            }
    }
}

/* -----------------------------------------------------------------------
 * Convenience: predict then update
 * ----------------------------------------------------------------------- */
void ekf7_step(ekf7_t *e,
               float wx, float wy, float wz,
               float ax_g, float ay_g, float az_g,
               float dt_s)
{
    ekf7_predict(e, wx, wy, wz, dt_s);
    ekf7_update_accel(e, ax_g, ay_g, az_g);
}

/* -----------------------------------------------------------------------
 * Accessors / diagnostics
 * ----------------------------------------------------------------------- */

float ekf7_trace_P(const ekf7_t *e)
{
    if (!e)
        return 0.0f;
    float tr = 0.0f;
    for (int i = 0; i < 7; i++)
        tr += e->P[i][i];
    return tr;
}

void ekf7_get_quat(const ekf7_t *e, float q_out[4])
{
    if (!e || !q_out)
        return;
    q_out[0] = e->q[0];
    q_out[1] = e->q[1];
    q_out[2] = e->q[2];
    q_out[3] = e->q[3];
}

void ekf7_get_bias(const ekf7_t *e, float b_out[3])
{
    if (!e || !b_out)
        return;
    b_out[0] = e->b[0];
    b_out[1] = e->b[1];
    b_out[2] = e->b[2];
}
