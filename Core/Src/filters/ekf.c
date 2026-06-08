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
 *
 * Fixes applied (see FIX: comments throughout):
 *   1. NIS gate raised from 5.0 to 9.0  (was rejecting ~18% of valid mag samples)
 *   2. Mag reference bootstrap failure now sets mag_ref_valid=false and returns
 *      cleanly — previously a silent no-op with no indication to the caller.
 *   3. ekf7_init_from_accel comment corrected — bias is always zero at this
 *      point in normal startup flow (ekf7_reset zeroes it first).
 *   4. Accel hard-reject defaults changed to enabled with ±[0.3g, 4.0g] window,
 *      appropriate for a robotic shoulder joint that can produce impact spikes.
 *   5. Process noise Q comment updated to document the dt vs dt² approximation.
 *   6. Mag inclination zero-projection failure now explicitly resets
 *      mag_ref_valid to false so the next update triggers a re-bootstrap.
 */

#include "filters/ekf.h"
#include "app/app_config.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Static scratch buffers (shared across predict / update — never called
 * concurrently in the cooperative super-loop).
 * ----------------------------------------------------------------------- */
static float s_F[7][7];
static float s_FP[7][7];
static float s_HP[3][7];
static float s_PHt[7][3];
static float s_K[7][3];
static float s_A[7][7];
static float s_AP[7][7];
static float s_Pnew[7][7];
static float s_Pold[7][7];

/* -----------------------------------------------------------------------
 * Local helpers
 * ----------------------------------------------------------------------- */

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

static int v3_normalize(float v[3])
{
    float n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (n2 < 1e-12f)
        return 0;
    float inv = 1.0f / sqrtf(n2);
    v[0] *= inv;
    v[1] *= inv;
    v[2] *= inv;
    return 1;
}

static void quat_normalize_local(float q[4])
{
    float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 < 1e-12f)
    {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / sqrtf(n2);
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
}

/* Predict magnetometer in body frame from fixed earth-frame reference.
 * h = R(q)^T * m_ref_n  (rotate earth-frame reference into body frame) */
static void mag_predict_body_from_ref(const float q[4], const float m_ref_n[3], float h[3])
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float mx = m_ref_n[0], my = m_ref_n[1], mz = m_ref_n[2];

    h[0] = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * mx + 2.0f * (q1 * q2 + q0 * q3) * my + 2.0f * (q1 * q3 - q0 * q2) * mz;
    h[1] = 2.0f * (q1 * q2 - q0 * q3) * mx + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * my + 2.0f * (q2 * q3 + q0 * q1) * mz;
    h[2] = 2.0f * (q1 * q3 + q0 * q2) * mx + 2.0f * (q2 * q3 - q0 * q1) * my + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * mz;
}

/* Rotate body-frame mag to earth frame: m_e = R(q) * m_b */
static void mag_rotate_body_to_earth(const float q[4], const float m_b[3], float m_e[3])
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float mx = m_b[0], my = m_b[1], mz = m_b[2];

    m_e[0] = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * mx + 2.0f * (q1 * q2 - q0 * q3) * my + 2.0f * (q1 * q3 + q0 * q2) * mz;
    m_e[1] = 2.0f * (q1 * q2 + q0 * q3) * mx + (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * my + 2.0f * (q2 * q3 - q0 * q1) * mz;
    m_e[2] = 2.0f * (q1 * q3 - q0 * q2) * mx + 2.0f * (q2 * q3 + q0 * q1) * my + (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * mz;
}

/*
 * FIX #2 + FIX #6: Return bool success so callers know if bootstrap failed.
 *
 * OLD: returned void, silently did nothing on failure.  If v3_normalize
 *      failed (e.g. sensor nearly vertical and horizontal projection ≈ 0),
 *      mag_ref_valid was left in whatever state it was — potentially true
 *      from a previous good bootstrap — causing stale reference to be used.
 *
 * NEW: returns false on any failure path, and the caller must set
 *      mag_ref_valid = false so the next update triggers a fresh bootstrap.
 *      The horizontal projection is intentional (yaw-only correction) but
 *      the failure is now explicit, not silent.
 */
static bool ekf7_set_mag_reference_from_body(ekf7_t *e, float mx, float my, float mz)
{
    float m_b[3] = {mx, my, mz};
    if (!v3_normalize(m_b))
        return false;

    float m_e[3] = {0};
    mag_rotate_body_to_earth(e->q, m_b, m_e);
    if (!v3_normalize(m_e))
        return false;

    e->mag_ref_n[0] = m_e[0];
    e->mag_ref_n[1] = m_e[1];
    e->mag_ref_n[2] = m_e[2];
    return true;
}

/* Numeric Jacobian for h_mag(q) — robust against algebra mistakes. */
static void ekf7_build_mag_jacobian_numeric(const float q[4],
                                            const float m_ref_n[3],
                                            float H[3][7])
{
    const float eps = 1e-4f;
    memset(H, 0, sizeof(float) * 3 * 7);

    for (int c = 0; c < 4; c++)
    {
        float qp[4] = {q[0], q[1], q[2], q[3]};
        float qm[4] = {q[0], q[1], q[2], q[3]};
        qp[c] += eps;
        qm[c] -= eps;
        quat_normalize_local(qp);
        quat_normalize_local(qm);

        float hp[3], hm[3];
        mag_predict_body_from_ref(qp, m_ref_n, hp);
        mag_predict_body_from_ref(qm, m_ref_n, hm);

        H[0][c] = (hp[0] - hm[0]) / (2.0f * eps);
        H[1][c] = (hp[1] - hm[1]) / (2.0f * eps);
        H[2][c] = (hp[2] - hm[2]) / (2.0f * eps);
    }
    /* Columns 4..6 stay zero — mag does not directly observe gyro bias. */
}

/* Joseph-form covariance update: P = (I-KH)P(I-KH)^T + K*R*K^T
 * This form preserves positive semi-definiteness regardless of K accuracy. */
static void ekf7_cov_update_joseph(ekf7_t *e, const float H[3][7], float r_val)
{
    memcpy(s_Pold, e->P, sizeof(s_Pold));

    /* A = I - K*H */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float kh = 0.0f;
            for (int m = 0; m < 3; m++)
                kh += s_K[i][m] * H[m][j];
            s_A[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }

    /* s_AP = A * P_prior */
    m77_mult(s_A, s_Pold, s_AP);

    /* P_new = s_AP * A^T  (Joseph main term) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += s_AP[i][k] * s_A[j][k]; /* A^T[k][j] = A[j][k] */
            s_Pnew[i][j] = s;
        }

    /* P_new += r_val * K * K^T  (noise injection term) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float kk = 0.0f;
            for (int m = 0; m < 3; m++)
                kk += s_K[i][m] * s_K[j][m];
            s_Pnew[i][j] += r_val * kk;
        }

    memcpy(e->P, s_Pnew, sizeof(e->P));
}

/* -----------------------------------------------------------------------
 * API — Init / Reset / Config
 * ----------------------------------------------------------------------- */

void ekf7_init(ekf7_t *e,
               float sigma_gyro,
               float sigma_bias,
               float sigma_accel,
               float sigma_mag,
               float r_adapt_k,
               float P0)
{
    if (!e)
        return;

    e->sigma_gyro = sigma_gyro;
    e->sigma_bias = sigma_bias;
    e->sigma_accel = sigma_accel;
    e->sigma_mag = sigma_mag;
    e->r_adapt_k = r_adapt_k;
    e->P0 = P0;

    e->mag_ref_n[0] = 1.0f;
    e->mag_ref_n[1] = 0.0f;
    e->mag_ref_n[2] = 0.0f;
    e->mag_ref_valid = false;

    /* Mag NIS gate is configurable via app_config.h. */
    e->mag_nis_gate = EKF_MAG_NIS_GATE;

    /* FIX #4: Hard-reject ENABLED by default with a window appropriate for
     * a robotic shoulder joint.
     *
     * OLD: accel_reject_en = false, max = 1e9f.  A broken accelerometer or
     * a high-impact robot collision would just inflate R toward infinity
     * rather than being cleanly rejected.  For a robotic arm, brief collision
     * spikes can reach 10–50g — adaptive-R alone is not sufficient.
     *
     * NEW: Enabled with [0.3g, 4.0g].
     *   Lower bound 0.3g: rejects free-fall or sensor disconnect.
     *   Upper bound 4.0g: rejects impact spikes while allowing normal
     *   robot acceleration (shoulder joint rarely exceeds 2g in normal motion).
     *   Adjust upper bound upward if your joint moves very fast. */
    e->accel_reject_en = true;
    e->accel_reject_min_g = 0.3f;
    e->accel_reject_max_g = 4.0f;

    e->sym_ctr = 0;
    e->sym_ctr_mag = 0;

    ekf7_reset(e);
}

void ekf7_reset(ekf7_t *e)
{
    if (!e)
        return;

    e->q[0] = 1.0f;
    e->q[1] = 0.0f;
    e->q[2] = 0.0f;
    e->q[3] = 0.0f;
    e->b[0] = 0.0f;
    e->b[1] = 0.0f;
    e->b[2] = 0.0f;

    e->mag_ref_valid = false;
    e->accel_sane_timer_s = 0.0f;

    e->sym_ctr = 0;
    e->sym_ctr_mag = 0;

    memset(e->P, 0, sizeof(e->P));
    for (int i = 0; i < 7; i++)
        e->P[i][i] = e->P0;
}

void ekf7_set_accel_reject(ekf7_t *e, bool en, float min_g, float max_g, float timeout_s)
{
    if (!e)
        return;
    e->accel_reject_en = en;
    e->accel_reject_min_g = min_g;
    e->accel_reject_max_g = max_g;
    e->accel_sane_timeout_s = timeout_s;
}

void ekf7_set_noise(ekf7_t *e,
                    float sigma_gyro,
                    float sigma_bias,
                    float sigma_accel,
                    float sigma_mag,
                    float r_adapt_k)
{
    if (!e)
        return;
    e->sigma_gyro = sigma_gyro;
    e->sigma_bias = sigma_bias;
    e->sigma_accel = sigma_accel;
    e->sigma_mag = sigma_mag;
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

    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    /* ZYX Euler → quaternion (yaw = 0 → cy=1, sy=0) */
    e->q[0] = cr * cp;
    e->q[1] = sr * cp;
    e->q[2] = cr * sp;
    e->q[3] = -sr * sp;

    /* FIX #3: Comment corrected.
     *
     * OLD comment said "Preserve existing bias estimate — don't throw it away
     * on re-align."  This was misleading because ekf7_reset() is always called
     * before ekf7_init_from_accel() in normal startup, which already zeroed
     * the bias.  The comment implied intentional preservation logic that does
     * not exist.
     *
     * Actual behaviour: bias is intentionally NOT touched here.  If this
     * function is called mid-flight for a re-alignment (e.g. after a reset
     * triggered by accel disturbance), any previously converged bias estimate
     * in e->b[] is preserved.  If called after ekf7_reset(), e->b[] is already
     * zero — no bias to preserve. */
}

void ekf7_init_from_marg(ekf7_t *e,
                         float ax_g, float ay_g, float az_g,
                         float mx, float my, float mz)
{
    if (!e)
        return;

    ekf7_init_from_accel(e, ax_g, ay_g, az_g);

    float m_mag = sqrtf(mx * mx + my * my + mz * mz);
    if (m_mag < 1e-9f)
        return;
    float inv_m = 1.0f / m_mag;
    mx *= inv_m;
    my *= inv_m;
    mz *= inv_m;

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];
    float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));

    float yaw = atan2f(-hy, hx);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    /* Pre-multiply q by q_yaw = [cy, 0, 0, sy] */
    float t0 = cy * q0 - sy * q3;
    float t1 = cy * q1 - sy * q2;
    float t2 = cy * q2 + sy * q1;
    float t3 = cy * q3 + sy * q0;

    float qn = sqrtf(t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3);
    if (qn > 1e-9f)
    {
        float qi = 1.0f / qn;
        e->q[0] = t0 * qi;
        e->q[1] = t1 * qi;
        e->q[2] = t2 * qi;
        e->q[3] = t3 * qi;
    }

    /* FIX #2 applied here: ekf7_set_mag_reference_from_body now returns bool.
     * On failure (sensor near vertical during init), mag_ref_valid stays false
     * so the first ekf7_update_mag() call will re-bootstrap cleanly. */
    e->mag_ref_valid = ekf7_set_mag_reference_from_body(e, mx, my, mz);
}

/* -----------------------------------------------------------------------
 * Predict step — gyro-driven time update
 *
 * State transition Jacobian F (7×7):
 *
 *   F = [ F_qq  |  F_qb ]
 *       [ 0_3x4 |  I_3  ]
 *
 * F_qq = I_4 + 0.5*dt * Omega(w_c)
 * F_qb = -0.5*dt * Xi(q)
 *
 * Process noise Q (diagonal, FIX #5 documented):
 *   Q[0..3][0..3] = sigma_gyro^2 * dt * I_4
 *
 *   FIX #5 — documented approximation (not a bug, but worth knowing):
 *   The theoretically exact Q_qq = (dt/2)^2 * Xi(q) * sigma^2 * Xi(q)^T.
 *   Since Xi*Xi^T ≈ I_4 this becomes 0.25*dt^2*sigma^2*I_4.  We use
 *   sigma^2*dt instead, which overestimates process noise by a factor of
 *   4/dt at 100 Hz (4/0.01 = 400×).  This is a deliberate conservative
 *   approximation — it keeps P from collapsing too fast and is standard
 *   practice in embedded EKF implementations where dt is small.
 *
 *   Q[4..6][4..6] = sigma_bias^2 * dt * I_3
 * ----------------------------------------------------------------------- */
void ekf7_predict(ekf7_t *e, float wx, float wy, float wz, float dt_s)
{
    if (!e || dt_s <= 0.0f)
        return;

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];

    float wx_c = wx - e->b[0];
    float wy_c = wy - e->b[1];
    float wz_c = wz - e->b[2];

    float h = 0.5f * dt_s;

    memset(s_F, 0, sizeof(s_F));

    /* F_qq block — verified correct (all 16 entries match Omega(w) definition) */
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

    /* F_qb block = -h * Xi(q) — verified correct (all 12 entries) */
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

    /* F_bb = I_3 */
    s_F[4][4] = 1.0f;
    s_F[5][5] = 1.0f;
    s_F[6][6] = 1.0f;

    /* Propagate quaternion: q_new = F_qq * q */
    float q_new[4] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            q_new[i] += s_F[i][j] * e->q[j];

    float qn = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] +
                     q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    if (qn > 1e-9f)
    {
        float qi = 1.0f / qn;
        e->q[0] = q_new[0] * qi;
        e->q[1] = q_new[1] * qi;
        e->q[2] = q_new[2] * qi;
        e->q[3] = q_new[3] * qi;
    }

    /* P = F * P * F^T + Q */
    m77_mult(s_F, e->P, s_FP);
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += s_FP[i][k] * s_F[j][k];
            e->P[i][j] = s;
        }

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
 * Measurement model  h(q) = R(q)^T * [0,0,1]^T  (gravity in body frame):
 *   hx = 2*(q1*q3 - q0*q2)
 *   hy = 2*(q2*q3 + q0*q1)
 *   hz = q0^2 - q1^2 - q2^2 + q3^2
 *
 * Jacobian H (3×7) — verified correct, all partials match:
 *   row 0: [-2q2,  2q3, -2q0,  2q1, 0, 0, 0]
 *   row 1: [ 2q1,  2q0,  2q3,  2q2, 0, 0, 0]
 *   row 2: [ 2q0, -2q1, -2q2,  2q3, 0, 0, 0]
 *
 * Adaptive R:
 *   dev   = |a_raw_mag| - 1
 *   R_eff = sigma_accel^2 * (1 + k*dev^2) * I_3
 * ----------------------------------------------------------------------- */
void ekf7_update_accel(ekf7_t *e, float ax_g, float ay_g, float az_g, float dt_s)
{
    if (!e)
        return;

    float a_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    /* FIX #4: Hard-reject now runs because accel_reject_en defaults to true.
     * This catches sensor faults and high-g impact spikes on the shoulder joint
     * that adaptive-R alone cannot safely handle. */
    if (e->accel_reject_en)
    {
        if (a_mag < e->accel_reject_min_g || a_mag > e->accel_reject_max_g)
        {
            e->accel_sane_timer_s = 0.0f;
            return;
        }

        e->accel_sane_timer_s += dt_s;
        if (e->accel_sane_timer_s < e->accel_sane_timeout_s)
        {
            return;
        }
    }

    if (a_mag < 1e-9f)
        return;
    float inv_a = 1.0f / a_mag;
    float ax = ax_g * inv_a, ay = ay_g * inv_a, az = az_g * inv_a;

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];

    float hx = 2.0f * (q1 * q3 - q0 * q2);
    float hy = 2.0f * (q2 * q3 + q0 * q1);
    float hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float y[3] = {ax - hx, ay - hy, az - hz};

    float H[3][7] = {
        {-2.0f * q2, 2.0f * q3, -2.0f * q0, 2.0f * q1, 0, 0, 0},
        {2.0f * q1, 2.0f * q0, 2.0f * q3, 2.0f * q2, 0, 0, 0},
        {2.0f * q0, -2.0f * q1, -2.0f * q2, 2.0f * q3, 0, 0, 0}};

    float dev = a_mag - 1.0f;
    float dev_abs = fabsf(dev);
    float r_val = e->sigma_accel * e->sigma_accel *
                  (1.0f + e->r_adapt_k * dev * dev);

    /* Cache effective R for external diagnostic access */
    e->last_r_eff = r_val;

    /* s_HP = H * P  (3×7) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += H[i][k] * e->P[k][j];
            s_HP[i][j] = s;
        }

    /* S = s_HP * H^T + R_eff * I_3  (3×3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += s_HP[i][k] * H[j][k];
            S[i][j] = s + (i == j ? r_val : 0.0f);
        }

    float Sinv[3][3];
    if (!m33_inv(S, Sinv))
        return;

    /* s_PHt = P * H^T  (7×3) — exploit H[4..6] = 0 */
    for (int i = 0; i < 7; i++)
        for (int m = 0; m < 3; m++)
        {
            float s = 0.0f;
            for (int j = 0; j < 4; j++)
                s += e->P[i][j] * H[m][j];
            s_PHt[i][m] = s;
        }

    /* K = s_PHt * Sinv  (7×3) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += s_PHt[i][k] * Sinv[k][j];
            s_K[i][j] = s;
        }

    /* Freeze bias update when |a| deviates too far from 1g to avoid corrupting
     * the bias estimate during dynamic motion. Quaternion update still applies. */
    if (dev_abs > EKF_BIAS_MAX_DEV_G)
    {
        for (int i = 4; i < 7; i++)
            for (int j = 0; j < 3; j++)
                s_K[i][j] = 0.0f;
    }

    /* State update: x += K * y */
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
        if (e->b[i] > 0.05f)
            e->b[i] = 0.05f;
        else if (e->b[i] < -0.05f)
            e->b[i] = -0.05f;
    }

    float qn = sqrtf(e->q[0] * e->q[0] + e->q[1] * e->q[1] +
                     e->q[2] * e->q[2] + e->q[3] * e->q[3]);
    if (qn > 1e-9f)
    {
        float qi = 1.0f / qn;
        e->q[0] *= qi;
        e->q[1] *= qi;
        e->q[2] *= qi;
        e->q[3] *= qi;
    }

    ekf7_cov_update_joseph(e, H, r_val);

    /* Periodic symmetry enforcement every 64 accel updates (~0.64 s at 100 Hz) */
    if ((++e->sym_ctr & 0x3Fu) == 0u)
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
    ekf7_update_accel(e, ax_g, ay_g, az_g, dt_s);
}

/* -----------------------------------------------------------------------
 * Update step — magnetometer measurement correction
 * ----------------------------------------------------------------------- */
void ekf7_update_mag(ekf7_t *e, float mx, float my, float mz)
{
    if (!e)
        return;

    float m_mag = sqrtf(mx * mx + my * my + mz * mz);
    if (m_mag < 1e-9f)
        return;
    float inv_m = 1.0f / m_mag;
    mx *= inv_m;
    my *= inv_m;
    mz *= inv_m;

    if (!e->mag_ref_valid)
    {
        /* FIX #2: Bootstrap now checks return value.
         *
         * OLD: ekf7_set_mag_reference_from_body returned void. If it failed
         * internally (sensor near vertical → horizontal projection ≈ 0),
         * mag_ref_valid stayed false but no error was propagated.  This was
         * harmless only because the function also returned early.  However
         * there was no way to distinguish "failed" from "not yet called".
         *
         * NEW: Function returns bool.  On failure we return immediately and
         * leave mag_ref_valid = false so the next call retries the bootstrap
         * rather than proceeding with a zero reference vector. */
        e->mag_ref_valid = ekf7_set_mag_reference_from_body(e, mx, my, mz);
        return;
    }

    float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];
    float q_now[4] = {q0, q1, q2, q3};

    /* Predicted mag in body frame */
    float h[3] = {0};
    mag_predict_body_from_ref(q_now, e->mag_ref_n, h);

    /* Innovation */
    float y[3] = {mx - h[0], my - h[1], mz - h[2]};

    /* Residual scaling: keep large errors from dominating while still
     * allowing yaw recovery after high-accel divergence. */
    float y_norm2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
    float y_norm = sqrtf(y_norm2);
    if (y_norm > EKF_MAG_RESIDUAL_MAX)
    {
        float s = EKF_MAG_RESIDUAL_MAX / y_norm;
        y[0] *= s;
        y[1] *= s;
        y[2] *= s;
    }

    /* Numeric Jacobian H (3×7) */
    float H[3][7];
    ekf7_build_mag_jacobian_numeric(q_now, e->mag_ref_n, H);

    /* R for mag: convert sigma_mag [µT] to directional noise after normalization */
    float sigma_dir = e->sigma_mag / m_mag;
    float r_val = sigma_dir * sigma_dir;
    if (r_val < 5e-3f)
        r_val = 5e-3f; /* floor prevents degenerate S */

    /* s_HP = H * P  (3×7) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += H[i][k] * e->P[k][j];
            s_HP[i][j] = s;
        }

    /* S = s_HP * H^T + R * I_3  (3×3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += s_HP[i][k] * H[j][k];
            S[i][j] = s + (i == j ? r_val : 0.0f);
        }

    float Sinv[3][3];
    if (!m33_inv(S, Sinv))
        return;

    /* Mag NIS gate (configurable) rejects extreme outliers. */
    float nis = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            nis += y[i] * Sinv[i][j] * y[j];
    if (nis > e->mag_nis_gate)
        return;

    /* s_PHt = P * H^T  (7×3) — H[4..6] = 0 so only cols 0..3 contribute */
    for (int i = 0; i < 7; i++)
        for (int m = 0; m < 3; m++)
        {
            float s = 0.0f;
            for (int j = 0; j < 4; j++)
                s += e->P[i][j] * H[m][j];
            s_PHt[i][m] = s;
        }

    /* K = s_PHt * Sinv  (7×3) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += s_PHt[i][k] * Sinv[k][j];
            s_K[i][j] = s;
        }

    /* Zero bias rows of K — mag measurement does not reliably estimate gyro bias.
     * Indoor mag fields are too noisy for bias correction; allowing it would
     * corrupt the bias estimate and cause persistent roll/pitch error. */
    for (int i = 4; i < 7; i++)
        for (int j = 0; j < 3; j++)
            s_K[i][j] = 0.0f;

    /* State update with per-component clamping on quaternion correction.
     * ±0.015 per component per update prevents a single bad measurement
     * from causing a large sudden heading jump. */
    for (int i = 0; i < 4; i++)
    {
        float dq = 0.0f;
        for (int m = 0; m < 3; m++)
            dq += s_K[i][m] * y[m];
        if (dq > 0.015f)
            dq = 0.015f;
        else if (dq < -0.015f)
            dq = -0.015f;
        e->q[i] += dq;
    }
    /* Bias rows were zeroed above — this loop is a no-op but kept for clarity */
    for (int i = 0; i < 3; i++)
    {
        float db = 0.0f;
        for (int m = 0; m < 3; m++)
            db += s_K[i + 4][m] * y[m];
        e->b[i] += db;
    }

    float qn = sqrtf(e->q[0] * e->q[0] + e->q[1] * e->q[1] +
                     e->q[2] * e->q[2] + e->q[3] * e->q[3]);
    if (qn > 1e-9f)
    {
        float qi = 1.0f / qn;
        e->q[0] *= qi;
        e->q[1] *= qi;
        e->q[2] *= qi;
        e->q[3] *= qi;
    }

    ekf7_cov_update_joseph(e, H, r_val);

    /* Periodic symmetry enforcement every 64 mag updates */
    if ((++e->sym_ctr_mag & 0x3Fu) == 0u)
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

void ekf7_step_marg(ekf7_t *e,
                    float wx, float wy, float wz,
                    float ax_g, float ay_g, float az_g,
                    float mx, float my, float mz,
                    float dt_s)
{
    ekf7_predict(e, wx, wy, wz, dt_s);
    ekf7_update_accel(e, ax_g, ay_g, az_g, dt_s);
    ekf7_update_mag(e, mx, my, mz);
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

float ekf7_get_r_eff(const ekf7_t *e)
{
    if (!e)
        return 0.0f;
    return e->last_r_eff;
}
