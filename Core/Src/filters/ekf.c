#include "filters/ekf.h"
#include "app/app_config.h"
#include "utils/math3d.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Scratch buffers are now stored inside each ekf7_t instance (fields
 * prefixed with '_') so that multiple IMU instances can be updated
 * concurrently from different RTOS tasks without corrupting each other.
 * See ekf.h for the full rationale and memory cost breakdown.
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 * Local helpers
 * ----------------------------------------------------------------------- */

static void m77_mult(const float A[7][7], const float B[7][7], float C[7][7])
{
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            /* Trip count is compile-time constant (7) — unroll eliminates
             * all SUB/CMP/BNE branch overhead from the innermost loop. */
#pragma GCC unroll 7
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

    if (fabsf(d) < 1e-12f)
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
    float inv = math3d_inv_sqrtf(n2);
    v[0] *= inv;
    v[1] *= inv;
    v[2] *= inv;
    return 1;
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

/* Analytic Jacobian for h_mag(q) = R(q)^T * m_ref_n.
 *
 * Derivation: h = R^T * m where R is the rotation matrix from q.
 * Closed-form partials ∂h/∂q_i (bias columns 4..6 remain zero):
 *
 *   H[0][0] =  2*(my*q3 - mz*q2)
 *   H[0][1] =  2*(my*q2 + mz*q3)
 *   H[0][2] =  2*(-2*mx*q2 + my*q1 - mz*q0)
 *   H[0][3] =  2*(-2*mx*q3 + my*q0 + mz*q1)
 *
 *   H[1][0] =  2*(-mx*q3 + mz*q1)
 *   H[1][1] =  2*(mx*q2 - 2*my*q1 + mz*q0)
 *   H[1][2] =  2*(mx*q1 + mz*q3)
 *   H[1][3] =  2*(-mx*q0 - 2*my*q3 + mz*q2)
 *
 *   H[2][0] =  2*(mx*q2 - my*q1)
 *   H[2][1] =  2*(mx*q3 - my*q0 - 2*mz*q1)
 *   H[2][2] =  2*(mx*q0 + my*q3 - 2*mz*q2)
 *   H[2][3] =  2*(mx*q1 + my*q2)
 *
 * Replaces the previous 8-evaluation numeric central-difference. */
static void ekf7_build_mag_jacobian(const float q[4],
                                    const float m_ref_n[3],
                                    float H[3][7])
{
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    const float mx = m_ref_n[0], my = m_ref_n[1], mz = m_ref_n[2];

    memset(H, 0, sizeof(float) * 3 * 7);

    H[0][0] =  2.0f * ( my * q3 - mz * q2);
    H[0][1] =  2.0f * ( my * q2 + mz * q3);
    H[0][2] =  2.0f * (-2.0f * mx * q2 + my * q1 - mz * q0);
    H[0][3] =  2.0f * (-2.0f * mx * q3 + my * q0 + mz * q1);

    H[1][0] =  2.0f * (-mx * q3 + mz * q1);
    H[1][1] =  2.0f * ( mx * q2 - 2.0f * my * q1 + mz * q0);
    H[1][2] =  2.0f * ( mx * q1 + mz * q3);
    H[1][3] =  2.0f * (-mx * q0 - 2.0f * my * q3 + mz * q2);

    H[2][0] =  2.0f * ( mx * q2 - my * q1);
    H[2][1] =  2.0f * ( mx * q3 - my * q0 - 2.0f * mz * q1);
    H[2][2] =  2.0f * ( mx * q0 + my * q3 - 2.0f * mz * q2);
    H[2][3] =  2.0f * ( mx * q1 + my * q2);
    /* Columns 4..6 remain zero — mag does not directly observe gyro bias. */
}

/* Joseph-form covariance update: P = (I-KH)P(I-KH)^T + K*R*K^T
 * This form preserves positive semi-definiteness regardless of K accuracy.
 * Uses per-instance scratch fields e->_A, _AP, _Pnew, _Pold, _K. */
static void ekf7_cov_update_joseph(ekf7_t *e, const float H[3][7], float r_val)
{
    memcpy(e->_Pold, e->P, sizeof(e->_Pold));

    /* A = I - K*H */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float kh = 0.0f;
            for (int m = 0; m < 3; m++)
                kh += e->_K[i][m] * H[m][j];
            e->_A[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }

    /* _AP = A * P_prior */
    m77_mult(e->_A, e->_Pold, e->_AP);

    /* P_new = _AP * A^T  (Joseph main term) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += e->_AP[i][k] * e->_A[j][k]; /* A^T[k][j] = A[j][k] */
            e->_Pnew[i][j] = s;
        }

    /* P_new += r_val * K * K^T  (noise injection term) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float kk = 0.0f;
            for (int m = 0; m < 3; m++)
                kk += e->_K[i][m] * e->_K[j][m];
            e->_Pnew[i][j] += r_val * kk;
        }

    memcpy(e->P, e->_Pnew, sizeof(e->P));
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

    e->sigma_gyro  = sigma_gyro;
    e->sigma_bias  = sigma_bias;
    e->sigma_accel = sigma_accel;
    e->sigma_mag   = sigma_mag;
    e->r_adapt_k   = r_adapt_k;
    e->P0          = P0;

    e->mag_ref_n[0] = 1.0f;
    e->mag_ref_n[1] = 0.0f;
    e->mag_ref_n[2] = 0.0f;
    e->mag_ref_valid = false;

    /* Mag NIS gate — from app_config.h. */
    e->mag_nis_gate = EKF_MAG_NIS_GATE;

    /* Accel hard-reject window — from app_config.h (B1 + B2 fix). */
    e->accel_reject_en      = (bool)EKF_ACCEL_REJECT_EN;
    e->accel_reject_min_g   = EKF_ACCEL_MIN_G;
    e->accel_reject_max_g   = EKF_ACCEL_MAX_G;
    e->accel_sane_timeout_s = EKF_ACCEL_TIMEOUT_S; /* B2: was never initialised */

    e->sym_ctr     = 0;
    e->sym_ctr_mag = 0;

    /* Health / convergence flag — cleared on init, set by update steps. */
    e->healthy     = false;
    e->healthy_ctr = 0u;

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
    /* D4: quaternion and bias states have very different initial uncertainties.
     * Using a single P0 for both over-inflates bias variance and causes
     * over-aggressive bias estimation in the first few seconds. */
    for (int i = 0; i < 4; i++)
        e->P[i][i] = e->P0;        /* quaternion uncertainty */
    for (int i = 4; i < 7; i++)
        e->P[i][i] = EKF_P0_BIAS;  /* bias uncertainty — much tighter */

    /* Reset health on state reset. */
    e->healthy     = false;
    e->healthy_ctr = 0u;
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

    memset(e->_F, 0, sizeof(e->_F));

    /* F_qq block — verified correct (all 16 entries match Omega(w) definition) */
    e->_F[0][0] = 1.0f;
    e->_F[0][1] = -h * wx_c;
    e->_F[0][2] = -h * wy_c;
    e->_F[0][3] = -h * wz_c;
    e->_F[1][0] = h * wx_c;
    e->_F[1][1] = 1.0f;
    e->_F[1][2] = h * wz_c;
    e->_F[1][3] = -h * wy_c;
    e->_F[2][0] = h * wy_c;
    e->_F[2][1] = -h * wz_c;
    e->_F[2][2] = 1.0f;
    e->_F[2][3] = h * wx_c;
    e->_F[3][0] = h * wz_c;
    e->_F[3][1] = h * wy_c;
    e->_F[3][2] = -h * wx_c;
    e->_F[3][3] = 1.0f;

    /* F_qb block = -h * Xi(q) — verified correct (all 12 entries) */
    e->_F[0][4] = h * q1;
    e->_F[0][5] = h * q2;
    e->_F[0][6] = h * q3;
    e->_F[1][4] = -h * q0;
    e->_F[1][5] = h * q3;
    e->_F[1][6] = -h * q2;
    e->_F[2][4] = -h * q3;
    e->_F[2][5] = -h * q0;
    e->_F[2][6] = h * q1;
    e->_F[3][4] = h * q2;
    e->_F[3][5] = -h * q1;
    e->_F[3][6] = -h * q0;

    /* F_bb = I_3 */
    e->_F[4][4] = 1.0f;
    e->_F[5][5] = 1.0f;
    e->_F[6][6] = 1.0f;

    /* Propagate quaternion: q_new = F_qq * q */
    float q_new[4] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            q_new[i] += e->_F[i][j] * e->q[j];

    float qn2 = q_new[0] * q_new[0] + q_new[1] * q_new[1] +
                 q_new[2] * q_new[2] + q_new[3] * q_new[3];
    if (qn2 > 1e-18f)
    {
        float qi = math3d_inv_sqrtf(qn2);
        e->q[0] = q_new[0] * qi;
        e->q[1] = q_new[1] * qi;
        e->q[2] = q_new[2] * qi;
        e->q[3] = q_new[3] * qi;
    }

    /* P = F * P * F^T + Q */
    m77_mult(e->_F, e->P, e->_FP);
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            /* Same trip-count-7 unroll as in m77_mult — eliminates branch
             * overhead on the hottest loop in the predict step. */
#pragma GCC unroll 7
            for (int k = 0; k < 7; k++)
                s += e->_FP[i][k] * e->_F[j][k];
            e->P[i][j] = s;
        }

    /* ---- Q injection: P += Q_d ----
     *
     * Exact discrete-time Q derived from the noise input model:
     *   q_dot = 0.5 * Xi(q) * (omega_true)   <- gyro noise enters here
     *   b_dot = n_b                            <- bias random-walk
     *
     * Q_qq = sigma_gyro^2 * dt * (I_4 - q*q^T)
     *
     * The (I_4 - q*q^T) term comes from Xi(q)*Xi(q)^T = I_4 - q*q^T for a
     * unit quaternion.  It correctly captures two things:
     *   1. Off-diagonal coupling between quaternion components.
     *   2. Zero variance along the quaternion direction (unit-norm constraint).
     *
     * Note: the 0.25 factor from the exact Xi derivation is absorbed into
     * sigma_gyro so the tuned EKF_SIGMA_GYRO parameter keeps its meaning.
     *
     * Bias block: sigma_bias^2 * dt * I_3  (unchanged, independent noise). */
    {
        const float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];
        const float qv =
            0.25f *
            e->sigma_gyro *
            e->sigma_gyro *
            dt_s;

        /* Diagonal: qv * (1 - q_i^2) */
        e->P[0][0] += qv * (1.0f - q0 * q0);
        e->P[1][1] += qv * (1.0f - q1 * q1);
        e->P[2][2] += qv * (1.0f - q2 * q2);
        e->P[3][3] += qv * (1.0f - q3 * q3);

        /* Off-diagonal: -qv * q_i * q_j  (symmetric) */
        float c01 = qv * q0 * q1;
        float c02 = qv * q0 * q2;
        float c03 = qv * q0 * q3;
        float c12 = qv * q1 * q2;
        float c13 = qv * q1 * q3;
        float c23 = qv * q2 * q3;

        e->P[0][1] -= c01;  e->P[1][0] -= c01;
        e->P[0][2] -= c02;  e->P[2][0] -= c02;
        e->P[0][3] -= c03;  e->P[3][0] -= c03;
        e->P[1][2] -= c12;  e->P[2][1] -= c12;
        e->P[1][3] -= c13;  e->P[3][1] -= c13;
        e->P[2][3] -= c23;  e->P[3][2] -= c23;

        /* Bias random-walk: diagonal only */
        const float b_var = e->sigma_bias * e->sigma_bias * dt_s;
        e->P[4][4] += b_var;
        e->P[5][5] += b_var;
        e->P[6][6] += b_var;
    }
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

    /* _HP = H * P  (3×7) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += H[i][k] * e->P[k][j];
            e->_HP[i][j] = s;
        }

    /* S = _HP * H^T + R_eff * I_3  (3×3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += e->_HP[i][k] * H[j][k];
            S[i][j] = s + (i == j ? r_val : 0.0f);
        }

    float Sinv[3][3];
    if (!m33_inv(S, Sinv))
        return;

    /* _PHt = P * H^T  (7×3) — exploit H[4..6] = 0 */
    for (int i = 0; i < 7; i++)
        for (int m = 0; m < 3; m++)
        {
            float s = 0.0f;
            for (int j = 0; j < 4; j++)
                s += e->P[i][j] * H[m][j];
            e->_PHt[i][m] = s;
        }

    /* _K = _PHt * Sinv  (7×3) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += e->_PHt[i][k] * Sinv[k][j];
            e->_K[i][j] = s;
        }

    /* Freeze bias update when |a| deviates too far from 1g to avoid corrupting
     * the bias estimate during dynamic motion. Quaternion update still applies. */
    if (dev_abs > EKF_BIAS_MAX_DEV_G)
    {
        for (int i = 4; i < 7; i++)
            for (int j = 0; j < 3; j++)
                e->_K[i][j] = 0.0f;
    }

    /* State update: x += K * y */
    for (int i = 0; i < 4; i++)
    {
        float dq = 0.0f;
        for (int m = 0; m < 3; m++)
            dq += e->_K[i][m] * y[m];
        e->q[i] += dq;
    }
    for (int i = 0; i < 3; i++)
    {
        float db = 0.0f;
        for (int m = 0; m < 3; m++)
            db += e->_K[i + 4][m] * y[m];
        e->b[i] += db;
        /* B3: clamp via named macro instead of magic number */
        if      (e->b[i] >  EKF_BIAS_CLAMP_RAD_S) e->b[i] =  EKF_BIAS_CLAMP_RAD_S;
        else if (e->b[i] < -EKF_BIAS_CLAMP_RAD_S) e->b[i] = -EKF_BIAS_CLAMP_RAD_S;
    }

    float qn2 = e->q[0] * e->q[0] + e->q[1] * e->q[1] +
                e->q[2] * e->q[2] + e->q[3] * e->q[3];
    if (qn2 > 1e-18f)
    {
        float qi = math3d_inv_sqrtf(qn2);
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

    /* D5: update health/convergence flag with 10-step hysteresis */
    if (ekf7_trace_P(e) < EKF_CONVERGENCE_TRACE)
    {
        if (++e->healthy_ctr >= 10u)
            e->healthy = true;
    }
    else
    {
        e->healthy_ctr = 0u;
        e->healthy     = false;
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

    /* ---- NaN / divergence guard ----
     *
     * A single I2C glitch can inject a NaN into the sensor reading.  Once NaN
     * enters q[] or P[][], every subsequent multiply propagates it and the
     * filter permanently outputs NaN — never recovering without a power cycle.
     *
     * If we detect NaN or trace(P) beyond EKF_MAX_TRACE, we force a hard reset
     * and re-align from the current accel reading.  The filter loses ~1 s of
     * convergence but resumes producing valid output immediately, which is far
     * better than permanently dead output for the rest of the session. */
    float tr = ekf7_trace_P(e);
    if (isnanf(tr) || tr > EKF_MAX_TRACE)
    {
        ekf7_reset(e);
        ekf7_init_from_accel(e, ax_g, ay_g, az_g);
    }
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
        if (!e->healthy)
            return;

        e->mag_ref_valid =
            ekf7_set_mag_reference_from_body(e,mx,my,mz);

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

    /* Analytic Jacobian H (3×7) — closed-form, ~8× fewer FLOPs than numeric. */
    float H[3][7];
    ekf7_build_mag_jacobian(q_now, e->mag_ref_n, H);

    /* B5: R for mag — derive noise variance in normalised (unit-vector) space.
     * sigma_mag is in µT; after normalising by m_mag the effective directional
     * std-dev scales as sigma_mag/m_mag, giving variance (sigma_mag/m_mag)^2.
     * This is consistent with the API contract (calibrated µT input). */
    float r_val = (e->sigma_mag / m_mag) * (e->sigma_mag / m_mag);
    if (r_val < 5e-3f)
        r_val = 5e-3f; /* floor prevents degenerate S when field is very strong */

    /* _HP = H * P  (3×7) */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 7; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += H[i][k] * e->P[k][j];
            e->_HP[i][j] = s;
        }

    /* S = _HP * H^T + R * I_3  (3×3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 7; k++)
                s += e->_HP[i][k] * H[j][k];
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

    /* _PHt = P * H^T  (7×3) — H[4..6] = 0 so only cols 0..3 contribute */
    for (int i = 0; i < 7; i++)
        for (int m = 0; m < 3; m++)
        {
            float s = 0.0f;
            for (int j = 0; j < 4; j++)
                s += e->P[i][j] * H[m][j];
            e->_PHt[i][m] = s;
        }

    /* _K = _PHt * Sinv  (7×3) */
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += e->_PHt[i][k] * Sinv[k][j];
            e->_K[i][j] = s;
        }

    /* Zero bias rows of K — mag measurement does not reliably estimate gyro bias.
     * Indoor mag fields are too noisy for bias correction; allowing it would
     * corrupt the bias estimate and cause persistent roll/pitch error. */
    for (int i = 4; i < 7; i++)
        for (int j = 0; j < 3; j++)
            e->_K[i][j] = 0.0f;

    /* State update with per-component clamping on quaternion correction.
     * B4: clamp via EKF_MAG_DQ_CLAMP (app_config.h) instead of magic ±0.015.
     * Prevents a single bad measurement from causing a large sudden heading jump. */
    for (int i = 0; i < 4; i++)
    {
        float dq = 0.0f;
        for (int m = 0; m < 3; m++)
            dq += e->_K[i][m] * y[m];
        if      (dq >  EKF_MAG_DQ_CLAMP) dq =  EKF_MAG_DQ_CLAMP;
        else if (dq < -EKF_MAG_DQ_CLAMP) dq = -EKF_MAG_DQ_CLAMP;
        e->q[i] += dq;
    }
    /* Bias rows were zeroed above — this loop is a no-op but kept for clarity */
    for (int i = 0; i < 3; i++)
    {
        float db = 0.0f;
        for (int m = 0; m < 3; m++)
            db += e->_K[i + 4][m] * y[m];
        e->b[i] += db;
    }

    float qn2 = e->q[0] * e->q[0] + e->q[1] * e->q[1] +
                e->q[2] * e->q[2] + e->q[3] * e->q[3];
    if (qn2 > 1e-18f)
    {
        float qi = math3d_inv_sqrtf(qn2);
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

    /* Same NaN / divergence guard as ekf7_step — see comment there. */
    float tr = ekf7_trace_P(e);
    if (isnanf(tr) || tr > EKF_MAX_TRACE)
    {
        ekf7_reset(e);
        ekf7_init_from_accel(e, ax_g, ay_g, az_g);
    }
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

bool ekf7_is_healthy(const ekf7_t *e)
{
    if (!e)
        return false;
    return e->healthy;
}
