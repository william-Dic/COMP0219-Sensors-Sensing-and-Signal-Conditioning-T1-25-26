#include "kalman2d.h"
#include <stddef.h>

void Kalman2D_Init(Kalman2D_t *kf,
                   float v0_init,
                   float a0_init,
                   float P0_v,
                   float P0_a,
                   float sigma_a2,
                   float R_plate,
                   float R_rpm)
{
  if (kf == NULL) {
    return;
  }

  kf->x[0] = v0_init;
  kf->x[1] = a0_init;

  kf->P00 = P0_v;
  kf->P01 = 0.0f;
  kf->P10 = 0.0f;
  kf->P11 = P0_a;

  kf->sigma_a2 = sigma_a2;
  kf->R_plate = R_plate;
  kf->R_rpm = R_rpm;
}

void Kalman2D_Predict(Kalman2D_t *kf, float dt_sec)
{
  if (kf == NULL) {
    return;
  }
  const float F00 = 1.0f;
  const float F01 = dt_sec;
  const float F10 = 0.0f;
  const float F11 = 1.0f;

  const float v_pred = F00 * kf->x[0] + F01 * kf->x[1];
  const float a_pred = F10 * kf->x[0] + F11 * kf->x[1];

  const float dt2 = dt_sec * dt_sec;
  const float dt3 = dt2 * dt_sec;
  const float dt4 = dt2 * dt2;

  const float Q00 = kf->sigma_a2 * (0.25f * dt4);
  const float Q01 = kf->sigma_a2 * (0.5f * dt3);
  const float Q10 = Q01;
  const float Q11 = kf->sigma_a2 * dt2;

  const float A00 = F00 * kf->P00 + F01 * kf->P10;
  const float A01 = F00 * kf->P01 + F01 * kf->P11;
  const float A10 = F10 * kf->P00 + F11 * kf->P10;
  const float A11 = F10 * kf->P01 + F11 * kf->P11;

  const float P00 = A00 * F00 + A01 * F01 + Q00;
  const float P01 = A00 * F10 + A01 * F11 + Q01;
  const float P10 = A10 * F00 + A11 * F01 + Q10;
  const float P11 = A10 * F10 + A11 * F11 + Q11;

  kf->x[0] = v_pred;
  kf->x[1] = a_pred;
  kf->P00 = P00;
  kf->P01 = P01;
  kf->P10 = P10;
  kf->P11 = P11;
}

static void Kalman2D_UpdateScalar(Kalman2D_t *kf, float z_meas, float R_meas)
{
  const float y = z_meas - kf->x[0];
  const float S = kf->P00 + R_meas;

  if (S <= 0.0f) {
    return;
  }

  const float invS = 1.0f / S;
  const float K0 = kf->P00 * invS;
  const float K1 = kf->P10 * invS;

  kf->x[0] += K0 * y;
  kf->x[1] += K1 * y;

  const float P00_new = (1.0f - K0) * kf->P00;
  const float P01_new = (1.0f - K0) * kf->P01;
  const float P10_new = kf->P10 - K1 * kf->P00;
  const float P11_new = kf->P11 - K1 * kf->P01;

  kf->P00 = P00_new;
  kf->P01 = P01_new;
  kf->P10 = P10_new;
  kf->P11 = P11_new;
}

void Kalman2D_UpdatePlate(Kalman2D_t *kf, float z_plate_mps)
{
  if (kf == NULL) {
    return;
  }
  Kalman2D_UpdateScalar(kf, z_plate_mps, kf->R_plate);
}

void Kalman2D_UpdateRPM(Kalman2D_t *kf, float z_rpm_mps, uint8_t rpm_is_reliable)
{
  if (kf == NULL) {
    return;
  }
  float R_eff = kf->R_rpm;
  if (!rpm_is_reliable) {
    R_eff = 1.0e6f;
  }
  Kalman2D_UpdateScalar(kf, z_rpm_mps, R_eff);
}
