#ifndef KALMAN2D_H
#define KALMAN2D_H

#include <stdint.h>

typedef struct {
  float x[2];
  float P00;
  float P01;
  float P10;
  float P11;
  float sigma_a2;
  float R_plate;
  float R_rpm;
} Kalman2D_t;

void Kalman2D_Init(Kalman2D_t *kf,
                   float v0_init,
                   float a0_init,
                   float P0_v,
                   float P0_a,
                   float sigma_a2,
                   float R_plate,
                   float R_rpm);

void Kalman2D_Predict(Kalman2D_t *kf, float dt_sec);

void Kalman2D_UpdatePlate(Kalman2D_t *kf, float z_plate_mps);

void Kalman2D_UpdateRPM(Kalman2D_t *kf, float z_rpm_mps, uint8_t rpm_is_reliable);

static inline float Kalman2D_GetWindMPS(const Kalman2D_t *kf)
{
  return kf->x[0];
}

static inline float Kalman2D_GetWindAccel(const Kalman2D_t *kf)
{
  return kf->x[1];
}

#endif /* KALMAN2D_H */
