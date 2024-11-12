#ifndef IMU_H
#define IMU_H

#include "vector_math.hpp"

Quaternion accel_to_quaternion(const Vector3 accel);
Quaternion integrate_gyro(const Quaternion orientation, const Vector3 gyro,
                          const float dt, const float tau);

Quaternion complementary_filter(const Quaternion &previous, const Vector3 &gyro,
                                const Vector3 &accel, const float dt,
                                const float tau);

#endif
