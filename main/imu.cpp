#include "imu.hpp"
#include "vector_math.hpp"
#include <cmath>
extern "C" {
#include <esp_err.h>
#include <esp_log.h>
}

Quaternion accel_to_quaternion(const Vector3 accel) {
  Vector3 minus_g = {0, 0, 1};
  auto a = normalize(accel);
  auto rotation_axis = cross(a, minus_g);
  if (norm(rotation_axis) < 1e-10) {
    if (accel.z > 0.0) {
      return Quaternion(1.0, 0.0, 0.0, 0.0);
    }
    return Quaternion(0.0, 0.0, 0.0, 1.0);
  }
  rotation_axis = normalize(rotation_axis);
  float a_dot_g = clip(dot(a, minus_g), -1, 1);
  float angle = acos(a_dot_g);
  float cos_half = cos(angle / 2);
  float sin_half = sin(angle / 2);
  return normalize(Quaternion(sin_half * rotation_axis.x,
                              sin_half * rotation_axis.y,
                              sin_half * rotation_axis.z, cos_half));
}

Quaternion integrate_gyro(const Quaternion orientation, const Vector3 gyro,
                          const float dt) {
  float max_angle = M_PI / 4;
  if (abs(gyro.x) > max_angle || abs(gyro.y) > max_angle ||
      abs(gyro.z) > max_angle) {
    size_t n_x = ceil(abs(gyro.x) / max_angle);
    size_t n_y = ceil(abs(gyro.y) / max_angle);
    size_t n_z = ceil(abs(gyro.z) / max_angle);
    size_t n = n_x > n_y ? n_x : n_y;
    n = n > n_z ? n : n_z;
    ESP_LOGW("GYRO", "Rotating too fast (%.2f, %.2f, %.2f)",
             (double)gyro.x / M_PI, (double)gyro.y / M_PI,
             (double)gyro.z / M_PI);
    ESP_LOGW("GYRO", "Looping for %d times", n);

    Quaternion q = orientation;
    for (size_t i = 0; i < n; i++) {
      Quaternion q_gyro = {gyro.x / n, gyro.y / n, gyro.z / n, 0.0};
      Quaternion dq = scale(multiply(q, q_gyro), 0.5);
      q = normalize(add(q, scale(dq, dt)));
    }
    return q;
  }
  Quaternion q_gyro = {gyro.x, gyro.y, gyro.z, 0.0};
  Quaternion dq = scale(multiply(orientation, q_gyro), 0.5);
  return normalize(add(orientation, scale(dq, dt)));
}

Quaternion complementary_filter(const Quaternion &previous, const Vector3 &gyro,
                                const Vector3 &accel, const float dt,
                                const float tau) {
  auto q_gyro = integrate_gyro(previous, gyro, dt);
  auto q_accel = accel_to_quaternion(accel);

  float t = 1.0 - tau;

  float dot_product = dot(q_gyro, q_accel);
  if (abs(dot_product - 1.0) < 1e-10) {
    return q_accel;
  }
  Vector3 e_gyro = quaternion_to_euler(q_gyro);
  Vector3 e_accel = quaternion_to_euler(q_accel);
  float gyro_yaw = e_gyro.z;
  e_gyro.z = 0;
  e_accel.z = 0;

  Quaternion q_new =
      slerp(euler_to_quaternion(e_gyro), euler_to_quaternion(e_accel), t);
  Vector3 e_new = quaternion_to_euler(q_new);
  e_new.z = gyro_yaw;
  auto q = euler_to_quaternion(e_new);
  if (q.x != q.x) {
    ESP_LOGE("CF", "dt = %f", (double)dt);
    ESP_LOGE("CF", "tau = %f", (double)tau);
    ESP_LOGE("CF", "prev = Quaternion(%f, %f, %f, %f)", (double)previous.x,
             (double)previous.y, (double)previous.z, (double)previous.w);
    ESP_LOGE("CF", "gyro = Vector3(%f, %f, %f)", (double)gyro.x, (double)gyro.y,
             (double)gyro.z);
    ESP_LOGE("CF", "q_gyro = Quaternion(%f, %f, %f, %f)", (double)q_gyro.x,
             (double)q_gyro.y, (double)q_gyro.z, (double)q_gyro.w);
    ESP_LOGE("CF", "q_accel = Quaternion(%f, %f, %f, %f)", (double)q_accel.x,
             (double)q_accel.y, (double)q_accel.z, (double)q_accel.w);
    if (q.x != q.x) {
      ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
    }
  }
  return q;
}
