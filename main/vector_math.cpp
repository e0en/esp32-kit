#include "vector_math.hpp"
#include <cmath>
extern "C" {
#include <esp_err.h>
#include <esp_log.h>
}

float clip(float x, float x_min, float x_max) {
  x = x > x_max ? x_max : x;
  x = x < x_min ? x_min : x;
  return x;
}

Vector3 add(const Vector3 &v1, const Vector3 &v2) {
  return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}
Vector3 scale(const Vector3 &v, float n) { return {v.x * n, v.y * n, v.z * n}; }

float norm(const Vector3 &v) { return sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

Vector3 normalize(const Vector3 &v) {
  float n = norm(v);
  return {v.x / n, v.y / n, v.z / n};
}

float dot(const Vector3 &v1, const Vector3 &v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3 cross(const Vector3 &v1, const Vector3 &v2) {
  return {
      v1.y * v2.z - v1.z * v2.y,
      v1.z * v2.x - v1.x * v2.z,
      v1.x * v2.y - v1.y * v2.x,
  };
}

Quaternion add(const Quaternion &q1, const Quaternion &q2) {
  return {
      q1.x + q2.x,
      q1.y + q2.y,
      q1.z + q2.z,
      q1.w + q2.w,
  };
}

Quaternion scale(const Quaternion &q1, float n) {
  return {
      q1.x * n,
      q1.y * n,
      q1.z * n,
      q1.w * n,
  };
}

Quaternion multiply(const Quaternion &q1, const Quaternion &q2) {
  return {
      q1.y * q2.z - q1.z * q2.y + q1.w * q2.x + q1.x * q2.w,
      q1.z * q2.x - q1.x * q2.z + q1.w * q2.y + q1.y * q2.w,
      q1.x * q2.y - q1.y * q2.x + q1.w * q2.z + q1.z * q2.w,
      -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w,
  };
}

float dot(const Quaternion &q1, const Quaternion &q2) {
  return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}

float norm(const Quaternion &q) {
  return sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

Quaternion normalize(const Quaternion &q) {
  float n = norm(q);
  if (n < 1e-10) {
    return {0.0, 0.0, 0.0, 0.0};
  }
  return scale(q, 1.0 / n);
}

Quaternion conjugate(const Quaternion &q) { return {-q.x, -q.y, -q.z, q.w}; }

Quaternion inverse(const Quaternion &q) {
  float n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (n < 1e-10) {
    return {0.0, 0.0, 0.0, 0.0};
  }
  return {
      -q.x / n,
      -q.y / n,
      -q.z / n,
      +q.w / n,
  };
}

Quaternion exp(const Quaternion &q) {
  float exp_w = exp(q.w);
  Vector3 v = {q.x, q.y, q.z};
  float v_norm = norm(v);
  float sin_v = sin(v_norm) / v_norm;
  float cos_v = cos(v_norm);
  return scale(Quaternion(v.x * sin_v, v.y * sin_v, v.z * sin_v, cos_v), exp_w);
}

Quaternion log(const Quaternion &q) {
  float q_norm = norm(q);
  if (q_norm < 1e-10) {
    return {0, 0, 0, 0};
  }
  Vector3 v = normalize(Vector3(q.x, q.y, q.z));
  float a = acos(q.w / q_norm);
  float log_q_norm = log(q_norm);
  return {
      v.x * a,
      v.y * a,
      v.z * a,
      log_q_norm,
  };
}

Quaternion pow(const Quaternion &q, float n) { return exp(scale(log(q), n)); }

Vector3 slerp(const Vector3 &v1, const Vector3 &v2, float t) {
  t = clip(t, 0, 1);
  if (t == 0) {
    return v1;
  } else if (t == 1) {
    return v2;
  }

  float dot_prod = clip(dot(v1, v2), -1, 1);
  if (dot_prod > 0.9995) {
    return normalize(add(scale(v1, 1 - t), scale(v2, t)));
  } else if (dot_prod < -0.9995) {
    // Rotate around a perpendicular vector
    Vector3 perp = cross(v1, {1, 0, 0});
    if (norm(perp) < 1e-6) {
      perp = cross(v1, {0, 1, 0});
    }
    perp = normalize(perp);
    float theta = M_PI * t;
    return normalize(add(scale(v1, cos(theta)), scale(perp, sin(theta))));
  }

  float theta = acos(dot_prod);
  float c1 = sin((1 - t) * theta);
  float c2 = sin(t * theta);
  return normalize(add(scale(v1, c1), scale(v2, c2)));
}

Vector3 rotate_vector(const Vector3 &v, const Quaternion &q) {
  Quaternion q_v = {v.x, v.y, v.z, 0.0};
  Quaternion tmp = multiply(multiply(q, q_v), conjugate(q));
  return {tmp.x, tmp.y, tmp.z};
}

Quaternion slerp(const Quaternion &q1, const Quaternion &q2, float t) {
  t = clip(t, 0, 1);
  if (t == 0) {
    return q1;
  } else if (t == 1) {
    return q2;
  }

  float dot_prod = clip(dot(q1, q2), -1, 1);

  Quaternion result;
  Quaternion q2_fixed;
  if (dot_prod < 0) {
    q2_fixed = scale(q2, -1);
  } else {
    q2_fixed = q2;
  }

  if (dot_prod > 0.995) {
    result = add(scale(q1, 1 - t), scale(q2_fixed, t));
  } else {
    result = multiply(q1, pow(multiply(inverse(q1), q2_fixed), t));
  }
  result = normalize(result);
  if (result.x != result.x) {
    ESP_LOGE("SLERP", "q1 dot q2 = %f", (double)dot_prod);
    ESP_LOGE("SLERP", "q1 = Quaternion(%f, %f, %f : %f)", (double)q1.x,
             (double)q1.y, (double)q1.z, (double)q1.w);
    ESP_LOGE("SLERP", "q2 = Quaternion(%f, %f, %f : %f)", (double)q2.x,
             (double)q2.y, (double)q2.z, (double)q2.w);
    ESP_LOGE("SLERP", "q2_fixed = Quaternion(%f, %f, %f : %f)",
             (double)q2_fixed.x, (double)q2_fixed.y, (double)q2_fixed.z,
             (double)q2_fixed.w);
    ESP_LOGE("SLERP", "result = Quaternion(%f, %f, %f : %f)", (double)result.x,
             (double)result.y, (double)result.z, (double)result.w);
    ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
  }
  return result;
}

Quaternion extract_yaw(const Quaternion &q) {
  float theta =
      atan2(2 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  float cos_half = cos(theta / 2.0);
  float sin_half = sin(theta / 2.0);
  return Quaternion{0, 0, sin_half, cos_half};
}
