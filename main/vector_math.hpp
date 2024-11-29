#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

float clip(float x, float x_min, float x_max);

struct Vector3 {
  float x;
  float y;
  float z;
};

Vector3 add(const Vector3 &v1, const Vector3 &v2);
Vector3 scale(const Vector3 &v, float n);
float norm(const Vector3 &v);
Vector3 normalize(const Vector3 &v);
float dot(const Vector3 &v1, const Vector3 &v2);
Vector3 cross(const Vector3 &v1, const Vector3 &v2);

struct Quaternion {
  float x;
  float y;
  float z;
  float w;
};

Quaternion add(const Quaternion &q1, const Quaternion &q2);
Quaternion multiply(const Quaternion &q1, const Quaternion &q2);
Quaternion scale(const Quaternion &q1, float n);
float dot(const Quaternion &q1, const Quaternion &q2);

float norm(const Quaternion &q);
Quaternion normalize(const Quaternion &q);

Quaternion conjugate(const Quaternion &q);
Quaternion inverse(const Quaternion &q);
Quaternion exp(const Quaternion &q);
Quaternion log(const Quaternion &q);
Quaternion pow(const Quaternion &q, float n);

Vector3 slerp(const Vector3 &v1, const Vector3 &v2, float t);
Quaternion slerp(const Quaternion &q1, const Quaternion &q2, float t);
Vector3 rotate_vector(const Vector3 &v, const Quaternion &q);

Quaternion extract_yaw(const Quaternion &q);

#endif
