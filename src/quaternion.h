#ifndef RAW_PHYSICS_QUATERNION_H
#define RAW_PHYSICS_QUATERNION_H
#include <gm.h>

typedef struct {
	r64 x, y, z, w;
} Quaternion;

Quaternion quaternion_new_radians(vec3 axis, r64 angle);
Quaternion quaternion_new(vec3 axis, r64 angle);
Quaternion quaternion_product(const Quaternion* q1, const Quaternion* q2);
Quaternion quaternion_nlerp(const Quaternion* q1, const Quaternion* q2, r64 t);
Quaternion quaternion_slerp(const Quaternion* q1, const Quaternion* q2, r64 t);
Quaternion quaternion_inverse(const Quaternion* q);
mat3 quaternion_get_matrix3(const Quaternion* quat);
mat4 quaternion_get_matrix(const Quaternion* quat);
vec3 quaternion_get_forward(const Quaternion* quat);
vec3 quaternion_get_up(const Quaternion* quat);
vec3 quaternion_get_right(const Quaternion* quat);
vec3 quaternion_get_forward_inverted(const Quaternion* quat);
vec3 quaternion_get_up_inverted(const Quaternion* quat);
vec3 quaternion_get_right_inverted(const Quaternion* quat);
Quaternion quaternion_normalize(const Quaternion* q);
Quaternion quaternion_from_matrix(const mat4* m);
vec3 quaternion_apply_to_vec3(const Quaternion* q, vec3 v);
vec3 quaternion_apply_inverse_to_vec3(const Quaternion* q, vec3 v);
#endif