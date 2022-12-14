#include "quaternion.h"

Quaternion quaternion_new_radians(vec3 axis, r64 angle) {
	if (gm_vec3_length(axis) != 0.0) {
		axis = gm_vec3_normalize(axis);
	}
	r64 sang = sin(angle / 2.0);

	Quaternion quat;
	quat.w = cos(angle / 2.0);
	quat.x = axis.x * sang;
	quat.y = axis.y * sang;
	quat.z = axis.z * sang;

	return quat;
}

Quaternion quaternion_new(vec3 axis, r64 angle) {
	if (gm_vec3_length(axis) != 0.0) {
		axis = gm_vec3_normalize(axis);
	}
	r64 sang = sin(gm_radians(angle) / 2.0);

	Quaternion quat;
	quat.w = cos(gm_radians(angle) / 2.0);
	quat.x = axis.x * sang;
	quat.y = axis.y * sang;
	quat.z = axis.z * sang;

	return quat;
}

vec3 quaternion_get_right_inverted(const Quaternion* quat) {
	return (vec3) {
		1.0 - 2.0 * quat->y * quat->y - 2.0 * quat->z * quat->z,
		2.0 * quat->x * quat->y - 2.0 * quat->w * quat->z, 2.0 * quat->x * quat->z + 2.0 * quat->w * quat->y
	};
}

vec3 quaternion_get_up_inverted(const Quaternion* quat) {
	return (vec3) {
		2.0 * quat->x * quat->y + 2.0 * quat->w * quat->z, 
		1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->z * quat->z),
		2.0 * quat->y * quat->z - 2.0 * quat->w * quat->x
	};
}

vec3 quaternion_get_forward_inverted(const Quaternion* quat) {
	return (vec3) {
		2.0 * quat->x * quat->z - 2.0 * quat->w * quat->y, 
		2.0 * quat->y * quat->z + 2.0 * quat->w * quat->x, 
		1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->y * quat->y)
	};
}

vec3 quaternion_get_right(const Quaternion* quat) {
	return (vec3) {
		1.0 - 2.0 * quat->y * quat->y - 2.0 * quat->z * quat->z,
		2.0 * quat->x * quat->y - 2.0 * -quat->w * quat->z, 
		2.0 * quat->x * quat->z + 2.0 * -quat->w * quat->y
	};
}

vec3 quaternion_get_up(const Quaternion* quat) {
	return (vec3) {
		2.0 * quat->x * quat->y + 2.0 * -quat->w * quat->z, 
		1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->z * quat->z),
		2.0 * quat->y * quat->z - 2.0 * -quat->w * quat->x
	};
}

vec3 quaternion_get_forward(const Quaternion* quat) {
	return (vec3) {
		2.0 * quat->x * quat->z - 2.0 * -quat->w * quat->y, 
		2.0 * quat->y * quat->z + 2.0 * -quat->w * quat->x, 
		1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->y * quat->y)
	};
}

Quaternion quaternion_inverse(const Quaternion* q) {
	Quaternion result;
	result.x = -q->x;
	result.y = -q->y;
	result.z = -q->z;
	result.w = q->w;
	return result;
}

mat3 quaternion_get_matrix3(const Quaternion* quat) {
	mat3 result;

	result.data[0][0] = 1.0 - 2.0 * quat->y * quat->y - 2.0 * quat->z * quat->z;
	result.data[1][0] = 2.0 * quat->x * quat->y + 2.0 * quat->w * quat->z;
	result.data[2][0] = 2.0 * quat->x * quat->z - 2.0 * quat->w * quat->y;

	result.data[0][1] = 2.0 * quat->x * quat->y - 2.0 * quat->w * quat->z;
	result.data[1][1] = 1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->z * quat->z);
	result.data[2][1] = 2.0 * quat->y * quat->z + 2.0 * quat->w * quat->x;

	result.data[0][2] = 2.0 * quat->x * quat->z + 2.0 * quat->w * quat->y;
	result.data[1][2] = 2.0 * quat->y * quat->z - 2.0 * quat->w * quat->x;
	result.data[2][2] = 1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->y * quat->y);

	return result;
}

mat4 quaternion_get_matrix(const Quaternion* quat) {
	mat4 result;

	result.data[0][0] = 1.0 - 2.0 * quat->y * quat->y - 2.0 * quat->z * quat->z;
	result.data[1][0] = 2.0 * quat->x * quat->y + 2.0 * quat->w * quat->z;
	result.data[2][0] = 2.0 * quat->x * quat->z - 2.0 * quat->w * quat->y;
	result.data[3][0] = 0.0;

	result.data[0][1] = 2.0 * quat->x * quat->y - 2.0 * quat->w * quat->z;
	result.data[1][1] = 1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->z * quat->z);
	result.data[2][1] = 2.0 * quat->y * quat->z + 2.0 * quat->w * quat->x;
	result.data[3][1] = 0.0;

	result.data[0][2] = 2.0 * quat->x * quat->z + 2.0 * quat->w * quat->y;
	result.data[1][2] = 2.0 * quat->y * quat->z - 2.0 * quat->w * quat->x;
	result.data[2][2] = 1.0 - (2.0 * quat->x * quat->x) - (2.0 * quat->y * quat->y);
	result.data[3][2] = 0.0;

	result.data[0][3] = 0.0;
	result.data[1][3] = 0.0;
	result.data[2][3] = 0.0;
	result.data[3][3] = 1.0;

	return result;
}

Quaternion quaternion_product(const Quaternion* q1, const Quaternion* q2) {
	Quaternion res;

	res.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
	res.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
	res.y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
	res.z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;

	return res;
}

Quaternion quaternion_normalize(const Quaternion* q) {
	r64 len = sqrt(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
	return (Quaternion) { q->x / len, q->y / len, q->z / len, q->w / len };
}

Quaternion quaternion_slerp(const Quaternion* _q1, const Quaternion* _q2, r64 t) {
	Quaternion q1 = *_q1, q2 = *_q2, qm;

	// Calculate angle between them.
	r64 cos_half_theta = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
	if (cos_half_theta < 0) {
		q2.w = -q2.w;
		q2.x = -q2.x;
		q2.y = -q2.y;
		q2.z = q2.z;
		cos_half_theta = -cos_half_theta;
	}
	// if qa=qb or qa=-qb then theta = 0 and we can return qa
	if (fabs(cos_half_theta) >= 1.0) {
		qm.w = q1.w;
		qm.x = q1.x;
		qm.y = q1.y;
		qm.z = q1.z;
		return qm;
	}
	// Calculate temporary values.
	r64 half_theta = acos(cos_half_theta);
	r64 sin_half_theta = sqrt(1.0 - cos_half_theta * cos_half_theta);

	// if theta = 180 degrees then result is not fully defined
	// we could rotate around any axis normal to qa or qb
	if (fabs(sin_half_theta) < 0.001) {
		qm.w = (q1.w * 0.5 + q2.w * 0.5);
		qm.x = (q1.x * 0.5 + q2.x * 0.5);
		qm.y = (q1.y * 0.5 + q2.y * 0.5);
		qm.z = (q1.z * 0.5 + q2.z * 0.5);
		return qm;
	}

	r64 ratio_a = sin((1 - t) * half_theta) / sin_half_theta;
	r64 ratio_b = sin(t * half_theta) / sin_half_theta;

	// Calculate Quaternion
	qm.w = (q1.w * ratio_a + q2.w * ratio_b);
	qm.x = (q1.x * ratio_a + q2.x * ratio_b);
	qm.y = (q1.y * ratio_a + q2.y * ratio_b);
	qm.z = (q1.z * ratio_a + q2.z * ratio_b);
	return qm;
}

Quaternion quaternion_nlerp(const Quaternion* _q1, const Quaternion* _q2, r64 t) {
	Quaternion q1 = *_q1, q2 = *_q2, qm;
	Quaternion result;
	r64 dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
	r64 blend_i = 1.0 - t;
	if (dot < 0) {
		result.w = blend_i * q1.w + t * -q2.w;
		result.x = blend_i * q1.x + t * -q2.x;
		result.y = blend_i * q1.y + t * -q2.y;
		result.z = blend_i * q1.z + t * -q2.z;
	} else {
		result.w = blend_i * q1.w + t * q2.w;
		result.x = blend_i * q1.x + t * q2.x;
		result.y = blend_i * q1.y + t * q2.y;
		result.z = blend_i * q1.z + t * q2.z;
	}

	return quaternion_normalize(&result);
}

// This function will only work properly if:
// the matrix is orthogonal
// the matrix is special orthogonal which gives additional condition: det(matrix)= +1
//
// when dealing with transform matrices, usually this should be true unless there is a scale
// (not sure if any scale or only for non-symetrical scales)
Quaternion quaternion_from_matrix(const mat4* m) {
	Quaternion q = { 0 };
	mat4 a = *m;

	r64 trace = a.data[0][0] + a.data[1][1] + a.data[2][2];	// I removed + 1.0; see discussion with Ethan

	if (trace > 0) { // I changed M_EPSILON to 0
		r64 s = 0.5 / sqrt(trace + 1.0);
		q.w = 0.25 / s;
		q.x = (a.data[2][1] - a.data[1][2]) * s;
		q.y = (a.data[0][2] - a.data[2][0]) * s;
		q.z = (a.data[1][0] - a.data[0][1]) * s;
	} else {
		if (a.data[0][0] > a.data[1][1] && a.data[0][0] > a.data[2][2]) {
			r64 s = 2.0 * sqrt(1.0 + a.data[0][0] - a.data[1][1] - a.data[2][2]);
			q.w = (a.data[2][1] - a.data[1][2]) / s;
			q.x = 0.25 * s;
			q.y = (a.data[0][1] + a.data[1][0]) / s;
			q.z = (a.data[0][2] + a.data[2][0]) / s;
		} else if (a.data[1][1] > a.data[2][2]) {
			r64 s = 2.0 * sqrt(1.0 + a.data[1][1] - a.data[0][0] - a.data[2][2]);
			q.w = (a.data[0][2] - a.data[2][0]) / s;
			q.x = (a.data[0][1] + a.data[1][0]) / s;
			q.y = 0.25 * s;
			q.z = (a.data[1][2] + a.data[2][1]) / s;
		} else {
			r64 s = 2.0 * sqrt(1.0 + a.data[2][2] - a.data[0][0] - a.data[1][1]);
			q.w = (a.data[1][0] - a.data[0][1]) / s;
			q.x = (a.data[0][2] + a.data[2][0]) / s;
			q.y = (a.data[1][2] + a.data[2][1]) / s;
			q.z = 0.25 * s;
		}
	}

	return q;
}

vec3 quaternion_apply_to_vec3(const Quaternion* q, vec3 v) {
	r64 ix = q->w * v.x + q->y * v.z - q->z * v.y;
	r64 iy = q->w * v.y + q->z * v.x - q->x * v.z;
	r64 iz = q->w * v.z + q->x * v.y - q->y * v.x;
	r64 iw = - q->x * v.x - q->y * v.y - q->z * v.z;

	return (vec3) {
		(ix * q->w) + (iw * -q->x) + (iy * -q->z) - (iz * -q->y),
		(iy * q->w) + (iw * -q->y) + (iz * -q->x) - (ix * -q->z),
		(iz * q->w) + (iw * -q->z) + (ix * -q->y) - (iy * -q->x)
	};
}

vec3 quaternion_apply_inverse_to_vec3(const Quaternion* q, vec3 v) {
	Quaternion inv = quaternion_inverse(q);
	return quaternion_apply_to_vec3(&inv, v);
}