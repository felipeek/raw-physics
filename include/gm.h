#ifndef BASIC_ENGINE_GM_H
#define BASIC_ENGINE_GM_H
#include <common.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#define PI_F 3.14159265358979

#pragma pack(push, 1)
typedef struct
{
	r64 data[4][4];
} mat4;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	r64 data[3][3];
} mat3;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	r64 data[2][2];
} mat2;
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
	struct
	{
		r64 x, y, z, w;
	};
	struct
	{
		r64 r, g, b, a;
	};
} vec4;
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
	struct
	{
		r64 x, y, z;
	};
	struct
	{
		r64 r, g, b;
	};
} vec3;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	r64 x, y;
} vec2;
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
	struct
	{
		r32 x, y, z, w;
	};
	struct
	{
		r32 r, g, b, a;
	};
} fvec4;
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
	struct
	{
		r32 x, y, z;
	};
	struct
	{
		r32 r, g, b;
	};
} fvec3;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	r32 x, y;
} fvec2;
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
	struct
	{
		s32 x, y, z, w;
	};
	struct
	{
		s32 r, g, b, a;
	};
} dvec4;
#pragma pack(pop)

#pragma pack(push, 1)
typedef union
{
	struct
	{
		s32 x, y, z;
	};
	struct
	{
		s32 r, g, b;
	};
} dvec3;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	s32 x, y;
} dvec2;
#pragma pack(pop)

// mat4
mat4  gm_mat4_ortho(r64 left, r64 right, r64 bottom, r64 top);
int   gm_mat4_inverse(const mat4* m, mat4* out);
vec4  gm_mat4_multiply_vec4(const mat4* m, vec4 v);
vec3  gm_mat4_multiply_vec3(const mat4* m, vec3 v, boolean is_point);
mat4  gm_mat4_multiply(const mat4* m1, const mat4* m2);
mat4  gm_mat4_transpose(const mat4* m);
mat4  gm_mat4_identity(void);
mat4  gm_mat4_scalar_product(r64 scalar, const mat4* m);
char* gm_mat4_to_string(char* buffer, const mat4* m);
mat4  gm_mat4_translate(const vec3 v);
mat4  gm_mat4_translate_transposed(const vec3 v);
mat4  gm_mat4_scale(const vec3 v);
vec3  gm_mat4_translation_from_matrix(const mat4* m);
mat3  gm_mat4_to_mat3(const mat4* m);

// mat3
int   gm_mat3_inverse(const mat3* m, mat3* out);
mat3  gm_mat3_multiply(const mat3* m1, const mat3* m2);
vec3  gm_mat3_multiply_vec3(const mat3* m, vec3 v);
mat3  gm_mat3_transpose(const mat3* m);
mat3  gm_mat3_scalar_product(r64 scalar, const mat3* m);
mat3  gm_mat3_identity(void);
char* gm_mat3_to_string(char* buffer, const mat3* m);

// mat2
mat2  gm_mat2_multiply(const mat2* m1, const mat2* m2);
mat2  gm_mat2_transpose(const mat2* m);
mat2  gm_mat2_scalar_product(r64 scalar, const mat2* m);
mat2  gm_mat2_identity(void);
char* gm_mat2_to_string(char* buffer, const mat2* m);

// vec4
int   gm_vec4_equal(vec4 v1, vec4 v2);
vec4  gm_vec4_scalar_product(r64 scalar, vec4 v);
vec4  gm_vec4_normalize(vec4 v);
r64   gm_vec4_length(vec4 v);
vec4  gm_vec4_add(vec4 v1, vec4 v2);
vec4  gm_vec4_subtract(vec4 v1, vec4 v2);
r64   gm_vec4_dot(vec4 v1, vec4 v2);
vec4  gm_vec4_cross(vec4 v1, vec4 v2);
char* gm_vec4_to_string(char* buffer, vec4 v);

// vec3
int   gm_vec3_equal(vec3 v1, vec3 v2);
vec3  gm_vec3_scalar_product(r64 scalar, vec3 v);
vec3  gm_vec3_normalize(vec3 v);
r64   gm_vec3_length(vec3 v);
vec3  gm_vec3_add(vec3 v1, vec3 v2);
vec3  gm_vec3_subtract(vec3 v1, vec3 v2);
r64   gm_vec3_dot(vec3 v1, vec3 v2);
vec3  gm_vec3_cross(vec3 v1, vec3 v2);
char* gm_vec3_to_string(char* buffer, vec3 v);
vec3  gm_vec4_to_vec3(vec4 v);
vec3  gm_vec3_invert(vec3 v);
int   gm_vec3_is_zero(vec3 v);

// vec2
vec2  gm_vec2_add(vec2 v1, vec2 v2);
int   gm_vec2_equal(vec2 v1, vec2 v2);
vec2  gm_vec2_scalar_product(r64 scalar, vec2 v);
vec2  gm_vec2_normalize(vec2 v);
r64   gm_vec2_length(vec2 v);
vec2  gm_vec2_subtract(vec2 v1, vec2 v2);
r64   gm_vec2_dot(vec2 v1, vec2 v2);
r64   gm_vec2_angle(vec2 v);
char* gm_vec2_to_string(char* buffer, vec2 v);

// Util
r64   gm_radians(r64 degrees);
r64   gm_degrees(r64 radians);
r64   gm_absolute(r64 x);

#ifdef GRAPHICS_MATH_IMPLEMENT
mat4 gm_mat4_ortho(r64 left, r64 right, r64 bottom, r64 top)
{
	mat4 result;
	result.data[0][0] = 2.0 / (right - left);	result.data[0][1] = 0;						result.data[0][2] = 0;	result.data[0][3] = -(right + left) / (right - left);
	result.data[1][0] = 0;						result.data[1][1] = 2.0 / (top - bottom);	result.data[1][2] = 0;	result.data[1][3] = -(top + bottom) / (top - bottom);
	result.data[2][0] = 0;						result.data[2][1] = 0;						result.data[2][2] = 1;	result.data[2][3] = 0;
	result.data[3][0] = 0;						result.data[3][1] = 0;						result.data[3][2] = 0;	result.data[3][3] = 1;

	return result;
}

int gm_mat4_inverse(const mat4* m, mat4* out)
{
	mat4 inv;
	r64 det;
	s32 i;

	r64* m_data = (r64*)m->data;
	r64* out_data = (r64*)out->data;
	r64* inv_data = (r64*)inv.data;

	inv_data[0] = m_data[5] * m_data[10] * m_data[15] -
		m_data[5] * m_data[11] * m_data[14] -
		m_data[9] * m_data[6] * m_data[15] +
		m_data[9] * m_data[7] * m_data[14] +
		m_data[13] * m_data[6] * m_data[11] -
		m_data[13] * m_data[7] * m_data[10];

	inv_data[4] = -m_data[4] * m_data[10] * m_data[15] +
		m_data[4] * m_data[11] * m_data[14] +
		m_data[8] * m_data[6] * m_data[15] -
		m_data[8] * m_data[7] * m_data[14] -
		m_data[12] * m_data[6] * m_data[11] +
		m_data[12] * m_data[7] * m_data[10];

	inv_data[8] = m_data[4] * m_data[9] * m_data[15] -
		m_data[4] * m_data[11] * m_data[13] -
		m_data[8] * m_data[5] * m_data[15] +
		m_data[8] * m_data[7] * m_data[13] +
		m_data[12] * m_data[5] * m_data[11] -
		m_data[12] * m_data[7] * m_data[9];

	inv_data[12] = -m_data[4] * m_data[9] * m_data[14] +
		m_data[4] * m_data[10] * m_data[13] +
		m_data[8] * m_data[5] * m_data[14] -
		m_data[8] * m_data[6] * m_data[13] -
		m_data[12] * m_data[5] * m_data[10] +
		m_data[12] * m_data[6] * m_data[9];

	inv_data[1] = -m_data[1] * m_data[10] * m_data[15] +
		m_data[1] * m_data[11] * m_data[14] +
		m_data[9] * m_data[2] * m_data[15] -
		m_data[9] * m_data[3] * m_data[14] -
		m_data[13] * m_data[2] * m_data[11] +
		m_data[13] * m_data[3] * m_data[10];

	inv_data[5] = m_data[0] * m_data[10] * m_data[15] -
		m_data[0] * m_data[11] * m_data[14] -
		m_data[8] * m_data[2] * m_data[15] +
		m_data[8] * m_data[3] * m_data[14] +
		m_data[12] * m_data[2] * m_data[11] -
		m_data[12] * m_data[3] * m_data[10];

	inv_data[9] = -m_data[0] * m_data[9] * m_data[15] +
		m_data[0] * m_data[11] * m_data[13] +
		m_data[8] * m_data[1] * m_data[15] -
		m_data[8] * m_data[3] * m_data[13] -
		m_data[12] * m_data[1] * m_data[11] +
		m_data[12] * m_data[3] * m_data[9];

	inv_data[13] = m_data[0] * m_data[9] * m_data[14] -
		m_data[0] * m_data[10] * m_data[13] -
		m_data[8] * m_data[1] * m_data[14] +
		m_data[8] * m_data[2] * m_data[13] +
		m_data[12] * m_data[1] * m_data[10] -
		m_data[12] * m_data[2] * m_data[9];

	inv_data[2] = m_data[1] * m_data[6] * m_data[15] -
		m_data[1] * m_data[7] * m_data[14] -
		m_data[5] * m_data[2] * m_data[15] +
		m_data[5] * m_data[3] * m_data[14] +
		m_data[13] * m_data[2] * m_data[7] -
		m_data[13] * m_data[3] * m_data[6];

	inv_data[6] = -m_data[0] * m_data[6] * m_data[15] +
		m_data[0] * m_data[7] * m_data[14] +
		m_data[4] * m_data[2] * m_data[15] -
		m_data[4] * m_data[3] * m_data[14] -
		m_data[12] * m_data[2] * m_data[7] +
		m_data[12] * m_data[3] * m_data[6];

	inv_data[10] = m_data[0] * m_data[5] * m_data[15] -
		m_data[0] * m_data[7] * m_data[13] -
		m_data[4] * m_data[1] * m_data[15] +
		m_data[4] * m_data[3] * m_data[13] +
		m_data[12] * m_data[1] * m_data[7] -
		m_data[12] * m_data[3] * m_data[5];

	inv_data[14] = -m_data[0] * m_data[5] * m_data[14] +
		m_data[0] * m_data[6] * m_data[13] +
		m_data[4] * m_data[1] * m_data[14] -
		m_data[4] * m_data[2] * m_data[13] -
		m_data[12] * m_data[1] * m_data[6] +
		m_data[12] * m_data[2] * m_data[5];

	inv_data[3] = -m_data[1] * m_data[6] * m_data[11] +
		m_data[1] * m_data[7] * m_data[10] +
		m_data[5] * m_data[2] * m_data[11] -
		m_data[5] * m_data[3] * m_data[10] -
		m_data[9] * m_data[2] * m_data[7] +
		m_data[9] * m_data[3] * m_data[6];

	inv_data[7] = m_data[0] * m_data[6] * m_data[11] -
		m_data[0] * m_data[7] * m_data[10] -
		m_data[4] * m_data[2] * m_data[11] +
		m_data[4] * m_data[3] * m_data[10] +
		m_data[8] * m_data[2] * m_data[7] -
		m_data[8] * m_data[3] * m_data[6];

	inv_data[11] = -m_data[0] * m_data[5] * m_data[11] +
		m_data[0] * m_data[7] * m_data[9] +
		m_data[4] * m_data[1] * m_data[11] -
		m_data[4] * m_data[3] * m_data[9] -
		m_data[8] * m_data[1] * m_data[7] +
		m_data[8] * m_data[3] * m_data[5];

	inv_data[15] = m_data[0] * m_data[5] * m_data[10] -
		m_data[0] * m_data[6] * m_data[9] -
		m_data[4] * m_data[1] * m_data[10] +
		m_data[4] * m_data[2] * m_data[9] +
		m_data[8] * m_data[1] * m_data[6] -
		m_data[8] * m_data[2] * m_data[5];

	det = m_data[0] * inv_data[0] + m_data[1] * inv_data[4] + m_data[2] * inv_data[8] + m_data[3] * inv_data[12];

	if (det == 0.0)
		return false;

	det = 1.0 / det;

	for (i = 0; i < 16; i++)
		out_data[i] = inv_data[i] * det;

	return true;
}

mat4 gm_mat4_multiply(const mat4* m1, const mat4* m2)
{
	mat4 result;

	result.data[0][0] = m1->data[0][0] * m2->data[0][0] + m1->data[0][1] * m2->data[1][0] + m1->data[0][2] * m2->data[2][0] + m1->data[0][3] * m2->data[3][0];
	result.data[0][1] = m1->data[0][0] * m2->data[0][1] + m1->data[0][1] * m2->data[1][1] + m1->data[0][2] * m2->data[2][1] + m1->data[0][3] * m2->data[3][1];
	result.data[0][2] = m1->data[0][0] * m2->data[0][2] + m1->data[0][1] * m2->data[1][2] + m1->data[0][2] * m2->data[2][2] + m1->data[0][3] * m2->data[3][2];
	result.data[0][3] = m1->data[0][0] * m2->data[0][3] + m1->data[0][1] * m2->data[1][3] + m1->data[0][2] * m2->data[2][3] + m1->data[0][3] * m2->data[3][3];
	result.data[1][0] = m1->data[1][0] * m2->data[0][0] + m1->data[1][1] * m2->data[1][0] + m1->data[1][2] * m2->data[2][0] + m1->data[1][3] * m2->data[3][0];
	result.data[1][1] = m1->data[1][0] * m2->data[0][1] + m1->data[1][1] * m2->data[1][1] + m1->data[1][2] * m2->data[2][1] + m1->data[1][3] * m2->data[3][1];
	result.data[1][2] = m1->data[1][0] * m2->data[0][2] + m1->data[1][1] * m2->data[1][2] + m1->data[1][2] * m2->data[2][2] + m1->data[1][3] * m2->data[3][2];
	result.data[1][3] = m1->data[1][0] * m2->data[0][3] + m1->data[1][1] * m2->data[1][3] + m1->data[1][2] * m2->data[2][3] + m1->data[1][3] * m2->data[3][3];
	result.data[2][0] = m1->data[2][0] * m2->data[0][0] + m1->data[2][1] * m2->data[1][0] + m1->data[2][2] * m2->data[2][0] + m1->data[2][3] * m2->data[3][0];
	result.data[2][1] = m1->data[2][0] * m2->data[0][1] + m1->data[2][1] * m2->data[1][1] + m1->data[2][2] * m2->data[2][1] + m1->data[2][3] * m2->data[3][1];
	result.data[2][2] = m1->data[2][0] * m2->data[0][2] + m1->data[2][1] * m2->data[1][2] + m1->data[2][2] * m2->data[2][2] + m1->data[2][3] * m2->data[3][2];
	result.data[2][3] = m1->data[2][0] * m2->data[0][3] + m1->data[2][1] * m2->data[1][3] + m1->data[2][2] * m2->data[2][3] + m1->data[2][3] * m2->data[3][3];
	result.data[3][0] = m1->data[3][0] * m2->data[0][0] + m1->data[3][1] * m2->data[1][0] + m1->data[3][2] * m2->data[2][0] + m1->data[3][3] * m2->data[3][0];
	result.data[3][1] = m1->data[3][0] * m2->data[0][1] + m1->data[3][1] * m2->data[1][1] + m1->data[3][2] * m2->data[2][1] + m1->data[3][3] * m2->data[3][1];
	result.data[3][2] = m1->data[3][0] * m2->data[0][2] + m1->data[3][1] * m2->data[1][2] + m1->data[3][2] * m2->data[2][2] + m1->data[3][3] * m2->data[3][2];
	result.data[3][3] = m1->data[3][0] * m2->data[0][3] + m1->data[3][1] * m2->data[1][3] + m1->data[3][2] * m2->data[2][3] + m1->data[3][3] * m2->data[3][3];

	return result;
}

int gm_mat3_inverse(const mat3* m, mat3* out)
{
    // computes the inverse of a matrix m
    double det = m->data[0][0] * (m->data[1][1] * m->data[2][2] - m->data[2][1] * m->data[1][2]) -
                 m->data[0][1] * (m->data[1][0] * m->data[2][2] - m->data[1][2] * m->data[2][0]) +
                 m->data[0][2] * (m->data[1][0] * m->data[2][1] - m->data[1][1] * m->data[2][0]);

    if (det == 0.0) {
        return false;
    }
    double invdet = 1 / det;

    out->data[0][0] = (m->data[1][1] * m->data[2][2] - m->data[2][1] * m->data[1][2]) * invdet;
    out->data[0][1] = (m->data[0][2] * m->data[2][1] - m->data[0][1] * m->data[2][2]) * invdet;
    out->data[0][2] = (m->data[0][1] * m->data[1][2] - m->data[0][2] * m->data[1][1]) * invdet;
    out->data[1][0] = (m->data[1][2] * m->data[2][0] - m->data[1][0] * m->data[2][2]) * invdet;
    out->data[1][1] = (m->data[0][0] * m->data[2][2] - m->data[0][2] * m->data[2][0]) * invdet;
    out->data[1][2] = (m->data[1][0] * m->data[0][2] - m->data[0][0] * m->data[1][2]) * invdet;
    out->data[2][0] = (m->data[1][0] * m->data[2][1] - m->data[2][0] * m->data[1][1]) * invdet;
    out->data[2][1] = (m->data[2][0] * m->data[0][1] - m->data[0][0] * m->data[2][1]) * invdet;
    out->data[2][2] = (m->data[0][0] * m->data[1][1] - m->data[1][0] * m->data[0][1]) * invdet;

    return true;
}

mat3 gm_mat3_multiply(const mat3* m1, const mat3* m2)
{
	mat3 result;

	result.data[0][0] = m1->data[0][0] * m2->data[0][0] + m1->data[0][1] * m2->data[1][0] + m1->data[0][2] * m2->data[2][0];
	result.data[0][1] = m1->data[0][0] * m2->data[0][1] + m1->data[0][1] * m2->data[1][1] + m1->data[0][2] * m2->data[2][1];
	result.data[0][2] = m1->data[0][0] * m2->data[0][2] + m1->data[0][1] * m2->data[1][2] + m1->data[0][2] * m2->data[2][2];
	result.data[1][0] = m1->data[1][0] * m2->data[0][0] + m1->data[1][1] * m2->data[1][0] + m1->data[1][2] * m2->data[2][0];
	result.data[1][1] = m1->data[1][0] * m2->data[0][1] + m1->data[1][1] * m2->data[1][1] + m1->data[1][2] * m2->data[2][1];
	result.data[1][2] = m1->data[1][0] * m2->data[0][2] + m1->data[1][1] * m2->data[1][2] + m1->data[1][2] * m2->data[2][2];
	result.data[2][0] = m1->data[2][0] * m2->data[0][0] + m1->data[2][1] * m2->data[1][0] + m1->data[2][2] * m2->data[2][0];
	result.data[2][1] = m1->data[2][0] * m2->data[0][1] + m1->data[2][1] * m2->data[1][1] + m1->data[2][2] * m2->data[2][1];
	result.data[2][2] = m1->data[2][0] * m2->data[0][2] + m1->data[2][1] * m2->data[1][2] + m1->data[2][2] * m2->data[2][2];

	return result;
}

mat2 gm_mat2_multiply(const mat2* m1, const mat2* m2)
{
	mat2 result;

  // @TODO(fek): verify this function

	result.data[0][0] = m1->data[0][0] * m2->data[0][0] + m1->data[0][1] * m2->data[1][0];
	result.data[0][1] = m1->data[0][0] * m2->data[0][1] + m1->data[0][1] * m2->data[1][1];
	result.data[1][0] = m1->data[1][0] * m2->data[0][0] + m1->data[1][1] * m2->data[1][0];
	result.data[1][1] = m1->data[1][0] * m2->data[0][1] + m1->data[1][1] * m2->data[1][1];

	return result;
}

vec4 gm_mat4_multiply_vec4(const mat4* m, vec4 v)
{
	vec4 result;

	result.x = v.x * m->data[0][0] + v.y * m->data[0][1] + v.z * m->data[0][2] + v.w * m->data[0][3];
	result.y = v.x * m->data[1][0] + v.y * m->data[1][1] + v.z * m->data[1][2] + v.w * m->data[1][3];
	result.z = v.x * m->data[2][0] + v.y * m->data[2][1] + v.z * m->data[2][2] + v.w * m->data[2][3];
	result.w = v.x * m->data[3][0] + v.y * m->data[3][1] + v.z * m->data[3][2] + v.w * m->data[3][3];

	return result;
}

vec3 gm_mat4_multiply_vec3(const mat4* m, vec3 v, boolean is_point) {
	vec3 result;
	if (is_point) {
		result.x = m->data[0][0] * v.x + m->data[0][1] * v.y + m->data[0][2] * v.z + m->data[0][3] * 1.0;
		result.y = m->data[1][0] * v.x + m->data[1][1] * v.y + m->data[1][2] * v.z + m->data[1][3] * 1.0;
		result.z = m->data[2][0] * v.x + m->data[2][1] * v.y + m->data[2][2] * v.z + m->data[2][3] * 1.0;
	} else {
		result.x = m->data[0][0] * v.x + m->data[0][1] * v.y + m->data[0][2] * v.z;
		result.y = m->data[1][0] * v.x + m->data[1][1] * v.y + m->data[1][2] * v.z;
		result.z = m->data[2][0] * v.x + m->data[2][1] * v.y + m->data[2][2] * v.z;
	}
	return result;
}

vec3 gm_mat3_multiply_vec3(const mat3* m, vec3 v) {
	vec3 result;
	result.x = m->data[0][0] * v.x + m->data[0][1] * v.y + m->data[0][2] * v.z;
	result.y = m->data[1][0] * v.x + m->data[1][1] * v.y + m->data[1][2] * v.z;
	result.z = m->data[2][0] * v.x + m->data[2][1] * v.y + m->data[2][2] * v.z;
	return result;
}

mat3 gm_mat4_to_mat3(const mat4* m) {
	mat3 result;
	result.data[0][0] = m->data[0][0];
	result.data[0][1] = m->data[0][1];
	result.data[0][2] = m->data[0][2];
	result.data[1][0] = m->data[1][0];
	result.data[1][1] = m->data[1][1];
	result.data[1][2] = m->data[1][2];
	result.data[2][0] = m->data[2][0];
	result.data[2][1] = m->data[2][1];
	result.data[2][2] = m->data[2][2];
	return result;
}

mat4 gm_mat4_transpose(const mat4* m)
{
	return (mat4) {
		m->data[0][0], m->data[1][0], m->data[2][0], m->data[3][0],
		m->data[0][1], m->data[1][1], m->data[2][1], m->data[3][1],
		m->data[0][2], m->data[1][2], m->data[2][2], m->data[3][2],
		m->data[0][3], m->data[1][3], m->data[2][3], m->data[3][3]
	};
}

mat3 gm_mat3_transpose(const mat3* m)
{
	return (mat3) {
		m->data[0][0], m->data[1][0], m->data[2][0],
		m->data[0][1], m->data[1][1], m->data[2][1],
		m->data[0][2], m->data[1][2], m->data[2][2]
	};
}

mat2 gm_mat2_transpose(const mat2* m)
{
	return (mat2) {
		m->data[0][0], m->data[1][0],
		m->data[0][1], m->data[1][1],
	};
}

mat4 gm_mat4_scalar_product(r64 scalar, const mat4* m)
{
	return (mat4) {
		scalar * m->data[0][0], scalar * m->data[0][1], scalar * m->data[0][2], scalar * m->data[0][3],
		scalar * m->data[1][0], scalar * m->data[1][1], scalar * m->data[1][2], scalar * m->data[1][3],
		scalar * m->data[2][0], scalar * m->data[2][1], scalar * m->data[2][2], scalar * m->data[2][3],
		scalar * m->data[3][0], scalar * m->data[3][1], scalar * m->data[3][2], scalar * m->data[3][3],
	};
}

mat3 gm_mat3_scalar_product(r64 scalar, const mat3* m)
{
	return (mat3) {
		scalar * m->data[0][0], scalar * m->data[0][1], scalar * m->data[0][2],
		scalar * m->data[1][0], scalar * m->data[1][1], scalar * m->data[1][2],
		scalar * m->data[2][0], scalar * m->data[2][1], scalar * m->data[2][2]
	};
}

mat2 gm_mat2_scalar_product(r64 scalar, const mat2* m)
{
	return (mat2) {
		scalar * m->data[0][0], scalar * m->data[0][1],
		scalar * m->data[1][0], scalar * m->data[1][1]
	};
}

mat4 gm_mat4_identity(void)
{
	return (mat4) {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};
}

mat3 gm_mat3_identity(void)
{
	return (mat3) {
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0,
	};
}

mat2 gm_mat2_identity(void)
{
	return (mat2) {
		1.0, 0.0,
		0.0, 1.0
	};
}

mat4 gm_mat4_scale(const vec3 v) 
{
	mat4 result = gm_mat4_identity();
	result.data[0][0] = v.x;
	result.data[1][1] = v.y;
	result.data[2][2] = v.z;
	return result;
}

mat4 gm_mat4_translate(const vec3 v)
{
	mat4 result = gm_mat4_identity();
	result.data[0][0] = 1.0;
	result.data[1][1] = 1.0;
	result.data[2][2] = 1.0;

	result.data[3][0] = v.x;
	result.data[3][1] = v.y;
	result.data[3][2] = v.z;

	result.data[3][3] = 1.0;

	return result;
}

mat4 gm_mat4_translate_transposed(const vec3 v)
{
	mat4 result = gm_mat4_identity();
	result.data[0][0] = 1.0;
	result.data[1][1] = 1.0;
	result.data[2][2] = 1.0;

	result.data[0][3] = v.x;
	result.data[1][3] = v.y;
	result.data[2][3] = v.z;

	result.data[3][3] = 1.0;

	return result;
}

vec3 gm_mat4_translation_from_matrix(const mat4* m)
{
	return (vec3){m->data[0][3], m->data[1][3], m->data[2][3]};
}

vec3 gm_vec3_invert(vec3 v) {
	return gm_vec3_subtract((vec3){0.0, 0.0, 0.0}, v);
}

int gm_vec3_is_zero(vec3 v) {
	return v.x == 0.0 && v.y == 0.0 && v.z == 0.0;
}

int gm_vec2_equal(vec2 v1, vec2 v2)
{
	if (v1.x == v2.x && v1.y == v2.y)
		return true;
	return false;
}

int gm_vec3_equal(vec3 v1, vec3 v2)
{
	if (v1.x == v2.x && v1.y == v2.y && v1.z == v2.z)
		return true;
	return false;
}

int gm_vec4_equal(vec4 v1, vec4 v2)
{
	if (v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.w == v2.w)
		return true;
	return false;
}

vec4 gm_vec4_scalar_product(r64 scalar, vec4 v)
{
	return (vec4) { scalar * v.x, scalar * v.y, scalar * v.z, scalar * v.w };
}

vec3 gm_vec3_scalar_product(r64 scalar, vec3 v)
{
	return (vec3) { scalar * v.x, scalar * v.y, scalar * v.z };
}

vec2 gm_vec2_scalar_product(r64 scalar, vec2 v)
{
	return (vec2) { scalar * v.x, scalar * v.y };
}

vec4 gm_vec4_normalize(vec4 v)
{
	if (!(v.x != 0.0 || v.y != 0.0 || v.z != 0.0 || v.w != 0.0)) {
		return (vec4) { 0.0, 0.0, 0.0, 0.0 };
	}
	r64 vector_length = gm_vec4_length(v);
	return (vec4) { v.x / vector_length, v.y / vector_length, v.z / vector_length, v.w / vector_length };
}

vec3 gm_vec3_normalize(vec3 v)
{
	if (!(v.x != 0.0 || v.y != 0.0 || v.z != 0.0)) {
		return (vec3) { 0.0, 0.0, 0.0 };
	}
	r64 vector_length = gm_vec3_length(v);
	return (vec3) { v.x / vector_length, v.y / vector_length, v.z / vector_length };
}

vec2 gm_vec2_normalize(vec2 v)
{
	if (!(v.x != 0.0 || v.y != 0.0)) {
		return (vec2) { 0.0, 0.0 };
	}
	r64 vector_length = gm_vec2_length(v);
	return (vec2) { v.x / vector_length, v.y / vector_length };
}

r64 gm_vec4_length(vec4 v)
{
	return sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
}

r64 gm_vec3_length(vec3 v)
{
	return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

r64 gm_vec2_length(vec2 v)
{
	return sqrt(v.x * v.x + v.y * v.y);
}

vec4 gm_vec4_add(vec4 v1, vec4 v2)
{
	return (vec4) { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w };
}

vec3 gm_vec3_add(vec3 v1, vec3 v2)
{
	return (vec3) { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

vec2 gm_vec2_add(vec2 v1, vec2 v2)
{
	return (vec2) { v1.x + v2.x, v1.y + v2.y };
}

vec4 gm_vec4_subtract(vec4 v1, vec4 v2)
{
	return (vec4) { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z, v1.w - v2.w };
}

vec3 gm_vec3_subtract(vec3 v1, vec3 v2)
{
	return (vec3) { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

vec2 gm_vec2_subtract(vec2 v1, vec2 v2)
{
	return (vec2) { v1.x - v2.x, v1.y - v2.y };
}

r64 gm_vec4_dot(vec4 v1, vec4 v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

r64 gm_vec3_dot(vec3 v1, vec3 v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

r64 gm_vec2_dot(vec2 v1, vec2 v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

r64 gm_vec2_angle(vec2 v)
{
	return atan2(v.y, v.x);
}

r64 gm_radians(r64 degrees)
{
	return PI_F * degrees / 180.0;
}

r64 gm_degrees(r64 radians)
{
	return (radians * 180.0) / PI_F;
}

vec4 gm_vec4_cross(vec4 v1, vec4 v2)
{
	assert(v1.w == 0.0 && v2.w == 0.0);
	vec4 result;

	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;
	result.w = 0.0;

	return result;
}

vec3 gm_vec3_cross(vec3 v1, vec3 v2)
{
	vec3 result;

	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;

	return result;
}

r64 gm_absolute(r64 x)
{
	return (x < 0) ? -x : x;
}

char* gm_mat4_to_string(char* buffer, const mat4* m)
{
	sprintf(buffer, "[%.5f, %.5f, %.5f, %.5f]\n[%.5f, %.5f, %.5f, %.5f]\n[%.5f, %.5f, %.5f, %.5f]\n[%.5f, %.5f, %.5f, %.5f]",
		m->data[0][0], m->data[0][1], m->data[0][2], m->data[0][3],
		m->data[1][0], m->data[1][1], m->data[1][2], m->data[1][3],
		m->data[2][0], m->data[2][1], m->data[2][2], m->data[2][3],
		m->data[3][0], m->data[3][1], m->data[3][2], m->data[3][3]);
	return buffer;
}

char* gm_mat3_to_string(char* buffer, const mat3* m)
{
	sprintf(buffer, "[%.5f, %.5f, %.5f]\n[%.5f, %.5f, %.5f]\n[%.5f, %.5f, %.5f]",
		m->data[0][0], m->data[0][1], m->data[0][2],
		m->data[1][0], m->data[1][1], m->data[1][2],
		m->data[2][0], m->data[2][1], m->data[2][2]);
	return buffer;
}

char* gm_mat2_to_string(char* buffer, const mat2* m)
{
	sprintf(buffer, "[%.5f, %.5f]\n[%.5f, %.5f]",
		m->data[0][0], m->data[0][1],
		m->data[1][0], m->data[1][1]);
	return buffer;
}

char* gm_vec4_to_string(char* buffer, vec4 v)
{
	sprintf(buffer, "<%f, %f, %f, %f>", v.x, v.y, v.z, v.w);
	return buffer;
}

char* gm_vec3_to_string(char* buffer, vec3 v)
{
	sprintf(buffer, "<%.9f, %.9f, %.9f>", v.x, v.y, v.z);
	return buffer;
}

char* gm_vec2_to_string(char* buffer, vec2 v)
{
	sprintf(buffer, "<%f, %f>", v.x, v.y);
	return buffer;
}

vec3 gm_vec4_to_vec3(vec4 v) {
	return (vec3) { v.x, v.y, v.z };
}
#endif
#endif
