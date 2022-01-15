#ifndef RAW_PHYSICS_PHYSICS_PBD_H
#define RAW_PHYSICS_PHYSICS_PBD_H
#include "../entity.h"

typedef enum {
	POSITIONAL_CONSTRAINT,
	COLLISION_CONSTRAINT,
	MUTUAL_ORIENTATION_CONSTRAINT,
	HINGE_JOINT_CONSTRAINT
} Constraint_Type;

typedef struct {
	eid e1_id;
	eid e2_id;
	vec3 r1_lc;
	vec3 r2_lc;
	r64 compliance;
	r64 lambda;
	vec3 distance;
} Positional_Constraint;

typedef struct {
	eid e1_id;
	eid e2_id;
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 normal;
	r64 lambda_t;
	r64 lambda_n;
} Collision_Constraint;

typedef struct {
	eid e1_id;
	eid e2_id;
	r64 compliance;
	r64 lambda;
} Mutual_Orientation_Constraint;

typedef struct {
	eid e1_id;
	eid e2_id;
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 e1_a;
	vec3 e1_b;
	vec3 e1_c;
	vec3 e2_a;
	vec3 e2_b;
	vec3 e2_c;
	r64 compliance;
	r64 lambda_rot;
	r64 lambda_pos;
} Hinge_Joint_Constraint;

typedef struct {
	Constraint_Type type;

	union {
		Positional_Constraint positional_constraint;
		Collision_Constraint collision_constraint;
		Mutual_Orientation_Constraint mutual_orientation_constraint;
		Hinge_Joint_Constraint hinge_joint_constraint;
	};
} Constraint;

void pbd_simulate(r64 dt, Entity** entities);
void pbd_simulate_with_constraints(r64 dt, Entity** entities, Constraint* external_constraints);

#endif