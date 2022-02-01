#ifndef RAW_PHYSICS_PHYSICS_PBD_H
#define RAW_PHYSICS_PHYSICS_PBD_H
#include "../entity.h"

typedef enum {
	PBD_POSITIVE_X_AXIS,
	PBD_NEGATIVE_X_AXIS,
	PBD_POSITIVE_Y_AXIS,
	PBD_NEGATIVE_Y_AXIS,
	PBD_POSITIVE_Z_AXIS,
	PBD_NEGATIVE_Z_AXIS
} PBD_Axis_Type;

typedef enum {
	POSITIONAL_CONSTRAINT,
	COLLISION_CONSTRAINT,
	MUTUAL_ORIENTATION_CONSTRAINT,
	HINGE_JOINT_CONSTRAINT,
	SPHERICAL_JOINT_CONSTRAINT
} Constraint_Type;

typedef struct {
	vec3 r1_lc;
	vec3 r2_lc;
	r64 compliance;
	r64 lambda;
	vec3 distance;
} Positional_Constraint;

typedef struct {
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 normal;
	r64 lambda_t;
	r64 lambda_n;
} Collision_Constraint;

typedef struct {
	r64 compliance;
	r64 lambda;
} Mutual_Orientation_Constraint;

typedef struct {
	vec3 r1_lc;
	vec3 r2_lc;
	r64 compliance;
	r64 lambda_pos;

	PBD_Axis_Type e1_aligned_axis;
	PBD_Axis_Type e2_aligned_axis;
	r64 lambda_aligned_axes;
	
	boolean limited;
	r64 upper_limit;
	r64 lower_limit;
	PBD_Axis_Type e1_limit_axis;
	PBD_Axis_Type e2_limit_axis;
	r64 lambda_limit_axes;
} Hinge_Joint_Constraint;

typedef struct {
	vec3 r1_lc;
	vec3 r2_lc;
	r64 lambda_pos;

	r64 lambda_swing;
	r64 swing_upper_limit;
	r64 swing_lower_limit;
	PBD_Axis_Type e1_swing_axis;
	PBD_Axis_Type e2_swing_axis;

	r64 lambda_twist;
	r64 twist_upper_limit;
	r64 twist_lower_limit;
	PBD_Axis_Type e1_twist_axis;
	PBD_Axis_Type e2_twist_axis;
} Spherical_Joint_Constraint;

typedef struct {
	Constraint_Type type;
	eid e1_id;
	eid e2_id;

	union {
		Positional_Constraint positional_constraint;
		Collision_Constraint collision_constraint;
		Mutual_Orientation_Constraint mutual_orientation_constraint;
		Hinge_Joint_Constraint hinge_joint_constraint;
		Spherical_Joint_Constraint spherical_joint_constraint;
	};
} Constraint;

void pbd_simulate(r64 dt, Entity** entities, u32 num_substeps, u32 num_pos_iters, boolean enable_collisions);
void pbd_simulate_with_constraints(r64 dt, Entity** entities, Constraint* external_constraints, u32 num_substeps, u32 num_pos_iters, boolean enable_collisions);

void pbd_positional_constraint_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, r64 compliance, vec3 distance);
void pbd_mutual_orientation_constraint_init(Constraint* constraint, eid e1_id, eid e2_id, r64 compliance);
void pbd_hinge_joint_constraint_unlimited_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, r64 compliance, PBD_Axis_Type e1_aligned_axis, PBD_Axis_Type e2_aligned_axis);
void pbd_hinge_joint_constraint_limited_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, r64 compliance, PBD_Axis_Type e1_aligned_axis, PBD_Axis_Type e2_aligned_axis,
	PBD_Axis_Type e1_limit_axis, PBD_Axis_Type e2_limit_axis, r64 lower_limit, r64 upper_limit);
void pbd_spherical_joint_constraint_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, PBD_Axis_Type e1_swing_axis, PBD_Axis_Type e2_swing_axis,
	PBD_Axis_Type e1_twist_axis, PBD_Axis_Type e2_twist_axis, r64 swing_lower_limit, r64 swing_upper_limit, r64 twist_lower_limit, r64 twist_upper_limit);

#endif