#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include <float.h>
#include "broad.h"
#include "pbd_base_constraints.h"
#include "../util.h"
#include "physics_util.h"

//#include <fenv.h> 

#define ENABLE_SIMULATION_ISLANDS
#define LINEAR_SLEEPING_THRESHOLD 0.15
#define ANGULAR_SLEEPING_THRESHOLD 0.15
#define DEACTIVATION_TIME_TO_BE_INACTIVE 1.0
#define USE_QUATERNIONS_LINEARIZED_FORMULAS
#define CALCULATE_IN_LOCAL_COORDS

void pbd_positional_constraint_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, r64 compliance, vec3 distance) {
	constraint->type = POSITIONAL_CONSTRAINT;
	constraint->positional_constraint.e1_id = e1_id;
	constraint->positional_constraint.e2_id = e2_id;
	constraint->positional_constraint.r1_lc = r1_lc;
	constraint->positional_constraint.r2_lc = r2_lc;
	constraint->positional_constraint.compliance = compliance;
	constraint->positional_constraint.distance = distance;
}

void pbd_mutual_orientation_constraint_init(Constraint* constraint, eid e1_id, eid e2_id, r64 compliance) {
	constraint->type = MUTUAL_ORIENTATION_CONSTRAINT;
	constraint->mutual_orientation_constraint.e1_id = e1_id;
	constraint->mutual_orientation_constraint.e2_id = e2_id;
	constraint->mutual_orientation_constraint.compliance = compliance;
}

void pbd_hinge_joint_constraint_unlimited_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, r64 compliance, PBD_Axis_Type e1_aligned_axis, PBD_Axis_Type e2_aligned_axis) {
	constraint->type = HINGE_JOINT_CONSTRAINT;
	constraint->hinge_joint_constraint.e1_id = e1_id;
	constraint->hinge_joint_constraint.e2_id = e2_id;
	constraint->hinge_joint_constraint.r1_lc = r1_lc;
	constraint->hinge_joint_constraint.r2_lc = r2_lc;
	constraint->hinge_joint_constraint.compliance = compliance;
	constraint->hinge_joint_constraint.e1_aligned_axis = e1_aligned_axis;
	constraint->hinge_joint_constraint.e2_aligned_axis = e2_aligned_axis;
	constraint->hinge_joint_constraint.limited = false;
}

void pbd_hinge_joint_constraint_limited_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, r64 compliance, PBD_Axis_Type e1_aligned_axis, PBD_Axis_Type e2_aligned_axis,
	PBD_Axis_Type e1_limit_axis, PBD_Axis_Type e2_limit_axis, r64 lower_limit, r64 upper_limit) {
	constraint->type = HINGE_JOINT_CONSTRAINT;
	constraint->hinge_joint_constraint.e1_id = e1_id;
	constraint->hinge_joint_constraint.e2_id = e2_id;
	constraint->hinge_joint_constraint.r1_lc = r1_lc;
	constraint->hinge_joint_constraint.r2_lc = r2_lc;
	constraint->hinge_joint_constraint.compliance = compliance;
	constraint->hinge_joint_constraint.e1_aligned_axis = e1_aligned_axis;
	constraint->hinge_joint_constraint.e2_aligned_axis = e2_aligned_axis;
	constraint->hinge_joint_constraint.limited = true;
	constraint->hinge_joint_constraint.e1_limit_axis = e1_limit_axis;
	constraint->hinge_joint_constraint.e2_limit_axis = e2_limit_axis;
	constraint->hinge_joint_constraint.lower_limit = lower_limit;
	constraint->hinge_joint_constraint.upper_limit = upper_limit;
}

void pbd_spherical_joint_constraint_init(Constraint* constraint, eid e1_id, eid e2_id, vec3 r1_lc, vec3 r2_lc, PBD_Axis_Type e1_swing_axis, PBD_Axis_Type e2_swing_axis,
	PBD_Axis_Type e1_twist_axis, PBD_Axis_Type e2_twist_axis, r64 swing_lower_limit, r64 swing_upper_limit, r64 twist_lower_limit, r64 twist_upper_limit) {
	constraint->type = SPHERICAL_JOINT_CONSTRAINT;
	constraint->spherical_joint_constraint.e1_id = e1_id;
	constraint->spherical_joint_constraint.e2_id = e2_id;
	constraint->spherical_joint_constraint.r1_lc = r1_lc;
	constraint->spherical_joint_constraint.r2_lc = r2_lc;
	constraint->spherical_joint_constraint.e1_swing_axis = e1_swing_axis;
	constraint->spherical_joint_constraint.e2_swing_axis = e2_swing_axis;
	constraint->spherical_joint_constraint.e1_twist_axis = e1_twist_axis;
	constraint->spherical_joint_constraint.e2_twist_axis = e2_twist_axis;
	constraint->spherical_joint_constraint.swing_lower_limit = swing_lower_limit;
	constraint->spherical_joint_constraint.swing_upper_limit = swing_upper_limit;
	constraint->spherical_joint_constraint.twist_lower_limit = twist_lower_limit;
	constraint->spherical_joint_constraint.twist_upper_limit = twist_upper_limit;
}

static void positional_constraint_solve(Constraint* constraint, r64 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Entity* e1 = entity_get_by_id(constraint->positional_constraint.e1_id);
	Entity* e2 = entity_get_by_id(constraint->positional_constraint.e2_id);

	vec3 attachment_distance = gm_vec3_subtract(e1->world_position, e2->world_position);
	vec3 delta_x = gm_vec3_subtract(attachment_distance, constraint->positional_constraint.distance);

	Position_Constraint_Preprocessed_Data pcpd;
	calculate_positional_constraint_preprocessed_data(e1, e2, constraint->positional_constraint.r1_lc,
		constraint->positional_constraint.r2_lc, &pcpd);
	r64 delta_lambda = positional_constraint_get_delta_lambda(&pcpd, h, constraint->positional_constraint.compliance,
		constraint->positional_constraint.lambda, delta_x);
	positional_constraint_apply(&pcpd, delta_lambda, delta_x);
	constraint->positional_constraint.lambda += delta_lambda;
}

static vec3 calculate_p_til(Entity* e, vec3 r_lc) {
	return gm_vec3_add(e->previous_world_position, quaternion_apply_to_vec3(&e->previous_world_rotation, r_lc));
}

static vec3 calculate_p(Entity* e, vec3 r_lc) {
	return gm_vec3_add(e->world_position, quaternion_apply_to_vec3(&e->world_rotation, r_lc));
}

static void collision_constraint_solve(Constraint* constraint, r64 h) {
	assert(constraint->type == COLLISION_CONSTRAINT);

	Entity* e1 = entity_get_by_id(constraint->collision_constraint.e1_id);
	Entity* e2 = entity_get_by_id(constraint->collision_constraint.e2_id);

	Position_Constraint_Preprocessed_Data pcpd;
	calculate_positional_constraint_preprocessed_data(e1, e2, constraint->collision_constraint.r1_lc, constraint->collision_constraint.r2_lc, &pcpd);

	// here we calculate 'p1' and 'p2' in order to calculate 'd', as stated in sec (3.5)
	vec3 p1 = gm_vec3_add(e1->world_position, pcpd.r1_wc);
	vec3 p2 = gm_vec3_add(e2->world_position, pcpd.r2_wc);
	r64 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), constraint->collision_constraint.normal);

	if (d > 0.0) {
		vec3 delta_x = gm_vec3_scalar_product(d, constraint->collision_constraint.normal);
		r64 delta_lambda = positional_constraint_get_delta_lambda(&pcpd, h, 0.0, constraint->collision_constraint.lambda_n, delta_x);
		positional_constraint_apply(&pcpd, delta_lambda, delta_x);
		constraint->collision_constraint.lambda_n += delta_lambda;

		// Recalculate entity pair preprocessed data and p1/p2
		calculate_positional_constraint_preprocessed_data(e1, e2, constraint->collision_constraint.r1_lc, constraint->collision_constraint.r2_lc, &pcpd);

		p1 = gm_vec3_add(e1->world_position, pcpd.r1_wc);
		p2 = gm_vec3_add(e2->world_position, pcpd.r2_wc);

		delta_lambda = positional_constraint_get_delta_lambda(&pcpd, h, 0.0, constraint->collision_constraint.lambda_t, delta_x);

		// We should also add a constraint for static friction, but only if lambda_t < u_s * lambda_n
		const r64 static_friction_coefficient = (e1->static_friction_coefficient + e2->static_friction_coefficient) / 2.0f;

		r64 lambda_n = constraint->collision_constraint.lambda_n;
		r64 lambda_t = constraint->collision_constraint.lambda_t + delta_lambda;
		// @NOTE(fek): This inequation shown in 3.5 was changed because the lambdas will always be negative!
		if (lambda_t > static_friction_coefficient * lambda_n) {
			vec3 p1_til = gm_vec3_add(e1->previous_world_position,
				quaternion_apply_to_vec3(&e1->previous_world_rotation, constraint->collision_constraint.r1_lc));
			vec3 p2_til = gm_vec3_add(e2->previous_world_position,
				quaternion_apply_to_vec3(&e2->previous_world_rotation, constraint->collision_constraint.r2_lc));
			vec3 delta_p = gm_vec3_subtract(gm_vec3_subtract(p1, p1_til), gm_vec3_subtract(p2, p2_til));
			vec3 delta_p_t = gm_vec3_subtract(delta_p, gm_vec3_scalar_product(
				gm_vec3_dot(delta_p, constraint->collision_constraint.normal), constraint->collision_constraint.normal));

			positional_constraint_apply(&pcpd, delta_lambda, delta_p_t);
			constraint->collision_constraint.lambda_t += delta_lambda;
		}
	}
}

static void mutual_orientation_constraint_solve(Constraint* constraint, r64 h) {
	assert(constraint->type == MUTUAL_ORIENTATION_CONSTRAINT);

	Entity* e1 = entity_get_by_id(constraint->mutual_orientation_constraint.e1_id);
	Entity* e2 = entity_get_by_id(constraint->mutual_orientation_constraint.e2_id);

	Angular_Constraint_Preprocessed_Data acpd;
	calculate_angular_constraint_preprocessed_data(e1, e2, &acpd);

	Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
	Quaternion aux = quaternion_product(&e1->world_rotation, &q2_inv);
	vec3 delta_q = (vec3){2.0 * aux.x, 2.0 * aux.y, 2.0 * aux.z};

	r64 delta_lambda = angular_constraint_get_delta_lambda(&acpd, h, constraint->mutual_orientation_constraint.compliance,
		constraint->mutual_orientation_constraint.lambda, delta_q);
	angular_constraint_apply(&acpd, delta_lambda, delta_q);
	constraint->mutual_orientation_constraint.lambda += delta_lambda;
}

static boolean limit_angle(vec3 n, vec3 n1, vec3 n2, r64 alpha, r64 beta, vec3* delta_q) {
	// Calculate phi, which is the angle between n1 and n2 with respect to the rotation vector n
	r64 phi = asin(gm_vec3_dot(gm_vec3_cross(n1, n2), n));
	// asin returns the angle in the interval [-pi/2,+pi/2], which is already correct if the angle between n1 and n2 is acute.
	// however, if n1 and n2 forms an obtuse angle, we need to manually differentiate. In this case, n1 dot n2 is less than 0.
	// For example, if the angle between n1 and n2 is 30 degrees, then sin(30)=0.5, but if the angle is 150, sin(150)=0.5 as well,
	// thus in both cases asin will return 30 degrees (pi/6)
	if (gm_vec3_dot(n1, n2) < 0.0) {
		phi = PI_F - phi; // this will do the trick and fix the angle
	}
	// now our angle is between [-pi/2, 3pi/2].

	// maps the inner range [pi, 3pi/2] to [-pi, -pi/2]
	if (phi > PI_F) {
		phi = phi - 2.0 * PI_F;
	}
	// now our angle is between [-pi, pi]

	// this is useless?
	if (phi < -PI_F) {
		phi = phi + 2.0 * PI_F;
	}

	if (phi < alpha || phi > beta) {
		// at this point, phi represents the angle between n1 and n2
		
		// clamp phi to get the limit angle, i.e., the angle that we wanna 'be at'
		phi = CLAMP(phi, alpha, beta);

		// create a quaternion that represents this rotation
		Quaternion rot = quaternion_new_radians(n, phi);

		// rotate n1 by the limit angle, so n1 will get very close to n2, except for the extra rotation that we wanna get rid of
		n1 = quaternion_apply_to_vec3(&rot, n1);

		// calculate delta_q based on this extra rotation

		*delta_q = gm_vec3_cross(n1, n2);
		return true;
	}

	return false;
}

static vec3 get_axis_in_world_coords(const Quaternion* entity_rotation, PBD_Axis_Type axis) {
	switch (axis) {
		case PBD_POSITIVE_X_AXIS: {
			return quaternion_get_right(entity_rotation);
		} break;
		case PBD_NEGATIVE_X_AXIS: {
			return quaternion_get_right_inverted(entity_rotation);
		} break;
		case PBD_POSITIVE_Y_AXIS: {
			return quaternion_get_up(entity_rotation);
		} break;
		case PBD_NEGATIVE_Y_AXIS: {
			return quaternion_get_up_inverted(entity_rotation);
		} break;
		case PBD_POSITIVE_Z_AXIS: {
			return quaternion_get_forward(entity_rotation);
		} break;
		case PBD_NEGATIVE_Z_AXIS: {
			return quaternion_get_forward_inverted(entity_rotation);
		} break;
	}

	assert(0);
	return (vec3) { 0.0, 0.0, 0.0 };
}

static void hinge_joint_constraint_solve(Constraint* constraint, r64 h) {
	assert(constraint->type == HINGE_JOINT_CONSTRAINT);

	Entity* e1 = entity_get_by_id(constraint->hinge_joint_constraint.e1_id);
	Entity* e2 = entity_get_by_id(constraint->hinge_joint_constraint.e2_id);

	// Angular Constraint to make sure the aligned axis are kept aligned
	Angular_Constraint_Preprocessed_Data acpd;
	calculate_angular_constraint_preprocessed_data(e1, e2, &acpd);

	vec3 e1_a_wc = get_axis_in_world_coords(&e1->world_rotation, constraint->hinge_joint_constraint.e1_aligned_axis);
	vec3 e2_a_wc = get_axis_in_world_coords(&e2->world_rotation, constraint->hinge_joint_constraint.e2_aligned_axis);
	vec3 delta_q = gm_vec3_cross(e1_a_wc, e2_a_wc);

	r64 delta_lambda = angular_constraint_get_delta_lambda(&acpd, h, constraint->hinge_joint_constraint.compliance,
		constraint->hinge_joint_constraint.lambda_aligned_axes, delta_q);
	angular_constraint_apply(&acpd, delta_lambda, delta_q);
	constraint->hinge_joint_constraint.lambda_aligned_axes += delta_lambda;

	// Positional constraint to ensure that the distance between both entities are correct
	// @TODO: optmize preprocessed datas
	Position_Constraint_Preprocessed_Data pcpd;
	calculate_positional_constraint_preprocessed_data(e1, e2, constraint->hinge_joint_constraint.r1_lc,
		constraint->hinge_joint_constraint.r2_lc, &pcpd);

	vec3 p1 = gm_vec3_add(e1->world_position, pcpd.r1_wc);
	vec3 p2 = gm_vec3_add(e2->world_position, pcpd.r2_wc);
	vec3 delta_r = gm_vec3_subtract(p1, p2);
	vec3 delta_x = delta_r;

	//printf("%f\n", gm_vec3_length(delta_x));

	delta_lambda = positional_constraint_get_delta_lambda(&pcpd, h, 0.0, constraint->hinge_joint_constraint.lambda_pos, delta_x);
	positional_constraint_apply(&pcpd, delta_lambda, delta_x);
	constraint->hinge_joint_constraint.lambda_pos += delta_lambda;

	// Finally, angular constraint to ensure the joint angle limit is respected
	if (constraint->hinge_joint_constraint.limited) {
		vec3 n1 = get_axis_in_world_coords(&e1->world_rotation, constraint->hinge_joint_constraint.e1_limit_axis);
		vec3 n2 = get_axis_in_world_coords(&e2->world_rotation, constraint->hinge_joint_constraint.e2_limit_axis);
		vec3 n = get_axis_in_world_coords(&e1->world_rotation, constraint->hinge_joint_constraint.e1_aligned_axis);
		r64 alpha = constraint->hinge_joint_constraint.lower_limit;
		r64 beta = constraint->hinge_joint_constraint.upper_limit;

		if (limit_angle(n, n1, n2, alpha, beta, &delta_q)) {
			// Angular Constraint
			Angular_Constraint_Preprocessed_Data acpd;
			calculate_angular_constraint_preprocessed_data(e1, e2, &acpd);

			r64 delta_lambda = angular_constraint_get_delta_lambda(&acpd, h, 0.0, constraint->hinge_joint_constraint.lambda_limit_axes, delta_q);
			angular_constraint_apply(&acpd, delta_lambda, delta_q);
			constraint->hinge_joint_constraint.lambda_limit_axes += delta_lambda;
		}
	}
}

static void spherical_joint_constraint_solve(Constraint* constraint, r64 h) {
	assert(constraint->type == SPHERICAL_JOINT_CONSTRAINT);

	const r64 EPSILON = 1e-50;

	Entity* e1 = entity_get_by_id(constraint->spherical_joint_constraint.e1_id);
	Entity* e2 = entity_get_by_id(constraint->spherical_joint_constraint.e2_id);

	// Positional constraint to ensure that the distance between both entities are correct
	Position_Constraint_Preprocessed_Data pcpd;
	calculate_positional_constraint_preprocessed_data(e1, e2, constraint->spherical_joint_constraint.r1_lc,
		constraint->spherical_joint_constraint.r2_lc, &pcpd);

	vec3 p1 = gm_vec3_add(e1->world_position, pcpd.r1_wc);
	vec3 p2 = gm_vec3_add(e2->world_position, pcpd.r2_wc);
	vec3 delta_r = gm_vec3_subtract(p1, p2);
	vec3 delta_x = delta_r;

	r64 delta_lambda = positional_constraint_get_delta_lambda(&pcpd, h, 0.0, constraint->spherical_joint_constraint.lambda_pos, delta_x);
	positional_constraint_apply(&pcpd, delta_lambda, delta_x);
	constraint->spherical_joint_constraint.lambda_pos += delta_lambda;

	// Angular constraint to ensure the swing angle limit is respected
	vec3 n1 = get_axis_in_world_coords(&e1->world_rotation, constraint->spherical_joint_constraint.e1_swing_axis);
	vec3 n2 = get_axis_in_world_coords(&e2->world_rotation, constraint->spherical_joint_constraint.e2_swing_axis);
	vec3 n = gm_vec3_cross(n1, n2);
	r64 n_len = gm_vec3_length(n);
	if (n_len > EPSILON) {
		n = (vec3) {n.x / n_len, n.y / n_len, n.z / n_len};

		r64 alpha = constraint->spherical_joint_constraint.swing_lower_limit;
		r64 beta = constraint->spherical_joint_constraint.swing_upper_limit;
		vec3 delta_q;

		if (limit_angle(n, n1, n2, alpha, beta, &delta_q)) {
			// Angular Constraint
			Angular_Constraint_Preprocessed_Data acpd;
			calculate_angular_constraint_preprocessed_data(e1, e2, &acpd);

			r64 delta_lambda = angular_constraint_get_delta_lambda(&acpd, h, 0.0, constraint->spherical_joint_constraint.lambda_swing, delta_q);
			angular_constraint_apply(&acpd, delta_lambda, delta_q);
			constraint->spherical_joint_constraint.lambda_swing += delta_lambda;
		}
	}

	// Angular constraint to ensure the twist angle limit is respected
	vec3 a1 = get_axis_in_world_coords(&e1->world_rotation, constraint->spherical_joint_constraint.e1_swing_axis);
	vec3 b1 = get_axis_in_world_coords(&e1->world_rotation, constraint->spherical_joint_constraint.e1_twist_axis);
	vec3 a2 = get_axis_in_world_coords(&e2->world_rotation, constraint->spherical_joint_constraint.e2_swing_axis);
	vec3 b2 = get_axis_in_world_coords(&e2->world_rotation, constraint->spherical_joint_constraint.e2_twist_axis);
	n = gm_vec3_add(a1, a2);
	n_len = gm_vec3_length(n);
	if (n_len > EPSILON) {
		n = (vec3) {n.x / n_len, n.y / n_len, n.z / n_len};

		n1 = gm_vec3_subtract(b1, gm_vec3_scalar_product(gm_vec3_dot(n, b1), n));
		n2 = gm_vec3_subtract(b2, gm_vec3_scalar_product(gm_vec3_dot(n, b2), n));
		r64 n1_len = gm_vec3_length(n1);
		r64 n2_len = gm_vec3_length(n2);
		if (n1_len > EPSILON && n2_len > EPSILON) {
			n1 = (vec3) {n1.x / n1_len, n1.y / n1_len, n1.z / n1_len};
			n2 = (vec3) {n2.x / n2_len, n2.y / n2_len, n2.z / n2_len};

			r64 alpha = constraint->spherical_joint_constraint.twist_lower_limit;
			r64 beta = constraint->spherical_joint_constraint.twist_upper_limit;
			vec3 delta_q;

			if (limit_angle(n, n1, n2, alpha, beta, &delta_q)) {
				// Angular Constraint
				Angular_Constraint_Preprocessed_Data acpd;
				calculate_angular_constraint_preprocessed_data(e1, e2, &acpd);

				r64 delta_lambda = angular_constraint_get_delta_lambda(&acpd, h, 0.0, constraint->spherical_joint_constraint.lambda_twist, delta_q);
				angular_constraint_apply(&acpd, delta_lambda, delta_q);
				constraint->spherical_joint_constraint.lambda_twist += delta_lambda;
			}
		}
	}
}

static void solve_constraint(Constraint* constraint, r64 h) {
	switch (constraint->type) {
		case POSITIONAL_CONSTRAINT: {
			positional_constraint_solve(constraint, h);
			return;
		} break;
		case COLLISION_CONSTRAINT: {
			collision_constraint_solve(constraint, h);
			return;
		} break;
		case MUTUAL_ORIENTATION_CONSTRAINT: {
			mutual_orientation_constraint_solve(constraint, h);
			return;
		} break;
		case HINGE_JOINT_CONSTRAINT: {
			hinge_joint_constraint_solve(constraint, h);
			return;
		} break;
		case SPHERICAL_JOINT_CONSTRAINT: {
			spherical_joint_constraint_solve(constraint, h);
			return;
		} break;
	}

	assert(0);
}

void clipping_contact_to_collision_constraint(Entity* e1, Entity* e2, Collider_Contact* contact, Constraint* constraint) {
	constraint->type = COLLISION_CONSTRAINT;
	constraint->collision_constraint.e1_id = e1->id;
	constraint->collision_constraint.e2_id = e2->id;
	constraint->collision_constraint.normal = contact->normal;
	constraint->collision_constraint.lambda_n = 0.0;
	constraint->collision_constraint.lambda_t = 0.0;

	vec3 r1_wc = gm_vec3_subtract(contact->collision_point1, e1->world_position);
	vec3 r2_wc = gm_vec3_subtract(contact->collision_point2, e2->world_position);

	Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
	constraint->collision_constraint.r1_lc = quaternion_apply_to_vec3(&q1_inv, r1_wc);

	Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
	constraint->collision_constraint.r2_lc = quaternion_apply_to_vec3(&q2_inv, r2_wc);
}

static Constraint* copy_constraints(Constraint* constraints) {
	if (constraints == NULL) {
		return array_new(Constraint);
	}

	Constraint* copied_constraints = (Constraint*)array_copy(constraints);

	for (u32 i = 0; i < array_length(copied_constraints); ++i) {
		Constraint* constraint = &copied_constraints[i];

		// Reset lambda
		switch (constraint->type) {
			case POSITIONAL_CONSTRAINT: {
				constraint->positional_constraint.lambda = 0.0;
			} break;
			case COLLISION_CONSTRAINT: {
				constraint->collision_constraint.lambda_t = 0.0;
				constraint->collision_constraint.lambda_n = 0.0;
			} break;
			case MUTUAL_ORIENTATION_CONSTRAINT: {
				constraint->mutual_orientation_constraint.lambda = 0.0;
			} break;
			case HINGE_JOINT_CONSTRAINT: {
				constraint->hinge_joint_constraint.lambda_pos = 0.0;
				constraint->hinge_joint_constraint.lambda_aligned_axes = 0.0;
				constraint->hinge_joint_constraint.lambda_limit_axes = 0.0;
			} break;
			case SPHERICAL_JOINT_CONSTRAINT: {
				constraint->spherical_joint_constraint.lambda_pos = 0.0;
				constraint->spherical_joint_constraint.lambda_swing = 0.0;
				constraint->spherical_joint_constraint.lambda_twist = 0.0;
			} break;
		}
	}

	return copied_constraints;
}

void pbd_simulate(r64 dt, Entity** entities, u32 num_substeps, u32 num_pos_iters) {
	pbd_simulate_with_constraints(dt, entities, NULL, num_substeps, num_pos_iters);
}

void pbd_simulate_with_constraints(r64 dt, Entity** entities, Constraint* external_constraints, u32 num_substeps, u32 num_pos_iters) {
	//feenableexcept(FE_INVALID | FE_OVERFLOW);

	if (dt <= 0.0) return;
	r64 h = dt / num_substeps;

	Broad_Collision_Pair* broad_collision_pairs = broad_get_collision_pairs(entities);

#ifdef ENABLE_SIMULATION_ISLANDS
	eid** simulation_islands = broad_collect_simulation_islands(entities, broad_collision_pairs);

	// All entities will be contained in the simulation islands.
	// Update deactivation time and also, at the same time, its active status
	for (u32 j = 0; j < array_length(simulation_islands); ++j) {
		eid* simulation_island = simulation_islands[j];

		boolean all_inactive = true;
		for (u32 k = 0; k < array_length(simulation_island); ++k) {
			Entity* e = entity_get_by_id(simulation_island[k]);

			r64 linear_velocity_len = gm_vec3_length(e->linear_velocity);
			r64 angular_velocity_len = gm_vec3_length(e->angular_velocity);
			if (linear_velocity_len < LINEAR_SLEEPING_THRESHOLD && angular_velocity_len < ANGULAR_SLEEPING_THRESHOLD) {
				e->deactivation_time += dt; // we should use 'dt' if doing once per frame
			} else {
				e->deactivation_time = 0.0;
			}

			if (e->deactivation_time < DEACTIVATION_TIME_TO_BE_INACTIVE) {
				all_inactive = false;
			}
		}

		// We only set entities to inactive if the whole island is inactive!
		for (u32 k = 0; k < array_length(simulation_island); ++k) {
			Entity* e = entity_get_by_id(simulation_island[k]);
			e->active = !all_inactive;
		}
	}
#if 0
	for (u32 j = 0; j < array_length(simulation_islands); ++j) {
		eid* simulation_island = simulation_islands[j];
		vec4 color = util_pallete(j);
		for (u32 k = 0; k < array_length(simulation_island); ++k) {
			Entity* e = entity_get_by_id(simulation_island[k]);
			e->color = color;
		}
	}
#else
/*
	for (u32 j = 0; j < array_length(simulation_islands); ++j) {
		eid* simulation_island = simulation_islands[j];
		for (u32 k = 0; k < array_length(simulation_island); ++k) {
			Entity* e = entity_get_by_id(simulation_island[k]);
			if (e->active) {
				e->color = util_pallete(1);
			} else {
				e->color = util_pallete(0);
			}
		}
	}
*/
#endif

	broad_simulation_islands_destroy(simulation_islands);
#endif

	// The main loop of the PBD simulation
	for (u32 i = 0; i < num_substeps; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = entities[j];
			// Stores the previous position and orientation of the entity
			e->previous_world_position = e->world_position;
			e->previous_world_rotation = e->world_rotation;

			if (e->fixed) continue;	
			if (!e->active) continue;

			// Calculate the external force and torque of the entity
			vec3 external_force = calculate_external_force(e);
			vec3 external_torque = calculate_external_torque(e);

			// Update the entity position and linear velocity based on the current velocity and applied forces
			e->linear_velocity = gm_vec3_add(e->linear_velocity, gm_vec3_scalar_product(h * e->inverse_mass, external_force));
			e->world_position = gm_vec3_add(e->world_position, gm_vec3_scalar_product(h, e->linear_velocity));

			// Update the entity orientation and angular velocity based on the current velocity and applied forces

#ifdef CALCULATE_IN_LOCAL_COORDS
			vec3 angular_velocity_lc = quaternion_apply_inverse_to_vec3(&e->world_rotation, e->angular_velocity);
			vec3 external_torque_lc = quaternion_apply_inverse_to_vec3(&e->world_rotation, external_torque);
			mat3 e_inverse_inertia_tensor = e->inverse_inertia_tensor;
			mat3 e_inertia_tensor = e->inertia_tensor;
			angular_velocity_lc = gm_vec3_add(angular_velocity_lc, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(external_torque_lc,
				gm_vec3_cross(angular_velocity_lc, gm_mat3_multiply_vec3(&e_inertia_tensor, angular_velocity_lc))))));
			e->angular_velocity = quaternion_apply_to_vec3(&e->world_rotation, angular_velocity_lc);
#else
			mat3 e_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e);
			mat3 e_inertia_tensor = get_dynamic_inertia_tensor(e);
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(external_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e_inertia_tensor, e->angular_velocity))))));
#endif

#ifdef USE_QUATERNIONS_LINEARIZED_FORMULAS
			Quaternion aux = (Quaternion){e->angular_velocity.x, e->angular_velocity.y, e->angular_velocity.z, 0.0};
			Quaternion q = quaternion_product(&aux, &e->world_rotation);
			e->world_rotation.x = e->world_rotation.x + h * 0.5 * q.x;
			e->world_rotation.y = e->world_rotation.y + h * 0.5 * q.y;
			e->world_rotation.z = e->world_rotation.z + h * 0.5 * q.z;
			e->world_rotation.w = e->world_rotation.w + h * 0.5 * q.w;
			// should we normalize?
			e->world_rotation = quaternion_normalize(&e->world_rotation);
#else
			r64 rotation_angle = gm_vec3_length(e->angular_velocity) * h;
			vec3 rotation_axis = gm_vec3_normalize(e->angular_velocity);
			Quaternion orientation_change = quaternion_new_radians(rotation_axis, rotation_angle);
			e->world_rotation = quaternion_product(&orientation_change, &e->world_rotation);
			// should we normalize?
			e->world_rotation = quaternion_normalize(&e->world_rotation);
#endif
		}

		// Create the constraints array
		Constraint* constraints = copy_constraints(external_constraints);

		// As explained in sec 3.5, in each substep we need to check for collisions
		for (u32 j = 0; j < array_length(broad_collision_pairs); ++j) {
			Entity* e1 = entity_get_by_id(broad_collision_pairs[j].e1_id);
			Entity* e2 = entity_get_by_id(broad_collision_pairs[j].e2_id);

			// If e1 is "colliding" with e2, they must be either both active or both inactive
			if (!e1->fixed && !e2->fixed) {
				assert((e1->active && e2->active) || (!e1->active && !e2->active));
			}

			// No need to solve the collision if both entities are either inactive or fixed
			if ((e1->fixed || !e1->active) && (e2->fixed || !e2->active)) {
				continue;
			}

			colliders_update(e1->colliders, e1->world_position, &e1->world_rotation);
			colliders_update(e2->colliders, e2->world_position, &e2->world_rotation);

			Collider_Contact* contacts = colliders_get_contacts(e1->colliders, e2->colliders);
			if (contacts) {
				for (u32 l = 0; l < array_length(contacts); ++l) {
					Collider_Contact* contact = &contacts[l];
					Constraint constraint;
					clipping_contact_to_collision_constraint(e1, e2, contact, &constraint);
					array_push(constraints, constraint);
				}
				array_free(contacts);
			}
		}

#if 0
		int size = array_length(constraints);
		u32* idxs = array_new(u32);
		for (u32 i = 0; i < size; ++i) {
			int idx;
			boolean n;
			do {
				n = true;
				idx = rand() % size;
				for (u32 j = 0; j < array_length(idxs); ++j) {
					if (idx == idxs[j]) {
						n = false;
						break;
					}
				}
			} while (!n);
			array_push(idxs, idx);
		}

		Constraint* abc = array_new(Constraint);
		for (u32 i = 0; i < array_length(idxs); ++i) {
			array_push(abc, constraints[idxs[i]]);
		}
		constraints = abc;
#endif

		// Now we run the PBD solver with NUM_POS_ITERS iterations
		for (u32 j = 0; j < num_pos_iters; ++j) {
			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				solve_constraint(constraint, h);
			}	
		}

		// The PBD velocity update
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = entities[j];
			if (e->fixed) continue;
			if (!e->active) continue;
			
			// We start by storing the current velocities (this is needed for the velocity solver that comes at the end of the loop)
			e->previous_linear_velocity = e->linear_velocity;
			e->previous_angular_velocity = e->angular_velocity;

			// Update the linear velocity based on the position difference
			e->linear_velocity = gm_vec3_scalar_product(1.0 / h, gm_vec3_subtract(e->world_position, e->previous_world_position));

			// Update the angular velocity based on the orientation difference
			Quaternion inv = quaternion_inverse(&e->previous_world_rotation);
			Quaternion delta_q = quaternion_product(&e->world_rotation, &inv);
			if (delta_q.w >= 0.0) {
				e->angular_velocity = gm_vec3_scalar_product(2.0 / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			} else {
				e->angular_velocity = gm_vec3_scalar_product(-2.0 / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			}
		}

		// The velocity solver - we run this additional solver for every collision that we found
		for (u32 j = 0; j < array_length(constraints); ++j) {
			Constraint* constraint = &constraints[j];
			if (constraint->type == COLLISION_CONSTRAINT) {
				Entity* e1 = entity_get_by_id(constraint->collision_constraint.e1_id);
				Entity* e2 = entity_get_by_id(constraint->collision_constraint.e2_id);
				vec3 n = constraint->collision_constraint.normal;
				r64 lambda_n = constraint->collision_constraint.lambda_n;
				r64 lambda_t = constraint->collision_constraint.lambda_t;

				Position_Constraint_Preprocessed_Data pcpd;
				calculate_positional_constraint_preprocessed_data(e1, e2, constraint->collision_constraint.r1_lc,
					constraint->collision_constraint.r2_lc, &pcpd);

				vec3 v1 = e1->linear_velocity;
				vec3 w1 = e1->angular_velocity;
				vec3 v2 = e2->linear_velocity;
				vec3 w2 = e2->angular_velocity;

				// We start by calculating the relative normal and tangential velocities at the contact point, as described in (3.6)
				// @NOTE: equation (29) was modified here
				vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, pcpd.r1_wc)), gm_vec3_add(v2, gm_vec3_cross(w2, pcpd.r2_wc)));
				r64 vn = gm_vec3_dot(n, v);
				vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));

				// delta_v stores the velocity change that we need to perform at the end of the solver
				vec3 delta_v = (vec3){0.0, 0.0, 0.0};
				
				// we start by applying Coloumb's dynamic friction force
				const r64 dynamic_friction_coefficient = (e1->dynamic_friction_coefficient + e2->dynamic_friction_coefficient) / 2.0f;
				r64 fn = lambda_n / h; // simplifly h^2 by ommiting h in the next calculation
				// @NOTE: equation (30) was modified here
				r64 fact = MIN(dynamic_friction_coefficient * fabs(fn), gm_vec3_length(vt));
				// update delta_v
				delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(-fact, gm_vec3_normalize(vt)));

				// Now we handle restitution
				vec3 old_v1 = e1->previous_linear_velocity;
				vec3 old_w1 = e1->previous_angular_velocity;
				vec3 old_v2 = e2->previous_linear_velocity;
				vec3 old_w2 = e2->previous_angular_velocity;
				vec3 v_til = gm_vec3_subtract(gm_vec3_add(old_v1, gm_vec3_cross(old_w1, pcpd.r1_wc)), gm_vec3_add(old_v2, gm_vec3_cross(old_w2, pcpd.r2_wc)));
				r64 vn_til = gm_vec3_dot(n, v_til);
				//r64 e = (fabs(vn) > 2.0 * GRAVITY * h) ? 0.8 : 0.0;
				r64 e = e1->restitution_coefficient * e2->restitution_coefficient;
				// @NOTE: equation (34) was modified here
				fact = -vn + MIN(-e * vn_til, 0.0);
				// update delta_v
				delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(fact, n));

				// Finally, we end the solver by applying delta_v, considering the inverse masses of both entities
#ifdef CALCULATE_IN_LOCAL_COORDS
				vec3 r1_lc = constraint->collision_constraint.r1_lc;
				vec3 r2_lc = constraint->collision_constraint.r2_lc;
				vec3 e1_n_lc = quaternion_apply_inverse_to_vec3(&e1->world_rotation, n);
				vec3 e2_n_lc = quaternion_apply_inverse_to_vec3(&e2->world_rotation, n);
				vec3 e1_cross = gm_vec3_cross(r1_lc, e1_n_lc);
				vec3 e2_cross = gm_vec3_cross(r2_lc, e2_n_lc);
				r64 _w1 = e1->inverse_mass + gm_vec3_dot(e1_cross, gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, e1_cross));
				r64 _w2 = e2->inverse_mass + gm_vec3_dot(e2_cross, gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, e2_cross));
#else
				vec3 e1_cross = gm_vec3_cross(pcpd.r1_wc, n);
				vec3 e2_cross = gm_vec3_cross(pcpd.r2_wc, n);
				r64 _w1 = e1->inverse_mass + gm_vec3_dot(e1_cross, gm_mat3_multiply_vec3(&pcpd.e1_inverse_inertia_tensor, e1_cross));
				r64 _w2 = e2->inverse_mass + gm_vec3_dot(e2_cross, gm_mat3_multiply_vec3(&pcpd.e2_inverse_inertia_tensor, e2_cross));
#endif
				vec3 p = gm_vec3_scalar_product(1.0 / (_w1 + _w2), delta_v);

				if (!e1->fixed) {
					e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(e1->inverse_mass, p));
#ifdef CALCULATE_IN_LOCAL_COORDS
					vec3 p_lc = quaternion_apply_inverse_to_vec3(&e1->world_rotation, p);
					vec3 delta_angular_velocity_lc = gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1_lc, p_lc));
					e1->angular_velocity = gm_vec3_add(e1->angular_velocity, quaternion_apply_to_vec3(&e1->world_rotation, delta_angular_velocity_lc));
#else
					e1->angular_velocity = gm_vec3_add(e1->angular_velocity,
						gm_mat3_multiply_vec3(&pcpd.e1_inverse_inertia_tensor, gm_vec3_cross(pcpd.r1_wc, p)));
#endif
				}
				if (!e2->fixed) {
					e2->linear_velocity = gm_vec3_add(e2->linear_velocity, gm_vec3_invert(gm_vec3_scalar_product(e2->inverse_mass, p)));
#ifdef CALCULATE_IN_LOCAL_COORDS
					vec3 p_lc = quaternion_apply_inverse_to_vec3(&e2->world_rotation, p);
					vec3 delta_angular_velocity_lc = gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2_lc, p_lc));
					e2->angular_velocity = gm_vec3_add(e2->angular_velocity,
						gm_vec3_invert(quaternion_apply_to_vec3(&e2->world_rotation, delta_angular_velocity_lc)));
#else
					e2->angular_velocity = gm_vec3_add(e2->angular_velocity,
						gm_vec3_invert(gm_mat3_multiply_vec3(&pcpd.e2_inverse_inertia_tensor, gm_vec3_cross(pcpd.r2_wc, p))));
#endif
				}
			} else if (0 && constraint->type == HINGE_JOINT_CONSTRAINT) {
				Entity* e1 = entity_get_by_id(constraint->hinge_joint_constraint.e1_id);
				Entity* e2 = entity_get_by_id(constraint->hinge_joint_constraint.e2_id);

				// angular damping
				vec3 omega_diff = gm_vec3_subtract(e2->angular_velocity, e1->angular_velocity);
				omega_diff = gm_vec3_scalar_product(MIN(1.0, 10.0 * h), omega_diff);
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, omega_diff);
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, omega_diff);

				// linear damping
				vec3 delta_v = gm_vec3_subtract(e2->linear_velocity, e1->linear_velocity);
				delta_v = gm_vec3_scalar_product(MIN(1.0, 10.0 * h), delta_v);

				// Finally, we end the solver by applying delta_v, considering the inverse masses of both entities
				r64 _w1 = e1->inverse_mass;
				r64 _w2 = e2->inverse_mass;
				vec3 p = gm_vec3_scalar_product(1.0 / (_w1 + _w2), delta_v);

				if (!e1->fixed) {
					e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(e1->inverse_mass, p));
				}
				if (!e2->fixed) {
					e2->linear_velocity = gm_vec3_add(e2->linear_velocity, gm_vec3_invert(gm_vec3_scalar_product(e2->inverse_mass, p)));
				}
			}
		}

		array_free(constraints);
	}

	array_free(broad_collision_pairs);
	//fedisableexcept(FE_INVALID | FE_OVERFLOW);
}