#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include <float.h>
#include "gjk.h"
#include "epa.h"
#include "clipping.h"
#include "broad.h"
#include "../util.h"

#define NUM_SUBSTEPS 10
#define NUM_POS_ITERS 1
#define USE_QUATERNIONS_LINEARIZED_FORMULAS
#define ENABLE_SIMULATION_ISLANDS
#define LINEAR_SLEEPING_THRESHOLD 0.15
#define ANGULAR_SLEEPING_THRESHOLD 0.15
#define DEACTIVATION_TIME_TO_BE_INACTIVE 1.0

extern boolean paused;

// Calculate the sum of all external forces acting on an entity
static vec3 calculate_external_force(Entity* e) {
	const vec3 center_of_mass = (vec3){0.0, 0.0, 0.0};
	vec3 total_force = (vec3){0.0, 0.0, 0.0};
	for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
		total_force = gm_vec3_add(total_force, e->forces[i].force);
	}
	return total_force;
}

// Calculate the sum of all external torques acting on an entity
static vec3 calculate_external_torque(Entity* e) {
	const vec3 center_of_mass = (vec3){0.0, 0.0, 0.0};
	vec3 total_torque = (vec3){0.0, 0.0, 0.0};
	for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
		vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
		total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, e->forces[i].force));
	}
	return total_torque;
}

// Calculate the dynamic inertia tensor of an entity, i.e., the inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inertia_tensor(Entity* e) {
#if 1
	// Can only be used if the local->world matrix is orthogonal
	mat3 rotation_matrix = quaternion_get_matrix3(&e->world_rotation);
	mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix);
	mat3 aux = gm_mat3_multiply(&rotation_matrix, &e->inertia_tensor);
	return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
#else
	// Can always be used
	mat3 local_to_world = quaternion_get_matrix3(&e->world_rotation);
	mat3 inverse_local_to_world;
	assert(gm_mat3_inverse(&local_to_world, &inverse_local_to_world));
	mat3 transposed_inverse_local_to_world = gm_mat3_transpose(&inverse_local_to_world);
	mat3 aux = gm_mat3_multiply(&transposed_inverse_local_to_world, &e->inertia_tensor);
	return gm_mat3_multiply(&aux, &inverse_local_to_world);
#endif
}

// Calculate the dynamic inverse inertia tensor of an entity, i.e., the inverse inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inverse_inertia_tensor(Entity* e) {
#if 1
	// Can only be used if the local->world matrix is orthogonal
	mat3 rotation_matrix = quaternion_get_matrix3(&e->world_rotation);
	mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix);
	mat3 aux = gm_mat3_multiply(&rotation_matrix, &e->inverse_inertia_tensor);
	return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
#else
	// Can always be used
	mat3 local_to_world = quaternion_get_matrix3(&e->world_rotation);
	mat3 inverse_local_to_world;
	assert(gm_mat3_inverse(&local_to_world, &inverse_local_to_world));
	mat3 transposed_inverse_local_to_world = gm_mat3_transpose(&inverse_local_to_world);
	mat3 aux = gm_mat3_multiply(&transposed_inverse_local_to_world, &e->inverse_inertia_tensor);
	return gm_mat3_multiply(&aux, &inverse_local_to_world);
#endif
}

static vec3 calculate_p_til(Entity* e, vec3 r_lc) {
	mat3 previous_rot_matrix = quaternion_get_matrix3(&e->previous_world_rotation);
	return gm_vec3_add(e->previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, r_lc));
}

static vec3 calculate_p(Entity* e, vec3 r_lc) {
	mat3 current_rot_matrix = quaternion_get_matrix3(&e->world_rotation);
	return gm_vec3_add(e->world_position, gm_mat3_multiply_vec3(&current_rot_matrix, r_lc));
}

// Apply the positional constraint, updating the position and orientation of the entities accordingly
static r64 apply_positional_constraint(Constraint* constraint, r64 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Entity* e1 = constraint->positional_constraint.e1;
	Entity* e2 = constraint->positional_constraint.e2;
	mat3 e1_rot = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1_lc = constraint->positional_constraint.r1_lc;
	vec3 r2_lc = constraint->positional_constraint.r2_lc;
	r64 lambda = constraint->positional_constraint.lambda;
	r64 compliance = constraint->positional_constraint.compliance;
	vec3 delta_x = constraint->positional_constraint.delta_x;

	mat3 e1_rot_matrix = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot_matrix = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1_wc = gm_mat3_multiply_vec3(&e1_rot_matrix, r1_lc);
	vec3 r2_wc = gm_mat3_multiply_vec3(&e2_rot_matrix, r2_lc);

	mat3 e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
	mat3 e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);

	// split delta_x into n and c.
	vec3 n = gm_vec3_normalize(delta_x);
	r64 c = gm_vec3_length(delta_x);

	// calculate the inverse masses of both entities
	r64 w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1_wc, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, n)));
	r64 w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2_wc, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, n)));

	// calculate the delta_lambda (XPBD) and updates the constraint
	r64 til_compliance = compliance / (h * h);
	r64 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);

	// calculates the positional impulse
	vec3 positional_impulse = gm_vec3_scalar_product(delta_lambda, n);

	// updates the position of the entities based on eq (6) and (7)
	if (!e1->fixed) {
		e1->world_position = gm_vec3_add(e1->world_position, gm_vec3_scalar_product(e1->inverse_mass, positional_impulse));
	}
	if (!e2->fixed) {
		e2->world_position = gm_vec3_add(e2->world_position, gm_vec3_scalar_product(-e2->inverse_mass, positional_impulse));
	}

	// updates the rotation of the entities based on eq (8) and (9)
	vec3 aux1 = gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, positional_impulse));
	vec3 aux2 = gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, positional_impulse));
#ifdef USE_QUATERNIONS_LINEARIZED_FORMULAS
	Quaternion aux_q1 = (Quaternion){aux1.x, aux1.y, aux1.z, 0.0};
	Quaternion aux_q2 = (Quaternion){aux2.x, aux2.y, aux2.z, 0.0};
	Quaternion q1 = quaternion_product(&aux_q1, &e1->world_rotation);
	Quaternion q2 = quaternion_product(&aux_q2, &e2->world_rotation);
	if (!e1->fixed) {
		e1->world_rotation.x = e1->world_rotation.x + 0.5 * q1.x;
		e1->world_rotation.y = e1->world_rotation.y + 0.5 * q1.y;
		e1->world_rotation.z = e1->world_rotation.z + 0.5 * q1.z;
		e1->world_rotation.w = e1->world_rotation.w + 0.5 * q1.w;
		// should we normalize?
		e1->world_rotation = quaternion_normalize(&e1->world_rotation);
	}
	if (!e2->fixed) {
		e2->world_rotation.x = e2->world_rotation.x - 0.5 * q2.x;
		e2->world_rotation.y = e2->world_rotation.y - 0.5 * q2.y;
		e2->world_rotation.z = e2->world_rotation.z - 0.5 * q2.z;
		e2->world_rotation.w = e2->world_rotation.w - 0.5 * q2.w;
		// should we normalize?
		e2->world_rotation = quaternion_normalize(&e2->world_rotation);
	}
#else
	if (!e1->fixed) {
		r64 e1_rotation_angle = gm_vec3_length(aux1);
		vec3 e1_rotation_axis = gm_vec3_normalize(aux1);
		Quaternion e1_orientation_change = quaternion_new_radians(e1_rotation_axis, e1_rotation_angle);
		e1->world_rotation = quaternion_product(&e1_orientation_change, &e1->world_rotation);
		// should we normalize?
		e1->world_rotation = quaternion_normalize(&e1->world_rotation);
	}

	if (!e2->fixed) {
		r64 e2_rotation_angle = -gm_vec3_length(aux2);
		vec3 e2_rotation_axis = gm_vec3_normalize(aux2);
		Quaternion e2_orientation_change = quaternion_new_radians(e2_rotation_axis, e2_rotation_angle);
		e2->world_rotation = quaternion_product(&e2_orientation_change, &e2->world_rotation);
		// should we normalize?
		e2->world_rotation = quaternion_normalize(&e2->world_rotation);
	}
#endif

	return delta_lambda;
}

static void solve_positional_constraint(Constraint* constraint, r64 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);
	r64 delta_lambda = apply_positional_constraint(constraint, h);
	constraint->positional_constraint.lambda += delta_lambda;
}

// Solves the collision constraint, updating the position and orientation of the entities accordingly
static void solve_collision_constraint(Constraint* constraint, r64 h) {
	assert(constraint->type == COLLISION_CONSTRAINT);

	Entity* e1 = constraint->collision_constraint.e1;
	Entity* e2 = constraint->collision_constraint.e2;
	vec3 r1_lc = constraint->collision_constraint.r1_lc;
	vec3 r2_lc = constraint->collision_constraint.r2_lc;
	vec3 normal = constraint->collision_constraint.normal;
	r64 lambda_n = constraint->collision_constraint.lambda_n;
	r64 lambda_t = constraint->collision_constraint.lambda_t;

	mat3 e1_rot_matrix = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot_matrix = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1_wc = gm_mat3_multiply_vec3(&e1_rot_matrix, r1_lc);
	vec3 r2_wc = gm_mat3_multiply_vec3(&e2_rot_matrix, r2_lc);

	// here we calculate 'p1' and 'p2' in order to calculate 'd', as stated in sec (3.5)
	vec3 p1 = gm_vec3_add(e1->world_position, r1_wc);
	vec3 p2 = gm_vec3_add(e2->world_position, r2_wc);
	r64 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), normal);
	vec3 delta_x = gm_vec3_scalar_product(d, normal);

	if (d > 0.0) {
		Constraint c;
		c.type = POSITIONAL_CONSTRAINT;
		c.positional_constraint.compliance = 0.0;
		c.positional_constraint.delta_x = delta_x;
		c.positional_constraint.e1 = e1;
		c.positional_constraint.e2 = e2;
		c.positional_constraint.lambda = lambda_n;
		c.positional_constraint.r1_lc = r1_lc;
		c.positional_constraint.r2_lc = r2_lc;
		r64 delta_lambda = apply_positional_constraint(&c, h);

		lambda_n += delta_lambda;
		constraint->collision_constraint.lambda_n = lambda_n;

		// Recalculate p1, p2, r1_wc and r2_wc
		e1_rot_matrix = quaternion_get_matrix3(&e1->world_rotation);
		e2_rot_matrix = quaternion_get_matrix3(&e2->world_rotation);
		r1_wc = gm_mat3_multiply_vec3(&e1_rot_matrix, r1_lc);
		r2_wc = gm_mat3_multiply_vec3(&e2_rot_matrix, r2_lc);

		p1 = gm_vec3_add(e1->world_position, r1_wc);
		p2 = gm_vec3_add(e2->world_position, r2_wc);

		// if 'd' is greater than 0.0, we should also add a constraint for static friction, but only if lambda_t < u_s * lambda_n
		//const r64 static_friction_coefficient = (e1->static_friction_coefficient + e2->static_friction_coefficient) / 2.0f;
		const r64 static_friction_coefficient = 0.0001;

		// @NOTE(fek): This inequation shown in 3.5 was changed because the lambdas will always be negative!
		if (static_friction_coefficient > 0.0 && lambda_t > static_friction_coefficient * lambda_n) {
			mat3 e1_previous_rot_matrix = quaternion_get_matrix3(&e1->previous_world_rotation);
			vec3 p1_til = gm_vec3_add(e1->previous_world_position, gm_mat3_multiply_vec3(&e1_previous_rot_matrix, r1_lc));
			mat3 e2_previous_rot_matrix = quaternion_get_matrix3(&e2->previous_world_rotation);
			vec3 p2_til = gm_vec3_add(e2->previous_world_position, gm_mat3_multiply_vec3(&e2_previous_rot_matrix, r2_lc));
			vec3 delta_p = gm_vec3_subtract(gm_vec3_subtract(p1, p1_til), gm_vec3_subtract(p2, p2_til));
			vec3 delta_p_t = gm_vec3_subtract(delta_p, gm_vec3_scalar_product(gm_vec3_dot(delta_p, normal), normal));

			Constraint c;
			c.type = POSITIONAL_CONSTRAINT;
			c.positional_constraint.compliance = 0.0;
			c.positional_constraint.delta_x = delta_p_t;
			c.positional_constraint.e1 = e1;
			c.positional_constraint.e2 = e2;
			c.positional_constraint.lambda = lambda_t;
			c.positional_constraint.r1_lc = r1_lc;
			c.positional_constraint.r2_lc = r2_lc;
			r64 delta_lambda = apply_positional_constraint(&c, h);

			lambda_t += delta_lambda;
			constraint->collision_constraint.lambda_t = lambda_t;
		}
	}
}

static void solve_constraint(Constraint* constraint, r64 h) {
	switch (constraint->type) {
		case POSITIONAL_CONSTRAINT: {
			solve_positional_constraint(constraint, h);
			return;
		} break;
		case COLLISION_CONSTRAINT: {
			solve_collision_constraint(constraint, h);
			return;
		} break;
	}

	assert(0);
}

void clipping_contact_to_constraint(Entity* e1, Entity* e2, vec3 normal, Clipping_Contact* contact, Constraint* constraint) {
	constraint->type = COLLISION_CONSTRAINT;
	constraint->collision_constraint.e1 = e1;
	constraint->collision_constraint.e2 = e2;
	constraint->collision_constraint.normal = normal;
	constraint->collision_constraint.lambda_n = 0.0;
	constraint->collision_constraint.lambda_t = 0.0;

	vec3 r1_wc = gm_vec3_subtract(contact->collision_point1, e1->world_position);
	vec3 r2_wc = gm_vec3_subtract(contact->collision_point2, e2->world_position);

	Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
	mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
	constraint->collision_constraint.r1_lc = gm_mat3_multiply_vec3(&q1_mat, r1_wc);

	Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
	mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
	constraint->collision_constraint.r2_lc = gm_mat3_multiply_vec3(&q2_mat, r2_wc);
}

void pbd_simulate(r64 dt, Entity* entities) {
	if (dt <= 0.0) return;
	r64 h = dt / NUM_SUBSTEPS;
	//r64 h = 0.01;

	// The main loop of the PBD simulation
	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
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
			mat3 e_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e);
			mat3 e_inertia_tensor = get_dynamic_inertia_tensor(e);
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(external_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e_inertia_tensor, e->angular_velocity))))));
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
		Constraint* constraints = array_new(Constraint);

		Broad_Collision_Pair* broad_collision_pairs = broad_get_collision_pairs(entities);

#ifdef ENABLE_SIMULATION_ISLANDS
		Entity*** simulation_islands = broad_collect_simulation_islands(entities, broad_collision_pairs);

		// All entities will be contained in the simulation islands.
		// Update deactivation time and also, at the same time, its active status
		for (u32 j = 0; j < array_length(simulation_islands); ++j) {
			Entity** simulation_island = simulation_islands[j];

			boolean all_inactive = true;
			for (u32 k = 0; k < array_length(simulation_island); ++k) {
				Entity* e = simulation_island[k];

				r64 linear_velocity_len = gm_vec3_length(e->linear_velocity);
				r64 angular_velocity_len = gm_vec3_length(e->angular_velocity);
				if (linear_velocity_len < LINEAR_SLEEPING_THRESHOLD && angular_velocity_len < ANGULAR_SLEEPING_THRESHOLD) {
					e->deactivationTime += h; // we should use 'dt' if doing once per frame
				} else {
					e->deactivationTime = 0.0;
				}

				if (e->deactivationTime < DEACTIVATION_TIME_TO_BE_INACTIVE) {
					all_inactive = false;
				}
			}

			// We only set entities to inactive if the whole island is inactive!
			for (u32 k = 0; k < array_length(simulation_island); ++k) {
				Entity* e = simulation_island[k];
				e->active = !all_inactive;
			}
		}
#if 0
		for (u32 j = 0; j < array_length(simulation_islands); ++j) {
			Entity** simulation_island = simulation_islands[j];
			vec4 color = util_pallete(j);
			for (u32 k = 0; k < array_length(simulation_island); ++k) {
				Entity* e = simulation_island[k];
				e->color = color;
			}
		}
#else
		for (u32 j = 0; j < array_length(simulation_islands); ++j) {
			Entity** simulation_island = simulation_islands[j];
			for (u32 k = 0; k < array_length(simulation_island); ++k) {
				Entity* e = simulation_island[k];
				if (e->active) {
					e->color = util_pallete(1);
				} else {
					e->color = util_pallete(0);
				}
			}
		}
#endif

		broad_simulation_islands_destroy(simulation_islands);
#endif
		// As explained in sec 3.5, in each substep we need to check for collisions
		// (I am not pre-collecting potential collision pairs.)
		// Here we just check the plane-cube collision and collect the intersections.
		//for (u32 j = 0; j < array_length(entities); ++j) {
		//	for (u32 k = j + 1; k < array_length(entities); ++k) {
		for (u32 j = 0; j < array_length(broad_collision_pairs); ++j) {
			Entity* e1 = broad_collision_pairs[j].e1;
			Entity* e2 = broad_collision_pairs[j].e2;

			// If e1 is "colliding" with e2, they must be either both active or both inactive
			if (!e1->fixed && !e2->fixed) {
				assert((e1->active && e2->active) || (!e1->active && !e2->active));
			}

			// No need to solve the collision if both entities are either inactive or fixed
			if ((e1->fixed || !e1->active) && (e2->fixed || !e2->active)) {
				continue;
			}

			mat4 e1_model_matrix = entity_get_model_matrix_no_scale(e1);
			mat4 e2_model_matrix = entity_get_model_matrix_no_scale(e2);
			collider_update(&e1->collider, e1_model_matrix);
			collider_update(&e2->collider, e2_model_matrix);

			Collider_Convex_Hull* convex_hull1 = &e1->collider.convex_hull;
			Collider_Convex_Hull* convex_hull2 = &e2->collider.convex_hull;
			GJK_Simplex simplex;
			if (gjk_collides(convex_hull1->transformed_vertices, convex_hull2->transformed_vertices, &simplex)) {
				vec3 normal;
				r64 penetration;
				if (epa(convex_hull1->transformed_vertices, convex_hull2->transformed_vertices, &simplex, &normal, &penetration)) {
					Clipping_Contact* contacts = clipping_get_contact_manifold(convex_hull1, convex_hull2, normal);
					for (u32 l = 0; l < array_length(contacts); ++l) {
						Clipping_Contact* contact = &contacts[l];
						Constraint constraint;
						clipping_contact_to_constraint(e1, e2, normal, contact, &constraint);
						array_push(constraints, constraint);
					}
					array_free(contacts);
				}
			}
		}

		array_free(broad_collision_pairs);

#if 0
		int size = array_length(constraints);
		u32* idxs = array_new(u32);
		for (u32 i = 0; i < size; ++i) {
			int idx;
			boolean new;
			do {
				new = true;
				idx = rand() % size;
				for (u32 j = 0; j < array_length(idxs); ++j) {
					if (idx == idxs[j]) {
						new = false;
						break;
					}
				}
			} while (!new);
			array_push(idxs, idx);
		}

		Constraint* abc = array_new(Constraint);
		for (u32 i = 0; i < array_length(idxs); ++i) {
			array_push(abc, constraints[idxs[i]]);
		}
		constraints = abc;
#endif

		// Now we run the PBD solver with NUM_POS_ITERS iterations
		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {
			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				solve_constraint(constraint, h);
				if (paused) {
					return;
				}
			}	
		}

		// The PBD velocity update
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
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

			e->updated_linear_velocity = (vec3){0.0, 0.0, 0.0};
			e->updated_angular_velocity = (vec3){0.0, 0.0, 0.0};
		}

		// The velocity solver - we run this additional solver for every collision that we found
		for (u32 j = 0; j < array_length(constraints); ++j) {
			Constraint* constraint = &constraints[j];
			if (constraint->type != COLLISION_CONSTRAINT) {
				continue;
			}

			Entity* e1 = constraint->collision_constraint.e1;
			Entity* e2 = constraint->collision_constraint.e2;
			vec3 r1_lc = constraint->collision_constraint.r1_lc;
			vec3 r2_lc = constraint->collision_constraint.r2_lc;
			vec3 n = constraint->collision_constraint.normal;
			r64 lambda_n = constraint->collision_constraint.lambda_n;
			r64 lambda_t = constraint->collision_constraint.lambda_t;

			vec3 v1 = e1->linear_velocity;
			vec3 w1 = e1->angular_velocity;
			vec3 v2 = e2->linear_velocity;
			vec3 w2 = e2->angular_velocity;

			mat3 q1_mat = quaternion_get_matrix3(&e1->world_rotation);
			vec3 r1_wc = gm_mat3_multiply_vec3(&q1_mat, r1_lc);
			mat3 q2_mat = quaternion_get_matrix3(&e2->world_rotation);
			vec3 r2_wc = gm_mat3_multiply_vec3(&q2_mat, r2_lc);

			// We start by calculating the relative normal and tangential velocities at the contact point, as described in (3.6)
			// @NOTE: equation (29) was modified here
			vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, r1_wc)), gm_vec3_add(v2, gm_vec3_cross(w2, r2_wc)));
			r64 vn = gm_vec3_dot(n, v);
			vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));

			// delta_v stores the velocity change that we need to perform at the end of the solver
			vec3 delta_v = (vec3){0.0, 0.0, 0.0};
			
			// we start by applying Coloumb's dynamic friction force
			const r64 dynamic_friction_coefficient = (e1->dynamic_friction_coefficient + e2->dynamic_friction_coefficient) / 2.0f;
			r64 fn = lambda_n / h; // simplifly h^2 by ommiting h in the next calculation
			// @NOTE: equation (30) was modified here
			r64 fact = MIN(dynamic_friction_coefficient * fabsf(fn), gm_vec3_length(vt));
			// update delta_v
			delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(-fact, gm_vec3_normalize(vt)));

			// Now we handle restitution
			vec3 old_v1 = e1->previous_linear_velocity;
			vec3 old_w1 = e1->previous_angular_velocity;
			vec3 old_v2 = e2->previous_linear_velocity;
			vec3 old_w2 = e2->previous_angular_velocity;
			vec3 v_til = gm_vec3_subtract(gm_vec3_add(old_v1, gm_vec3_cross(old_w1, r1_wc)), gm_vec3_add(old_v2, gm_vec3_cross(old_w2, r2_wc)));
			r64 vn_til = gm_vec3_dot(n, v_til);
			//r64 e = (fabsf(vn) > 2.0 * GRAVITY * h) ? 0.8 : 0.0;
			r64 e = e1->restitution_coefficient * e2->restitution_coefficient;
			// @NOTE: equation (34) was modified here
			fact = -vn + MIN(-e * vn_til, 0.0);
			// update delta_v
			delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(fact, n));

			// Finally, we end the solver by applying delta_v, considering the inverse masses of both entities
			mat3 e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
			mat3 e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);
			r64 _w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1_wc, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, n)));
			r64 _w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2_wc, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, n)));
			vec3 p = gm_vec3_scalar_product(1.0 / (_w1 + _w2), delta_v);

			if (!e1->fixed) {
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(e1->inverse_mass, p));
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, p)));
			}
			if (!e2->fixed) {
				e2->linear_velocity = gm_vec3_add(e2->linear_velocity, gm_vec3_negative(gm_vec3_scalar_product(e2->inverse_mass, p)));
				e2->angular_velocity = gm_vec3_add(e2->angular_velocity, gm_vec3_negative(gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, p))));
			}
		}

		array_free(constraints);
	}
}