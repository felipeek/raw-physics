#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include <float.h>
#include "gjk.h"
#include "epa.h"
#include "clipping.h"

#define NUM_SUBSTEPS 5
#define NUM_POS_ITERS 1

// Calculate the sum of all external forces acting on an entity
static vec3 calculate_external_force(Entity* e) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        total_force = gm_vec3_add(total_force, e->forces[i].force);
    }
	return total_force;
}

// Calculate the sum of all external torques acting on an entity
static vec3 calculate_external_torque(Entity* e) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, e->forces[i].force));
    }
	return total_torque;
}

// Calculate the dynamic inertia tensor of an entity, i.e., the inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inertia_tensor(Entity* e) {
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

// Calculate the dynamic inverse inertia tensor of an entity, i.e., the inverse inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inverse_inertia_tensor(Entity* e) {
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inverse_inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

static vec3 calculate_p_til(Entity* e, vec3 r_lc) {
	mat3 previous_rot_matrix = quaternion_get_matrix3(&e->previous_world_rotation);
	return gm_vec3_add(e->previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, r_lc));
}

static vec3 calculate_p(Entity* e, vec3 r_lc) {
	mat3 current_rot_matrix = quaternion_get_matrix3(&e->world_rotation);
	return gm_vec3_add(e->world_position, gm_mat3_multiply_vec3(&current_rot_matrix, r_lc));
}

// Solves the positional constraint, updating the position and orientation of the entities accordingly
static void solve_positional_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Entity* e1 = constraint->positional_constraint.e1;
	Entity* e2 = constraint->positional_constraint.e2;
	mat3 e1_rot = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1 = constraint->positional_constraint.r1_wc;	// in world coordinates
	vec3 r2 = constraint->positional_constraint.r2_wc; // in world coordinates
	r32 lambda = *constraint->positional_constraint.lambda;
	r32 compliance = constraint->positional_constraint.compliance;
	vec3 delta_x = constraint->positional_constraint.delta_x;

	mat3 e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
	mat3 e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);

	// split delta_x into n and c.
	vec3 n = gm_vec3_normalize(delta_x);
	r32 c = gm_vec3_length(delta_x);

	// calculate the inverse masses of both entities
	r32 w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, n)));
	r32 w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, n)));

	// calculate the delta_lambda (XPBD) and updates the constraint
	r32 til_compliance = compliance / (h * h);
	r32 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);
	*constraint->positional_constraint.lambda += delta_lambda;

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
	vec3 aux1 = gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, positional_impulse));
	vec3 aux2 = gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, positional_impulse));
	Quaternion aux_q1 = (Quaternion){aux1.x, aux1.y, aux1.z, 0.0f};
	Quaternion aux_q2 = (Quaternion){aux2.x, aux2.y, aux2.z, 0.0f};
	Quaternion q1 = quaternion_product(&aux_q1, &e1->world_rotation);
	Quaternion q2 = quaternion_product(&aux_q2, &e2->world_rotation);
	if (!e1->fixed) {
		e1->world_rotation.x = e1->world_rotation.x + 0.5f * q1.x;
		e1->world_rotation.y = e1->world_rotation.y + 0.5f * q1.y;
		e1->world_rotation.z = e1->world_rotation.z + 0.5f * q1.z;
		e1->world_rotation.w = e1->world_rotation.w + 0.5f * q1.w;
		// should we normalize?
		e1->world_rotation = quaternion_normalize(&e1->world_rotation);
	}
	if (!e2->fixed) {
		e2->world_rotation.x = e2->world_rotation.x - 0.5f * q2.x;
		e2->world_rotation.y = e2->world_rotation.y - 0.5f * q2.y;
		e2->world_rotation.z = e2->world_rotation.z - 0.5f * q2.z;
		e2->world_rotation.w = e2->world_rotation.w - 0.5f * q2.w;
		// should we normalize?
		e2->world_rotation = quaternion_normalize(&e2->world_rotation);
	}
}

extern boolean paused;

// Solves the collision constraint, updating the position and orientation of the entities accordingly
static void solve_collision_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == COLLISION_CONSTRAINT);

	Entity* e1 = constraint->collision_constraint.e1;
	Entity* e2 = constraint->collision_constraint.e2;
	vec3 r1_lc = constraint->collision_constraint.r1_lc;
	vec3 r2_lc = constraint->collision_constraint.r2_lc;
	vec3 normal = constraint->collision_constraint.normal;
	r32* lambda_n = constraint->collision_constraint.lambda_n;
	r32* lambda_t = constraint->collision_constraint.lambda_t;

	mat3 e1_rot_matrix = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot_matrix = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1_wc = gm_mat3_multiply_vec3(&e1_rot_matrix, r1_lc);
	vec3 r2_wc = gm_mat3_multiply_vec3(&e2_rot_matrix, r2_lc);

	// here we calculate 'p1' and 'p2' in order to calculate 'd', as stated in sec (3.5)
	vec3 p1 = calculate_p(e1, r1_lc);
	vec3 p2 = calculate_p(e2, r2_lc);
	r32 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), normal);
	if (fabsf(d) > 0.1f) {
		printf("d: %f\n", d);
		paused = true;
		return;
	}
	vec3 delta_x = gm_vec3_scalar_product(d, normal);

	if (d > 0.0f) {
		Constraint c;
		c.type = POSITIONAL_CONSTRAINT;
		c.positional_constraint.compliance = 0.0f;
		c.positional_constraint.delta_x = delta_x;
		c.positional_constraint.e1 = e1;
		c.positional_constraint.e2 = e2;
		c.positional_constraint.lambda = lambda_n;
		c.positional_constraint.r1_wc = r1_wc;
		c.positional_constraint.r2_wc = r2_wc;
		solve_positional_constraint(&c, h);

		// @TODO: check if we need to recalculate r1_wc and r2_wc...
		e1_rot_matrix = quaternion_get_matrix3(&e1->world_rotation);
		e2_rot_matrix = quaternion_get_matrix3(&e2->world_rotation);
		r1_wc = gm_mat3_multiply_vec3(&e1_rot_matrix, r1_lc);
		r2_wc = gm_mat3_multiply_vec3(&e2_rot_matrix, r2_lc);

		// if 'd' is greater than 0.0, we should also add a constraint for static friction, but only if lambda_t < u_s * lambda_n
		// This is only known once the normal constraint is solved, so we delegate this check to the solver.
		const r32 static_friction_coefficient = 0.0f;
		vec3 p1_til = calculate_p_til(e1, r1_lc);
		vec3 p2_til = calculate_p_til(e2, r2_lc);
		vec3 delta_p = gm_vec3_subtract(gm_vec3_subtract(p1, p1_til), gm_vec3_subtract(p2, p2_til));
		vec3 delta_p_t = gm_vec3_subtract(delta_p, gm_vec3_scalar_product(gm_vec3_dot(delta_p, normal), normal));

		// @NOTE(fek): This inequation shown in 3.5 was changed because the lambdas will always be negative!
		if (*lambda_t > static_friction_coefficient * *lambda_n) {
			//printf("Applying static friction\n");
			Constraint c = {0};
			c.type = POSITIONAL_CONSTRAINT;
			c.positional_constraint.compliance = 0.0f;
			c.positional_constraint.delta_x = delta_p_t;
			c.positional_constraint.e1 = e1;
			c.positional_constraint.e2 = e2;
			c.positional_constraint.lambda = lambda_t;
			c.positional_constraint.r1_wc = r1_wc;
			c.positional_constraint.r2_wc = r2_wc;
			solve_positional_constraint(&c, h);
		}
	}
}

static void solve_constraint(Constraint* constraint, r32 h) {
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

// Create 'artificial' position constraints for a given collision that was detected in the simulation
static void create_constraints_for_contacts(Temporary_Contact* tmp_contacts, Constraint** constraints) {
	for (u32 i = 0; i < array_length(tmp_contacts); ++i) {
		Temporary_Contact* contact = &tmp_contacts[i];

		Constraint constraint = {0};
		constraint.type = COLLISION_CONSTRAINT;
		constraint.collision_constraint.e1 = contact->e1;
		constraint.collision_constraint.e2 = contact->e2;
		// we make lambda a reference to the value stored in the collision_info, so we can accumulate per iteration
		constraint.collision_constraint.lambda_n = &contact->lambda_n;
		constraint.collision_constraint.lambda_t = &contact->lambda_t;
		constraint.collision_constraint.r1_lc = contact->r1_lc;
		constraint.collision_constraint.r2_lc = contact->r2_lc;
		constraint.collision_constraint.normal = contact->normal;

		array_push(*constraints, constraint);
	}
}

Temporary_Contact* tmp_contacts;

void pbd_simulate(r32 dt, Entity* entities) {
	if (dt <= 0.0f) return;
	r32 h = dt / NUM_SUBSTEPS;
	//r32 h = 0.01f;

	// The main loop of the PBD simulation
	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			if (e->fixed) continue;	

			// Calculate the external force and torque of the entity
			vec3 external_force = calculate_external_force(e);
			vec3 external_torque = calculate_external_torque(e);

			// Stores the previous position and orientation of the entity
			e->previous_world_position = e->world_position;
			e->previous_world_rotation = e->world_rotation;

			// Update the entity position and linear velocity based on the current velocity and applied forces
			e->linear_velocity = gm_vec3_add(e->linear_velocity, gm_vec3_scalar_product(h * e->inverse_mass, external_force));
			e->world_position = gm_vec3_add(e->world_position, gm_vec3_scalar_product(h, e->linear_velocity));

			// Update the entity orientation and angular velocity based on the current velocity and applied forces
			mat3 e_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e);
			mat3 e_inertia_tensor = get_dynamic_inertia_tensor(e);
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(external_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e_inertia_tensor, e->angular_velocity))))));
			Quaternion aux = (Quaternion){e->angular_velocity.x, e->angular_velocity.y, e->angular_velocity.z, 0.0f};
			Quaternion q = quaternion_product(&aux, &e->world_rotation);
			e->world_rotation.x = e->world_rotation.x + h * 0.5f * q.x;
			e->world_rotation.y = e->world_rotation.y + h * 0.5f * q.y;
			e->world_rotation.z = e->world_rotation.z + h * 0.5f * q.z;
			e->world_rotation.w = e->world_rotation.w + h * 0.5f * q.w;
			// should we normalize?
			e->world_rotation = quaternion_normalize(&e->world_rotation);
		}

		if (!tmp_contacts) {
			tmp_contacts = array_new(Temporary_Contact);
		} else {
			array_clear(tmp_contacts);
		}

		// As explained in sec 3.5, in each substep we need to check for collisions
		// (I am not pre-collecting potential collision pairs.)
		// Here we just check the plane-cube collision and collect the intersections.
		for (u32 j = 0; j < array_length(entities); ++j) {
			for (u32 k = j + 1; k < array_length(entities); ++k) {
				Entity* e1 = &entities[j];
				Entity* e2 = &entities[k];

				mat4 e1_model_matrix = graphics_entity_get_model_matrix(e1);
				mat4 e2_model_matrix = graphics_entity_get_model_matrix(e2);
				collider_update(&e1->mesh.collider, e1_model_matrix);
				collider_update(&e2->mesh.collider, e2_model_matrix);

				Collider_Convex_Hull* convex_hull1 = &e1->mesh.collider.convex_hull;
				Collider_Convex_Hull* convex_hull2 = &e2->mesh.collider.convex_hull;
				GJK_Simplex simplex;
				if (gjk_collides(convex_hull1->transformed_vertices, convex_hull2->transformed_vertices, &simplex)) {
					vec3 normal;
					r32 penetration;
					if (epa(convex_hull1->transformed_vertices, convex_hull2->transformed_vertices, &simplex, &normal, &penetration)) {
						//if (penetration > 10) {
						//	printf("e1: <%f, %f, %f>\n", e1->world_position.x, e1->world_position.y, e1->world_position.z);
						//	printf("e2: <%f, %f, %f>\n", e2->world_position.x, e2->world_position.y, e2->world_position.z);
						//	printf("e2 rot: <%f, %f, %f, %f>\n", e2->world_rotation.x, e2->world_rotation.y, e2->world_rotation.z, e2->world_rotation.w);
						//	printf("e1 rot: <%f, %f, %f, %f>\n", e1->world_rotation.x, e1->world_rotation.y, e1->world_rotation.z, e1->world_rotation.w);
						//}

						Clipping_Contact* contacts = clipping_get_contact_manifold(convex_hull1, convex_hull2, normal);
						for (u32 l = 0; l < array_length(contacts); ++l) {
							Clipping_Contact* contact = &contacts[l];
							Temporary_Contact tmp;
							tmp.e1 = e1;
							tmp.e2 = e2;
							tmp.normal = normal;
							tmp.lambda_n = 0.0f;
							tmp.lambda_t = 0.0f;
							tmp.r1_wc = gm_vec3_subtract(contact->collision_point1, e1->world_position);
							tmp.r2_wc = gm_vec3_subtract(contact->collision_point2, e2->world_position);
							Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
							mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
							tmp.r1_lc = gm_mat3_multiply_vec3(&q1_mat, tmp.r1_wc);

							Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
							mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
							tmp.r2_lc = gm_mat3_multiply_vec3(&q2_mat, tmp.r2_wc);
							array_push(tmp_contacts, tmp);
						}
					}
				}
			}
		}

		// Now we run the PBD solver with NUM_POS_ITERS iterations
		Constraint* constraints = array_new(Constraint);

		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {
			create_constraints_for_contacts(tmp_contacts, &constraints);

			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				solve_constraint(constraint, h);
				if (paused) {
					return;
				}
			}	

			// Here we should only clear the constraints generated because of collisions, but for now all of them fall into this category
			array_clear(constraints);
		}

		// The PBD velocity update
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			if (e->fixed) continue;
			
			// We start by storing the current velocities (this is needed for the velocity solver that comes at the end of the loop)
			e->previous_linear_velocity = e->linear_velocity;
			e->previous_angular_velocity = e->angular_velocity;

			// Update the linear velocity based on the position difference
			e->linear_velocity = gm_vec3_scalar_product(1.0f / h, gm_vec3_subtract(e->world_position, e->previous_world_position));

			// Update the angular velocity based on the orientation difference
			Quaternion inv = quaternion_inverse(&e->previous_world_rotation);
			Quaternion delta_q = quaternion_product(&e->world_rotation, &inv);
			if (delta_q.w >= 0.0f) {
				e->angular_velocity = gm_vec3_scalar_product(2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			} else {
				e->angular_velocity = gm_vec3_scalar_product(-2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			}
		}

		// The velocity solver - we run this additional solver for every collision that we found
		for (u32 j = 0; j < array_length(tmp_contacts); ++j) {
			Temporary_Contact* contact = &tmp_contacts[j];
			vec3 v1 = contact->e1->linear_velocity;
			vec3 w1 = contact->e1->angular_velocity;
			vec3 v2 = contact->e2->linear_velocity;
			vec3 w2 = contact->e2->angular_velocity;
			vec3 n = contact->normal;
			r32 lambda_n = contact->lambda_n;
			r32 lambda_t = contact->lambda_t;
			Entity* e1 = contact->e1;
			Entity* e2 = contact->e2;
			#if 0
			vec3 r1_wc = contact->r1_wc;
			vec3 r2_wc = contact->r2_wc;
			#else
			mat3 q1_mat = quaternion_get_matrix3(&e1->world_rotation);
			vec3 r1_wc = gm_mat3_multiply_vec3(&q1_mat, contact->r1_lc);
			mat3 q2_mat = quaternion_get_matrix3(&e2->world_rotation);
			vec3 r2_wc = gm_mat3_multiply_vec3(&q2_mat, contact->r2_lc);
			#endif

			// We start by calculating the relative normal and tangential velocities at the contact point, as described in (3.6)
			// @NOTE: equation (29) was modified here
			vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, r1_wc)), gm_vec3_add(v2, gm_vec3_cross(w2, r2_wc)));
			r32 vn = gm_vec3_dot(n, v);
			vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));
			printf("VT: %f\n", gm_vec3_length(vt));

			// delta_v stores the velocity change that we need to perform at the end of the solver
			vec3 delta_v = (vec3){0.0f, 0.0f, 0.0f};
			
			// we start by applying Coloumb's dynamic friction force
			const r32 dynamic_friction_coefficient = 1.0f;
			r32 fn = lambda_n / h; // simplifly h^2 by ommiting h in the next calculation
			// @NOTE: equation (30) was modified here
			r32 fact = MIN(dynamic_friction_coefficient * fabsf(fn), gm_vec3_length(vt));
			// update delta_v
			delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(-fact, gm_vec3_normalize(vt)));

			// Now we handle restitution
			vec3 old_v1 = e1->previous_linear_velocity;
			vec3 old_w1 = e1->previous_angular_velocity;
			vec3 old_v2 = e2->previous_linear_velocity;
			vec3 old_w2 = e2->previous_angular_velocity;
			vec3 v_til = gm_vec3_subtract(gm_vec3_add(old_v1, gm_vec3_cross(old_w1, r1_wc)), gm_vec3_add(old_v2, gm_vec3_cross(old_w2, r2_wc)));
			r32 vn_til = gm_vec3_dot(n, v_til);
			//r32 e = (fabsf(vn) > 2.0f * GRAVITY * h) ? 0.8f : 0.0f;
			r32 e = 0.0f;
			// @NOTE: equation (34) was modified here
			fact = -vn + MIN(-e * vn_til, 0.0f);
			// update delta_v
			delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(fact, n));

			// Finally, we end the solver by applying delta_v, considering the inverse masses of both entities
			mat3 e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
			mat3 e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);
			r32 _w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1_wc, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, n)));
			r32 _w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2_wc, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, n)));
			vec3 p = gm_vec3_scalar_product(1.0f / (_w1 + _w2), delta_v);
			if (!e1->fixed) {
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(e1->inverse_mass, p));
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, p)));
			}
			if (!e2->fixed) {
				e2->linear_velocity = gm_vec3_subtract(e2->linear_velocity, gm_vec3_scalar_product(e2->inverse_mass, p));
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, p)));
			}
		}

		array_free(constraints);
		//array_free(collision_infos);
	}
}