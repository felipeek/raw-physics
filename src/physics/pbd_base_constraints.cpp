#include "pbd_base_constraints.h"
#include "physics_util.h"

#define USE_QUATERNIONS_LINEARIZED_FORMULAS

void calculate_positional_constraint_preprocessed_data(Entity* e1, Entity* e2, vec3 r1_lc, vec3 r2_lc, Position_Constraint_Preprocessed_Data* pcpd) {
	pcpd->e1 = e1;
	pcpd->e2 = e2;

	pcpd->r1_wc = quaternion_apply_to_vec3(&e1->world_rotation, r1_lc);
	pcpd->r2_wc = quaternion_apply_to_vec3(&e2->world_rotation, r2_lc);

	pcpd->e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
	pcpd->e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);
}

r64 positional_constraint_get_delta_lambda(Position_Constraint_Preprocessed_Data* pcpd, r64 h, r64 compliance, r64 lambda, vec3 delta_x) {
	r64 c = gm_vec3_length(delta_x);

	// We need to avoid calculations when delta_x is zero or very very close to zero, otherwise we will might run into
	// big problems because of floating-point precision
	const r64 EPSILON = 1e-50;
	if (c <= EPSILON) {
		return 0.0;
	}

	Entity* e1 = pcpd->e1;
	Entity* e2 = pcpd->e2;
	vec3 r1_wc = pcpd->r1_wc;
	vec3 r2_wc = pcpd->r2_wc;
	mat3 e1_inverse_inertia_tensor = pcpd->e1_inverse_inertia_tensor;
	mat3 e2_inverse_inertia_tensor = pcpd->e2_inverse_inertia_tensor;

	vec3 n = (vec3) {delta_x.x / c, delta_x.y / c, delta_x.z / c};

	// calculate the inverse masses of both entities
	r64 w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1_wc, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, n)));
	r64 w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2_wc, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, n)));

	assert(w1 + w2 != 0.0);

	// calculate the delta_lambda (XPBD) and updates the constraint
	r64 til_compliance = compliance / (h * h);
	r64 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);

	return delta_lambda;
}

// Apply the positional constraint, updating the position and orientation of the entities accordingly
void positional_constraint_apply(Position_Constraint_Preprocessed_Data* pcpd, r64 delta_lambda, vec3 delta_x) {
	r64 c = gm_vec3_length(delta_x);

	// We need to avoid calculations when delta_x is zero or very very close to zero, otherwise we will might run into
	// big problems because of floating-point precision
	const r64 EPSILON = 1e-50;
	if (c <= EPSILON) {
		return;
	}

	Entity* e1 = pcpd->e1;
	Entity* e2 = pcpd->e2;
	vec3 r1_wc = pcpd->r1_wc;
	vec3 r2_wc = pcpd->r2_wc;
	mat3 e1_inverse_inertia_tensor = pcpd->e1_inverse_inertia_tensor;
	mat3 e2_inverse_inertia_tensor = pcpd->e2_inverse_inertia_tensor;

	vec3 n = (vec3) {delta_x.x / c, delta_x.y / c, delta_x.z / c};

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
}

void calculate_angular_constraint_preprocessed_data(Entity* e1, Entity* e2, Angular_Constraint_Preprocessed_Data* acpd) {
	acpd->e1 = e1;
	acpd->e2 = e2;

	acpd->e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
	acpd->e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);
}

r64 angular_constraint_get_delta_lambda(Angular_Constraint_Preprocessed_Data* acpd, r64 h, r64 compliance, r64 lambda, vec3 delta_q) {
	r64 theta = gm_vec3_length(delta_q);

	// We need to avoid calculations when delta_q is zero or very very close to zero, otherwise we will might run into
	// big problems because of floating-point precision
	const r64 EPSILON = 1e-50;
	if (theta <= EPSILON) {
		return 0.0;
	}

	Entity* e1 = acpd->e1;
	Entity* e2 = acpd->e2;
	mat3 e1_inverse_inertia_tensor = acpd->e1_inverse_inertia_tensor;
	mat3 e2_inverse_inertia_tensor = acpd->e2_inverse_inertia_tensor;

	vec3 n = (vec3) {delta_q.x / theta, delta_q.y / theta, delta_q.z / theta};

	// calculate the inverse masses of both entities
	r64 w1 = gm_vec3_dot(n, gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, n));
	r64 w2 = gm_vec3_dot(n, gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, n));

	assert(w1 + w2 != 0.0);

	// calculate the delta_lambda (XPBD) and updates the constraint
	r64 til_compliance = compliance / (h * h);
	r64 delta_lambda = (- theta - til_compliance * lambda) / (w1 + w2 + til_compliance);

	return delta_lambda;
}

// Apply the angular constraint, updating the orientation of the entities accordingly
void angular_constraint_apply(Angular_Constraint_Preprocessed_Data* acpd, r64 delta_lambda, vec3 delta_q) {
	r64 theta = gm_vec3_length(delta_q);

	// We need to avoid calculations when delta_q is zero or very very close to zero, otherwise we will might run into
	// big problems because of floating-point precision
	const r64 EPSILON = 1e-50;
	if (theta <= EPSILON) {
		return;
	}

	Entity* e1 = acpd->e1;
	Entity* e2 = acpd->e2;
	mat3 e1_inverse_inertia_tensor = acpd->e1_inverse_inertia_tensor;
	mat3 e2_inverse_inertia_tensor = acpd->e2_inverse_inertia_tensor;

	vec3 n = (vec3) {delta_q.x / theta, delta_q.y / theta, delta_q.z / theta};

	// calculates the positional impulse
	vec3 positional_impulse = gm_vec3_scalar_product(-delta_lambda, n);

	// updates the rotation of the entities based on eq (8) and (9)
	vec3 aux1 = gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, positional_impulse);
	vec3 aux2 = gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, positional_impulse);
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
}