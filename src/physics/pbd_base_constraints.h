#ifndef RAW_PHYSICS_PHYSICS_PBD_BASE_CONSTRAINTS_H
#define RAW_PHYSICS_PHYSICS_PBD_BASE_CONSTRAINTS_H
#include "../render/graphics.h"

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_wc;
	vec3 r2_wc;
	mat3 e1_inverse_inertia_tensor;
	mat3 e2_inverse_inertia_tensor;
} Position_Constraint_Preprocessed_Data;

typedef struct {
	Entity* e1;
	Entity* e2;
	mat3 e1_inverse_inertia_tensor;
	mat3 e2_inverse_inertia_tensor;
} Angular_Constraint_Preprocessed_Data;

// Positional Constraint
void calculate_positional_constraint_preprocessed_data(Entity* e1, Entity* e2, vec3 r1_lc, vec3 r2_lc, Position_Constraint_Preprocessed_Data* pcpd);
r64 positional_constraint_get_delta_lambda(Position_Constraint_Preprocessed_Data* pcpd, r64 h, r64 compliance, r64 lambda, vec3 delta_x);
void positional_constraint_apply(Position_Constraint_Preprocessed_Data* pcpd, r64 delta_lambda, vec3 delta_x);

// Angular Constraint
void calculate_angular_constraint_preprocessed_data(Entity* e1, Entity* e2, Angular_Constraint_Preprocessed_Data* acpd);
r64 angular_constraint_get_delta_lambda(Angular_Constraint_Preprocessed_Data* acpd, r64 h, r64 compliance, r64 lambda, vec3 delta_q);
void angular_constraint_apply(Angular_Constraint_Preprocessed_Data* acpd, r64 delta_lambda, vec3 delta_q);

#endif