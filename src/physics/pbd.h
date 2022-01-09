#ifndef RAW_PHYSICS_PHYSICS_PBD_H
#define RAW_PHYSICS_PHYSICS_PBD_H
#include "../render/graphics.h"

typedef enum {
	POSITIONAL_STATIC_CONSTRAINT,
} Static_Constraint_Type;

typedef struct {
	eid e1_id;
	eid e2_id;
	vec3 r1_lc;
	vec3 r2_lc;
	r64 compliance;
	vec3 distance;
} Static_Positional_Constraint;

typedef struct {
	Static_Constraint_Type type;

	union {
		Static_Positional_Constraint positional_constraint;
	};
} Static_Constraint;

void pbd_simulate(r64 dt, Entity** entities);
void pbd_simulate_with_static_constraints(r64 dt, Entity** entities, Static_Constraint* constraints);

#endif