#ifndef RAW_PHYSICS_PBD_H
#define RAW_PHYSICS_PBD_H
#include "../render/graphics.h"

typedef enum {
	POSITIONAL_CONSTRAINT,
	COLLISION_CONSTRAINT
} Constraint_Type;

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_lc;
	vec3 r2_lc;
	r64 compliance;
	vec3 delta_x;
	r64 lambda;
} Positional_Constaint;

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 normal;
	r64 lambda_t;
	r64 lambda_n;
} Collision_Constraint;

typedef struct {
	Constraint_Type type;

	union {
		Positional_Constaint positional_constraint;
		Collision_Constraint collision_constraint;
	};
} Constraint;

void pbd_simulate(r64 dt, Entity* entities);

#endif