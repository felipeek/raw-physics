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
	vec3 r1_wc;
	vec3 r2_wc;
	r32 compliance;
	vec3 delta_x;
	r32* lambda;
} Positional_Constaint;

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 normal;
	r32* lambda_t;
	r32* lambda_n;
} Collision_Constraint;

typedef struct {
	Entity* e1;
	Entity* e2;
	r32 lambda_n;
	r32 lambda_t;
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 r1_wc;
	vec3 r2_wc;
	vec3 normal;
} Temporary_Contact;

typedef struct {
	Constraint_Type type;

	union {
		Positional_Constaint positional_constraint;
		Collision_Constraint collision_constraint;
	};
} Constraint;

void pbd_simulate(r32 dt, Entity* entities);
void collect_collisions(Entity* entities);

#endif