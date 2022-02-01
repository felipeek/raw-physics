#ifndef RAW_PHYSICS_PHYSICS_BROAD_H
#define RAW_PHYSICS_PHYSICS_BROAD_H
#include "../render/graphics.h"
#include "pbd.h"

typedef struct {
	eid e1_id;
	eid e2_id;
} Broad_Collision_Pair;

Broad_Collision_Pair* broad_get_collision_pairs(Entity** entities);
eid** broad_collect_simulation_islands(Entity** entities, Broad_Collision_Pair* collision_pairs, const Constraint* constraints);
void broad_simulation_islands_destroy(eid** simulation_islands);

#endif