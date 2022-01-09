#ifndef RAW_PHYSICS_PHYSICS_BROAD_H
#define RAW_PHYSICS_PHYSICS_BROAD_H
#include "../render/graphics.h"

typedef struct {
	u32 e1_idx;
	u32 e2_idx;
} Broad_Collision_Pair;

Broad_Collision_Pair* broad_get_collision_pairs(Entity** entities);
u32** broad_collect_simulation_islands(Entity** entities, Broad_Collision_Pair* _collision_pairs);
void broad_simulation_islands_destroy(u32** simulation_islands);

#endif