#include "broad.h"
#include <light_array.h>
#include <hash_map.h>

Broad_Collision_Pair* broad_get_collision_pairs(Entity* entities) {
	Broad_Collision_Pair pair;
	Broad_Collision_Pair* collision_pairs = array_new(Broad_Collision_Pair);

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e1 = &entities[i];
		for (u32 j = i + 1; j < array_length(entities); ++j) {
			Entity* e2 = &entities[j];

			r64 entities_distance = gm_vec3_length(gm_vec3_subtract(e1->world_position, e2->world_position));

			// Increase the distance a little to account for moving objects.
			// @TODO: We should derivate this value from delta_time, forces, velocities, etc
			r64 max_distance_for_collision = e1->collider.bounding_sphere_radius + e2->collider.bounding_sphere_radius + 0.1;
			if (entities_distance <= max_distance_for_collision) {
				pair.e1_idx = i;
				pair.e2_idx = j;
				array_push(collision_pairs, pair);
			}
		}
	}

	return collision_pairs;
}

static u32 uf_find(const u32* parents, u32 x) {
	u32 p = parents[x];
	if (p == x) {
		return x;
	}

	return uf_find(parents, p);
}

static void uf_union(u32* parents, u32 x, u32 y) {
	parents[uf_find(parents, y)] = parents[uf_find(parents, x)];
}

static u32* uf_collect_all(Entity* entities, Broad_Collision_Pair* collision_pairs) {
	u32* parents = array_new_len(u32, array_length(entities));
	for (u32 i = 0; i < array_length(entities); ++i) {
		array_push(parents, i);
	}

	for (u32 i = 0; i < array_length(collision_pairs); ++i) {
		Broad_Collision_Pair collision_pair = collision_pairs[i];
		Entity* e1 = &entities[collision_pair.e1_idx];
		Entity* e2 = &entities[collision_pair.e2_idx];
		if (!e1->fixed && !e2->fixed) {
			uf_union(parents, collision_pair.e1_idx, collision_pair.e2_idx);
		}
	}

	return parents;
}

static int u32_compare(const void *key1, const void *key2) {
	u32 n1 = *(u32*)key1;
	u32 n2 = *(u32*)key2;
	return n1 == n2;
}

static unsigned int u32_hash(const void *key) {
	u32 n = *(u32*)key;
	return n;
}

u32** broad_collect_simulation_islands(Entity* entities, Broad_Collision_Pair* collision_pairs) {
	Hash_Map simulation_islands_map;
	u32** simulation_islands = array_new(u32*);
	u32* parents = uf_collect_all(entities, collision_pairs);
	assert(!hash_map_create(&simulation_islands_map, 2 * array_length(entities), sizeof(u32), sizeof(u32), u32_compare, u32_hash));

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		if (e->fixed) {
			continue;
		}
		u32 parent = uf_find(parents, i);
		u32 simulation_island_idx;
		if (hash_map_get(&simulation_islands_map, &parent, &simulation_island_idx)) {
			// Simulation Island not created yet.
			u32* new_simulation_island = array_new(u32);
			simulation_island_idx = array_length(simulation_islands);
			array_push(simulation_islands, new_simulation_island);
			assert(!hash_map_put(&simulation_islands_map, &parent, &simulation_island_idx));
		}

		array_push(simulation_islands[simulation_island_idx], i);
	}

	hash_map_destroy(&simulation_islands_map);
	array_free(parents);
	return simulation_islands;
}

void broad_simulation_islands_destroy(u32** simulation_islands) {
	for (u32 i = 0; i < array_length(simulation_islands); ++i) {
		u32* simulation_island = simulation_islands[i];
		array_free(simulation_island);
	}

	array_free(simulation_islands);
}