#include "broad.h"
#include <light_array.h>
#include <hash_map.h>
#include "../util.h"

Broad_Collision_Pair* broad_get_collision_pairs(Entity** entities) {
	Broad_Collision_Pair pair;
	Broad_Collision_Pair* collision_pairs = array_new(Broad_Collision_Pair);

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e1 = entities[i];
		for (u32 j = i + 1; j < array_length(entities); ++j) {
			Entity* e2 = entities[j];

			r64 entities_distance = gm_vec3_length(gm_vec3_subtract(e1->world_position, e2->world_position));

			// Increase the distance a little to account for moving objects.
			// @TODO: We should derivate this value from delta_time, forces, velocities, etc
			r64 max_distance_for_collision = e1->collider.bounding_sphere_radius + e2->collider.bounding_sphere_radius + 0.1;
			if (entities_distance <= max_distance_for_collision) {
				pair.e1_id = e1->id;
				pair.e2_id = e2->id;
				array_push(collision_pairs, pair);
			}
		}
	}

	return collision_pairs;
}

static eid uf_find(Hash_Map* entity_to_parent_map, eid x) {
	eid p;
	assert(!hash_map_get(entity_to_parent_map, &x, &p));
	if (p == x) {
		return x;
	}

	return uf_find(entity_to_parent_map, p);
}

static void uf_union(Hash_Map* entity_to_parent_map, eid x, eid y) {
	eid key = uf_find(entity_to_parent_map, y);
	eid value = uf_find(entity_to_parent_map, x);
	assert(hash_map_put(entity_to_parent_map, &key, &value) == 0);
}

static Hash_Map uf_collect_all(Entity** entities, Broad_Collision_Pair* collision_pairs) {
	Hash_Map entity_to_parent_map;
	assert(!hash_map_create(&entity_to_parent_map, 1024, sizeof(eid), sizeof(eid), util_eid_compare, util_eid_hash));

	for (u32 i = 0; i < array_length(entities); ++i) {
		eid id = entities[i]->id;
		assert(!hash_map_put(&entity_to_parent_map, &id, &id));
	}

	for (u32 i = 0; i < array_length(collision_pairs); ++i) {
		Broad_Collision_Pair collision_pair = collision_pairs[i];
		eid id1 = collision_pair.e1_id;
		eid id2 = collision_pair.e2_id;
		Entity* e1 = entity_get_by_id(id1);
		Entity* e2 = entity_get_by_id(id2);
		if (!e1->fixed && !e2->fixed) {
			uf_union(&entity_to_parent_map, collision_pair.e1_id, collision_pair.e2_id);
		}
	}

	return entity_to_parent_map;
}

eid** broad_collect_simulation_islands(Entity** entities, Broad_Collision_Pair* collision_pairs) {
	eid** simulation_islands = array_new(eid*);
	Hash_Map entity_to_parent_map = uf_collect_all(entities, collision_pairs);

	Hash_Map simulation_islands_map;
	assert(!hash_map_create(&simulation_islands_map, 2 * array_length(entities), sizeof(eid), sizeof(u32), util_eid_compare, util_eid_hash));

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		if (e->fixed) {
			continue;
		}
		eid parent = uf_find(&entity_to_parent_map, e->id);
		u32 simulation_island_idx;
		if (hash_map_get(&simulation_islands_map, &parent, &simulation_island_idx)) {
			// Simulation Island not created yet.
			eid* new_simulation_island = array_new(eid);
			simulation_island_idx = array_length(simulation_islands);
			array_push(simulation_islands, new_simulation_island);
			assert(!hash_map_put(&simulation_islands_map, &parent, &simulation_island_idx));
		}

		array_push(simulation_islands[simulation_island_idx], e->id);
	}

	hash_map_destroy(&entity_to_parent_map);
	hash_map_destroy(&simulation_islands_map);
	return simulation_islands;
}

void broad_simulation_islands_destroy(eid** simulation_islands) {
	for (u32 i = 0; i < array_length(simulation_islands); ++i) {
		eid* simulation_island = simulation_islands[i];
		array_free(simulation_island);
	}

	array_free(simulation_islands);
}