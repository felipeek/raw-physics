#include "broad.h"
#include <light_array.h>

Broad_Collision_Pair* broad_get_collision_pairs(Entity* entities) {
	Broad_Collision_Pair pair;
	Broad_Collision_Pair* collision_pairs = array_new(Broad_Collision_Pair);

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e1 = &entities[i];
		for (u32 j = i + 1; j < array_length(entities); ++j) {
			Entity* e2 = &entities[j];

			r32 entities_distance = gm_vec3_length(gm_vec3_subtract(e1->world_position, e2->world_position));
			if (entities_distance <= e1->collider.bounding_sphere_radius + e2->collider.bounding_sphere_radius) {
				pair.e1 = e1;
				pair.e2 = e2;
				array_push(collision_pairs, pair);
			}
		}
	}

	return collision_pairs;
}

static boolean is_entity_already_in_simulation_island(Entity*** simulation_island, Entity* entity) {
	for (u32 i = 0; i < array_length(*simulation_island); ++i) {
		Entity* current = (*simulation_island)[i];
		if (current == entity) {
			return true;
		}
	}

	return false;
}

static void remove_computed_entity_from_array(Entity*** entities_to_compute, Entity* entity) {
	for (u32 i = 0; i < array_length(*entities_to_compute); ++i) {
		Entity* current = (*entities_to_compute)[i];
		if (current == entity) {
			array_remove(*entities_to_compute, i);
			return;
		}
	}
}

static void collect_simulation_island_recursively(Entity*** entities_to_compute, Broad_Collision_Pair** collision_pairs,
	Entity*** simulation_island, Entity* target_entity) {
	if (is_entity_already_in_simulation_island(simulation_island, target_entity)) {
		return;
	}

	array_push(*simulation_island, target_entity);

	for (u32 i = 0; i < array_length(*collision_pairs); ++i) {
		Broad_Collision_Pair* collision_pair = &(*collision_pairs)[i];
		if (collision_pair->e1 == target_entity) {
			collect_simulation_island_recursively(entities_to_compute, collision_pairs, simulation_island, collision_pair->e2);
		} else if (collision_pair->e2 == target_entity) {
			collect_simulation_island_recursively(entities_to_compute, collision_pairs, simulation_island, collision_pair->e1);
		}
	}

	remove_computed_entity_from_array(entities_to_compute, target_entity);
}

// TEMPORARY: Brute-force algorithm, needs to be rewritten!
Entity*** broad_collect_simulation_islands(Entity* entities, Broad_Collision_Pair* _collision_pairs) {
	// @TEMPORARY: Hack to overcome the fact that the entities are not being dynamically allocated,
	// i.e., their data is being directly stored in the array
	// we need to implement an 'eid' squiggly-like scheme or, at least, malloc the entities in core and use an Entity** array there
	Entity** entities_to_compute = array_new(Entity*);
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		if (!e->fixed) {
			array_push(entities_to_compute, e);
		}
	}

	Broad_Collision_Pair* collision_pairs = array_new(Broad_Collision_Pair);
	for (u32 i = 0; i < array_length(_collision_pairs); ++i) {
		Broad_Collision_Pair collision_pair = _collision_pairs[i];
		if (!collision_pair.e1->fixed && !collision_pair.e2->fixed) {
			array_push(collision_pairs, collision_pair);
		}
	}
	Entity*** simulation_islands = array_new(Entity**);

	while (array_length(entities_to_compute) > 0) {
		Entity* e = entities_to_compute[0];
		Entity** simulation_island = array_new(Entity*);
		collect_simulation_island_recursively(&entities_to_compute, &collision_pairs, &simulation_island, e);
		array_push(simulation_islands, simulation_island);
	}

	array_free(entities_to_compute);
	array_free(collision_pairs);
	return simulation_islands;
}