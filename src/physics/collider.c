#include "collider.h"
#include <hash_map.h>
#include <memory.h>
#include <light_array.h>
#include <limits.h>

static int vertex_compare(const void *key1, const void *key2) {
	vec3 v1 = *(vec3*)key1;
	vec3 v2 = *(vec3*)key2;
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

static unsigned int vertex_hash(const void *key) {
	vec3 v = *(vec3*)key;
	unsigned long long x, y, z;
	assert(sizeof(r64) == sizeof(unsigned long long));
	memcpy(&x, &v.x, sizeof(r64));
	memcpy(&y, &v.y, sizeof(r64));
	memcpy(&z, &v.z, sizeof(r64));
	return (unsigned int)((x + y + z) % UINT_MAX);
}

static boolean do_triangles_share_same_vertex(dvec3 t1, dvec3 t2) {
	return t1.x == t2.x || t1.x == t2.y || t1.x == t2.z ||
		t1.y == t2.x || t1.y == t2.y || t1.y == t2.z ||
		t1.z == t2.x || t1.z == t2.y || t1.z == t2.z;
}

static boolean do_faces_share_same_vertex(u32* e1, u32* e2) {
	for (u32 i = 0; i < array_length(e1); ++i) {
		u32 i1 = e1[i];
		for (u32 j = 0; j < array_length(e2); ++j) {
			u32 i2 = e2[j];
			if (i1 == i2) {
				return true;
			}
		}
	}

	return false;
}

static void collect_faces_planar_to(vec3* hull, dvec3* hull_triangle_faces, u32** triangle_faces_to_neighbor_faces_map,
	boolean* is_triangle_face_already_processed_arr, u32 face_to_test_idx, vec3 target_normal, dvec3** out) {
	const r64 EPSILON = 0.000001;
	dvec3 face_to_test = hull_triangle_faces[face_to_test_idx];
	vec3 v1 = hull[face_to_test.x];
	vec3 v2 = hull[face_to_test.y];
	vec3 v3 = hull[face_to_test.z];
	
	vec3 v12 = gm_vec3_subtract(v2, v1);
	vec3 v13 = gm_vec3_subtract(v3, v1);
	vec3 face_normal = gm_vec3_normalize(gm_vec3_cross(v12, v13));

	if (is_triangle_face_already_processed_arr[face_to_test_idx]) {
		return;
	}

	r64 projection = gm_vec3_dot(face_normal, target_normal);

	if ((projection - 1.0) > -EPSILON && (projection - 1.0) < EPSILON) {
		array_push(*out, face_to_test);
		is_triangle_face_already_processed_arr[face_to_test_idx] = true;

		u32* neighbor_faces = triangle_faces_to_neighbor_faces_map[face_to_test_idx];

		for (u32 i = 0; i < array_length(neighbor_faces); ++i) {
			u32 neighbor_face_idx = neighbor_faces[i];
			collect_faces_planar_to(hull, hull_triangle_faces, triangle_faces_to_neighbor_faces_map,
				is_triangle_face_already_processed_arr, neighbor_face_idx, target_normal, out);
		}
	}
}

static s32 get_edge_index(const dvec2* edges, dvec2 edge) {
	for (s32 i = 0; i < array_length(edges); ++i) {
		dvec2 current_edge = edges[i];
		if (current_edge.x == edge.x && current_edge.y == edge.y) {
			return i;
		}
		if (current_edge.x == edge.y && current_edge.y == edge.x) {
			return i;
		}
	}

	return -1;
}

static Collider_Convex_Hull_Face create_convex_hull_face(dvec3* triangles, vec3 face_normal) {
	dvec2* edges = array_new(dvec2);

	// Collect the edges that form the border of the face
	for (u32 i = 0; i < array_length(triangles); ++i) {
		dvec3 triangle = triangles[i];

		dvec2 edge1 = (dvec2){triangle.x, triangle.y};
		dvec2 edge2 = (dvec2){triangle.y, triangle.z};
		dvec2 edge3 = (dvec2){triangle.z, triangle.x};

		s32 edge1_idx = get_edge_index(edges, edge1);
		s32 edge2_idx = get_edge_index(edges, edge2);
		s32 edge3_idx = get_edge_index(edges, edge3);

		if (edge1_idx >= 0) {
			array_remove(edges, edge1_idx);
		} else {
			array_push(edges, edge1);
		}

		if (edge2_idx >= 0) {
			array_remove(edges, edge2_idx);
		} else {
			array_push(edges, edge2);
		}

		if (edge3_idx >= 0) {
			array_remove(edges, edge3_idx);
		} else {
			array_push(edges, edge3);
		}
	}

	// Nicely order the edges
	for (u32 i = 0; i < array_length(edges); ++i) {
		dvec2 current_edge = edges[i];
		for (u32 j = i + 1; j < array_length(edges); ++j) {
			dvec2 candidate_edge = edges[j];

			if (current_edge.y != candidate_edge.x && current_edge.y != candidate_edge.y) {
				continue;
			}

			if (current_edge.y == candidate_edge.y) {
				u32 tmp = candidate_edge.x;
				candidate_edge.x = candidate_edge.y;
				candidate_edge.y = tmp;
			}

			dvec2 tmp = edges[i + 1];
			edges[i + 1] = candidate_edge;
			edges[j] = tmp;
		}
	}

	assert(edges[0].x == edges[array_length(edges) - 1].y);

	// Simply create the face elements based on the edges
	u32* face_elements = array_new(u32);
	for (u32 i = 0; i < array_length(edges); ++i) {
		dvec2 current_edge = edges[i];
		array_push(face_elements, current_edge.x);
	}

	array_free(edges);

	Collider_Convex_Hull_Face face;
	face.elements = face_elements;
	face.normal = face_normal;
	return face;
}

static boolean is_neighbor_already_in_vertex_to_neighbors_map(u32* vertex_to_neighbors, u32 neighbor) {
	for (u32 i = 0; i < array_length(vertex_to_neighbors); ++i) {
		if (vertex_to_neighbors[i] == neighbor) {
			return true;
		}
	}

	return false;
}

static r64 get_convex_hull_bounding_sphere_radius(const vec3* hull) {
	r64 max_distance = 0.0;
	for (u32 i = 0; i < array_length(hull); ++i) {
		vec3 v = hull[i];
		r64 distance = gm_vec3_length(v);
		if (distance > max_distance) {
			max_distance = distance;
		}
	}

	return max_distance;
}

// Create a convex hull from the vertices+indices
// For now, we assume that the mesh is already a convex hull
// This function only makes sure that vertices are unique - duplicated vertices will be merged.
static Collider collider_convex_hull_create(const vec3* vertices, const u32* indices) {
	Hash_Map vertex_to_idx_map;
	hash_map_create(&vertex_to_idx_map, 1024, sizeof(vec3), sizeof(u32), vertex_compare, vertex_hash);

	vec3* hull = array_new(vec3);
	
	// Build hull, eliminating duplicated vertex
	for (u32 i = 0; i < array_length(vertices); ++i) {
		vec3 current_vertex = vertices[i];
		u32 current_index;
		if (hash_map_get(&vertex_to_idx_map, &current_vertex, &current_index)) {
			current_index = array_length(hull);
			array_push(hull, current_vertex);
			hash_map_put(&vertex_to_idx_map, &current_vertex, &current_index);
		}
	}

	// Collect all triangle faces that compose the just-built hull
	dvec3* hull_triangle_faces = array_new(dvec3);
	for (u32 i = 0; i < array_length(indices); i += 3) {
		u32 i1 = indices[i];
		u32 i2 = indices[i + 1];
		u32 i3 = indices[i + 2];
		vec3 v1 = vertices[i1];
		vec3 v2 = vertices[i2];
		vec3 v3 = vertices[i3];

		u32 new_i1, new_i2, new_i3;
		assert(hash_map_get(&vertex_to_idx_map, &v1, &new_i1) == 0);
		assert(hash_map_get(&vertex_to_idx_map, &v2, &new_i2) == 0);
		assert(hash_map_get(&vertex_to_idx_map, &v3, &new_i3) == 0);
		
		dvec3 triangle = (dvec3){new_i1, new_i2, new_i3};
		array_push(hull_triangle_faces, triangle);
	}

	// Prepare vertex to faces map
	u32** vertex_to_faces_map = malloc(sizeof(u32*) * array_length(hull));
	for (u32 i = 0; i < array_length(hull); ++i) {
		vertex_to_faces_map[i] = array_new(u32);
	}

	// Prepare vertex to neighbors map
	u32** vertex_to_neighbors_map = malloc(sizeof(u32*) * array_length(hull));
	for (u32 i = 0; i < array_length(hull); ++i) {
		vertex_to_neighbors_map[i] = array_new(u32);
	}

	// Prepare triangle faces to neighbors map
	u32** triangle_faces_to_neighbor_faces_map = malloc(sizeof(u32*) * array_length(hull_triangle_faces));
	for (u32 i = 0; i < array_length(hull_triangle_faces); ++i) {
		triangle_faces_to_neighbor_faces_map[i] = array_new(u32);
	}

	// Create the vertex to neighbors map
	for (u32 i = 0; i < array_length(hull_triangle_faces); ++i) {
		dvec3 triangle_face = hull_triangle_faces[i];

		for (u32 j = 0; j < array_length(hull_triangle_faces); ++j) {
			if (i == j) {
				continue;
			}

			dvec3 face_to_test = hull_triangle_faces[j];
			if (do_triangles_share_same_vertex(triangle_face, face_to_test)) {
				array_push(triangle_faces_to_neighbor_faces_map[i], j);
			}
		}

		// Fill vertex to edges map
		if (!is_neighbor_already_in_vertex_to_neighbors_map(vertex_to_neighbors_map[triangle_face.x], triangle_face.y)) {
			array_push(vertex_to_neighbors_map[triangle_face.x], triangle_face.y);
		}
		if (!is_neighbor_already_in_vertex_to_neighbors_map(vertex_to_neighbors_map[triangle_face.x], triangle_face.z)) {
			array_push(vertex_to_neighbors_map[triangle_face.x], triangle_face.z);
		}
		if (!is_neighbor_already_in_vertex_to_neighbors_map(vertex_to_neighbors_map[triangle_face.y], triangle_face.x)) {
			array_push(vertex_to_neighbors_map[triangle_face.y], triangle_face.x);
		}
		if (!is_neighbor_already_in_vertex_to_neighbors_map(vertex_to_neighbors_map[triangle_face.y], triangle_face.z)) {
			array_push(vertex_to_neighbors_map[triangle_face.y], triangle_face.z);
		}
		if (!is_neighbor_already_in_vertex_to_neighbors_map(vertex_to_neighbors_map[triangle_face.z], triangle_face.x)) {
			array_push(vertex_to_neighbors_map[triangle_face.z], triangle_face.x);
		}
		if (!is_neighbor_already_in_vertex_to_neighbors_map(vertex_to_neighbors_map[triangle_face.z], triangle_face.y)) {
			array_push(vertex_to_neighbors_map[triangle_face.z], triangle_face.y);
		}
	}

	// Collect all 'de facto' faces of the convex hull
	Collider_Convex_Hull_Face* faces = array_new(Collider_Convex_Hull_Face);
	boolean* is_triangle_face_already_processed_arr = calloc(array_length(hull_triangle_faces), sizeof(boolean));

	for (u32 i = 0; i < array_length(hull_triangle_faces); ++i) {
		if (is_triangle_face_already_processed_arr[i]) {
			continue;
		}

		dvec3 triangle_face = hull_triangle_faces[i];
		vec3 v1 = hull[triangle_face.x];
		vec3 v2 = hull[triangle_face.y];
		vec3 v3 = hull[triangle_face.z];

		vec3 v12 = gm_vec3_subtract(v2, v1);
		vec3 v13 = gm_vec3_subtract(v3, v1);
		vec3 normal = gm_vec3_normalize(gm_vec3_cross(v12, v13));

		dvec3* planar_faces = array_new(dvec3);
		collect_faces_planar_to(hull, hull_triangle_faces, triangle_faces_to_neighbor_faces_map,
			is_triangle_face_already_processed_arr, i, normal, &planar_faces);

		Collider_Convex_Hull_Face new_face = create_convex_hull_face(planar_faces, normal);
		u32 new_face_index = array_length(faces);
		array_push(faces, new_face);

		// Fill vertex to faces map accordingly
		for (u32 j = 0; j < array_length(planar_faces); ++j) {
			dvec3 planar_face = planar_faces[j];
			array_push(vertex_to_faces_map[planar_face.x], new_face_index);
			array_push(vertex_to_faces_map[planar_face.y], new_face_index);
			array_push(vertex_to_faces_map[planar_face.z], new_face_index);
		}

		array_free(planar_faces);
	}

	// Prepare face to neighbors map
	u32** face_to_neighbor_faces_map = malloc(sizeof(u32*) * array_length(faces));
	for (u32 i = 0; i < array_length(faces); ++i) {
		face_to_neighbor_faces_map[i] = array_new(u32);
	}

	// Fill faces to neighbor faces map
	for (u32 i = 0; i < array_length(faces); ++i) {
		Collider_Convex_Hull_Face face = faces[i];

		for (u32 j = 0; j < array_length(faces); ++j) {
			if (i == j) {
				continue;
			}

			Collider_Convex_Hull_Face candidate_face = faces[j];
			if (do_faces_share_same_vertex(face.elements, candidate_face.elements)) {
				array_push(face_to_neighbor_faces_map[i], j);
			}
		}
	}

	hash_map_destroy(&vertex_to_idx_map);
	for (u32 i = 0; i < array_length(hull_triangle_faces); ++i) {
		array_free(triangle_faces_to_neighbor_faces_map[i]);
	}
	free(triangle_faces_to_neighbor_faces_map);
	free(is_triangle_face_already_processed_arr);
	array_free(hull_triangle_faces);

	Collider_Convex_Hull convex_hull;
	convex_hull.faces = faces;
	convex_hull.transformed_faces = array_copy(faces);
	convex_hull.vertices = hull;
	convex_hull.transformed_vertices = array_copy(hull);
	convex_hull.vertex_to_faces = vertex_to_faces_map;
	convex_hull.vertex_to_neighbors = vertex_to_neighbors_map;
	convex_hull.face_to_neighbors = face_to_neighbor_faces_map;

	Collider collider;
	collider.type = COLLIDER_TYPE_CONVEX_HULL;
	collider.bounding_sphere_radius = get_convex_hull_bounding_sphere_radius(hull);
	collider.convex_hull = convex_hull;
	return collider;
}

static void collider_convex_hull_destroy(Collider* collider) {
	for (u32 i = 0; i < array_length(collider->convex_hull.vertices); ++i) {
		array_free(collider->convex_hull.vertex_to_faces[i]);
	}
	free(collider->convex_hull.vertex_to_faces);
	for (u32 i = 0; i < array_length(collider->convex_hull.vertices); ++i) {
		array_free(collider->convex_hull.vertex_to_neighbors[i]);
	}
	free(collider->convex_hull.vertex_to_neighbors);
	for (u32 i = 0; i < array_length(collider->convex_hull.faces); ++i) {
		array_free(collider->convex_hull.face_to_neighbors[i]);
	}
	free(collider->convex_hull.face_to_neighbors);

	array_free(collider->convex_hull.vertices);
	array_free(collider->convex_hull.transformed_vertices);
	for (u32 i = 0; i < array_length(collider->convex_hull.faces); ++i) {
		array_free(collider->convex_hull.faces[i].elements);
	}
	array_free(collider->convex_hull.faces);

	// The 'elements' array of the transformed faces is the same as the one stored in 'faces', so we don't need to release it again.
	array_free(collider->convex_hull.transformed_faces);
}

Collider collider_create(const vec3* vertices, const u32* indices, Collider_Type type) {
	switch (type) {
		case COLLIDER_TYPE_CONVEX_HULL: {
			return collider_convex_hull_create(vertices, indices);
		} break;
	}
	assert(0);
}

void collider_destroy(Collider* collider) {
	switch (collider->type) {
		case COLLIDER_TYPE_CONVEX_HULL: {
			collider_convex_hull_destroy(collider);
		} break;
	}
}

void collider_update(Collider* collider, mat4 model_matrix_no_scale) {
	switch (collider->type) {
		case COLLIDER_TYPE_CONVEX_HULL: {
			for (u32 i = 0; i < array_length(collider->convex_hull.transformed_vertices); ++i) {
				vec4 vertex = (vec4) {
					collider->convex_hull.vertices[i].x,
					collider->convex_hull.vertices[i].y,
					collider->convex_hull.vertices[i].z,
					1.0
				};
				vec4 transformed_vertex = gm_mat4_multiply_vec4(&model_matrix_no_scale, vertex);
				transformed_vertex = gm_vec4_scalar_product(1.0 / transformed_vertex.w, transformed_vertex);
				collider->convex_hull.transformed_vertices[i] = gm_vec4_to_vec3(transformed_vertex);
			}

			for (u32 i = 0; i < array_length(collider->convex_hull.transformed_faces); ++i) {
				vec3 normal = collider->convex_hull.faces[i].normal;
				vec3 transformed_normal = gm_mat4_multiply_vec3(&model_matrix_no_scale, normal, false);
				collider->convex_hull.transformed_faces[i].normal = gm_vec3_normalize(transformed_normal);
			}
		} break;
		default: {
			assert(0);
		} break;
	}
}
