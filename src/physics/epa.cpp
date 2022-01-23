#include "epa.h"
#include <light_array.h>
#include <float.h>
#include "support.h"

static const r64 EPSILON = 0.0001;

void polytope_from_gjk_simplex(const GJK_Simplex* s, vec3** _polytope, dvec3** _faces) {
	assert(s->num == 4);
	vec3* polytope = array_new_len(vec3, 4);
	dvec3* faces = array_new_len(dvec3, 4);

	array_push(polytope, s->a);
	array_push(polytope, s->b);
	array_push(polytope, s->c);
	array_push(polytope, s->d);

	dvec3 i1 = (dvec3){0, 1, 2}; // ABC
	dvec3 i2 = (dvec3){0, 2, 3}; // ACD
	dvec3 i3 = (dvec3){0, 3, 1}; // ADB
	dvec3 i4 = (dvec3){1, 2, 3}; // BCD

	array_push(faces, i1);
	array_push(faces, i2);
	array_push(faces, i3);
	array_push(faces, i4);

	*_polytope = polytope;
	*_faces = faces;
}

void get_face_normal_and_distance_to_origin(dvec3 face, vec3* polytope, vec3* _normal, r64* _distance) {
	vec3 a = polytope[face.x];
	vec3 b = polytope[face.y];
	vec3 c = polytope[face.z];

	vec3 ab = gm_vec3_subtract(b, a);
	vec3 ac = gm_vec3_subtract(c, a);
	vec3 normal = gm_vec3_normalize(gm_vec3_cross(ab, ac));

	assert(normal.x != 0.0 || normal.y != 0.0 || normal.z != 0.0);

	// When this value is not 0, it is possible that the normals are not found even if the polytope is not degenerate
	const r64 DISTANCE_TO_ORIGIN_TOLERANCE = 0.0000000000000;

	// the distance from the face's *plane* to the origin (considering an infinite plane).
	r64 distance = gm_vec3_dot(normal, a);
	if (distance < -DISTANCE_TO_ORIGIN_TOLERANCE) {
		// if the distance is less than 0, it means that our normal is point inwards instead of outwards
		// in this case, we just invert both normal and distance
		// this way, we don't need to worry about face's winding
		normal = gm_vec3_invert(normal);
		distance = -distance;
	} else if (distance >= -DISTANCE_TO_ORIGIN_TOLERANCE && distance <= DISTANCE_TO_ORIGIN_TOLERANCE) {
		// if the distance is exactly 0.0, then it means that the origin is lying exactly on the face.
		// in this case, we can't directly infer the orientation of the normal.
		// since our shape is convex, we analyze the other vertices of the hull to deduce the orientation
		boolean was_able_to_calculate_normal = false;
		for (u32 i = 0; i < array_length(polytope); ++i) {
			vec3 current = polytope[i];
			r64 auxiliar_distance = gm_vec3_dot(normal, current);
			if (auxiliar_distance < -DISTANCE_TO_ORIGIN_TOLERANCE || auxiliar_distance > DISTANCE_TO_ORIGIN_TOLERANCE) {
				// since the shape is convex, the other vertices should always be "behind" the normal plane
				normal = auxiliar_distance < -DISTANCE_TO_ORIGIN_TOLERANCE ? normal : gm_vec3_invert(normal);
				was_able_to_calculate_normal = true;
				break;
			}
		}

		// If we were not able to calculate the normal, it means that ALL points of the polytope are in the same plane
		// Therefore, we either have a degenerate polytope or our tolerance is not big enough
		assert(was_able_to_calculate_normal);
	}

	*_normal = normal;
	*_distance = distance;
}

void add_edge(dvec2** edges, dvec2 edge, vec3* polytope) {
	// @TODO: we can use a hash table here
	for (u32 i = 0; i < array_length(*edges); ++i) {
		dvec2 current = (*edges)[i];
		if (edge.x == current.x && edge.y == current.y) {
			array_remove(*edges, i);
			return;
		}
		if (edge.x == current.y && edge.y == current.x) {
			array_remove(*edges, i);
			return;
		}

		// @TEMPORARY: Once indexes point to unique vertices, this won't be needed.
		vec3 current_v1 = polytope[current.x];
		vec3 current_v2 = polytope[current.y];
		vec3 edge_v1 = polytope[edge.x];
		vec3 edge_v2 = polytope[edge.y];

		if (gm_vec3_equal(current_v1, edge_v1) && gm_vec3_equal(current_v2, edge_v2)) {
			array_remove(*edges, i);
			return;
		}

		if (gm_vec3_equal(current_v1, edge_v2) && gm_vec3_equal(current_v2, edge_v1)) {
			array_remove(*edges, i);
			return;
		}
	}

	array_push(*edges, edge);
}

static vec3 triangle_centroid(vec3 p1, vec3 p2, vec3 p3) {
	vec3 centroid = gm_vec3_add(gm_vec3_add(p2, p3), p1);
	centroid = gm_vec3_scalar_product(1.0 / 3.0, centroid);
	return centroid;
}

boolean epa(Collider* collider1, Collider* collider2, GJK_Simplex* simplex, vec3* _normal, r64* _penetration) {
	vec3* polytope;
	dvec3* faces;

	// build initial polytope from GJK simplex
	polytope_from_gjk_simplex(simplex, &polytope, &faces);

	vec3* normals = array_new_len(vec3, 128);
	r64* faces_distance_to_origin = array_new_len(r64, 128);

	vec3 min_normal;
	r64 min_distance = DBL_MAX;

	for (u32 i = 0; i < array_length(faces); ++i) {
		vec3 normal;
		r64 distance;
		dvec3 face = faces[i];

		get_face_normal_and_distance_to_origin(face, polytope, &normal, &distance);

		array_push(normals, normal);
		array_push(faces_distance_to_origin, distance);

		if (distance < min_distance) {
			min_distance = distance;
			min_normal = normal;
		}
	}

	dvec2* edges = array_new_len(dvec2, 1024);
	boolean converged = false;
	for (u32 it = 0; it < 100; ++it) {
		vec3 support_point = support_point_of_minkowski_difference(collider1, collider2, min_normal);

		// If the support time lies on the face currently set as the closest to the origin, we are done.
		r64 d = gm_vec3_dot(min_normal, support_point);
		if (fabs(d - min_distance) < EPSILON) {
			*_normal = min_normal;
			*_penetration = min_distance;
			converged = true;
			//printf("epa: took %d iterations to converge\n", it);
			break;
		}

		// add new point to polytope
		u32 new_point_index = array_length(polytope);
		array_push(polytope, support_point);

		// Expand Polytope
		for (u32 i = 0; i < array_length(normals); ++i) {
			vec3 normal = normals[i];
			dvec3 face = faces[i];

			// If the face normal points towards the support point, we need to reconstruct it.
			vec3 centroid = triangle_centroid(
				polytope[face.x],
				polytope[face.y],
				polytope[face.z]
			);

			// If the face normal points towards the support point, we need to reconstruct it.
			if (gm_vec3_dot(normal, gm_vec3_subtract(support_point, centroid)) > 0.0) {
				dvec3 face = faces[i];

				dvec2 edge1 = (dvec2){face.x, face.y};
				dvec2 edge2 = (dvec2){face.y, face.z};
				dvec2 edge3 = (dvec2){face.z, face.x};

				add_edge(&edges, edge1, polytope);
				add_edge(&edges, edge2, polytope);
				add_edge(&edges, edge3, polytope);

				// Relative order between the two arrays should be kept.
				array_remove(faces, i);
				array_remove(faces_distance_to_origin, i);
				array_remove(normals, i);

				--i;
			}
		}

		for (u32 i = 0; i < array_length(edges); ++i) {
			dvec2 edge = edges[i];
			dvec3 new_face;
			new_face.x = edge.x;
			new_face.y = edge.y;
			new_face.z = new_point_index;
			array_push(faces, new_face);

			vec3 new_face_normal;
			r64 new_face_distance;
			get_face_normal_and_distance_to_origin(new_face, polytope, &new_face_normal, &new_face_distance);

			array_push(normals, new_face_normal);
			array_push(faces_distance_to_origin, new_face_distance);
		}

		min_distance = DBL_MAX;
		for (u32 i = 0; i < array_length(faces_distance_to_origin); ++i) {
			r64 distance = faces_distance_to_origin[i];
			if (distance < min_distance) {
				min_distance = distance;
				min_normal = normals[i];
			}
		}

		array_clear(edges);
	}

	array_free(faces);
	array_free(polytope);
	array_free(normals);
	array_free(faces_distance_to_origin);
	array_free(edges);

	if (!converged) {
		printf("EPA did not converge.\n");
	}

	return converged;
}