#include "clipping.h"
#include <light_array.h>
#include <float.h>
#include "gjk.h"
#include "support.h"

typedef struct {
	vec3 normal;
	vec3 point;
} Plane;

static boolean is_point_in_plane(const Plane* plane, vec3 position) {
	float distance = -gm_vec3_dot(plane->normal, plane->point);
	if (gm_vec3_dot(position, plane->normal) + distance < 0.0) {
		return false;
	}

	return true;
}

static boolean plane_edge_intersection(const Plane* plane, const vec3 start, const vec3 end, vec3* out_point) {
	const r64 EPSILON = 0.000001;
	vec3 ab = gm_vec3_subtract(end, start);

	// Check that the edge and plane are not parallel and thus never intersect
	// We do this by projecting the line (start - A, End - B) ab along the plane
	float ab_p = gm_vec3_dot(plane->normal, ab);
	if (fabs(ab_p) > EPSILON) {
		// Generate a random point on the plane (any point on the plane will suffice)
		float distance = -gm_vec3_dot(plane->normal, plane->point);
		vec3 p_co = gm_vec3_scalar_product(-distance, plane->normal);

		// Work out the edge factor to scale edge by
		// e.g. how far along the edge to traverse before it meets the plane.
		// This is computed by: -proj<plane_nrml>(edge_start - any_planar_point) / proj<plane_nrml>(edge_start - edge_end)
		float fac = -gm_vec3_dot(plane->normal, gm_vec3_subtract(start, p_co)) / ab_p;

		// Stop any large floating point divide issues with almost parallel planes
		fac = MIN(MAX(fac, 0.0), 1.0); 

		// Return point on edge
		*out_point = gm_vec3_add(start, gm_vec3_scalar_product(fac, ab));
		return true;
	}

	return false;
}

// Clips the input polygon to the input clip planes
// If remove_instead_of_clipping is true, vertices that are lying outside the clipping planes will be removed instead of clipped
// Based on https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics5collisionmanifolds/
static void sutherland_hodgman(vec3* input_polygon, int num_clip_planes, const Plane* clip_planes, vec3** out_polygon,
	boolean remove_instead_of_clipping) {
	assert(out_polygon != NULL);
	assert(num_clip_planes > 0);

	// Create temporary list of vertices
	// We will keep ping-pong'ing between the two lists updating them as we go.
	vec3* input = (vec3*)array_copy(input_polygon);
	vec3* output = array_new(vec3);

	for (int i = 0; i < num_clip_planes; ++i) {
		// If every single point has already been removed previously, just exit
		if (array_length(input) == 0) {
			break;
		}

		const Plane* plane = &clip_planes[i];

		// Loop through each edge of the polygon and clip that edge against the current plane.
		vec3 temp_point, start_point = input[array_length(input) - 1];
		for (u32 j = 0; j < array_length(input); ++j) {
			vec3 end_point = input[j];
			boolean start_in_plane = is_point_in_plane(plane, start_point);
			boolean end_in_plane = is_point_in_plane(plane, end_point);

			if (remove_instead_of_clipping) {
				if (end_in_plane) {
					array_push(output, end_point);
				}
			} else {
				// If the edge is entirely within the clipping plane, keep it as it is
				if (start_in_plane && end_in_plane) {
					array_push(output, end_point);
				}
				// If the edge interesects the clipping plane, cut the edge along clip plane
				else if (start_in_plane && !end_in_plane) {
					if (plane_edge_intersection(plane, start_point, end_point, &temp_point)) {
						array_push(output, temp_point);
					}
				} else if (!start_in_plane && end_in_plane) {
					if (plane_edge_intersection(plane, start_point, end_point, &temp_point)) {
						array_push(output, temp_point);
					}

					array_push(output, end_point);
				}
			}
			// ..otherwise the edge is entirely outside the clipping plane and should be removed/ignored

			start_point = end_point;
		}

		// Swap input/output polygons, and clear output list for us to generate afresh
		vec3* tmp = input;
		input = output;
		output = tmp;
		array_clear(output);
	}

	*out_polygon = input;
	array_free(output);
}

static vec3 get_closest_point_polygon(vec3 position, Plane* reference_plane) {
	r64 d = gm_vec3_dot(gm_vec3_scalar_product(-1.0, reference_plane->normal), reference_plane->point);
	return gm_vec3_subtract(position,
		gm_vec3_scalar_product(gm_vec3_dot(reference_plane->normal, position) + d, reference_plane->normal));
}

static Plane* build_boundary_planes(Collider_Convex_Hull* convex_hull, u32 target_face_idx) {
	Plane* result = array_new(Plane);
	u32* face_neighbors = convex_hull->face_to_neighbors[target_face_idx];

	for (u32 i = 0; i < array_length(face_neighbors); ++i) {
		Collider_Convex_Hull_Face neighbor_face = convex_hull->transformed_faces[face_neighbors[i]];
		Plane p;
		p.point = convex_hull->transformed_vertices[neighbor_face.elements[0]];
		p.normal = gm_vec3_negative(neighbor_face.normal);
		array_push(result, p);
	}

	return result;
}

static u32 get_face_with_most_fitting_normal(u32 support_idx, const Collider_Convex_Hull* convex_hull, vec3 normal) {
	const r64 EPSILON = 0.000001;
	u32* support_faces = convex_hull->vertex_to_faces[support_idx];

	r64 max_proj = -DBL_MAX;
	u32 selected_face_idx;
	for (u32 i = 0; i < array_length(support_faces); ++i) {
		Collider_Convex_Hull_Face face = convex_hull->transformed_faces[support_faces[i]];
		r64 proj = gm_vec3_dot(face.normal, normal);
		if (proj > max_proj) {
			max_proj = proj;
			selected_face_idx = support_faces[i];
		}
	}

	return selected_face_idx;
}

static dvec4 get_edge_with_most_fitting_normal(u32 support1_idx, u32 support2_idx, const Collider_Convex_Hull* convex_hull1,
	const Collider_Convex_Hull* convex_hull2, vec3 normal, vec3* edge_normal) {
	vec3 inverted_normal = gm_vec3_negative(normal);

	vec3 support1 = convex_hull1->transformed_vertices[support1_idx];
	vec3 support2 = convex_hull2->transformed_vertices[support2_idx];

	u32* support1_neighbors = convex_hull1->vertex_to_neighbors[support1_idx];
	u32* support2_neighbors = convex_hull2->vertex_to_neighbors[support2_idx];

	r64 max_dot = -DBL_MAX;
	dvec4 selected_edges;

	for (u32 i = 0; i < array_length(support1_neighbors); ++i) {
		vec3 neighbor1 = convex_hull1->transformed_vertices[support1_neighbors[i]];
		vec3 edge1 = gm_vec3_subtract(support1, neighbor1);
		for (u32 j = 0; j < array_length(support2_neighbors); ++j) {
			vec3 neighbor2 = convex_hull2->transformed_vertices[support2_neighbors[j]];
			vec3 edge2 = gm_vec3_subtract(support2, neighbor2);

			vec3 current_normal = gm_vec3_normalize(gm_vec3_cross(edge1, edge2));
			vec3 current_normal_inverted = gm_vec3_negative(current_normal);

			r64 dot = gm_vec3_dot(current_normal, normal);
			if (dot > max_dot) {
				max_dot = dot;
				selected_edges.x = support1_idx;
				selected_edges.y = support1_neighbors[i];
				selected_edges.z = support2_idx;
				selected_edges.w = support2_neighbors[j];
				*edge_normal = current_normal;
			}

			dot = gm_vec3_dot(current_normal_inverted, normal);
			if (dot > max_dot) {
				max_dot = dot;
				selected_edges.x = support1_idx;
				selected_edges.y = support1_neighbors[i];
				selected_edges.z = support2_idx;
				selected_edges.w = support2_neighbors[j];
				*edge_normal = current_normal_inverted;
			}
		}
	}

	return selected_edges;
}

// This function calculates the distance between two indepedent skew lines in the 3D world
// The first line is given by a known point P1 and a direction vector D1
// The second line is given by a known point P2 and a direction vector D2
// Outputs:
// L1 is the closest POINT to the second line that belongs to the first line
// L2 is the closest POINT to the first line that belongs to the second line
// _N is the number that satisfies L1 = P1 + _N * D1
// _M is the number that satisfies L2 = P2 + _M * D2
static boolean collision_distance_between_skew_lines(vec3 p1, vec3 d1, vec3 p2, vec3 d2, vec3 *l1, vec3 *l2, r64 * _n, r64 * _m) {
	r64 n1 = d1.x * d2.x + d1.y * d2.y + d1.z * d2.z;
	r64 n2 = d2.x * d2.x + d2.y * d2.y + d2.z * d2.z;
	r64 m1 = -d1.x * d1.x - d1.y * d1.y - d1.z * d1.z;
	r64 m2 = -d2.x * d1.x - d2.y * d1.y - d2.z * d1.z;
	r64 r1 = -d1.x * p2.x + d1.x * p1.x - d1.y * p2.y + d1.y * p1.y - d1.z * p2.z + d1.z * p1.z;
	r64 r2 = -d2.x * p2.x + d2.x * p1.x - d2.y * p2.y + d2.y * p1.y - d2.z * p2.z + d2.z * p1.z;

	// Solve 2x2 linear system
	if ((n1 * m2) - (n2 * m1) == 0) {
		return false;
	}
	r64 n = ((r1 * m2) - (r2 * m1)) / ((n1 * m2) - (n2 * m1));
	r64 m = ((n1 * r2) - (n2 * r1)) / ((n1 * m2) - (n2 * m1));

	if (l1) {
		*l1 = gm_vec3_add(p1, gm_vec3_scalar_product(m, d1));
	}
	if (l2) {
		*l2 = gm_vec3_add(p2, gm_vec3_scalar_product(n, d2));
	}
	if (_n) {
		*_n = n;
	}
	if (_m) {
		*_m = m;
	}

	return true;
}

static vec3* get_vertices_of_faces(Collider_Convex_Hull* hull, Collider_Convex_Hull_Face face) {
	vec3* vertices = array_new(vec3);
	for (u32 i = 0; i < array_length(face.elements); ++i) {
		array_push(vertices, hull->transformed_vertices[face.elements[i]]);
	}
	return vertices;
}

Collider_Contact* convex_convex_contact_manifold(Collider* collider1, Collider* collider2, vec3 normal) {
	assert(collider1->type == COLLIDER_TYPE_CONVEX_HULL);
	assert(collider2->type == COLLIDER_TYPE_CONVEX_HULL);
	Collider_Convex_Hull* convex_hull1 = &collider1->convex_hull;
	Collider_Convex_Hull* convex_hull2 = &collider2->convex_hull;

	const r64 EPSILON = 0.0001;
	Collider_Contact* contacts = array_new(Collider_Contact);

	vec3 inverted_normal = gm_vec3_negative(normal);

	vec3 edge_normal;
	u32 support1_idx = support_point_get_index(convex_hull1, normal);
	u32 support2_idx = support_point_get_index(convex_hull2, inverted_normal);
	u32 face1_idx = get_face_with_most_fitting_normal(support1_idx, convex_hull1, normal);
	u32 face2_idx = get_face_with_most_fitting_normal(support2_idx, convex_hull2, inverted_normal);
	Collider_Convex_Hull_Face face1 = convex_hull1->transformed_faces[face1_idx];
	Collider_Convex_Hull_Face face2 = convex_hull2->transformed_faces[face2_idx];
	dvec4 edges = get_edge_with_most_fitting_normal(support1_idx, support2_idx, convex_hull1, convex_hull2, normal, &edge_normal);

	r64 chosen_normal1_dot = gm_vec3_dot(face1.normal, normal);
	r64 chosen_normal2_dot = gm_vec3_dot(face2.normal, inverted_normal);
	r64 edge_normal_dot = gm_vec3_dot(edge_normal, normal);

	if (edge_normal_dot > chosen_normal1_dot + EPSILON && edge_normal_dot > chosen_normal2_dot + EPSILON) {
		//printf("EDGE\n");
		vec3 l1, l2;
		vec3 p1 = convex_hull1->transformed_vertices[edges.x];
		vec3 d1 = gm_vec3_subtract(convex_hull1->transformed_vertices[edges.y], p1);
		vec3 p2 = convex_hull2->transformed_vertices[edges.z];
		vec3 d2 = gm_vec3_subtract(convex_hull2->transformed_vertices[edges.w], p2);
		assert(collision_distance_between_skew_lines(p1, d1, p2, d2, &l1, &l2, 0, 0));
		Collider_Contact contact = (Collider_Contact){l1, l2};
		array_push(contacts, contact);
	} else {
		//printf("FACE\n");
		boolean is_face1_the_reference_face = chosen_normal1_dot > chosen_normal2_dot;
		vec3* reference_face_support_points = is_face1_the_reference_face ?
			get_vertices_of_faces(convex_hull1, face1) : get_vertices_of_faces(convex_hull2, face2);
		vec3* incident_face_support_points = is_face1_the_reference_face ?
			get_vertices_of_faces(convex_hull2, face2) : get_vertices_of_faces(convex_hull1, face1);

		Plane* boundary_planes = is_face1_the_reference_face ? build_boundary_planes(convex_hull1, face1_idx) :
			build_boundary_planes(convex_hull2, face2_idx);

		vec3* clipped_points;
		sutherland_hodgman(incident_face_support_points, array_length(boundary_planes), boundary_planes, &clipped_points, false);

		Plane reference_plane;
		reference_plane.normal = is_face1_the_reference_face ? gm_vec3_negative(face1.normal) :
			gm_vec3_negative(face2.normal);
		reference_plane.point = reference_face_support_points[0];

		vec3* final_clipped_points;
		sutherland_hodgman(clipped_points, 1, &reference_plane, &final_clipped_points, true);

		for (u32 i = 0; i < array_length(final_clipped_points); ++i) {
			vec3 point = final_clipped_points[i];
			//vec3 closest_point = get_closest_pointPolygon(point, reference_face_support_points);
			vec3 closest_point = get_closest_point_polygon(point, &reference_plane);
			vec3 point_diff = gm_vec3_subtract(point, closest_point);
			r64 contact_penetration;

			// we are projecting the points that are in the incident face on the reference planes
			// so the points that we have are part of the incident object.
			Collider_Contact contact;
			if (is_face1_the_reference_face) {
				contact_penetration = gm_vec3_dot(point_diff, normal);
				contact.collision_point1 = gm_vec3_subtract(point, gm_vec3_scalar_product(contact_penetration, normal));
				contact.collision_point2 = point;
			} else {
				contact_penetration = - gm_vec3_dot(point_diff, normal);
				contact.collision_point1 = point;
				contact.collision_point2 = gm_vec3_add(point, gm_vec3_scalar_product(contact_penetration, normal));
			}

			if (contact_penetration < 0.0) {
				array_push(contacts, contact);
			}
		}

		array_free(reference_face_support_points);
		array_free(incident_face_support_points);
		array_free(boundary_planes);
		array_free(clipped_points);
		array_free(final_clipped_points);
	}

	if (array_length(contacts) == 0) {
		//printf("Warning: no intersection was found\n");
	}

	return contacts;
}

Collider_Contact* clipping_get_contact_manifold(Collider* collider1, Collider* collider2, vec3 normal, r64 penetration) {
	// TODO: For now, we only consider CONVEX and SPHERE colliders.
	// If new colliders are added, we can think about making this more generic.

	if (collider1->type == COLLIDER_TYPE_SPHERE) {
		Collider_Contact* contacts = array_new(Collider_Contact);
		vec3 sphere_collision_point = support_point(collider1, normal);

		Collider_Contact contact;
		contact.collision_point1 = sphere_collision_point;
		contact.collision_point2 = gm_vec3_subtract(sphere_collision_point, gm_vec3_scalar_product(penetration, normal));
		array_push(contacts, contact);

		return contacts;
	} else if (collider2->type == COLLIDER_TYPE_SPHERE) {
		Collider_Contact* contacts = array_new(Collider_Contact);
		vec3 inverse_normal = gm_vec3_negative(normal);
		vec3 sphere_collision_point = support_point(collider2, inverse_normal);

		Collider_Contact contact;
		contact.collision_point1 = gm_vec3_add(sphere_collision_point, gm_vec3_scalar_product(penetration, normal));
		contact.collision_point2 = sphere_collision_point;
		array_push(contacts, contact);

		return contacts;
	} else {
		// For now, this case must be convex-convex
		assert(collider1->type == COLLIDER_TYPE_CONVEX_HULL);
		assert(collider2->type == COLLIDER_TYPE_CONVEX_HULL);
		return convex_convex_contact_manifold(collider1, collider2, normal);
	}
}