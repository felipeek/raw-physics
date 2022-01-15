#ifndef RAW_PHYSICS_PHYSICS_COLLIDER_H
#define RAW_PHYSICS_PHYSICS_COLLIDER_H
#include <gm.h>
#include "../quaternion.h"

typedef struct {
	vec3 collision_point1;
	vec3 collision_point2;
	vec3 normal;
} Collider_Contact;

typedef struct {
	u32* elements;
	vec3 normal;
} Collider_Convex_Hull_Face;

typedef struct {
	vec3* vertices;
	vec3* transformed_vertices;
	Collider_Convex_Hull_Face* faces;
	Collider_Convex_Hull_Face* transformed_faces;

	u32** vertex_to_faces;
	u32** vertex_to_neighbors;
	u32** face_to_neighbors;
} Collider_Convex_Hull;

typedef struct {
	r32 radius;
	vec3 center;
} Collider_Sphere;

typedef enum {
	COLLIDER_TYPE_SPHERE,
	COLLIDER_TYPE_CONVEX_HULL
} Collider_Type;

typedef struct {
	Collider_Type type;
	union {
		Collider_Convex_Hull convex_hull;
		Collider_Sphere sphere;
	};
} Collider;

// @NOTE: for simplicity (and speed), we don't deal with scaling in the colliders.
// therefore, if the object is scaled, the collider needs to be recreated (and the vertices should be already scaled when creating it)
Collider collider_convex_hull_create(const vec3* vertices, const u32* indices);
Collider collider_sphere_create(const r32 radius);

void colliders_update(Collider* colliders, vec3 translation, const Quaternion* rotation);
void colliders_destroy(Collider* collider);
mat3 colliders_get_default_inertia_tensor(Collider* colliders, r64 mass);
r64 colliders_get_bounding_sphere_radius(const Collider* colliders);
Collider_Contact* colliders_get_contacts(Collider* colliders1, Collider* colliders2);

#endif