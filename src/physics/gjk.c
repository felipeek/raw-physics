#include "gjk.h"
#include <light_array.h>
#include <float.h>
#include <math.h>

u32 gjk_get_support_point_index(vec3* shape, vec3 direction) {
	u32 selected_index;
	r32 max_dot = -FLT_MAX;
	for (u32 i = 0; i < array_length(shape); ++i) {
		r32 dot = gm_vec3_dot(shape[i], direction);
		if (dot > max_dot) {
			selected_index = i;
			max_dot = dot;
		}
	}

	return selected_index;
}

vec3 gjk_get_support_point(vec3* shape, vec3 direction) {
	u32 selected_index = gjk_get_support_point_index(shape, direction);
	return shape[selected_index];
}

vec3 gjk_get_support_point_of_minkowski_difference(vec3* shape1, vec3* shape2, vec3 direction) {
	vec3 support1 = gjk_get_support_point(shape1, direction);
	vec3 support2 = gjk_get_support_point(shape2, gm_vec3_scalar_product(-1.0f, direction));

	return gm_vec3_subtract(support1, support2);
}

static void add_to_simplex(GJK_Simplex* simplex, vec3 point) {
	switch (simplex->num) {
		case 1: {
			simplex->b = simplex->a;
			simplex->a = point;
		} break;
		case 2: {
			simplex->c = simplex->b;
			simplex->b = simplex->a;
			simplex->a = point;
		} break;
		case 3: {
			simplex->d = simplex->c;
			simplex->c = simplex->b;
			simplex->b = simplex->a;
			simplex->a = point;
		} break;
		default: {
			assert(0);
		} break;
	}

	++simplex->num;
}

static vec3 triple_cross(vec3 a, vec3 b, vec3 c) {
	return gm_vec3_cross(gm_vec3_cross(a, b), c);
}

static boolean do_simplex_2(GJK_Simplex* simplex, vec3* direction) {
	vec3 a = simplex->a; // the last point added
	vec3 b = simplex->b;

	vec3 ao = gm_vec3_negative(a);
	vec3 ab = gm_vec3_subtract(b, a);

	if (gm_vec3_dot(ab, ao) >= 0.0f) {
		simplex->a = a;
		simplex->b = b;
		simplex->num = 2;
		*direction = triple_cross(ab, ao, ab);
	} else {
		simplex->a = a;
		simplex->num = 1;
		*direction = ao;
	}

	return false;
}

static boolean do_simplex_3(GJK_Simplex* simplex, vec3* direction) {
	vec3 a = simplex->a; // the last point added
	vec3 b = simplex->b;
	vec3 c = simplex->c;

	vec3 ao = gm_vec3_negative(a);
	vec3 ab = gm_vec3_subtract(b, a);
	vec3 ac = gm_vec3_subtract(c, a);
	vec3 abc = gm_vec3_cross(ab, ac);

	if (gm_vec3_dot(gm_vec3_cross(abc, ac), ao) >= 0.0f) {
		if (gm_vec3_dot(ac, ao) >= 0.0f) {
			// AC region
			simplex->a = a;
			simplex->b = c;
			simplex->num = 2;
			*direction = triple_cross(ac, ao, ac);
		} else {
			if (gm_vec3_dot(ab, ao) >= 0.0f) {
				// AB region
				simplex->a = a;
				simplex->b = b;
				simplex->num = 2;
				*direction = triple_cross(ab, ao, ab);
			} else {
				// A region
				simplex->a = a;
				*direction = ao;
			}
		}
	} else {
		if (gm_vec3_dot(gm_vec3_cross(ab, abc), ao) >= 0.0f) {
			if (gm_vec3_dot(ab, ao) >= 0.0f) {
				// AB region
				simplex->a = a;
				simplex->b = b;
				simplex->num = 2;
				*direction = triple_cross(ab, ao, ab);
			} else {
				// A region
				simplex->a = a;
				*direction = ao;
			}
		} else {
			if (gm_vec3_dot(abc, ao) >= 0.0f) {
				// ABC region ("up")
				simplex->a = a;
				simplex->b = b;
				simplex->c = c;
				simplex->num = 3;
				*direction = abc;
			} else {
				// ABC region ("down")
				simplex->a = a;
				simplex->b = c;
				simplex->c = b;
				simplex->num = 3;
				*direction = gm_vec3_negative(abc);
			}
		}
	}

	return false;
}

static boolean do_simplex_4(GJK_Simplex* simplex, vec3* direction) {
	vec3 a = simplex->a; // the last point added
	vec3 b = simplex->b;
	vec3 c = simplex->c;
	vec3 d = simplex->d;

	vec3 ao = gm_vec3_negative(a);
	vec3 ab = gm_vec3_subtract(b, a);
	vec3 ac = gm_vec3_subtract(c, a);
	vec3 ad = gm_vec3_subtract(d, a);
	vec3 abc = gm_vec3_cross(ab, ac);
	vec3 acd = gm_vec3_cross(ac, ad);
	vec3 adb = gm_vec3_cross(ad, ab);

	unsigned char plane_information = 0x0;

	if (gm_vec3_dot(abc, ao) >= 0.0f) {
		plane_information |= 0x1;
	}
	if (gm_vec3_dot(acd, ao) >= 0.0f) {
		plane_information |= 0x2;
	}
	if (gm_vec3_dot(adb, ao) >= 0.0f) {
		plane_information |= 0x4;
	}

	switch (plane_information) {
		case 0x0: {
			// Intersection
			return true;
		} break;
		case 0x1: {
			// Triangle ABC
			if (gm_vec3_dot(gm_vec3_cross(abc, ac), ao) >= 0.0f) {
				if (gm_vec3_dot(ac, ao) >= 0.0f) {
					// AC region
					simplex->a = a;
					simplex->b = c;
					simplex->num = 2;
					*direction = triple_cross(ac, ao, ac);
				} else {
					if (gm_vec3_dot(ab, ao) >= 0.0f) {
						// AB region
						simplex->a = a;
						simplex->b = b;
						simplex->num = 2;
						*direction = triple_cross(ab, ao, ab);
					} else {
						// A region
						simplex->a = a;
						*direction = ao;
					}
				}
			} else {
				if (gm_vec3_dot(gm_vec3_cross(ab, abc), ao) >= 0.0f) {
					if (gm_vec3_dot(ab, ao) >= 0.0f) {
						// AB region
						simplex->a = a;
						simplex->b = b;
						simplex->num = 2;
						*direction = triple_cross(ab, ao, ab);
					} else {
						// A region
						simplex->a = a;
						*direction = ao;
					}
				} else {
					// ABC region
					simplex->a = a;
					simplex->b = b;
					simplex->c = c;
					simplex->num = 3;
					*direction = abc;
				}
			}
		} break;
		case 0x2: {
			// Triangle ACD
			if (gm_vec3_dot(gm_vec3_cross(acd, ad), ao) >= 0.0f) {
				if (gm_vec3_dot(ad, ao) >= 0.0f) {
					// AD region
					simplex->a = a;
					simplex->b = d;
					simplex->num = 2;
					*direction = triple_cross(ad, ao, ad);
				} else {
					if (gm_vec3_dot(ac, ao) >= 0.0f) {
						// AC region
						simplex->a = a;
						simplex->b = c;
						simplex->num = 2;
						*direction = triple_cross(ab, ao, ab);
					} else {
						// A region
						simplex->a = a;
						*direction = ao;
					}
				}
			} else {
				if (gm_vec3_dot(gm_vec3_cross(ac, acd), ao) >= 0.0f) {
					if (gm_vec3_dot(ac, ao) >= 0.0f) {
						// AC region
						simplex->a = a;
						simplex->b = c;
						simplex->num = 2;
						*direction = triple_cross(ac, ao, ac);
					} else {
						// A region
						simplex->a = a;
						*direction = ao;
					}
				} else {
					// ACD region
					simplex->a = a;
					simplex->b = c;
					simplex->c = d;
					simplex->num = 3;
					*direction = acd;
				}
			}
		} break;
		case 0x3: {
			// Line AC
			if (gm_vec3_dot(ac, ao) >= 0.0f) {
				simplex->a = a;
				simplex->b = c;
				simplex->num = 2;
				*direction = triple_cross(ac, ao, ac);
			} else {
				simplex->a = a;
				simplex->num = 1;
				*direction = ao;
			}

		} break;
		case 0x4: {
			// Triangle ADB
			if (gm_vec3_dot(gm_vec3_cross(adb, ab), ao) >= 0.0f) {
				if (gm_vec3_dot(ab, ao) >= 0.0f) {
					// AB region
					simplex->a = a;
					simplex->b = b;
					simplex->num = 2;
					*direction = triple_cross(ab, ao, ab);
				} else {
					if (gm_vec3_dot(ad, ao) >= 0.0f) {
						// AD region
						simplex->a = a;
						simplex->b = d;
						simplex->num = 2;
						*direction = triple_cross(ad, ao, ad);
					} else {
						// A region
						simplex->a = a;
						*direction = ao;
					}
				}
			} else {
				if (gm_vec3_dot(gm_vec3_cross(ad, adb), ao) >= 0.0f) {
					if (gm_vec3_dot(ad, ao) >= 0.0f) {
						// AD region
						simplex->a = a;
						simplex->b = d;
						simplex->num = 2;
						*direction = triple_cross(ad, ao, ad);
					} else {
						// A region
						simplex->a = a;
						*direction = ao;
					}
				} else {
					// ADB region
					simplex->a = a;
					simplex->b = d;
					simplex->c = b;
					simplex->num = 3;
					*direction = adb;
				}
			}
		} break;
		case 0x5: {
			// Line AB
			if (gm_vec3_dot(ab, ao) >= 0.0f) {
				simplex->a = a;
				simplex->b = b;
				simplex->num = 2;
				*direction = triple_cross(ab, ao, ab);
			} else {
				simplex->a = a;
				simplex->num = 1;
				*direction = ao;
			}
		} break;
		case 0x6: {
			// Line AD
			if (gm_vec3_dot(ad, ao) >= 0.0f) {
				simplex->a = a;
				simplex->b = d;
				simplex->num = 2;
				*direction = triple_cross(ad, ao, ad);
			} else {
				simplex->a = a;
				simplex->num = 1;
				*direction = ao;
			}
		} break;
		case 0x7: {
			// Point A
			simplex->a = a;
			simplex->num = 1;
			*direction = ao;
		} break;
	}

	return false;
}

static boolean do_simplex(GJK_Simplex* simplex, vec3* direction) {
	switch (simplex->num) {
		case 2: return do_simplex_2(simplex, direction);
		case 3: return do_simplex_3(simplex, direction);
		case 4: return do_simplex_4(simplex, direction);
	}

	assert(0);
}

boolean gjk_collides(vec3* shape1, vec3* shape2, GJK_Simplex* _simplex) {
	GJK_Simplex simplex;

	simplex.a = gjk_get_support_point_of_minkowski_difference(shape1, shape2, (vec3){0.0f, 0.0f, 1.0f});
	simplex.num = 1; 

	vec3 direction = gm_vec3_scalar_product(-1.0f, simplex.a);

	for (u32 i = 0; i < 100; ++i) {
		vec3 next_point = gjk_get_support_point_of_minkowski_difference(shape1, shape2, direction);
		
		if (gm_vec3_dot(next_point, direction) < 0.0f) {
			// No intersection.
			return false;
		}

		add_to_simplex(&simplex, next_point);

		if (do_simplex(&simplex, &direction)) {
			// Intersection.
			if (_simplex) {
				*_simplex = simplex;
			}
			return true;
		}
	}

	//printf("GJK did not converge.\n");
	return false;
}