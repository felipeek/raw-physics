#ifndef H_LIGHT_ARRAY
#define H_LIGHT_ARRAY

/*
    Author: Pedro Sassen Veiga
    The MIT License

    This library is C89 compatible

    ----------------------------------------------------------------------------------

    Warning: since the array_push function modifies the original array pointer,
    be aware that passing a light_array as a parameter to a function that pushes
    on it will not work by default. To do this you must pass the address of the original 
    array, otherwise all previous references to the array will keep the already freed
    memory.

    ----------------------------------------------------------------------------------

    define LIGHT_ARRAY_NO_CRT if you don't want the c runtime library included
    if that is defined, you must provide implementations for the following functions:

    void* calloc(size_t num, size_t size)
    void* realloc(void* ptr, size_t new_size)
    void  free(void* block)
    void* memmove(void* dest, void* src, size_t count)

    ----------------------------------------------------------------------------------

    Usage:
    An example usage of creating, pushing, popping, removing and freeing an array:

    #include "light_array.h"

    int main(int argc, char** argv)
    {
        int* buffer = array_new(int);

        array_push(buffer, 1);
        array_push(buffer, 2);
        array_push(buffer, 3);
        array_push(buffer, 4);
        array_push(buffer, 5);
        
        int five = array_pop(buffer);

        array_remove(buffer, 1);
        array_free(buffer);

        return 0;
    }

    array_remove is an unordered remove, if you want an ordered version, which is more expensive
    use array_remove_ordered instead

    To get the length of an array just use array_length, this will work as an lvalue, meaning you
    can do as follows:

    array_length(buffer)++;
    array_length(buffer) = 0;

    And it should work as expected, incrementing the array length artificially or setting it to
    zero, which is also the behaviour of array_clear

*/

#if !defined(LIGHT_ARRAY_NO_CRT)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

typedef struct {
    size_t capacity;
    size_t length;
} Dynamic_ArrayBase;

#define LIGHT_ARRAY_MIN(x, y) (y ^ ((x ^ y) & -(x < y)))
#define LIGHT_ARRAY_MAX(x, y) (x ^ ((x ^ y) & -(x < y)))
#if defined(__GNUC__)
#define LIGHT_ARRAY_API __attribute__((unused))
#else
#define LIGHT_ARRAY_API
#endif

/* gets the base of the array where the capacity and length info are, not recommended to use (internal of the library). */
#define array_base(A) ((Dynamic_ArrayBase*)(((char*)(A)) - sizeof(Dynamic_ArrayBase)))
/* gets the length of the array. */
#define array_length(A) array_base(A)->length
/* gets the capacity currently allocated of the array, that is, the array won't grow while the capacity isn't reached. */
#define array_capacity(A) array_base(A)->capacity

#if defined(__cplusplus)
/* creates a new array of type T */
#define array_new(T) (T*)((char*)&(((Dynamic_ArrayBase*)calloc(1, sizeof(Dynamic_ArrayBase) + sizeof(T)))->capacity = 1) + sizeof(Dynamic_ArrayBase))
#else
static LIGHT_ARRAY_API void* array_dyn_allocate(size_t size) {
    void* res = calloc(1, size);
    ((Dynamic_ArrayBase*)res)->capacity = 1;
    return (void*)((char*)res + sizeof(Dynamic_ArrayBase));
}
static LIGHT_ARRAY_API void* array_dyn_allocate_capacity(size_t size_element, size_t capacity) {
    void* res = calloc(1, size_element * capacity + sizeof(Dynamic_ArrayBase));
    ((Dynamic_ArrayBase*)res)->capacity = capacity;
    return (void*)((char*)res + sizeof(Dynamic_ArrayBase));
}
#define array_new(T) array_dyn_allocate(sizeof(T) + sizeof(Dynamic_ArrayBase))
/* creates a new array of type T with starting capacity L */
#define array_new_len(T, L) array_dyn_allocate_capacity(sizeof(T), L)
#endif

/* given an array created by array_new and a value (rvalue) of the base type of the array, puts that value in the last
   position of the current array, it allocates memory automatically when the capacity is reached. The policy to allocate
   is exponential (doubles every allocation). */
#define array_push(A, V) ((array_length(A) == array_capacity(A)) \
    ? *((void**)&(A)) = (void*)((Dynamic_ArrayBase*)realloc((Dynamic_ArrayBase*)(A) - 1, sizeof(Dynamic_ArrayBase) + sizeof(*(A)) * array_capacity(A) * 2) + 1), \
    array_capacity(A) = array_capacity(A) * 2 : 0, \
    (A)[array_length(A)++] = (V))

/* given an array created by array_new and an integer value V. Allocates on top of the existing memory V additional
   bytes of memory, changing the array capacity but not its length.
 */
#define array_allocate(A, V) ((array_length(A) + (V) >= array_capacity(A)) \
    ? *((void**)&(A)) = (void*)((Dynamic_ArrayBase*)realloc((Dynamic_ArrayBase*)(A) - 1, sizeof(Dynamic_ArrayBase) + sizeof(*(A)) * (array_length(A) + (V))) + 1), \
    array_capacity(A) = (array_length(A) + (V)) : 0)

/* inserts into a given array A the value V (rvalue of type of the array) in the index I and pushes every value after
   the index forward in the array. */
#define array_insert(A, V, I) array_push(A, V), memmove((A) + (I) + 1, (A) + (I), sizeof(*A) * (array_length(A) - (I) - 1)), (A)[I] = (V)

/* returns and removes the last value in the array. */
#define array_pop(A) (array_length(A) > 0) ? (A)[--array_length(A)] : 0

/* frees the memory of the array, the array pointer becomes invalid. */
#define array_free(A) free(array_base(A))

/* clears the array but keeps the current capacity, that is, keeps the memory allocated. */
#define array_clear(A) array_length(A) = 0

/* removes the value in a given Index from the array moving every value after it back. */
#define array_remove_ordered(A, Index) ((Index) < (array_length(A) - 1) && array_length(A) > 0) ? \
    memmove((A) + Index, (A) + Index + 1, (array_length(A) - 1 - Index) * sizeof(*A)) : 0, array_length(A)--

/* removes the last value of the array in an unordered way, putting the last element in its place. */
#define array_remove(A, Index) (array_length(A)--, (A)[Index] = (A)[array_length(A)])

/* copies an array to a new one, its capacity and length are also preserved */
#define array_copy(A) ((void*)((Dynamic_ArrayBase*) \
    memmove((Dynamic_ArrayBase*)array_dyn_allocate_capacity(sizeof(*A), array_capacity(A)) - 1, \
    array_base(A), sizeof(Dynamic_ArrayBase) + array_length(A) * sizeof(*A)) + 1))

/* appends all elements from A2 at the end of A1. A2 remains unchanged */
#define array_append(A1, A2) (array_length(A1) + array_length(A2) >= array_capacity(A1)) \
    ? *((void**)&(A1)) = (void*)((Dynamic_ArrayBase*)realloc((Dynamic_ArrayBase*)(A1) - 1, sizeof(Dynamic_ArrayBase) + sizeof(*(A1)) * (array_capacity(A1) + array_capacity(A2)) * 2) + 1), \
    array_capacity(A1) = (array_capacity(A1) + array_capacity(A2)) * 2 : 0, \
    memcpy(A1 + array_length(A1), A2, array_length(A2) * sizeof(*(A1))), \
    array_length(A1) += array_length(A2)

#endif /* H_LIGHT_ARRAY */