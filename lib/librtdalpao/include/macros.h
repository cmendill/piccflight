#ifndef _macros_h
#define _macros_h
/* this header contains some preprocessor functions and other #defines used throughout */

/* pre-processor number to string function */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/* pre-processor array length function */
#define N_ELEMENTS(array) (sizeof(array)/sizeof(array[0]))

/* pre-processor ceiling function */
#define CEILING(x,y) (((x) + (y) - 1) / (y))

/* pre-processor floor function */
#define FLOOR(x,y) ((x) / (y))

#endif /* _macros_h */