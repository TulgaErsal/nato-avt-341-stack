//
// MATLAB Compiler: 8.6 (R2023a)
// Date: Wed Aug 21 17:06:06 2024
// Arguments:
// "-B""macro_default""-W""cpplib:lib_tracker_wrapper,all,version=1.0""-T""link:
// lib""-d""C:\Users\Nic\Documents\git\PerceptionDemo\PerceptionDemo\tracker\lib
// _tracker_wrapper\lib_tracker_wrapper\for_testing""-v""C:\Users\Nic\Documents\
// git\PerceptionDemo\PerceptionDemo\tracker\trackerWrapper.m"
//

#ifndef lib_tracker_wrapper_h
#define lib_tracker_wrapper_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" { // sbcheck:ok:extern_c
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_lib_tracker_wrapper_C_API 
#define LIB_lib_tracker_wrapper_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_lib_tracker_wrapper_C_API 
bool MW_CALL_CONV lib_tracker_wrapperInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_lib_tracker_wrapper_C_API 
bool MW_CALL_CONV lib_tracker_wrapperInitialize(void);
extern LIB_lib_tracker_wrapper_C_API 
void MW_CALL_CONV lib_tracker_wrapperTerminate(void);

extern LIB_lib_tracker_wrapper_C_API 
void MW_CALL_CONV lib_tracker_wrapperPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_lib_tracker_wrapper_C_API 
bool MW_CALL_CONV mlxTrackerWrapper(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_lib_tracker_wrapper
#define PUBLIC_lib_tracker_wrapper_CPP_API __declspec(dllexport)
#else
#define PUBLIC_lib_tracker_wrapper_CPP_API __declspec(dllimport)
#endif

#define LIB_lib_tracker_wrapper_CPP_API PUBLIC_lib_tracker_wrapper_CPP_API

#else

#if !defined(LIB_lib_tracker_wrapper_CPP_API)
#if defined(LIB_lib_tracker_wrapper_C_API)
#define LIB_lib_tracker_wrapper_CPP_API LIB_lib_tracker_wrapper_C_API
#else
#define LIB_lib_tracker_wrapper_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_lib_tracker_wrapper_CPP_API void MW_CALL_CONV trackerWrapper(int nargout, mwArray& vehicleFound, mwArray& vehiclePosition, const mwArray& pcWidth, const mwArray& pcHeight, const mwArray& pcPointStep, const mwArray& pcRowStep, const mwArray& rawLidar, const mwArray& xMinBB, const mwArray& yMinBB, const mwArray& xMaxBB, const mwArray& yMaxBB, const mwArray& detectionScore, const mwArray& x, const mwArray& y, const mwArray& z, const mwArray& qw, const mwArray& qx, const mwArray& qy, const mwArray& qz);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
