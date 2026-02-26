//
// MATLAB Compiler: 24.1 (R2024a)
// Date: Sat Feb 28 14:23:21 2026
//

#ifndef lib_uab_perception_wrapper_h
#define lib_uab_perception_wrapper_h 1

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
#ifndef LIB_lib_uab_perception_wrapper_C_API 
#define LIB_lib_uab_perception_wrapper_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_lib_uab_perception_wrapper_C_API 
bool MW_CALL_CONV lib_uab_perception_wrapperInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_lib_uab_perception_wrapper_C_API 
bool MW_CALL_CONV lib_uab_perception_wrapperInitialize(void);
extern LIB_lib_uab_perception_wrapper_C_API 
void MW_CALL_CONV lib_uab_perception_wrapperTerminate(void);

extern LIB_lib_uab_perception_wrapper_C_API 
void MW_CALL_CONV lib_uab_perception_wrapperPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_lib_uab_perception_wrapper_C_API 
bool MW_CALL_CONV mlxPerception_wrapper(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                        *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_lib_uab_perception_wrapper
#define PUBLIC_lib_uab_perception_wrapper_CPP_API __declspec(dllexport)
#else
#define PUBLIC_lib_uab_perception_wrapper_CPP_API __declspec(dllimport)
#endif

#define LIB_lib_uab_perception_wrapper_CPP_API PUBLIC_lib_uab_perception_wrapper_CPP_API

#else

#if !defined(LIB_lib_uab_perception_wrapper_CPP_API)
#if defined(LIB_lib_uab_perception_wrapper_C_API)
#define LIB_lib_uab_perception_wrapper_CPP_API LIB_lib_uab_perception_wrapper_C_API
#else
#define LIB_lib_uab_perception_wrapper_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_lib_uab_perception_wrapper_CPP_API void MW_CALL_CONV perception_wrapper(int nargout, mwArray& terrainSubGrid, mwArray& terrainSubgridSize, mwArray& terrainModifiedCellIdxs, mwArray& obstacleSubGrid, mwArray& obstacleSubgridSize, mwArray& obstacleModifiedCellIdxs, const mwArray& img, const mwArray& pc, const mwArray& odom, const mwArray& cameraInfo, const mwArray& cameraToLidarTform, const mwArray& lidarToVboxTform, const mwArray& invertLidarZRot, const mwArray& grid_width, const mwArray& grid_height, const mwArray& cell_size, const mwArray& grid_llx, const mwArray& grid_lly);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
