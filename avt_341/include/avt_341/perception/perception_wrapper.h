//
// MATLAB Compiler: 8.4 (R2022a)
// Date: Mon Aug 21 17:31:57 2023
// Arguments:
// "-B""macro_default""-W""cpplib:perception_wrapper,all,version=1.0""-T""link:l
// ib""-d""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\perception_wrapper_P
// C2\for_testing""-v""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\percepti
// on_wrapper.m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\augmentIma
// geAndLabel.m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\calibratio
// n
// images""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\dataColorMap.m""
// -a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\dataPixelLabelIDs.m""-a"
// "C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\Deeplab_Node.mlx""-a""C:\Us
// ers\avmi\Documents\Code\AVMI\PerceptionDemo\dlnet_RUGD_singleTile.mat""-a""C:
// \Users\avmi\Documents\Code\AVMI\PerceptionDemo\ex_PerceptionAlgorithm.m""-a""
// C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\generateDatastores.mlx""-a""
// C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\generateDeeplabV3.mlx""-a""C
// :\Users\avmi\Documents\Code\AVMI\PerceptionDemo\generateDLN.mlx""-a""C:\Users
// \avmi\Documents\Code\AVMI\PerceptionDemo\partitionRUGDData.mlx""-a""C:\Users\
// avmi\Documents\Code\AVMI\PerceptionDemo\perception_wrapper.m""-a""C:\Users\av
// mi\Documents\Code\AVMI\PerceptionDemo\pixelLabelColorbar.m"
//

#ifndef perception_wrapper_h
#define perception_wrapper_h 1

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
#ifndef LIB_perception_wrapper_C_API 
#define LIB_perception_wrapper_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_perception_wrapper_C_API 
bool MW_CALL_CONV perception_wrapperInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_perception_wrapper_C_API 
bool MW_CALL_CONV perception_wrapperInitialize(void);

extern LIB_perception_wrapper_C_API 
void MW_CALL_CONV perception_wrapperTerminate(void);

extern LIB_perception_wrapper_C_API 
void MW_CALL_CONV perception_wrapperPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_perception_wrapper_C_API 
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

#ifdef EXPORTING_perception_wrapper
#define PUBLIC_perception_wrapper_CPP_API __declspec(dllexport)
#else
#define PUBLIC_perception_wrapper_CPP_API __declspec(dllimport)
#endif

#define LIB_perception_wrapper_CPP_API PUBLIC_perception_wrapper_CPP_API

#else

#if !defined(LIB_perception_wrapper_CPP_API)
#if defined(LIB_perception_wrapper_C_API)
#define LIB_perception_wrapper_CPP_API LIB_perception_wrapper_C_API
#else
#define LIB_perception_wrapper_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_perception_wrapper_CPP_API void MW_CALL_CONV perception_wrapper(int nargout, mwArray& costmapMatrix, const mwArray& rawImage, const mwArray& rawLidar, const mwArray& lidarWidth, const mwArray& lidarHeight, const mwArray& lidarPointStep, const mwArray& lidarRowStep, const mwArray& pose_point_x, const mwArray& pose_point_y, const mwArray& pose_point_z, const mwArray& pose_quat_w, const mwArray& pose_quat_x, const mwArray& pose_quat_y, const mwArray& pose_quat_z, const mwArray& grid_width, const mwArray& grid_height, const mwArray& grid_res, const mwArray& grid_llx, const mwArray& grid_lly, const mwArray& debug, const mwArray& CL_model, const mwArray& C_model, const mwArray& useProbabilisticGrid);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
