//
// MATLAB Compiler: 8.4 (R2022a)
// Date: Wed Dec 21 15:27:19 2022
// Arguments:
// "-B""macro_default""-W""cpplib:perception_wrapper,all,version=1.0""-T""link:l
// ib""-d""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\perception_wrapper_P
// C2\for_testing""-v""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\percepti
// on_wrapper.m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\augmentIma
// geAndLabel.m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\dataColorM
// ap.m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\dataPixelLabelIDs.
// m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\Deeplab_Node.mlx""-a"
// "C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\dlnet10.mat""-a""C:\Users\a
// vmi\Documents\Code\AVMI\PerceptionDemo\ex_PerceptionAlgorithm.m""-a""C:\Users
// \avmi\Documents\Code\AVMI\PerceptionDemo\generateDatastores.mlx""-a""C:\Users
// \avmi\Documents\Code\AVMI\PerceptionDemo\generateDeeplabV3.mlx""-a""C:\Users\
// avmi\Documents\Code\AVMI\PerceptionDemo\generateDLN.mlx""-a""C:\Users\avmi\Do
// cuments\Code\AVMI\PerceptionDemo\netTraff_adam_cls4_mbs128_150x240_8core.mat"
// "-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\netTraff_adam_cls5_rugd
// _fixed.mat""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\partitionRUG
// DData.mlx""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\perception_wr
// apper.m""-a""C:\Users\avmi\Documents\Code\AVMI\PerceptionDemo\pixelLabelColor
// bar.m"
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

extern LIB_perception_wrapper_CPP_API void MW_CALL_CONV perception_wrapper(int nargout, mwArray& costNow, const mwArray& rawImage, const mwArray& rawLidar, const mwArray& pose_point_x, const mwArray& pose_point_y, const mwArray& pose_point_z, const mwArray& pose_quat_w, const mwArray& pose_quat_x, const mwArray& pose_quat_y, const mwArray& pose_quat_z);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
