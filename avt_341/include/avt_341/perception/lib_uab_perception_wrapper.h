//
// MATLAB Compiler: 25.2 (R2025b)
// Date: Fri Jun 26 12:47:19 2026
// Arguments:
// "-B""macro_default""-W""cpplib:lib_uab_perception_wrapper,legacy,version=1.0.
// 0.0""-T""link:lib""-d""/home/matlab/lib_uab_perception_wrapper_out""-a""C:\Us
// ers\stefan\Documents\MATLAB\uab-terrain-segmentation\calibration_images""-a""
// C:\Users\stefan\Documents\MATLAB\uab-terrain-segmentation\main.m""-a""C:\User
// s\stefan\Documents\MATLAB\uab-terrain-segmentation\percep2occ.m""-a""C:\Users
// \stefan\Documents\MATLAB\uab-terrain-segmentation\perception.m""-a""C:\Users\
// stefan\Documents\MATLAB\uab-terrain-segmentation\models\sae_net_5class_RUGD_G
// OD_544_688_package.mat""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-segm
// entation\models\CAE.m""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-segme
// ntation\models\ILidarProcessingAlgorithm.m""-a""C:\Users\stefan\Documents\MAT
// LAB\uab-terrain-segmentation\models\ISegmentationAlgorithm.m""-a""C:\Users\st
// efan\Documents\MATLAB\uab-terrain-segmentation\models\KNN.m""-a""C:\Users\ste
// fan\Documents\MATLAB\uab-terrain-segmentation\models\KNN_weighted.mat""-a""C:
// \Users\stefan\Documents\MATLAB\uab-terrain-segmentation\models\LESN.m""-a""C:
// \Users\stefan\Documents\MATLAB\uab-terrain-segmentation\models\LESN_32R_Z_4k.
// mat""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-segmentation\models\Sen
// sorInfo.m""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-segmentation\func
// tions\data_split.m""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-segmenta
// tion\functions\GetCameraIntrinsics.m""-a""C:\Users\stefan\Documents\MATLAB\ua
// b-terrain-segmentation\functions\GetSensorTform.m""-a""C:\Users\stefan\Docume
// nts\MATLAB\uab-terrain-segmentation\functions\globalParams.m""-a""C:\Users\st
// efan\Documents\MATLAB\uab-terrain-segmentation\functions\GridBuilder.m""-a""C
// :\Users\stefan\Documents\MATLAB\uab-terrain-segmentation\functions\ParseRosOd
// ometry.m""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-segmentation\funct
// ions\reservoir_update.m""-a""C:\Users\stefan\Documents\MATLAB\uab-terrain-seg
// mentation\functions\reservoir_update_cesn.m""-a""C:\Users\stefan\Documents\MA
// TLAB\uab-terrain-segmentation\functions\standalone_CESN_v2.m""-Z""autodetect"
// "C:\Users\stefan\Documents\MATLAB\uab-terrain-segmentation\perception_wrapper
// .m"
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

extern LIB_lib_uab_perception_wrapper_CPP_API void MW_CALL_CONV perception_wrapper(int nargout, mwArray& terrainSubGrid, mwArray& terrainSubgridSize, mwArray& terrainModifiedCellIdxs, mwArray& obstacleSubGrid, mwArray& obstacleSubgridSize, mwArray& obstacleModifiedCellIdxs, mwArray& debugSegmentationMask, const mwArray& img, const mwArray& xyz, const mwArray& odom, const mwArray& cameraInfo, const mwArray& lidarToCameraTform, const mwArray& lidarToBaseLinkTform, const mwArray& brightnessOffset, const mwArray& grid_width, const mwArray& grid_height, const mwArray& cell_size, const mwArray& grid_llx, const mwArray& grid_lly, const mwArray& debugVisSegmentation);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
