#ifndef __c2_DroneControllerCollisionAvoidance_h__
#define __c2_DroneControllerCollisionAvoidance_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_SL_Bus_ROSVariableLengthArrayInfo_tag
#define struct_SL_Bus_ROSVariableLengthArrayInfo_tag

struct SL_Bus_ROSVariableLengthArrayInfo_tag
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif                                 /*struct_SL_Bus_ROSVariableLengthArrayInfo_tag*/

#ifndef typedef_c2_SL_Bus_ROSVariableLengthArrayInfo
#define typedef_c2_SL_Bus_ROSVariableLengthArrayInfo

typedef struct SL_Bus_ROSVariableLengthArrayInfo_tag
  c2_SL_Bus_ROSVariableLengthArrayInfo;

#endif                                 /*typedef_c2_SL_Bus_ROSVariableLengthArrayInfo*/

#ifndef struct_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time_tag
#define struct_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time_tag

struct SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time_tag
{
  real_T Sec;
  real_T Nsec;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time
#define typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time

typedef struct SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time_tag
  c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time*/

#ifndef struct_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header_tag
#define struct_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header_tag

struct SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header_tag
{
  uint32_T Seq;
  uint8_T FrameId[128];
  c2_SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time Stamp;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header
#define typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header

typedef struct SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header_tag
  c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_Point_3iqyla_tag
#define struct_SL_Bus_DroneControllerCollisionA_Point_3iqyla_tag

struct SL_Bus_DroneControllerCollisionA_Point_3iqyla_tag
{
  real_T X;
  real_T Y;
  real_T Z;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_Point_3iqyla_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla
#define typedef_c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla

typedef struct SL_Bus_DroneControllerCollisionA_Point_3iqyla_tag
  c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw_tag
#define struct_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw_tag

struct SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw_tag
{
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw
#define typedef_c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw

typedef struct SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw_tag
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw*/

#ifndef struct_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose_tag
#define struct_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose_tag

struct SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose_tag
{
  c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla Position;
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw Orientation;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose
#define typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose

typedef struct SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose_tag
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u_tag
#define struct_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u_tag

struct SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u_tag
{
  real_T Covariance[36];
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose Pose;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u
#define typedef_c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u

typedef struct SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u_tag
  c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi_tag
#define struct_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi_tag

struct SL_Bus_DroneControllerCollisionA_Vector3_wtscbi_tag
{
  real_T X;
  real_T Y;
  real_T Z;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi
#define typedef_c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi

typedef struct SL_Bus_DroneControllerCollisionA_Vector3_wtscbi_tag
  c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_Twist_3geo7b_tag
#define struct_SL_Bus_DroneControllerCollisionA_Twist_3geo7b_tag

struct SL_Bus_DroneControllerCollisionA_Twist_3geo7b_tag
{
  c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi Linear;
  c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi Angular;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_Twist_3geo7b_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b
#define typedef_c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b

typedef struct SL_Bus_DroneControllerCollisionA_Twist_3geo7b_tag
  c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8_tag
#define struct_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8_tag

struct SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8_tag
{
  real_T Covariance[36];
  c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b Twist;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8
#define typedef_c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8

typedef struct SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8_tag
  c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8*/

#ifndef struct_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry_tag
#define struct_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry_tag

struct SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry_tag
{
  uint8_T ChildFrameId[128];
  c2_SL_Bus_ROSVariableLengthArrayInfo ChildFrameId_SL_Info;
  c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header Header;
  c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u Pose;
  c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 Twist;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry
#define typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry

typedef struct SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry_tag
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry*/

#ifndef struct_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa_tag
#define struct_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa_tag

struct SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa_tag
{
  real32_T AngleMin;
  real32_T AngleMax;
  real32_T AngleIncrement;
  real32_T TimeIncrement;
  real32_T ScanTime;
  real32_T RangeMin;
  real32_T RangeMax;
  real32_T Ranges[2048];
  c2_SL_Bus_ROSVariableLengthArrayInfo Ranges_SL_Info;
  real32_T Intensities[2048];
  c2_SL_Bus_ROSVariableLengthArrayInfo Intensities_SL_Info;
  c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header Header;
};

#endif                                 /*struct_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa_tag*/

#ifndef typedef_c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa
#define typedef_c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa

typedef struct SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa_tag
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa;

#endif                                 /*typedef_c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa*/

#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray__common*/

#ifndef typedef_c2_emxArray__common
#define typedef_c2_emxArray__common

typedef struct emxArray__common c2_emxArray__common;

#endif                                 /*typedef_c2_emxArray__common*/

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  real32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real32_T*/

#ifndef typedef_c2_emxArray_real32_T
#define typedef_c2_emxArray_real32_T

typedef struct emxArray_real32_T c2_emxArray_real32_T;

#endif                                 /*typedef_c2_emxArray_real32_T*/

#ifndef typedef_c2_struct_T
#define typedef_c2_struct_T

typedef struct {
  char_T f1[6];
  char_T f2[6];
} c2_struct_T;

#endif                                 /*typedef_c2_struct_T*/

#ifndef typedef_c2_b_struct_T
#define typedef_c2_b_struct_T

typedef struct {
  char_T f1[8];
  char_T f2[4];
  char_T f3[2];
  char_T f4[5];
  real_T f5;
} c2_b_struct_T;

#endif                                 /*typedef_c2_b_struct_T*/

#ifndef struct_tag_sbz5YcIbiGQmS5RgvQnKlSH
#define struct_tag_sbz5YcIbiGQmS5RgvQnKlSH

struct tag_sbz5YcIbiGQmS5RgvQnKlSH
{
  real32_T RangeLimits[2];
};

#endif                                 /*struct_tag_sbz5YcIbiGQmS5RgvQnKlSH*/

#ifndef typedef_c2_sbz5YcIbiGQmS5RgvQnKlSH
#define typedef_c2_sbz5YcIbiGQmS5RgvQnKlSH

typedef struct tag_sbz5YcIbiGQmS5RgvQnKlSH c2_sbz5YcIbiGQmS5RgvQnKlSH;

#endif                                 /*typedef_c2_sbz5YcIbiGQmS5RgvQnKlSH*/

#ifndef typedef_SFc2_DroneControllerCollisionAvoidanceInstanceStruct
#define typedef_SFc2_DroneControllerCollisionAvoidanceInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_DroneControllerCollisionAvoidance;
  void *c2_fEmlrtCtx;
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *c2_odomMsg;
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *c2_laserMsg;
  real_T (*c2_posePub)[4];
  real_T (*c2_target)[4];
} SFc2_DroneControllerCollisionAvoidanceInstanceStruct;

#endif                                 /*typedef_SFc2_DroneControllerCollisionAvoidanceInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_DroneControllerCollisionAvoidance_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_DroneControllerCollisionAvoidance_get_check_sum(mxArray *plhs[]);
extern void c2_DroneControllerCollisionAvoidance_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
