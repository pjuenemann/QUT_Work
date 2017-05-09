/* Include files */

#include "DroneControllerCollisionAvoidance_sfun.h"
#include "c2_DroneControllerCollisionAvoidance.h"
#include <string.h>
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "DroneControllerCollisionAvoidance_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[13] = { "originTarget",
  "lastPositions", "pose", "minDist", "data", "v", "u", "target", "nargin",
  "nargout", "odomMsg", "laserMsg", "posePub" };

static const char * c2_b_debug_family_names[11] = { "poseMsg", "xpos", "ypos",
  "zpos", "quat", "angles", "theta", "nargin", "nargout", "odomMsg", "pose" };

static const char * c2_c_debug_family_names[15] = { "defaults", "R", "validIdx",
  "angles", "cartAngles", "x", "y", "cart", "th", "nargin", "nargout", "obj",
  "pose", "minDist", "dataWorld" };

static const char * c2_d_debug_family_names[6] = { "nargin", "nargout", "k_att",
  "q", "target", "F_att" };

static const char * c2_e_debug_family_names[12] = { "d", "p0", "k_att", "k_rep",
  "F", "nargin", "nargout", "target", "pose", "pose_obs", "minDist", "u" };

static emlrtRTEInfo c2_emlrtRTEI = { 27,/* lineNo */
  33,                                  /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_b_emlrtRTEI = { 32,/* lineNo */
  15,                                  /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_c_emlrtRTEI = { 33,/* lineNo */
  17,                                  /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_d_emlrtRTEI = { 44,/* lineNo */
  10,                                  /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2017a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtRTEInfo c2_e_emlrtRTEI = { 44,/* lineNo */
  5,                                   /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2017a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtRTEInfo c2_f_emlrtRTEI = { 33,/* lineNo */
  1,                                   /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_g_emlrtRTEI = { 253,/* lineNo */
  19,                                  /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2017a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtRTEInfo c2_h_emlrtRTEI = { 34,/* lineNo */
  1,                                   /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_i_emlrtRTEI = { 35,/* lineNo */
  1,                                   /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_j_emlrtRTEI = { 36,/* lineNo */
  9,                                   /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_k_emlrtRTEI = { 36,/* lineNo */
  24,                                  /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_l_emlrtRTEI = { 37,/* lineNo */
  9,                                   /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

static emlrtRTEInfo c2_m_emlrtRTEI = { 98,/* lineNo */
  9,                                   /* colNo */
  "colon",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2017a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pName */
};

static emlrtRTEInfo c2_n_emlrtRTEI = { 37,/* lineNo */
  24,                                  /* colNo */
  "Subsystem1/MATLAB Function",        /* fName */
  "#DroneControllerCollisionAvoidance:318"/* pName */
};

/* Function Declarations */
static void initialize_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void initialize_params_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void enable_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void disable_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void set_sim_state_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_st);
static void finalize_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void sf_gateway_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void mdl_start_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void c2_chartstep_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void initSimStructsc2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_emlrt_marshallOut
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   char_T c2_u[30]);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_b_posePub, const char_T *c2_identifier, real_T c2_y[4]);
static void c2_b_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_d_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real32_T c2_f_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *c2_y);
static void c2_h_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, uint8_T c2_y[128]);
static c2_SL_Bus_ROSVariableLengthArrayInfo c2_i_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static uint32_T c2_j_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_k_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *c2_y);
static c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time
  c2_l_emlrt_marshallIn(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_m_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *c2_y);
static void c2_n_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[36]);
static c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose
  c2_o_emlrt_marshallIn(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla c2_p_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw
  c2_q_emlrt_marshallIn(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_r_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *c2_y);
static c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b c2_s_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi c2_t_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_u_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *c2_y);
static void c2_v_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y[2048]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_size[1]);
static void c2_w_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
   int32_T c2_y_size[1]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_size[1]);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, real32_T
  c2_inData_data[], int32_T c2_inData_size[1]);
static void c2_x_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y_data[],
   int32_T c2_y_size[1]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real32_T c2_outData_data[],
  int32_T c2_outData_size[1]);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, real32_T
  c2_inData_data[], int32_T c2_inData_size[2]);
static void c2_y_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y_data[],
   int32_T c2_y_size[2]);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real32_T c2_outData_data[],
  int32_T c2_outData_size[2]);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid,
  c2_emxArray_real32_T *c2_inData);
static void c2_ab_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, c2_emxArray_real32_T
   *c2_y);
static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, c2_emxArray_real32_T *c2_outData);
static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_bb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, boolean_T c2_y[2048]);
static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static c2_sbz5YcIbiGQmS5RgvQnKlSH c2_cb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_db_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y[2]);
static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_quat2eul(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, real_T c2_q[4], real_T c2_eul[3]);
static void c2_error(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
                     *chartInstance);
static void c2_b_error(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance);
static void c2_c_error(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance);
static void c2_power(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
                     *chartInstance, real_T c2_a[3], real_T c2_y[3]);
static void c2_findObstacle(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *c2_obj,
  real_T c2_pose[4], real32_T *c2_minDist, real_T c2_dataWorld[2]);
static void c2_indexShapeCheck
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, int32_T
   c2_matrixSize, int32_T c2_indexSize[2]);
static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_eb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_odomMsg_bus_io(void *chartInstanceVoid, void *c2_pData);
static const mxArray *c2_laserMsg_bus_io(void *chartInstanceVoid, void *c2_pData);
static uint8_T c2_fb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_DroneControllerCollisionAvoidance, const char_T
   *c2_identifier);
static uint8_T c2_gb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_emxEnsureCapacity
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance,
   c2_emxArray__common *c2_emxArray, int32_T c2_oldNumel, uint32_T
   c2_elementSize, const emlrtRTEInfo *c2_srcLocation);
static void c2_emxInit_real32_T
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance,
   c2_emxArray_real32_T **c2_pEmxArray, int32_T c2_numDimensions, const
   emlrtRTEInfo *c2_srcLocation);
static void c2_emxFree_real32_T
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance,
   c2_emxArray_real32_T **c2_pEmxArray);
static int32_T c2_div_nzp_s32
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, int32_T
   c2_numerator, int32_T c2_denominator, uint32_T c2_ssid_src_loc, int32_T
   c2_offset_src_loc, int32_T c2_length_src_loc);
static void init_dsm_address_info
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_DroneControllerCollisionAvoidance(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_DroneControllerCollisionAvoidance = 0U;
}

static void initialize_params_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  const mxArray *c2_c_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(2, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", *chartInstance->c2_posePub, 0, 0U,
    1U, 0U, 2, 1, 4), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal =
    chartInstance->c2_is_active_c2_DroneControllerCollisionAvoidance;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_hoistedGlobal, 3, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i0;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("posePub", c2_u,
    0)), "posePub", c2_dv0);
  for (c2_i0 = 0; c2_i0 < 4; c2_i0++) {
    (*chartInstance->c2_posePub)[c2_i0] = c2_dv0[c2_i0];
  }

  chartInstance->c2_is_active_c2_DroneControllerCollisionAvoidance =
    c2_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(
    "is_active_c2_DroneControllerCollisionAvoidance", c2_u, 1)),
    "is_active_c2_DroneControllerCollisionAvoidance");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_DroneControllerCollisionAvoidance(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  int32_T c2_i1;
  int32_T c2_i2;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i1 = 0; c2_i1 < 4; c2_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_target)[c2_i1], 2U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_DroneControllerCollisionAvoidance(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_DroneControllerCollisionAvoidanceMachineNumber_,
     chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c2_i2 = 0; c2_i2 < 4; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_posePub)[c2_i2], 3U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }
}

static void mdl_start_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_chartstep_c2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  int32_T c2_i3;
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry c2_b_odomMsg;
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa c2_b_laserMsg;
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  int32_T c2_i10;
  uint32_T c2_debug_family_var_map[13];
  real_T c2_b_target[4];
  real_T c2_originTarget[3];
  real_T c2_lastPositions[30];
  real_T c2_pose[4];
  real32_T c2_minDist;
  real_T c2_data[2];
  real_T c2_v;
  real_T c2_u[3];
  real_T c2_c_target[3];
  real_T c2_b_data[3];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 1.0;
  real_T c2_b_posePub[4];
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_i14;
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry c2_c_odomMsg;
  uint32_T c2_b_debug_family_var_map[11];
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose c2_poseMsg;
  real_T c2_xpos;
  real_T c2_ypos;
  real_T c2_zpos;
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw c2_quat;
  real_T c2_angles[3];
  real_T c2_theta;
  real_T c2_b_nargin = 1.0;
  real_T c2_b_nargout = 1.0;
  real_T c2_b_quat[4];
  real_T c2_dv1[3];
  int32_T c2_i15;
  const mxArray *c2_y = NULL;
  int32_T c2_i16;
  real_T c2_b_pose[3];
  real_T c2_x[3];
  real_T c2_b_y;
  int32_T c2_k;
  real_T c2_b_x;
  real_T c2_d0;
  real_T c2_c_x;
  boolean_T c2_p;
  boolean_T c2_b_p;
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa c2_c_laserMsg;
  int32_T c2_i17;
  int32_T c2_i18;
  real_T c2_c_pose[4];
  real32_T c2_b_minDist;
  real_T c2_c_data[2];
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  real_T c2_d_pose[3];
  real_T c2_c_y;
  int32_T c2_b_k;
  real_T c2_d_x;
  real_T c2_e_x;
  boolean_T c2_c_p;
  boolean_T c2_d_p;
  real_T c2_f_x;
  real_T c2_g_x;
  int32_T c2_i25;
  int32_T c2_i;
  real_T c2_varargin_1[30];
  int32_T c2_i26;
  boolean_T c2_e_p;
  int32_T c2_inpageroot;
  int32_T c2_c_k;
  int32_T c2_outpageroot;
  int32_T c2_ixstart;
  int32_T c2_ix;
  int32_T c2_i27;
  real_T c2_d_k;
  real_T c2_xbar;
  int32_T c2_e_k;
  int32_T c2_f_k;
  real_T c2_d_y[3];
  int32_T c2_i28;
  real_T c2_d_target[3];
  boolean_T c2_f_p;
  real_T c2_h_x;
  real_T c2_i_x;
  int32_T c2_i29;
  real_T c2_e_pose[3];
  real_T c2_e_y;
  real_T c2_j_x;
  int32_T c2_g_k;
  real_T c2_k_x;
  real_T c2_b_xbar;
  real32_T c2_c_minDist;
  real_T c2_pose_obs[3];
  real_T c2_r;
  uint32_T c2_c_debug_family_var_map[12];
  real_T c2_l_x;
  real_T c2_f_y;
  real_T c2_d;
  real_T c2_d1;
  int32_T c2_h_k;
  real_T c2_p0;
  real_T c2_m_x;
  real_T c2_k_att;
  boolean_T c2_g_p;
  real_T c2_k_rep;
  boolean_T c2_h_p;
  real_T c2_n_x;
  real_T c2_F[3];
  real_T c2_o_x;
  real_T c2_c_nargin = 4.0;
  real_T c2_c_xbar;
  real_T c2_c_nargout = 1.0;
  real_T c2_b_r;
  real_T c2_g_y;
  real_T c2_b_k_att;
  int32_T c2_i30;
  int32_T c2_i31;
  real_T c2_q[3];
  uint32_T c2_d_debug_family_var_map[6];
  real_T c2_e_target[3];
  real_T c2_d_nargin = 3.0;
  real_T c2_d_nargout = 1.0;
  int32_T c2_i32;
  int32_T c2_i33;
  real32_T c2_B;
  int32_T c2_i34;
  real32_T c2_h_y;
  real32_T c2_i_y;
  real32_T c2_j_y;
  int32_T c2_i35;
  real32_T c2_b;
  real32_T c2_k_y;
  real32_T c2_a;
  real32_T c2_b_a;
  real32_T c2_c_a;
  real32_T c2_p_x;
  int32_T c2_i36;
  real32_T c2_d_a;
  real32_T c2_c;
  boolean_T c2_i_p;
  real32_T c2_b_B;
  real32_T c2_l_y;
  real32_T c2_m_y;
  real32_T c2_n_y;
  int32_T c2_i37;
  real32_T c2_c_B;
  real32_T c2_o_y;
  real32_T c2_p_y;
  int32_T c2_i38;
  real32_T c2_e_a;
  real32_T c2_q_y[3];
  int32_T c2_i39;
  int32_T c2_i40;
  boolean_T guard1 = false;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i3 = 0; c2_i3 < 128; c2_i3++) {
    c2_b_odomMsg.ChildFrameId[c2_i3] = ((uint8_T *)&((char_T *)
      chartInstance->c2_odomMsg)[0])[c2_i3];
  }

  c2_b_odomMsg.ChildFrameId_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    chartInstance->c2_odomMsg)[128])[0];
  c2_b_odomMsg.ChildFrameId_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    chartInstance->c2_odomMsg)[128])[4];
  c2_b_odomMsg.Header.Seq = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_odomMsg)[136])[0];
  for (c2_i4 = 0; c2_i4 < 128; c2_i4++) {
    c2_b_odomMsg.Header.FrameId[c2_i4] = ((uint8_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
      chartInstance->c2_odomMsg)[136])[4])[c2_i4];
  }

  c2_b_odomMsg.Header.FrameId_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_odomMsg)[136])[136])[0];
  c2_b_odomMsg.Header.FrameId_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_odomMsg)[136])[136])[4];
  c2_b_odomMsg.Header.Stamp.Sec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_odomMsg)[136])[144])[0];
  c2_b_odomMsg.Header.Stamp.Nsec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_odomMsg)[136])[144])[8];
  for (c2_i5 = 0; c2_i5 < 36; c2_i5++) {
    c2_b_odomMsg.Pose.Covariance[c2_i5] = ((real_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T
      *)chartInstance->c2_odomMsg)[296])[0])[c2_i5];
  }

  c2_b_odomMsg.Pose.Pose.Position.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[0])[0];
  c2_b_odomMsg.Pose.Pose.Position.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[0])[8];
  c2_b_odomMsg.Pose.Pose.Position.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[0])[16];
  c2_b_odomMsg.Pose.Pose.Orientation.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[24])[0];
  c2_b_odomMsg.Pose.Pose.Orientation.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[24])[8];
  c2_b_odomMsg.Pose.Pose.Orientation.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[24])[16];
  c2_b_odomMsg.Pose.Pose.Orientation.W = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    chartInstance->c2_odomMsg)[296])[288])[24])[24];
  for (c2_i6 = 0; c2_i6 < 36; c2_i6++) {
    c2_b_odomMsg.Twist.Covariance[c2_i6] = ((real_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)
      &((char_T *)chartInstance->c2_odomMsg)[640])[0])[c2_i6];
  }

  c2_b_odomMsg.Twist.Twist.Linear.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    chartInstance->c2_odomMsg)[640])[288])[0])[0];
  c2_b_odomMsg.Twist.Twist.Linear.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    chartInstance->c2_odomMsg)[640])[288])[0])[8];
  c2_b_odomMsg.Twist.Twist.Linear.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    chartInstance->c2_odomMsg)[640])[288])[0])[16];
  c2_b_odomMsg.Twist.Twist.Angular.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    chartInstance->c2_odomMsg)[640])[288])[24])[0];
  c2_b_odomMsg.Twist.Twist.Angular.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    chartInstance->c2_odomMsg)[640])[288])[24])[8];
  c2_b_odomMsg.Twist.Twist.Angular.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    chartInstance->c2_odomMsg)[640])[288])[24])[16];
  c2_b_laserMsg.AngleMin = *(real32_T *)&((char_T *)chartInstance->c2_laserMsg)
    [0];
  c2_b_laserMsg.AngleMax = *(real32_T *)&((char_T *)chartInstance->c2_laserMsg)
    [4];
  c2_b_laserMsg.AngleIncrement = *(real32_T *)&((char_T *)
    chartInstance->c2_laserMsg)[8];
  c2_b_laserMsg.TimeIncrement = *(real32_T *)&((char_T *)
    chartInstance->c2_laserMsg)[12];
  c2_b_laserMsg.ScanTime = *(real32_T *)&((char_T *)chartInstance->c2_laserMsg)
    [16];
  c2_b_laserMsg.RangeMin = *(real32_T *)&((char_T *)chartInstance->c2_laserMsg)
    [20];
  c2_b_laserMsg.RangeMax = *(real32_T *)&((char_T *)chartInstance->c2_laserMsg)
    [24];
  for (c2_i7 = 0; c2_i7 < 2048; c2_i7++) {
    c2_b_laserMsg.Ranges[c2_i7] = ((real32_T *)&((char_T *)
      chartInstance->c2_laserMsg)[28])[c2_i7];
  }

  c2_b_laserMsg.Ranges_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    chartInstance->c2_laserMsg)[8224])[0];
  c2_b_laserMsg.Ranges_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    chartInstance->c2_laserMsg)[8224])[4];
  for (c2_i8 = 0; c2_i8 < 2048; c2_i8++) {
    c2_b_laserMsg.Intensities[c2_i8] = ((real32_T *)&((char_T *)
      chartInstance->c2_laserMsg)[8232])[c2_i8];
  }

  c2_b_laserMsg.Intensities_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    chartInstance->c2_laserMsg)[16424])[0];
  c2_b_laserMsg.Intensities_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    chartInstance->c2_laserMsg)[16424])[4];
  c2_b_laserMsg.Header.Seq = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_laserMsg)[16432])[0];
  for (c2_i9 = 0; c2_i9 < 128; c2_i9++) {
    c2_b_laserMsg.Header.FrameId[c2_i9] = ((uint8_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
      chartInstance->c2_laserMsg)[16432])[4])[c2_i9];
  }

  c2_b_laserMsg.Header.FrameId_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_laserMsg)[16432])[136])[0];
  c2_b_laserMsg.Header.FrameId_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_laserMsg)[16432])[136])[4];
  c2_b_laserMsg.Header.Stamp.Sec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_laserMsg)[16432])[144])[0];
  c2_b_laserMsg.Header.Stamp.Nsec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    chartInstance->c2_laserMsg)[16432])[144])[8];
  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_b_target[c2_i10] = (*chartInstance->c2_target)[c2_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 15U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_originTarget, 0U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_lastPositions, 1U, c2_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_pose, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_minDist, 3U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_data, MAX_uint32_T,
    c2_f_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_v, 5U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u, 6U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_c_target, MAX_uint32_T,
    c2_e_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_data, MAX_uint32_T,
    c2_e_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 8U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 9U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_odomMsg, 10U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_laserMsg, 11U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_target, 7U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_posePub, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    c2_b_posePub[c2_i11] = c2_b_target[c2_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i12 = 0; c2_i12 < 3; c2_i12++) {
    c2_c_target[c2_i12] = c2_b_target[c2_i12];
  }

  _SFD_SYMBOL_SWITCH(7U, 7U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  for (c2_i13 = 0; c2_i13 < 3; c2_i13++) {
    c2_originTarget[c2_i13] = c2_c_target[c2_i13];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  for (c2_i14 = 0; c2_i14 < 30; c2_i14++) {
    c2_lastPositions[c2_i14] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_c_odomMsg = c2_b_odomMsg;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c2_b_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_poseMsg, 0U, c2_j_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_xpos, 1U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ypos, 2U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_zpos, 3U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_quat, 4U, c2_i_sf_marshallOut,
    c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_angles, 5U, c2_e_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta, 6U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 7U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 8U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_odomMsg, 9U, c2_c_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_pose, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 49);
  c2_poseMsg = c2_c_odomMsg.Pose.Pose;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
  c2_xpos = c2_poseMsg.Position.X;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
  c2_ypos = c2_poseMsg.Position.Y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
  c2_zpos = c2_poseMsg.Position.Z;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
  c2_quat = c2_poseMsg.Orientation;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
  c2_b_quat[0] = c2_quat.W;
  c2_b_quat[1] = c2_quat.X;
  c2_b_quat[2] = c2_quat.Y;
  c2_b_quat[3] = c2_quat.Z;
  c2_quat2eul(chartInstance, c2_b_quat, c2_dv1);
  for (c2_i15 = 0; c2_i15 < 3; c2_i15++) {
    c2_angles[c2_i15] = c2_dv1[c2_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 55);
  c2_theta = c2_angles[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 56);
  c2_pose[0] = c2_xpos;
  c2_pose[1] = c2_ypos;
  c2_pose[2] = c2_zpos;
  c2_pose[3] = c2_theta;
  sf_mex_printf("%s =\\n", "pose");
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_pose, 0, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14, c2_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -56);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  for (c2_i16 = 0; c2_i16 < 3; c2_i16++) {
    c2_b_pose[c2_i16] = c2_pose[c2_i16] - c2_c_target[c2_i16];
  }

  c2_power(chartInstance, c2_b_pose, c2_x);
  c2_b_y = c2_x[0];
  for (c2_k = 1; c2_k + 1 < 4; c2_k++) {
    c2_b_y += c2_x[c2_k];
  }

  c2_b_x = c2_b_y;
  c2_d0 = c2_b_x;
  c2_c_x = c2_d0;
  c2_p = (c2_c_x < 0.0);
  c2_b_p = c2_p;
  if (c2_b_p) {
    c2_b_error(chartInstance);
  }

  c2_d0 = muDoubleScalarSqrt(c2_d0);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c2_d0, 0.1, -1, 4U, c2_d0
        > 0.1))) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    c2_c_laserMsg = c2_b_laserMsg;
    for (c2_i17 = 0; c2_i17 < 4; c2_i17++) {
      c2_c_pose[c2_i17] = c2_pose[c2_i17];
    }

    c2_findObstacle(chartInstance, &c2_c_laserMsg, c2_c_pose, &c2_b_minDist,
                    c2_c_data);
    c2_minDist = c2_b_minDist;
    for (c2_i19 = 0; c2_i19 < 2; c2_i19++) {
      c2_data[c2_i19] = c2_c_data[c2_i19];
    }

    _SFD_SYMBOL_SWITCH(4U, 4U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
    for (c2_i20 = 0; c2_i20 < 2; c2_i20++) {
      c2_b_data[c2_i20] = c2_data[c2_i20];
    }

    c2_b_data[2] = c2_pose[2];
    _SFD_SYMBOL_SWITCH(4U, 8U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
    for (c2_i21 = 0; c2_i21 < 30; c2_i21++) {
      c2_lastPositions[c2_i21] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
    c2_i22 = 0;
    for (c2_i23 = 0; c2_i23 < 3; c2_i23++) {
      c2_lastPositions[c2_i22] = c2_pose[c2_i23];
      c2_i22 += 10;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
    for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
      c2_d_pose[c2_i24] = c2_pose[c2_i24] - c2_c_target[c2_i24];
    }

    c2_power(chartInstance, c2_d_pose, c2_x);
    c2_c_y = c2_x[0];
    for (c2_b_k = 1; c2_b_k + 1 < 4; c2_b_k++) {
      c2_c_y += c2_x[c2_b_k];
    }

    c2_d_x = c2_c_y;
    c2_v = c2_d_x;
    c2_e_x = c2_v;
    c2_c_p = (c2_e_x < 0.0);
    c2_d_p = c2_c_p;
    if (c2_d_p) {
      c2_b_error(chartInstance);
    }

    c2_f_x = c2_v;
    c2_v = c2_f_x;
    c2_g_x = c2_v;
    c2_v = c2_g_x;
    c2_v = muDoubleScalarSqrt(c2_v);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
    guard1 = false;
    if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 1, c2_v, 1.0, -1, 4U,
          c2_v > 1.0))) {
      for (c2_i25 = 0; c2_i25 < 30; c2_i25++) {
        c2_varargin_1[c2_i25] = c2_lastPositions[c2_i25];
      }

      for (c2_i = 0; c2_i + 1 < 4; c2_i++) {
        c2_inpageroot = c2_i * 10;
        c2_outpageroot = c2_i;
        c2_ixstart = c2_inpageroot;
        c2_ix = c2_ixstart;
        c2_xbar = c2_varargin_1[c2_ixstart];
        for (c2_e_k = 2; c2_e_k < 11; c2_e_k++) {
          c2_ix++;
          c2_xbar += c2_varargin_1[c2_ix];
        }

        c2_h_x = c2_xbar;
        c2_xbar = c2_h_x / 10.0;
        c2_ix = c2_ixstart;
        c2_k_x = c2_varargin_1[c2_ixstart];
        c2_b_xbar = c2_xbar;
        c2_r = c2_k_x - c2_b_xbar;
        c2_f_y = c2_r * c2_r;
        for (c2_h_k = 2; c2_h_k < 11; c2_h_k++) {
          c2_ix++;
          c2_o_x = c2_varargin_1[c2_ix];
          c2_c_xbar = c2_xbar;
          c2_b_r = c2_o_x - c2_c_xbar;
          c2_g_y = c2_b_r * c2_b_r;
          c2_f_y += c2_g_y;
        }

        c2_n_x = c2_f_y;
        c2_f_y = c2_n_x / 9.0;
        c2_d_y[c2_outpageroot] = c2_f_y;
      }

      c2_e_p = false;
      for (c2_c_k = 0; c2_c_k < 3; c2_c_k++) {
        c2_d_k = 1.0 + (real_T)c2_c_k;
        if (c2_e_p || (c2_d_y[(int32_T)c2_d_k - 1] < 0.0)) {
          c2_f_p = true;
        } else {
          c2_f_p = false;
        }

        c2_e_p = c2_f_p;
      }

      if (c2_e_p) {
        c2_b_error(chartInstance);
      }

      for (c2_f_k = 0; c2_f_k + 1 < 4; c2_f_k++) {
        c2_i_x = c2_d_y[c2_f_k];
        c2_j_x = c2_i_x;
        c2_j_x = muDoubleScalarSqrt(c2_j_x);
        c2_d_y[c2_f_k] = c2_j_x;
      }

      c2_power(chartInstance, c2_d_y, c2_x);
      c2_e_y = c2_x[0];
      for (c2_g_k = 1; c2_g_k + 1 < 4; c2_g_k++) {
        c2_e_y += c2_x[c2_g_k];
      }

      c2_l_x = c2_e_y;
      c2_d1 = c2_l_x;
      c2_m_x = c2_d1;
      c2_g_p = (c2_m_x < 0.0);
      c2_h_p = c2_g_p;
      if (c2_h_p) {
        c2_b_error(chartInstance);
      }

      c2_d1 = muDoubleScalarSqrt(c2_d1);
      if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 2, c2_d1, 0.075, -1,
            2U, c2_d1 < 0.075))) {
        CV_EML_MCDC(0, 1, 0, true);
        CV_EML_IF(0, 1, 1, true);
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
        c2_c_target[2] += 0.5;
        _SFD_SYMBOL_SWITCH(7U, 7U);
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      CV_EML_MCDC(0, 1, 0, false);
      CV_EML_IF(0, 1, 1, false);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
      for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
        c2_c_target[c2_i26] = c2_originTarget[c2_i26];
      }

      _SFD_SYMBOL_SWITCH(7U, 7U);
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
    for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
      c2_d_target[c2_i27] = c2_c_target[c2_i27];
    }

    for (c2_i28 = 0; c2_i28 < 3; c2_i28++) {
      c2_e_pose[c2_i28] = c2_pose[c2_i28];
    }

    for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
      c2_pose_obs[c2_i29] = c2_b_data[c2_i29];
    }

    c2_c_minDist = c2_minDist;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c2_e_debug_family_names,
      c2_c_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML(&c2_d, 0U, c2_d_sf_marshallOut);
    _SFD_SYMBOL_SCOPE_ADD_EML(&c2_p0, 1U, c2_d_sf_marshallOut);
    _SFD_SYMBOL_SCOPE_ADD_EML(&c2_k_att, 2U, c2_d_sf_marshallOut);
    _SFD_SYMBOL_SCOPE_ADD_EML(&c2_k_rep, 3U, c2_d_sf_marshallOut);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_F, 4U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 5U, c2_d_sf_marshallOut,
      c2_b_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 6U, c2_d_sf_marshallOut,
      c2_b_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_d_target, 7U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_e_pose, 8U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_pose_obs, 9U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_minDist, 10U, c2_g_sf_marshallOut,
      c2_e_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u, 11U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    CV_SCRIPT_FCN(0, 0);
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 4);
    c2_d = 0.1;
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 5);
    c2_p0 = 5.0;
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 6);
    c2_k_att = 1.0;
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 7);
    c2_k_rep = -5.0;
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 9);
    c2_b_k_att = 1.0;
    for (c2_i30 = 0; c2_i30 < 3; c2_i30++) {
      c2_q[c2_i30] = c2_e_pose[c2_i30];
    }

    for (c2_i31 = 0; c2_i31 < 3; c2_i31++) {
      c2_e_target[c2_i31] = c2_d_target[c2_i31];
    }

    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c2_d_debug_family_names,
      c2_d_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargin, 0U, c2_d_sf_marshallOut,
      c2_b_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargout, 1U, c2_d_sf_marshallOut,
      c2_b_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_k_att, 2U, c2_d_sf_marshallOut,
      c2_b_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 3U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_e_target, 4U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_F, 5U, c2_e_sf_marshallOut,
      c2_c_sf_marshallIn);
    CV_SCRIPT_FCN(1, 0);
    _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 4);
    for (c2_i32 = 0; c2_i32 < 3; c2_i32++) {
      c2_x[c2_i32] = c2_q[c2_i32] - c2_e_target[c2_i32];
    }

    for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
      c2_F[c2_i33] = -c2_x[c2_i33];
    }

    _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, -4);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 10);
    if (CV_SCRIPT_IF(0, 0, CV_RELATIONAL_EVAL(14U, 0U, 0, (real_T)c2_c_minDist,
          5.0, -1, 3U, c2_c_minDist <= 5.0F))) {
      _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 11);
      c2_B = c2_c_minDist;
      c2_h_y = c2_B;
      c2_i_y = c2_h_y;
      c2_j_y = 1.0F / c2_i_y;
      c2_b = c2_j_y - 0.2F;
      c2_k_y = -5.0F * c2_b;
      c2_a = c2_c_minDist;
      c2_b_a = c2_a;
      c2_c_a = c2_b_a;
      c2_p_x = c2_c_a;
      c2_d_a = c2_p_x;
      c2_c = c2_d_a * c2_d_a;
      c2_i_p = false;
      if (c2_i_p) {
        c2_error(chartInstance);
      }

      c2_b_B = c2_c;
      c2_l_y = c2_b_B;
      c2_m_y = c2_l_y;
      c2_n_y = 1.0F / c2_m_y;
      for (c2_i37 = 0; c2_i37 < 3; c2_i37++) {
        c2_x[c2_i37] = c2_e_pose[c2_i37] - c2_pose_obs[c2_i37];
      }

      c2_c_B = c2_c_minDist;
      c2_o_y = c2_c_B;
      c2_p_y = c2_o_y;
      for (c2_i38 = 0; c2_i38 < 3; c2_i38++) {
        c2_q_y[c2_i38] = (real32_T)c2_x[c2_i38] / c2_p_y;
      }

      c2_e_a = c2_k_y * c2_n_y;
      for (c2_i39 = 0; c2_i39 < 3; c2_i39++) {
        c2_q_y[c2_i39] *= c2_e_a;
      }

      for (c2_i40 = 0; c2_i40 < 3; c2_i40++) {
        c2_F[c2_i40] = (real32_T)c2_F[c2_i40] - c2_q_y[c2_i40];
      }
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 13);
    for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
      c2_x[c2_i34] = c2_F[c2_i34];
    }

    for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
      c2_u[c2_i35] = 0.1 * c2_x[c2_i35];
    }

    _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -13);
    _SFD_SYMBOL_SCOPE_POP();
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
    for (c2_i36 = 0; c2_i36 < 3; c2_i36++) {
      c2_b_posePub[c2_i36] = c2_pose[c2_i36] + c2_u[c2_i36];
    }

    c2_b_posePub[3] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -23);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i18 = 0; c2_i18 < 4; c2_i18++) {
    (*chartInstance->c2_posePub)[c2_i18] = c2_b_posePub[c2_i18];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_DroneControllerCollisionAvoidance
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)(c2_machineNumber);
  (void)(c2_chartNumber);
  (void)(c2_instanceNumber);
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, c2_instanceNumber, 0U,
    sf_debug_get_script_id(
    "C:\\Users\\Philipp\\Desktop\\Australien\\QUT_Work\\Simulink_Ros\\Quadrotor\\EKF_Gazebo\\hectorQuadrotorComputePotentialField.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, c2_instanceNumber, 1U,
    sf_debug_get_script_id(
    "C:\\Users\\Philipp\\Desktop\\Australien\\QUT_Work\\Simulink_Ros\\Quadrotor\\EKF_Gazebo\\getF_attractiv.m"));
}

static const mxArray *c2_emlrt_marshallOut
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   char_T c2_u[30])
{
  const mxArray *c2_y = NULL;
  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  return c2_y;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i41;
  const mxArray *c2_y = NULL;
  real_T c2_u[4];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
    c2_u[c2_i41] = (*(real_T (*)[4])c2_inData)[c2_i41];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_b_posePub, const char_T *c2_identifier, real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_posePub), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_posePub);
}

static void c2_b_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv2[4];
  int32_T c2_i42;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv2, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c2_i42 = 0; c2_i42 < 4; c2_i42++) {
    c2_y[c2_i42] = c2_dv2[c2_i42];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_posePub;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i43;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_b_posePub = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_posePub), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_posePub);
  for (c2_i43 = 0; c2_i43 < 4; c2_i43++) {
    (*(real_T (*)[4])c2_outData)[c2_i43] = c2_y[c2_i43];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa c2_u;
  const mxArray *c2_y = NULL;
  real32_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  real32_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real32_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real32_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  real32_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  real32_T c2_g_u;
  const mxArray *c2_g_y = NULL;
  real32_T c2_h_u;
  const mxArray *c2_h_y = NULL;
  int32_T c2_i44;
  const mxArray *c2_i_y = NULL;
  real32_T c2_i_u[2048];
  c2_SL_Bus_ROSVariableLengthArrayInfo c2_j_u;
  const mxArray *c2_j_y = NULL;
  uint32_T c2_k_u;
  const mxArray *c2_k_y = NULL;
  uint32_T c2_l_u;
  const mxArray *c2_l_y = NULL;
  int32_T c2_i45;
  const mxArray *c2_m_y = NULL;
  const mxArray *c2_n_y = NULL;
  uint32_T c2_m_u;
  const mxArray *c2_o_y = NULL;
  uint32_T c2_n_u;
  const mxArray *c2_p_y = NULL;
  c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header c2_o_u;
  const mxArray *c2_q_y = NULL;
  uint32_T c2_p_u;
  const mxArray *c2_r_y = NULL;
  int32_T c2_i46;
  const mxArray *c2_s_y = NULL;
  uint8_T c2_q_u[128];
  const mxArray *c2_t_y = NULL;
  uint32_T c2_r_u;
  const mxArray *c2_u_y = NULL;
  uint32_T c2_s_u;
  const mxArray *c2_v_y = NULL;
  c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time c2_t_u;
  const mxArray *c2_w_y = NULL;
  real_T c2_u_u;
  const mxArray *c2_x_y = NULL;
  real_T c2_v_u;
  const mxArray *c2_y_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.AngleMin;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_b_y, "AngleMin", "AngleMin", 0);
  c2_c_u = c2_u.AngleMax;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_c_y, "AngleMax", "AngleMax", 0);
  c2_d_u = c2_u.AngleIncrement;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_d_y, "AngleIncrement", "AngleIncrement", 0);
  c2_e_u = c2_u.TimeIncrement;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_e_y, "TimeIncrement", "TimeIncrement", 0);
  c2_f_u = c2_u.ScanTime;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_f_y, "ScanTime", "ScanTime", 0);
  c2_g_u = c2_u.RangeMin;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_g_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_g_y, "RangeMin", "RangeMin", 0);
  c2_h_u = c2_u.RangeMax;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_h_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_h_y, "RangeMax", "RangeMax", 0);
  for (c2_i44 = 0; c2_i44 < 2048; c2_i44++) {
    c2_i_u[c2_i44] = c2_u.Ranges[c2_i44];
  }

  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", c2_i_u, 1, 0U, 1U, 0U, 1, 2048),
                false);
  sf_mex_addfield(c2_y, c2_i_y, "Ranges", "Ranges", 0);
  c2_j_u = c2_u.Ranges_SL_Info;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_k_u = c2_j_u.CurrentLength;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_k_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_j_y, c2_k_y, "CurrentLength", "CurrentLength", 0);
  c2_l_u = c2_j_u.ReceivedLength;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_l_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_j_y, c2_l_y, "ReceivedLength", "ReceivedLength", 0);
  sf_mex_addfield(c2_y, c2_j_y, "Ranges_SL_Info", "Ranges_SL_Info", 0);
  for (c2_i45 = 0; c2_i45 < 2048; c2_i45++) {
    c2_i_u[c2_i45] = c2_u.Intensities[c2_i45];
  }

  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", c2_i_u, 1, 0U, 1U, 0U, 1, 2048),
                false);
  sf_mex_addfield(c2_y, c2_m_y, "Intensities", "Intensities", 0);
  c2_j_u = c2_u.Intensities_SL_Info;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_m_u = c2_j_u.CurrentLength;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_m_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_n_y, c2_o_y, "CurrentLength", "CurrentLength", 0);
  c2_n_u = c2_j_u.ReceivedLength;
  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_n_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_n_y, c2_p_y, "ReceivedLength", "ReceivedLength", 0);
  sf_mex_addfield(c2_y, c2_n_y, "Intensities_SL_Info", "Intensities_SL_Info", 0);
  c2_o_u = c2_u.Header;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_p_u = c2_o_u.Seq;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", &c2_p_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_q_y, c2_r_y, "Seq", "Seq", 0);
  for (c2_i46 = 0; c2_i46 < 128; c2_i46++) {
    c2_q_u[c2_i46] = c2_o_u.FrameId[c2_i46];
  }

  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", c2_q_u, 3, 0U, 1U, 0U, 1, 128),
                false);
  sf_mex_addfield(c2_q_y, c2_s_y, "FrameId", "FrameId", 0);
  c2_j_u = c2_o_u.FrameId_SL_Info;
  c2_t_y = NULL;
  sf_mex_assign(&c2_t_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_r_u = c2_j_u.CurrentLength;
  c2_u_y = NULL;
  sf_mex_assign(&c2_u_y, sf_mex_create("y", &c2_r_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_t_y, c2_u_y, "CurrentLength", "CurrentLength", 0);
  c2_s_u = c2_j_u.ReceivedLength;
  c2_v_y = NULL;
  sf_mex_assign(&c2_v_y, sf_mex_create("y", &c2_s_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_t_y, c2_v_y, "ReceivedLength", "ReceivedLength", 0);
  sf_mex_addfield(c2_q_y, c2_t_y, "FrameId_SL_Info", "FrameId_SL_Info", 0);
  c2_t_u = c2_o_u.Stamp;
  c2_w_y = NULL;
  sf_mex_assign(&c2_w_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_u_u = c2_t_u.Sec;
  c2_x_y = NULL;
  sf_mex_assign(&c2_x_y, sf_mex_create("y", &c2_u_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_w_y, c2_x_y, "Sec", "Sec", 0);
  c2_v_u = c2_t_u.Nsec;
  c2_y_y = NULL;
  sf_mex_assign(&c2_y_y, sf_mex_create("y", &c2_v_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_w_y, c2_y_y, "Nsec", "Nsec", 0);
  sf_mex_addfield(c2_q_y, c2_w_y, "Stamp", "Stamp", 0);
  sf_mex_addfield(c2_y, c2_q_y, "Header", "Header", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry c2_u;
  const mxArray *c2_y = NULL;
  int32_T c2_i47;
  const mxArray *c2_b_y = NULL;
  uint8_T c2_b_u[128];
  c2_SL_Bus_ROSVariableLengthArrayInfo c2_c_u;
  const mxArray *c2_c_y = NULL;
  uint32_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  uint32_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header c2_f_u;
  const mxArray *c2_f_y = NULL;
  uint32_T c2_g_u;
  const mxArray *c2_g_y = NULL;
  int32_T c2_i48;
  const mxArray *c2_h_y = NULL;
  const mxArray *c2_i_y = NULL;
  uint32_T c2_h_u;
  const mxArray *c2_j_y = NULL;
  uint32_T c2_i_u;
  const mxArray *c2_k_y = NULL;
  c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time c2_j_u;
  const mxArray *c2_l_y = NULL;
  real_T c2_k_u;
  const mxArray *c2_m_y = NULL;
  real_T c2_l_u;
  const mxArray *c2_n_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u c2_m_u;
  const mxArray *c2_o_y = NULL;
  int32_T c2_i49;
  const mxArray *c2_p_y = NULL;
  real_T c2_n_u[36];
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose c2_o_u;
  const mxArray *c2_q_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla c2_p_u;
  const mxArray *c2_r_y = NULL;
  real_T c2_q_u;
  const mxArray *c2_s_y = NULL;
  real_T c2_r_u;
  const mxArray *c2_t_y = NULL;
  real_T c2_s_u;
  const mxArray *c2_u_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw c2_t_u;
  const mxArray *c2_v_y = NULL;
  real_T c2_u_u;
  const mxArray *c2_w_y = NULL;
  real_T c2_v_u;
  const mxArray *c2_x_y = NULL;
  real_T c2_w_u;
  const mxArray *c2_y_y = NULL;
  real_T c2_x_u;
  const mxArray *c2_ab_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 c2_y_u;
  const mxArray *c2_bb_y = NULL;
  int32_T c2_i50;
  const mxArray *c2_cb_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b c2_ab_u;
  const mxArray *c2_db_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi c2_bb_u;
  const mxArray *c2_eb_y = NULL;
  real_T c2_cb_u;
  const mxArray *c2_fb_y = NULL;
  real_T c2_db_u;
  const mxArray *c2_gb_y = NULL;
  real_T c2_eb_u;
  const mxArray *c2_hb_y = NULL;
  const mxArray *c2_ib_y = NULL;
  real_T c2_fb_u;
  const mxArray *c2_jb_y = NULL;
  real_T c2_gb_u;
  const mxArray *c2_kb_y = NULL;
  real_T c2_hb_u;
  const mxArray *c2_lb_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)
    c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c2_i47 = 0; c2_i47 < 128; c2_i47++) {
    c2_b_u[c2_i47] = c2_u.ChildFrameId[c2_i47];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 3, 0U, 1U, 0U, 1, 128),
                false);
  sf_mex_addfield(c2_y, c2_b_y, "ChildFrameId", "ChildFrameId", 0);
  c2_c_u = c2_u.ChildFrameId_SL_Info;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_d_u = c2_c_u.CurrentLength;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_c_y, c2_d_y, "CurrentLength", "CurrentLength", 0);
  c2_e_u = c2_c_u.ReceivedLength;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_c_y, c2_e_y, "ReceivedLength", "ReceivedLength", 0);
  sf_mex_addfield(c2_y, c2_c_y, "ChildFrameId_SL_Info", "ChildFrameId_SL_Info",
                  0);
  c2_f_u = c2_u.Header;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_g_u = c2_f_u.Seq;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_g_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_g_y, "Seq", "Seq", 0);
  for (c2_i48 = 0; c2_i48 < 128; c2_i48++) {
    c2_b_u[c2_i48] = c2_f_u.FrameId[c2_i48];
  }

  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", c2_b_u, 3, 0U, 1U, 0U, 1, 128),
                false);
  sf_mex_addfield(c2_f_y, c2_h_y, "FrameId", "FrameId", 0);
  c2_c_u = c2_f_u.FrameId_SL_Info;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_h_u = c2_c_u.CurrentLength;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_h_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_i_y, c2_j_y, "CurrentLength", "CurrentLength", 0);
  c2_i_u = c2_c_u.ReceivedLength;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_i_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_i_y, c2_k_y, "ReceivedLength", "ReceivedLength", 0);
  sf_mex_addfield(c2_f_y, c2_i_y, "FrameId_SL_Info", "FrameId_SL_Info", 0);
  c2_j_u = c2_f_u.Stamp;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_k_u = c2_j_u.Sec;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_l_y, c2_m_y, "Sec", "Sec", 0);
  c2_l_u = c2_j_u.Nsec;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_l_y, c2_n_y, "Nsec", "Nsec", 0);
  sf_mex_addfield(c2_f_y, c2_l_y, "Stamp", "Stamp", 0);
  sf_mex_addfield(c2_y, c2_f_y, "Header", "Header", 0);
  c2_m_u = c2_u.Pose;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c2_i49 = 0; c2_i49 < 36; c2_i49++) {
    c2_n_u[c2_i49] = c2_m_u.Covariance[c2_i49];
  }

  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", c2_n_u, 0, 0U, 1U, 0U, 1, 36), false);
  sf_mex_addfield(c2_o_y, c2_p_y, "Covariance", "Covariance", 0);
  c2_o_u = c2_m_u.Pose;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_p_u = c2_o_u.Position;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_q_u = c2_p_u.X;
  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", &c2_q_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_r_y, c2_s_y, "X", "X", 0);
  c2_r_u = c2_p_u.Y;
  c2_t_y = NULL;
  sf_mex_assign(&c2_t_y, sf_mex_create("y", &c2_r_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_r_y, c2_t_y, "Y", "Y", 0);
  c2_s_u = c2_p_u.Z;
  c2_u_y = NULL;
  sf_mex_assign(&c2_u_y, sf_mex_create("y", &c2_s_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_r_y, c2_u_y, "Z", "Z", 0);
  sf_mex_addfield(c2_q_y, c2_r_y, "Position", "Position", 0);
  c2_t_u = c2_o_u.Orientation;
  c2_v_y = NULL;
  sf_mex_assign(&c2_v_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_u_u = c2_t_u.X;
  c2_w_y = NULL;
  sf_mex_assign(&c2_w_y, sf_mex_create("y", &c2_u_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_v_y, c2_w_y, "X", "X", 0);
  c2_v_u = c2_t_u.Y;
  c2_x_y = NULL;
  sf_mex_assign(&c2_x_y, sf_mex_create("y", &c2_v_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_v_y, c2_x_y, "Y", "Y", 0);
  c2_w_u = c2_t_u.Z;
  c2_y_y = NULL;
  sf_mex_assign(&c2_y_y, sf_mex_create("y", &c2_w_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_v_y, c2_y_y, "Z", "Z", 0);
  c2_x_u = c2_t_u.W;
  c2_ab_y = NULL;
  sf_mex_assign(&c2_ab_y, sf_mex_create("y", &c2_x_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_v_y, c2_ab_y, "W", "W", 0);
  sf_mex_addfield(c2_q_y, c2_v_y, "Orientation", "Orientation", 0);
  sf_mex_addfield(c2_o_y, c2_q_y, "Pose", "Pose", 0);
  sf_mex_addfield(c2_y, c2_o_y, "Pose", "Pose", 0);
  c2_y_u = c2_u.Twist;
  c2_bb_y = NULL;
  sf_mex_assign(&c2_bb_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c2_i50 = 0; c2_i50 < 36; c2_i50++) {
    c2_n_u[c2_i50] = c2_y_u.Covariance[c2_i50];
  }

  c2_cb_y = NULL;
  sf_mex_assign(&c2_cb_y, sf_mex_create("y", c2_n_u, 0, 0U, 1U, 0U, 1, 36),
                false);
  sf_mex_addfield(c2_bb_y, c2_cb_y, "Covariance", "Covariance", 0);
  c2_ab_u = c2_y_u.Twist;
  c2_db_y = NULL;
  sf_mex_assign(&c2_db_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_bb_u = c2_ab_u.Linear;
  c2_eb_y = NULL;
  sf_mex_assign(&c2_eb_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_cb_u = c2_bb_u.X;
  c2_fb_y = NULL;
  sf_mex_assign(&c2_fb_y, sf_mex_create("y", &c2_cb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_eb_y, c2_fb_y, "X", "X", 0);
  c2_db_u = c2_bb_u.Y;
  c2_gb_y = NULL;
  sf_mex_assign(&c2_gb_y, sf_mex_create("y", &c2_db_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_eb_y, c2_gb_y, "Y", "Y", 0);
  c2_eb_u = c2_bb_u.Z;
  c2_hb_y = NULL;
  sf_mex_assign(&c2_hb_y, sf_mex_create("y", &c2_eb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_eb_y, c2_hb_y, "Z", "Z", 0);
  sf_mex_addfield(c2_db_y, c2_eb_y, "Linear", "Linear", 0);
  c2_bb_u = c2_ab_u.Angular;
  c2_ib_y = NULL;
  sf_mex_assign(&c2_ib_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_fb_u = c2_bb_u.X;
  c2_jb_y = NULL;
  sf_mex_assign(&c2_jb_y, sf_mex_create("y", &c2_fb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_ib_y, c2_jb_y, "X", "X", 0);
  c2_gb_u = c2_bb_u.Y;
  c2_kb_y = NULL;
  sf_mex_assign(&c2_kb_y, sf_mex_create("y", &c2_gb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_ib_y, c2_kb_y, "Y", "Y", 0);
  c2_hb_u = c2_bb_u.Z;
  c2_lb_y = NULL;
  sf_mex_assign(&c2_lb_y, sf_mex_create("y", &c2_hb_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_ib_y, c2_lb_y, "Z", "Z", 0);
  sf_mex_addfield(c2_db_y, c2_ib_y, "Angular", "Angular", 0);
  sf_mex_addfield(c2_bb_y, c2_db_y, "Twist", "Twist", 0);
  sf_mex_addfield(c2_y, c2_bb_y, "Twist", "Twist", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d2;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d2, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d2;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i51;
  const mxArray *c2_y = NULL;
  real_T c2_u[3];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i51 = 0; c2_i51 < 3; c2_i51++) {
    c2_u[c2_i51] = (*(real_T (*)[3])c2_inData)[c2_i51];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_d_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv3[3];
  int32_T c2_i52;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c2_i52 = 0; c2_i52 < 3; c2_i52++) {
    c2_y[c2_i52] = c2_dv3[c2_i52];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_data;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i53;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_data = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_data), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_data);
  for (c2_i53 = 0; c2_i53 < 3; c2_i53++) {
    (*(real_T (*)[3])c2_outData)[c2_i53] = c2_y[c2_i53];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i54;
  const mxArray *c2_y = NULL;
  real_T c2_u[2];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i54 = 0; c2_i54 < 2; c2_i54++) {
    c2_u[c2_i54] = (*(real_T (*)[2])c2_inData)[c2_i54];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2])
{
  real_T c2_dv4[2];
  int32_T c2_i55;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv4, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i55 = 0; c2_i55 < 2; c2_i55++) {
    c2_y[c2_i55] = c2_dv4[c2_i55];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_data;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[2];
  int32_T c2_i56;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_data = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_data), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_data);
  for (c2_i56 = 0; c2_i56 < 2; c2_i56++) {
    (*(real_T (*)[2])c2_outData)[c2_i56] = c2_y[c2_i56];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  real32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(real32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real32_T c2_f_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real32_T c2_y;
  real32_T c2_f0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_f0, 1, 1, 0U, 0, 0U, 0);
  c2_y = c2_f0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_minDist;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_minDist = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_minDist), &c2_thisId);
  sf_mex_destroy(&c2_minDist);
  *(real32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i57;
  int32_T c2_i58;
  const mxArray *c2_y = NULL;
  int32_T c2_i59;
  real_T c2_u[30];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_i57 = 0;
  for (c2_i58 = 0; c2_i58 < 3; c2_i58++) {
    for (c2_i59 = 0; c2_i59 < 10; c2_i59++) {
      c2_u[c2_i59 + c2_i57] = (*(real_T (*)[30])c2_inData)[c2_i59 + c2_i57];
    }

    c2_i57 += 10;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 10, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[5] = { "ChildFrameId",
    "ChildFrameId_SL_Info", "Header", "Pose", "Twist" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 5, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "ChildFrameId";
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "ChildFrameId", "ChildFrameId", 0)), &c2_thisId, c2_y->ChildFrameId);
  c2_thisId.fIdentifier = "ChildFrameId_SL_Info";
  c2_y->ChildFrameId_SL_Info = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "ChildFrameId_SL_Info", "ChildFrameId_SL_Info", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "Header";
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u, "Header",
    "Header", 0)), &c2_thisId, &c2_y->Header);
  c2_thisId.fIdentifier = "Pose";
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u, "Pose",
    "Pose", 0)), &c2_thisId, &c2_y->Pose);
  c2_thisId.fIdentifier = "Twist";
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u, "Twist",
    "Twist", 0)), &c2_thisId, &c2_y->Twist);
  sf_mex_destroy(&c2_u);
}

static void c2_h_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, uint8_T c2_y[128])
{
  uint8_T c2_uv0[128];
  int32_T c2_i60;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_uv0, 1, 3, 0U, 1, 0U, 1, 128);
  for (c2_i60 = 0; c2_i60 < 128; c2_i60++) {
    c2_y[c2_i60] = c2_uv0[c2_i60];
  }

  sf_mex_destroy(&c2_u);
}

static c2_SL_Bus_ROSVariableLengthArrayInfo c2_i_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_ROSVariableLengthArrayInfo c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "CurrentLength", "ReceivedLength" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "CurrentLength";
  c2_y.CurrentLength = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "CurrentLength", "CurrentLength", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "ReceivedLength";
  c2_y.ReceivedLength = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "ReceivedLength", "ReceivedLength", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static uint32_T c2_j_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint32_T c2_y;
  uint32_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 7, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_k_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[4] = { "Seq", "FrameId", "FrameId_SL_Info",
    "Stamp" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 4, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Seq";
  c2_y->Seq = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Seq", "Seq", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "FrameId";
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "FrameId", "FrameId", 0)), &c2_thisId, c2_y->FrameId);
  c2_thisId.fIdentifier = "FrameId_SL_Info";
  c2_y->FrameId_SL_Info = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "FrameId_SL_Info", "FrameId_SL_Info", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Stamp";
  c2_y->Stamp = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Stamp", "Stamp", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time
  c2_l_emlrt_marshallIn(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Sec", "Nsec" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Sec";
  c2_y.Sec = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Sec", "Sec", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Nsec";
  c2_y.Nsec = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Nsec", "Nsec", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_m_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Covariance", "Pose" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Covariance";
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Covariance", "Covariance", 0)), &c2_thisId, c2_y->Covariance);
  c2_thisId.fIdentifier = "Pose";
  c2_y->Pose = c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Pose", "Pose", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static void c2_n_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[36])
{
  real_T c2_dv5[36];
  int32_T c2_i61;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv5, 1, 0, 0U, 1, 0U, 1, 36);
  for (c2_i61 = 0; c2_i61 < 36; c2_i61++) {
    c2_y[c2_i61] = c2_dv5[c2_i61];
  }

  sf_mex_destroy(&c2_u);
}

static c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose
  c2_o_emlrt_marshallIn(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Position", "Orientation" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Position";
  c2_y.Position = c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "Position", "Position", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Orientation";
  c2_y.Orientation = c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "Orientation", "Orientation", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla c2_p_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[3] = { "X", "Y", "Z" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 3, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "X";
  c2_y.X = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "X", "X", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Y";
  c2_y.Y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Y", "Y", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Z";
  c2_y.Z = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Z", "Z", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw
  c2_q_emlrt_marshallIn(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[4] = { "X", "Y", "Z", "W" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 4, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "X";
  c2_y.X = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "X", "X", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Y";
  c2_y.Y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Y", "Y", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Z";
  c2_y.Z = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Z", "Z", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "W";
  c2_y.W = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "W", "W", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_r_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Covariance", "Twist" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Covariance";
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Covariance", "Covariance", 0)), &c2_thisId, c2_y->Covariance);
  c2_thisId.fIdentifier = "Twist";
  c2_y->Twist = c2_s_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Twist", "Twist", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b c2_s_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Linear", "Angular" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Linear";
  c2_y.Linear = c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Linear", "Linear", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Angular";
  c2_y.Angular = c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Angular", "Angular", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi c2_t_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[3] = { "X", "Y", "Z" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 3, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "X";
  c2_y.X = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "X", "X", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Y";
  c2_y.Y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Y", "Y", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Z";
  c2_y.Z = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Z", "Z", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_odomMsg;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_b_odomMsg = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_odomMsg), &c2_thisId,
                        &c2_y);
  sf_mex_destroy(&c2_b_odomMsg);
  *(c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_outData =
    c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw c2_u;
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.X;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_b_y, "X", "X", 0);
  c2_c_u = c2_u.Y;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_c_y, "Y", "Y", 0);
  c2_d_u = c2_u.Z;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_d_y, "Z", "Z", 0);
  c2_e_u = c2_u.W;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_e_y, "W", "W", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_quat;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_quat = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_quat), &c2_thisId);
  sf_mex_destroy(&c2_quat);
  *(c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose c2_u;
  const mxArray *c2_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla c2_b_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw c2_f_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_g_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_h_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_i_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_j_u;
  const mxArray *c2_j_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)
    c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.Position;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_c_u = c2_b_u.X;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_b_y, c2_c_y, "X", "X", 0);
  c2_d_u = c2_b_u.Y;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_b_y, c2_d_y, "Y", "Y", 0);
  c2_e_u = c2_b_u.Z;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_b_y, c2_e_y, "Z", "Z", 0);
  sf_mex_addfield(c2_y, c2_b_y, "Position", "Position", 0);
  c2_f_u = c2_u.Orientation;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_g_u = c2_f_u.X;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_g_y, "X", "X", 0);
  c2_h_u = c2_f_u.Y;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_h_y, "Y", "Y", 0);
  c2_i_u = c2_f_u.Z;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_i_y, "Z", "Z", 0);
  c2_j_u = c2_f_u.W;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_j_y, "W", "W", 0);
  sf_mex_addfield(c2_y, c2_f_y, "Orientation", "Orientation", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_poseMsg;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_poseMsg = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_poseMsg), &c2_thisId);
  sf_mex_destroy(&c2_poseMsg);
  *(c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)c2_outData =
    c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_u_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
   c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[12] = { "AngleMin", "AngleMax",
    "AngleIncrement", "TimeIncrement", "ScanTime", "RangeMin", "RangeMax",
    "Ranges", "Ranges_SL_Info", "Intensities", "Intensities_SL_Info", "Header" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 12, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "AngleMin";
  c2_y->AngleMin = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "AngleMin", "AngleMin", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "AngleMax";
  c2_y->AngleMax = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "AngleMax", "AngleMax", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "AngleIncrement";
  c2_y->AngleIncrement = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "AngleIncrement", "AngleIncrement", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "TimeIncrement";
  c2_y->TimeIncrement = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "TimeIncrement", "TimeIncrement", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "ScanTime";
  c2_y->ScanTime = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "ScanTime", "ScanTime", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "RangeMin";
  c2_y->RangeMin = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "RangeMin", "RangeMin", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "RangeMax";
  c2_y->RangeMax = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "RangeMax", "RangeMax", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Ranges";
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u, "Ranges",
    "Ranges", 0)), &c2_thisId, c2_y->Ranges);
  c2_thisId.fIdentifier = "Ranges_SL_Info";
  c2_y->Ranges_SL_Info = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "Ranges_SL_Info", "Ranges_SL_Info", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Intensities";
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Intensities", "Intensities", 0)), &c2_thisId, c2_y->Intensities);
  c2_thisId.fIdentifier = "Intensities_SL_Info";
  c2_y->Intensities_SL_Info = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "Intensities_SL_Info", "Intensities_SL_Info", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "Header";
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u, "Header",
    "Header", 0)), &c2_thisId, &c2_y->Header);
  sf_mex_destroy(&c2_u);
}

static void c2_v_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y[2048])
{
  real32_T c2_fv0[2048];
  int32_T c2_i62;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_fv0, 1, 1, 0U, 1, 0U, 1, 2048);
  for (c2_i62 = 0; c2_i62 < 2048; c2_i62++) {
    c2_y[c2_i62] = c2_fv0[c2_i62];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_obj;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_obj = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_obj), &c2_thisId, &c2_y);
  sf_mex_destroy(&c2_obj);
  *(c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[], int32_T c2_inData_size[1])
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_u_size[1];
  int32_T c2_loop_ub;
  int32_T c2_i63;
  const mxArray *c2_y = NULL;
  real_T c2_u_data[2048];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u_size[0] = c2_inData_size[0];
  c2_loop_ub = c2_inData_size[0] - 1;
  for (c2_i63 = 0; c2_i63 <= c2_loop_ub; c2_i63++) {
    c2_u_data[c2_i63] = c2_inData_data[c2_i63];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", (void *)&c2_u_data, 0, 0U, 1U, 0U, 1,
    c2_u_size[0]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_w_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y_data[],
   int32_T c2_y_size[1])
{
  uint32_T c2_uv1[1];
  int32_T c2_tmp_size[1];
  boolean_T c2_bv0[1];
  real_T c2_tmp_data[2048];
  int32_T c2_loop_ub;
  int32_T c2_i64;
  (void)chartInstance;
  c2_uv1[0] = 2048U;
  c2_tmp_size[0] = sf_mex_get_dimension(c2_u, 0);
  c2_bv0[0] = true;
  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), (void *)&c2_tmp_data, 1, 0, 0U,
                   1, 0U, 1, c2_bv0, c2_uv1, c2_tmp_size);
  c2_y_size[0] = c2_tmp_size[0];
  c2_loop_ub = c2_tmp_size[0] - 1;
  for (c2_i64 = 0; c2_i64 <= c2_loop_ub; c2_i64++) {
    c2_y_data[c2_i64] = c2_tmp_data[c2_i64];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[], int32_T
  c2_outData_size[1])
{
  const mxArray *c2_validIdx;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y_data[2048];
  int32_T c2_y_size[1];
  int32_T c2_loop_ub;
  int32_T c2_i65;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_validIdx = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_w_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_validIdx), &c2_thisId,
                        c2_y_data, c2_y_size);
  sf_mex_destroy(&c2_validIdx);
  c2_outData_size[0] = c2_y_size[0];
  c2_loop_ub = c2_y_size[0] - 1;
  for (c2_i65 = 0; c2_i65 <= c2_loop_ub; c2_i65++) {
    c2_outData_data[c2_i65] = c2_y_data[c2_i65];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, real32_T
  c2_inData_data[], int32_T c2_inData_size[1])
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_u_size[1];
  int32_T c2_loop_ub;
  int32_T c2_i66;
  const mxArray *c2_y = NULL;
  real32_T c2_u_data[2048];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u_size[0] = c2_inData_size[0];
  c2_loop_ub = c2_inData_size[0] - 1;
  for (c2_i66 = 0; c2_i66 <= c2_loop_ub; c2_i66++) {
    c2_u_data[c2_i66] = c2_inData_data[c2_i66];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", (void *)&c2_u_data, 1, 0U, 1U, 0U, 1,
    c2_u_size[0]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_x_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y_data[],
   int32_T c2_y_size[1])
{
  uint32_T c2_uv2[1];
  int32_T c2_tmp_size[1];
  boolean_T c2_bv1[1];
  real32_T c2_tmp_data[2048];
  int32_T c2_loop_ub;
  int32_T c2_i67;
  (void)chartInstance;
  c2_uv2[0] = 2048U;
  c2_tmp_size[0] = sf_mex_get_dimension(c2_u, 0);
  c2_bv1[0] = true;
  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), (void *)&c2_tmp_data, 1, 1, 0U,
                   1, 0U, 1, c2_bv1, c2_uv2, c2_tmp_size);
  c2_y_size[0] = c2_tmp_size[0];
  c2_loop_ub = c2_tmp_size[0] - 1;
  for (c2_i67 = 0; c2_i67 <= c2_loop_ub; c2_i67++) {
    c2_y_data[c2_i67] = c2_tmp_data[c2_i67];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real32_T c2_outData_data[],
  int32_T c2_outData_size[1])
{
  const mxArray *c2_R;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y_data[2048];
  int32_T c2_y_size[1];
  int32_T c2_loop_ub;
  int32_T c2_i68;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_R = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_R), &c2_thisId, c2_y_data,
                        c2_y_size);
  sf_mex_destroy(&c2_R);
  c2_outData_size[0] = c2_y_size[0];
  c2_loop_ub = c2_y_size[0] - 1;
  for (c2_i68 = 0; c2_i68 <= c2_loop_ub; c2_i68++) {
    c2_outData_data[c2_i68] = c2_y_data[c2_i68];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, real32_T
  c2_inData_data[], int32_T c2_inData_size[2])
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_u_size[2];
  int32_T c2_u;
  int32_T c2_b_u;
  int32_T c2_loop_ub;
  int32_T c2_i69;
  const mxArray *c2_y = NULL;
  real32_T c2_u_data[2048];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u_size[0] = 1;
  c2_u_size[1] = c2_inData_size[1];
  c2_u = c2_u_size[0];
  c2_b_u = c2_u_size[1];
  c2_loop_ub = c2_inData_size[0] * c2_inData_size[1] - 1;
  for (c2_i69 = 0; c2_i69 <= c2_loop_ub; c2_i69++) {
    c2_u_data[c2_i69] = c2_inData_data[c2_i69];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", (void *)&c2_u_data, 1, 0U, 1U, 0U, 2,
    c2_u_size[0], c2_u_size[1]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_y_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y_data[],
   int32_T c2_y_size[2])
{
  int32_T c2_i70;
  int32_T c2_tmp_size[2];
  uint32_T c2_uv3[2];
  int32_T c2_i71;
  real32_T c2_tmp_data[2048];
  boolean_T c2_bv2[2];
  static boolean_T c2_bv3[2] = { false, true };

  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i72;
  (void)chartInstance;
  for (c2_i70 = 0; c2_i70 < 2; c2_i70++) {
    c2_uv3[c2_i70] = 1U + 2047U * (uint32_T)c2_i70;
  }

  c2_tmp_size[0] = sf_mex_get_dimension(c2_u, 0);
  c2_tmp_size[1] = sf_mex_get_dimension(c2_u, 1);
  for (c2_i71 = 0; c2_i71 < 2; c2_i71++) {
    c2_bv2[c2_i71] = c2_bv3[c2_i71];
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), (void *)&c2_tmp_data, 1, 1, 0U,
                   1, 0U, 2, c2_bv2, c2_uv3, c2_tmp_size);
  c2_y_size[0] = 1;
  c2_y_size[1] = c2_tmp_size[1];
  c2_y = c2_y_size[0];
  c2_b_y = c2_y_size[1];
  c2_loop_ub = c2_tmp_size[0] * c2_tmp_size[1] - 1;
  for (c2_i72 = 0; c2_i72 <= c2_loop_ub; c2_i72++) {
    c2_y_data[c2_i72] = c2_tmp_data[c2_i72];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real32_T c2_outData_data[],
  int32_T c2_outData_size[2])
{
  const mxArray *c2_cartAngles;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y_data[2048];
  int32_T c2_y_size[2];
  int32_T c2_loop_ub;
  int32_T c2_i73;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_cartAngles = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_cartAngles), &c2_thisId,
                        c2_y_data, c2_y_size);
  sf_mex_destroy(&c2_cartAngles);
  c2_outData_size[0] = 1;
  c2_outData_size[1] = c2_y_size[1];
  c2_loop_ub = c2_y_size[1] - 1;
  for (c2_i73 = 0; c2_i73 <= c2_loop_ub; c2_i73++) {
    c2_outData_data[c2_outData_size[0] * c2_i73] = c2_y_data[c2_y_size[0] *
      c2_i73];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid,
  c2_emxArray_real32_T *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_emxArray_real32_T *c2_u;
  int32_T c2_i74;
  int32_T c2_b_u;
  int32_T c2_c_u;
  int32_T c2_loop_ub;
  int32_T c2_i75;
  const mxArray *c2_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_emxInit_real32_T(chartInstance, &c2_u, 2, (emlrtRTEInfo *)NULL);
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_i74 = c2_u->size[0] * c2_u->size[1];
  c2_u->size[0] = 1;
  c2_u->size[1] = c2_inData->size[1];
  c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_u, c2_i74,
                       sizeof(real32_T), (emlrtRTEInfo *)NULL);
  c2_b_u = c2_u->size[0];
  c2_c_u = c2_u->size[1];
  c2_loop_ub = c2_inData->size[0] * c2_inData->size[1] - 1;
  for (c2_i75 = 0; c2_i75 <= c2_loop_ub; c2_i75++) {
    c2_u->data[c2_i75] = c2_inData->data[c2_i75];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u->data, 1, 0U, 1U, 0U, 2,
    c2_u->size[0], c2_u->size[1]), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  c2_emxFree_real32_T(chartInstance, &c2_u);
  return c2_mxArrayOutData;
}

static void c2_ab_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, c2_emxArray_real32_T
   *c2_y)
{
  c2_emxArray_real32_T *c2_r0;
  int32_T c2_i76;
  int32_T c2_i77;
  uint32_T c2_uv4[2];
  int32_T c2_i78;
  boolean_T c2_bv4[2];
  static boolean_T c2_bv5[2] = { false, true };

  int32_T c2_i79;
  int32_T c2_b_y;
  int32_T c2_c_y;
  int32_T c2_loop_ub;
  int32_T c2_i80;
  c2_emxInit_real32_T(chartInstance, &c2_r0, 2, (emlrtRTEInfo *)NULL);
  for (c2_i76 = 0; c2_i76 < 2; c2_i76++) {
    c2_uv4[c2_i76] = 1U + 4294967294U * (uint32_T)c2_i76;
  }

  c2_i77 = c2_r0->size[0] * c2_r0->size[1];
  c2_r0->size[0] = sf_mex_get_dimension(c2_u, 0);
  c2_r0->size[1] = sf_mex_get_dimension(c2_u, 1);
  c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_r0, c2_i77,
                       sizeof(real32_T), (emlrtRTEInfo *)NULL);
  for (c2_i78 = 0; c2_i78 < 2; c2_i78++) {
    c2_bv4[c2_i78] = c2_bv5[c2_i78];
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_r0->data, 1, 1, 0U, 1, 0U,
                   2, c2_bv4, c2_uv4, c2_r0->size);
  c2_i79 = c2_y->size[0] * c2_y->size[1];
  c2_y->size[0] = 1;
  c2_y->size[1] = c2_r0->size[1];
  c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_y, c2_i79,
                       sizeof(real32_T), (emlrtRTEInfo *)NULL);
  c2_b_y = c2_y->size[0];
  c2_c_y = c2_y->size[1];
  c2_loop_ub = c2_r0->size[0] * c2_r0->size[1] - 1;
  for (c2_i80 = 0; c2_i80 <= c2_loop_ub; c2_i80++) {
    c2_y->data[c2_i80] = c2_r0->data[c2_i80];
  }

  sf_mex_destroy(&c2_u);
  c2_emxFree_real32_T(chartInstance, &c2_r0);
}

static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, c2_emxArray_real32_T *c2_outData)
{
  c2_emxArray_real32_T *c2_y;
  const mxArray *c2_angles;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_i81;
  int32_T c2_loop_ub;
  int32_T c2_i82;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_emxInit_real32_T(chartInstance, &c2_y, 2, (emlrtRTEInfo *)NULL);
  c2_angles = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_angles), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_angles);
  c2_i81 = c2_outData->size[0] * c2_outData->size[1];
  c2_outData->size[0] = 1;
  c2_outData->size[1] = c2_y->size[1];
  c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_outData, c2_i81,
                       sizeof(real32_T), (emlrtRTEInfo *)NULL);
  c2_loop_ub = c2_y->size[1] - 1;
  for (c2_i82 = 0; c2_i82 <= c2_loop_ub; c2_i82++) {
    c2_outData->data[c2_outData->size[0] * c2_i82] = c2_y->data[c2_y->size[0] *
      c2_i82];
  }

  c2_emxFree_real32_T(chartInstance, &c2_y);
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i83;
  const mxArray *c2_y = NULL;
  boolean_T c2_u[2048];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i83 = 0; c2_i83 < 2048; c2_i83++) {
    c2_u[c2_i83] = (*(boolean_T (*)[2048])c2_inData)[c2_i83];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 11, 0U, 1U, 0U, 1, 2048), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_bb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, boolean_T c2_y[2048])
{
  boolean_T c2_bv6[2048];
  int32_T c2_i84;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_bv6, 1, 11, 0U, 1, 0U, 1, 2048);
  for (c2_i84 = 0; c2_i84 < 2048; c2_i84++) {
    c2_y[c2_i84] = c2_bv6[c2_i84];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_validIdx;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  boolean_T c2_y[2048];
  int32_T c2_i85;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_validIdx = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_validIdx), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_validIdx);
  for (c2_i85 = 0; c2_i85 < 2048; c2_i85++) {
    (*(boolean_T (*)[2048])c2_outData)[c2_i85] = c2_y[c2_i85];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i86;
  const mxArray *c2_y = NULL;
  real32_T c2_u[2048];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i86 = 0; c2_i86 < 2048; c2_i86++) {
    c2_u[c2_i86] = (*(real32_T (*)[2048])c2_inData)[c2_i86];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 1, 0U, 1U, 0U, 1, 2048), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_R;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y[2048];
  int32_T c2_i87;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_R = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_R), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_R);
  for (c2_i87 = 0; c2_i87 < 2048; c2_i87++) {
    (*(real32_T (*)[2048])c2_outData)[c2_i87] = c2_y[c2_i87];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_sbz5YcIbiGQmS5RgvQnKlSH c2_u;
  const mxArray *c2_y = NULL;
  int32_T c2_i88;
  const mxArray *c2_b_y = NULL;
  real32_T c2_b_u[2];
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_sbz5YcIbiGQmS5RgvQnKlSH *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c2_i88 = 0; c2_i88 < 2; c2_i88++) {
    c2_b_u[c2_i88] = c2_u.RangeLimits[c2_i88];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 1, 0U, 1U, 0U, 2, 1, 2),
                false);
  sf_mex_addfield(c2_y, c2_b_y, "RangeLimits", "RangeLimits", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static c2_sbz5YcIbiGQmS5RgvQnKlSH c2_cb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_sbz5YcIbiGQmS5RgvQnKlSH c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[1] = { "RangeLimits" };

  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 1, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "RangeLimits";
  c2_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "RangeLimits", "RangeLimits", 0)), &c2_thisId, c2_y.RangeLimits);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_db_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real32_T c2_y[2])
{
  real32_T c2_fv1[2];
  int32_T c2_i89;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_fv1, 1, 1, 0U, 1, 0U, 2, 1, 2);
  for (c2_i89 = 0; c2_i89 < 2; c2_i89++) {
    c2_y[c2_i89] = c2_fv1[c2_i89];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_defaults;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_sbz5YcIbiGQmS5RgvQnKlSH c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_defaults = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_defaults),
    &c2_thisId);
  sf_mex_destroy(&c2_defaults);
  *(c2_sbz5YcIbiGQmS5RgvQnKlSH *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray
  *sf_c2_DroneControllerCollisionAvoidance_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  const char * c2_data[5] = {
    "789ced55dd4ae340149e888a5e2805c10710af1d04f5c22b351aad7fb4b5b58a484ddb633b667e6232d12a78bf0fb10fb397fb04b277fb0efb041b8d69938121"
    "c54a44706ece7c7c93f39df97286838ce29181109a45d12acc447126c66f710ca597ca1bca39237d1c4da0f1d47731fff32db60497d09311a084c371c09ae085",
    "80db0cfa69da82116e73597d700179e00b7a07ed57e69a50a81206872201f648089895a0fae0857ad99b5d68392701435ed71f944b930025fcb9d2dc7f3cc31f"
    "75a9fea8e7b2f42633f40c4d8cd7349a4aa0d58d58ef87265fd6fde2fc8b1abd82c287ae4be19503bbed8970630ae606124a420297c4a61601fafa5f47f53dae",
    "6352c183ba22c627bc4361a0f76744bd9e562fcd5fec5c9aebb8e683e7e3529750e2ba781b7c470a176f06bef46c4a80e372addaa80bcfc1278405e1f3701a15"
    "e1e3be7d78e7c06aecda8fd01478186797587efdf56bee9ff9bc996f3fe7fd7e3e4f2feea3f7f6e9bc46afa0f07b7074f660d69d7dfa7853edf173b8272ba635",
    "a8a394a1935507d2e0bcf27fbff7e1ee395c7f2ef4e789abc937acafb30a46cab998ef80b41ab60cfd6b497297dfdc688ba0999c1bbf47d46b6af5d2fcc7f751"
    "dac12516e9e4d8377f9fd6bee7c4579f139d95facd72b102a7b78e5361b5c02ddf1e6e6f7dfd39f11f62a7efe8",
    "" };

  c2_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(c2_data, 3368U, &c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_quat2eul(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, real_T c2_q[4], real_T c2_eul[3])
{
  int32_T c2_k;
  real_T c2_y;
  real_T c2_b_y[4];
  real_T c2_a;
  int32_T c2_b_k;
  real_T c2_c_y;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  boolean_T c2_p;
  boolean_T c2_b_p;
  real_T c2_d_y;
  real_T c2_e_y;
  real_T c2_z;
  int32_T c2_i90;
  real_T c2_b;
  int32_T c2_ak;
  int32_T c2_ck;
  real_T c2_qw;
  real_T c2_av;
  real_T c2_qx;
  real_T c2_cv;
  real_T c2_qy;
  real_T c2_qz;
  real_T c2_aSinInput;
  real_T c2_dv6[1];
  int32_T c2_i;
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_d_x;
  real_T c2_d_a;
  real_T c2_f_y;
  boolean_T c2_c_p;
  real_T c2_e_a;
  real_T c2_f_a;
  real_T c2_e_x;
  real_T c2_g_a;
  real_T c2_g_y;
  boolean_T c2_d_p;
  real_T c2_h_a;
  real_T c2_i_a;
  real_T c2_f_x;
  real_T c2_j_a;
  real_T c2_h_y;
  boolean_T c2_e_p;
  real_T c2_k_a;
  real_T c2_l_a;
  real_T c2_g_x;
  real_T c2_m_a;
  real_T c2_i_y;
  boolean_T c2_f_p;
  real_T c2_j_y;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_k_y;
  real_T c2_l_y;
  real_T c2_j_x;
  real_T c2_r;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_m_x;
  boolean_T c2_b0;
  boolean_T c2_b1;
  boolean_T c2_g_p;
  boolean_T c2_h_p;
  real_T c2_n_a;
  real_T c2_o_a;
  real_T c2_n_x;
  real_T c2_p_a;
  real_T c2_m_y;
  boolean_T c2_i_p;
  real_T c2_q_a;
  real_T c2_r_a;
  real_T c2_o_x;
  real_T c2_s_a;
  real_T c2_n_y;
  boolean_T c2_j_p;
  real_T c2_t_a;
  real_T c2_u_a;
  real_T c2_p_x;
  real_T c2_v_a;
  real_T c2_o_y;
  boolean_T c2_k_p;
  real_T c2_w_a;
  real_T c2_x_a;
  real_T c2_q_x;
  real_T c2_y_a;
  real_T c2_p_y;
  boolean_T c2_l_p;
  real_T c2_q_y;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_r_y;
  real_T c2_s_y;
  real_T c2_t_x;
  real_T c2_b_r;
  for (c2_k = 0; c2_k + 1 < 5; c2_k++) {
    c2_a = c2_q[c2_k];
    c2_c_y = c2_a * c2_a;
    c2_b_y[c2_k] = c2_c_y;
  }

  c2_y = c2_b_y[0];
  for (c2_b_k = 1; c2_b_k + 1 < 5; c2_b_k++) {
    c2_y += c2_b_y[c2_b_k];
  }

  c2_x = c2_y;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_p = (c2_c_x < 0.0);
  c2_b_p = c2_p;
  if (c2_b_p) {
    c2_b_error(chartInstance);
  }

  c2_b_x = muDoubleScalarSqrt(c2_b_x);
  c2_d_y = c2_b_x;
  c2_e_y = c2_d_y;
  c2_z = 1.0 / c2_e_y;
  for (c2_i90 = 0; c2_i90 < 4; c2_i90++) {
    c2_b_y[c2_i90] = c2_q[c2_i90];
  }

  c2_b = c2_z;
  c2_ak = 0;
  for (c2_ck = 0; c2_ck < 4; c2_ck++) {
    c2_av = c2_b_y[c2_ak];
    c2_cv = c2_av * c2_b;
    c2_q[c2_ck] = c2_cv;
    c2_ak++;
  }

  c2_qw = c2_q[0];
  c2_qx = c2_q[1];
  c2_qy = c2_q[2];
  c2_qz = c2_q[3];
  c2_aSinInput = -2.0 * (c2_qx * c2_qz - c2_qw * c2_qy);
  c2_dv6[0] = c2_aSinInput;
  for (c2_i = 0; c2_i < 1; c2_i++) {
    if (c2_aSinInput > 1.0) {
      c2_dv6[c2_i] = 1.0;
    }
  }

  c2_aSinInput = c2_dv6[0];
  c2_b_a = c2_qw;
  c2_c_a = c2_b_a;
  c2_d_x = c2_c_a;
  c2_d_a = c2_d_x;
  c2_f_y = c2_d_a * c2_d_a;
  c2_c_p = false;
  if (c2_c_p) {
    c2_error(chartInstance);
  }

  c2_e_a = c2_qx;
  c2_f_a = c2_e_a;
  c2_e_x = c2_f_a;
  c2_g_a = c2_e_x;
  c2_g_y = c2_g_a * c2_g_a;
  c2_d_p = false;
  if (c2_d_p) {
    c2_error(chartInstance);
  }

  c2_h_a = c2_qy;
  c2_i_a = c2_h_a;
  c2_f_x = c2_i_a;
  c2_j_a = c2_f_x;
  c2_h_y = c2_j_a * c2_j_a;
  c2_e_p = false;
  if (c2_e_p) {
    c2_error(chartInstance);
  }

  c2_k_a = c2_qz;
  c2_l_a = c2_k_a;
  c2_g_x = c2_l_a;
  c2_m_a = c2_g_x;
  c2_i_y = c2_m_a * c2_m_a;
  c2_f_p = false;
  if (c2_f_p) {
    c2_error(chartInstance);
  }

  c2_j_y = 2.0 * (c2_qx * c2_qy + c2_qw * c2_qz);
  c2_h_x = ((c2_f_y + c2_g_y) - c2_h_y) - c2_i_y;
  c2_i_x = c2_j_y;
  c2_k_y = c2_h_x;
  c2_l_y = c2_i_x;
  c2_j_x = c2_k_y;
  c2_r = muDoubleScalarAtan2(c2_l_y, c2_j_x);
  c2_k_x = c2_aSinInput;
  c2_l_x = c2_k_x;
  c2_m_x = c2_l_x;
  c2_b0 = (c2_m_x < -1.0);
  c2_b1 = (c2_m_x > 1.0);
  c2_g_p = (c2_b0 || c2_b1);
  c2_h_p = c2_g_p;
  if (c2_h_p) {
    c2_c_error(chartInstance);
  }

  c2_l_x = muDoubleScalarAsin(c2_l_x);
  c2_n_a = c2_qw;
  c2_o_a = c2_n_a;
  c2_n_x = c2_o_a;
  c2_p_a = c2_n_x;
  c2_m_y = c2_p_a * c2_p_a;
  c2_i_p = false;
  if (c2_i_p) {
    c2_error(chartInstance);
  }

  c2_q_a = c2_qx;
  c2_r_a = c2_q_a;
  c2_o_x = c2_r_a;
  c2_s_a = c2_o_x;
  c2_n_y = c2_s_a * c2_s_a;
  c2_j_p = false;
  if (c2_j_p) {
    c2_error(chartInstance);
  }

  c2_t_a = c2_qy;
  c2_u_a = c2_t_a;
  c2_p_x = c2_u_a;
  c2_v_a = c2_p_x;
  c2_o_y = c2_v_a * c2_v_a;
  c2_k_p = false;
  if (c2_k_p) {
    c2_error(chartInstance);
  }

  c2_w_a = c2_qz;
  c2_x_a = c2_w_a;
  c2_q_x = c2_x_a;
  c2_y_a = c2_q_x;
  c2_p_y = c2_y_a * c2_y_a;
  c2_l_p = false;
  if (c2_l_p) {
    c2_error(chartInstance);
  }

  c2_q_y = 2.0 * (c2_qy * c2_qz + c2_qw * c2_qx);
  c2_r_x = ((c2_m_y - c2_n_y) - c2_o_y) + c2_p_y;
  c2_s_x = c2_q_y;
  c2_r_y = c2_r_x;
  c2_s_y = c2_s_x;
  c2_t_x = c2_r_y;
  c2_b_r = muDoubleScalarAtan2(c2_s_y, c2_t_x);
  c2_eul[0] = c2_r;
  c2_eul[1] = c2_l_x;
  c2_eul[2] = c2_b_r;
}

static void c2_error(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
                     *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_cv0[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv0, 10, 0U, 1U, 0U, 2, 1, 31),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static void c2_b_error(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_cv1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c2_b_y = NULL;
  static char_T c2_cv2[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv2, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static void c2_c_error(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_cv3[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c2_b_y = NULL;
  static char_T c2_cv4[4] = { 'a', 's', 'i', 'n' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv3, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv4, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static void c2_power(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
                     *chartInstance, real_T c2_a[3], real_T c2_y[3])
{
  int32_T c2_k;
  real_T c2_b_a;
  real_T c2_b_y;
  (void)chartInstance;
  for (c2_k = 0; c2_k + 1 < 4; c2_k++) {
    c2_b_a = c2_a[c2_k];
    c2_b_y = c2_b_a * c2_b_a;
    c2_y[c2_k] = c2_b_y;
  }
}

static void c2_findObstacle(SFc2_DroneControllerCollisionAvoidanceInstanceStruct
  *chartInstance, c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *c2_obj,
  real_T c2_pose[4], real32_T *c2_minDist, real_T c2_dataWorld[2])
{
  c2_emxArray_real32_T *c2_angles;
  uint32_T c2_debug_family_var_map[15];
  c2_sbz5YcIbiGQmS5RgvQnKlSH c2_defaults;
  real32_T c2_R[2048];
  boolean_T c2_validIdx[2048];
  real32_T c2_cartAngles_data[2048];
  int32_T c2_cartAngles_size[2];
  real32_T c2_x;
  real32_T c2_y;
  real_T c2_cart[2];
  real_T c2_th;
  real32_T c2_R_data[2048];
  int32_T c2_R_size[1];
  real_T c2_validIdx_data[2048];
  int32_T c2_validIdx_size[1];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 2.0;
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_i93;
  real32_T c2_b_x[2048];
  int32_T c2_i94;
  boolean_T c2_b[2048];
  int32_T c2_i95;
  int32_T c2_i96;
  boolean_T c2_b_b[2048];
  int32_T c2_i97;
  int32_T c2_i98;
  int32_T c2_i99;
  real32_T c2_b_defaults;
  int32_T c2_i100;
  int32_T c2_i101;
  int32_T c2_trueCount;
  int32_T c2_i;
  int32_T c2_partialTrueCount;
  int32_T c2_b_i;
  int32_T c2_varargin_1_size[1];
  int32_T c2_loop_ub;
  int32_T c2_i102;
  real32_T c2_varargin_1_data[2048];
  boolean_T c2_b2;
  const mxArray *c2_b_y = NULL;
  static char_T c2_cv5[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c', 'o',
    'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  const mxArray *c2_c_y = NULL;
  static char_T c2_cv6[39] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'e', 'm', 'l', '_', 'm', 'i', 'n', '_', 'o', 'r', '_',
    'm', 'a', 'x', '_', 'v', 'a', 'r', 'D', 'i', 'm', 'Z', 'e', 'r', 'o' };

  int32_T c2_ixstart;
  int32_T c2_n;
  real32_T c2_mtmp;
  real32_T c2_c_x;
  boolean_T c2_foundnan;
  int32_T c2_x_size[1];
  int32_T c2_ix;
  int32_T c2_b_loop_ub;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_b_ix;
  real32_T c2_d_x;
  int32_T c2_nx;
  boolean_T c2_x_data[2048];
  boolean_T c2_c_b;
  int32_T c2_idx;
  real32_T c2_a;
  int32_T c2_ii_size[1];
  real32_T c2_d_b;
  int32_T c2_ii;
  boolean_T c2_p;
  int32_T c2_b_a;
  static char_T c2_cv7[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'b', 'u', 'i', 'l',
    't', 'i', 'n', 's', ':', 'A', 's', 's', 'e', 'r', 't', 'i', 'o', 'n', 'F',
    'a', 'i', 'l', 'e', 'd' };

  int32_T c2_ii_data[2048];
  boolean_T c2_b3;
  boolean_T c2_b4;
  boolean_T c2_b5;
  int32_T c2_i105;
  int32_T c2_c_loop_ub;
  int32_T c2_i106;
  int32_T c2_tmp_size[2];
  int32_T c2_d_loop_ub;
  int32_T c2_i107;
  real32_T c2_c_a;
  real32_T c2_d;
  int32_T c2_tmp_data[2048];
  real32_T c2_e_b;
  real32_T c2_e_x;
  boolean_T c2_f_b;
  real32_T c2_f_x;
  boolean_T c2_g_b;
  int32_T c2_i108;
  real32_T c2_g_x;
  boolean_T c2_h_b;
  int32_T c2_b_angles;
  int32_T c2_c_angles;
  int32_T c2_i109;
  real32_T c2_h_x;
  int32_T c2_i110;
  boolean_T c2_i_b;
  int32_T c2_d_angles;
  real32_T c2_i_x;
  int32_T c2_e_angles;
  int32_T c2_indexSize;
  int32_T c2_matrixSize[2];
  boolean_T c2_j_b;
  real32_T c2_j_x;
  boolean_T c2_k_b;
  boolean_T c2_l_b;
  int32_T c2_sz;
  real32_T c2_k_x;
  boolean_T c2_c;
  boolean_T c2_nonSingletonDimFound;
  boolean_T c2_m_b;
  int32_T c2_i111;
  boolean_T c2_b_c;
  real32_T c2_l_x;
  int32_T c2_i112;
  boolean_T c2_n_b;
  real32_T c2_m_x;
  const mxArray *c2_d_y = NULL;
  int32_T c2_f_angles;
  static char_T c2_cv8[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'F', 'E', ':', 'P',
    'o', 't', 'e', 'n', 't', 'i', 'a', 'l', 'V', 'e', 'c', 't', 'o', 'r', 'V',
    'e', 'c', 't', 'o', 'r' };

  int32_T c2_g_angles[1];
  int32_T c2_h_angles;
  int32_T c2_i_angles;
  real32_T c2_n_x;
  int32_T c2_j_angles;
  real32_T c2_d_a;
  real32_T c2_o_x;
  int32_T c2_k_angles;
  real32_T c2_b_d;
  int32_T c2_e_loop_ub;
  real32_T c2_o_b;
  int32_T c2_i113;
  real32_T c2_e_a;
  int32_T c2_i114;
  real32_T c2_c_d;
  real32_T c2_p_b;
  real32_T c2_anew;
  int32_T c2_b_x_size[2];
  real_T c2_p_x;
  int32_T c2_f_loop_ub;
  real_T c2_ndbl;
  int32_T c2_i115;
  int32_T c2_q_x;
  int32_T c2_r_x;
  real_T c2_apnd;
  int32_T c2_g_loop_ub;
  int32_T c2_i116;
  real_T c2_cdiff;
  real_T c2_s_x;
  int32_T c2_b_nx;
  real32_T c2_b_x_data[2048];
  real_T c2_t_x;
  int32_T c2_k;
  real_T c2_u_x;
  real_T c2_e_y;
  real_T c2_f_a;
  int32_T c2_i117;
  real32_T c2_v_x;
  real_T c2_q_b;
  real32_T c2_w_x;
  real_T c2_x_x;
  real_T c2_y_x;
  int32_T c2_i118;
  int32_T c2_iv0[2];
  real_T c2_ab_x;
  real_T c2_absa;
  real_T c2_bb_x;
  int32_T c2_cb_x[2];
  real_T c2_db_x;
  int32_T c2_b_R[2];
  real_T c2_eb_x;
  real_T c2_absb;
  int32_T c2_c_R;
  real_T c2_c_c;
  real_T c2_r_b;
  int32_T c2_h_loop_ub;
  real32_T c2_f_y;
  int32_T c2_i119;
  real32_T c2_bnew;
  int32_T c2_iv1[1];
  int32_T c2_varargin_1[1];
  real_T c2_g_a;
  real_T c2_h_a;
  real_T c2_flt;
  boolean_T c2_n_too_large;
  int32_T c2_b_n;
  int32_T c2_fb_x;
  int32_T c2_gb_x;
  boolean_T c2_b_p;
  int32_T c2_i_loop_ub;
  int32_T c2_i120;
  const mxArray *c2_g_y = NULL;
  static char_T c2_cv9[21] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'p', 'm', 'a', 'x', 's', 'i', 'z', 'e' };

  int32_T c2_i121;
  int32_T c2_c_nx;
  int32_T c2_b_k;
  int32_T c2_i122;
  real32_T c2_hb_x;
  real32_T c2_ib_x;
  int32_T c2_i_a;
  int32_T c2_i123;
  int32_T c2_iv2[2];
  int32_T c2_nm1;
  int32_T c2_j_a;
  int32_T c2_nm1d2;
  int32_T c2_jb_x[2];
  int32_T c2_k_a;
  int32_T c2_d_R[2];
  int32_T c2_i124;
  int32_T c2_c_k;
  int32_T c2_e_R;
  int32_T c2_j_loop_ub;
  int32_T c2_s_b;
  real32_T c2_kd;
  int32_T c2_i125;
  int32_T c2_d_c;
  int32_T c2_l_a;
  int32_T c2_e_c;
  int32_T c2_m_a;
  int32_T c2_iv3[1];
  int32_T c2_n_a;
  int32_T c2_f_c;
  int32_T c2_o_a;
  int32_T c2_b_varargin_1[1];
  int32_T c2_g_c;
  int32_T c2_t_b;
  int32_T c2_h_c;
  int32_T c2_p_a;
  int32_T c2_i_c;
  real_T c2_q_a[2];
  int32_T c2_i126;
  int32_T c2_i127;
  int32_T c2_i128;
  int32_T c2_i129;
  real_T c2_kb_x;
  real_T c2_lb_x;
  static real_T c2_u_b[4] = { 0.0, 1.0, 1.0, 0.0 };

  real_T c2_mb_x;
  real_T c2_nb_x;
  real_T c2_ob_x;
  real_T c2_pb_x;
  real_T c2_qb_x;
  real_T c2_rb_x;
  int32_T c2_i130;
  real_T c2_v_b[4];
  int32_T c2_i131;
  int32_T c2_i132;
  int32_T c2_i133;
  real_T c2_h_y[2];
  int32_T c2_i134;
  int32_T c2_jcol;
  int32_T c2_i135;
  int32_T c2_iacol;
  int32_T c2_ibmat;
  int32_T c2_ibcol;
  real_T c2_w_b[2];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T exitg1;
  c2_emxInit_real32_T(chartInstance, &c2_angles, 2, &c2_h_emlrtRTEI);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 15U, 17U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_defaults, 0U, c2_q_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, MAX_uint32_T, c2_p_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_validIdx, MAX_uint32_T,
    c2_o_sf_marshallOut, c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_EMX_IMPORTABLE(c2_angles->data, (const int32_T *)
    c2_angles->size, NULL, 0, 3, (void *)c2_n_sf_marshallOut, (void *)
    c2_m_sf_marshallIn, (void *)c2_angles, true);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_EMX_IMPORTABLE((void *)&c2_cartAngles_data, (
    const int32_T *)&c2_cartAngles_size, NULL, 0, 4, (void *)c2_m_sf_marshallOut,
    (void *)c2_l_sf_marshallIn, (void *)&c2_cartAngles_data, false);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x, 5U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y, 6U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_cart, 7U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_th, 8U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_EMX_IMPORTABLE((void *)&c2_R_data, (const
    int32_T *)&c2_R_size, NULL, 0, -1, (void *)c2_l_sf_marshallOut, (void *)
    c2_k_sf_marshallIn, (void *)&c2_R_data, false);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_EMX_IMPORTABLE((void *)&c2_validIdx_data, (const
    int32_T *)&c2_validIdx_size, NULL, 0, -1, (void *)c2_k_sf_marshallOut, (void
    *)c2_j_sf_marshallIn, (void *)&c2_validIdx_data, false);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 9U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 10U, c2_d_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_obj, 11U, c2_b_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_pose, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_minDist, 13U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dataWorld, 14U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_defaults.RangeLimits[0] = 0.5F;
  c2_defaults.RangeLimits[1] = c2_obj->RangeMax;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  for (c2_i91 = 0; c2_i91 < 2048; c2_i91++) {
    c2_R[c2_i91] = c2_obj->Ranges[c2_i91];
  }

  _SFD_SYMBOL_SWITCH(1U, 1U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  for (c2_i92 = 0; c2_i92 < 2048; c2_i92++) {
    c2_b_x[c2_i92] = c2_R[c2_i92];
  }

  for (c2_i93 = 0; c2_i93 < 2048; c2_i93++) {
    c2_b[c2_i93] = muSingleScalarIsInf(c2_b_x[c2_i93]);
  }

  for (c2_i94 = 0; c2_i94 < 2048; c2_i94++) {
    c2_b[c2_i94] = !c2_b[c2_i94];
  }

  for (c2_i95 = 0; c2_i95 < 2048; c2_i95++) {
    c2_b_b[c2_i95] = muSingleScalarIsNaN(c2_b_x[c2_i95]);
  }

  for (c2_i96 = 0; c2_i96 < 2048; c2_i96++) {
    c2_b_b[c2_i96] = !c2_b_b[c2_i96];
  }

  for (c2_i97 = 0; c2_i97 < 2048; c2_i97++) {
    c2_b[c2_i97] = (c2_b[c2_i97] && c2_b_b[c2_i97]);
  }

  for (c2_i98 = 0; c2_i98 < 2048; c2_i98++) {
    c2_b_b[c2_i98] = (c2_R[c2_i98] >= 0.5F);
  }

  for (c2_i99 = 0; c2_i99 < 2048; c2_i99++) {
    c2_b[c2_i99] = (c2_b[c2_i99] && c2_b_b[c2_i99]);
  }

  c2_b_defaults = c2_defaults.RangeLimits[1];
  for (c2_i100 = 0; c2_i100 < 2048; c2_i100++) {
    c2_b_b[c2_i100] = (c2_R[c2_i100] <= c2_b_defaults);
  }

  for (c2_i101 = 0; c2_i101 < 2048; c2_i101++) {
    c2_validIdx[c2_i101] = (c2_b[c2_i101] && c2_b_b[c2_i101]);
  }

  _SFD_SYMBOL_SWITCH(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_trueCount = 0;
  c2_i = 0;
  while (c2_i <= 2047) {
    if (c2_validIdx[c2_i]) {
      c2_trueCount++;
    }

    c2_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  c2_R_size[0] = c2_trueCount;
  _SFD_SYMBOL_SWITCH(1U, 9U);
  c2_partialTrueCount = 0;
  c2_b_i = 0;
  while (c2_b_i <= 2047) {
    if (c2_validIdx[c2_b_i]) {
      c2_R_data[c2_partialTrueCount] = c2_R[c2_b_i];
      _SFD_SYMBOL_SWITCH(1U, 9U);
      c2_partialTrueCount++;
    }

    c2_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_varargin_1_size[0] = c2_R_size[0];
  c2_loop_ub = c2_R_size[0] - 1;
  for (c2_i102 = 0; c2_i102 <= c2_loop_ub; c2_i102++) {
    c2_varargin_1_data[c2_i102] = c2_R_data[c2_i102];
  }

  if ((c2_varargin_1_size[0] == 1) || ((real_T)c2_varargin_1_size[0] != 1.0)) {
    c2_b2 = true;
  } else {
    c2_b2 = false;
  }

  if (c2_b2) {
  } else {
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_cv5, 10, 0U, 1U, 0U, 2, 1, 36),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_b_y));
  }

  if ((real_T)c2_varargin_1_size[0] > 0.0) {
  } else {
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_cv6, 10, 0U, 1U, 0U, 2, 1, 39),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_c_y));
  }

  c2_ixstart = 1;
  c2_n = c2_varargin_1_size[0];
  c2_mtmp = c2_varargin_1_data[0];
  if (c2_n > 1) {
    c2_c_x = c2_mtmp;
    c2_foundnan = muSingleScalarIsNaN(c2_c_x);
    if (c2_foundnan) {
      c2_ix = 1;
      exitg1 = false;
      while ((!exitg1) && (c2_ix + 1 <= c2_n)) {
        c2_ixstart = c2_ix + 1;
        c2_d_x = c2_varargin_1_data[c2_ix];
        c2_c_b = muSingleScalarIsNaN(c2_d_x);
        if (!c2_c_b) {
          c2_mtmp = c2_varargin_1_data[c2_ix];
          exitg1 = true;
        } else {
          c2_ix++;
        }
      }
    }

    if (c2_ixstart < c2_n) {
      c2_i104 = c2_ixstart;
      for (c2_b_ix = c2_i104; c2_b_ix + 1 <= c2_n; c2_b_ix++) {
        c2_a = c2_varargin_1_data[c2_b_ix];
        c2_d_b = c2_mtmp;
        c2_p = (c2_a < c2_d_b);
        if (c2_p) {
          c2_mtmp = c2_varargin_1_data[c2_b_ix];
        }
      }
    }
  }

  *c2_minDist = c2_mtmp;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
  c2_x_size[0] = c2_R_size[0];
  c2_b_loop_ub = c2_R_size[0] - 1;
  for (c2_i103 = 0; c2_i103 <= c2_b_loop_ub; c2_i103++) {
    c2_x_data[c2_i103] = (c2_R_data[c2_i103] == *c2_minDist);
  }

  c2_nx = c2_x_size[0];
  c2_idx = 0;
  c2_ii_size[0] = c2_nx;
  c2_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (c2_ii <= c2_nx)) {
    if (c2_x_data[c2_ii - 1]) {
      c2_b_a = c2_idx + 1;
      c2_idx = c2_b_a;
      c2_ii_data[c2_idx - 1] = c2_ii;
      if (c2_idx >= c2_nx) {
        exitg1 = true;
      } else {
        c2_ii++;
      }
    } else {
      c2_ii++;
    }
  }

  if (c2_idx <= c2_nx) {
  } else {
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_emlrt_marshallOut(chartInstance, c2_cv7)));
  }

  if (c2_nx == 1) {
    if (c2_idx == 0) {
      c2_ii_size[0] = 0;
    }
  } else {
    c2_b3 = (1 > c2_idx);
    c2_b4 = c2_b3;
    c2_b5 = c2_b4;
    if (c2_b5) {
      c2_i105 = 0;
    } else {
      c2_i105 = c2_idx;
    }

    c2_tmp_size[0] = 1;
    c2_tmp_size[1] = c2_i105;
    c2_d_loop_ub = c2_i105 - 1;
    for (c2_i107 = 0; c2_i107 <= c2_d_loop_ub; c2_i107++) {
      c2_tmp_data[c2_tmp_size[0] * c2_i107] = 1 + c2_i107;
    }

    c2_indexShapeCheck(chartInstance, c2_ii_size[0], c2_tmp_size);
    c2_ii_size[0] = c2_i105;
  }

  c2_validIdx_size[0] = c2_ii_size[0];
  c2_c_loop_ub = c2_ii_size[0] - 1;
  for (c2_i106 = 0; c2_i106 <= c2_c_loop_ub; c2_i106++) {
    c2_validIdx_data[c2_i106] = (real_T)c2_ii_data[c2_i106];
  }

  _SFD_SYMBOL_SWITCH(2U, 10U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 34);
  c2_c_a = c2_obj->AngleMin;
  c2_d = c2_obj->AngleIncrement;
  c2_e_b = c2_obj->AngleMax;
  c2_e_x = c2_c_a;
  c2_f_b = muSingleScalarIsNaN(c2_e_x);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (c2_f_b) {
    guard2 = true;
  } else {
    c2_f_x = c2_d;
    c2_g_b = muSingleScalarIsNaN(c2_f_x);
    if (c2_g_b) {
      guard2 = true;
    } else {
      c2_g_x = c2_e_b;
      c2_h_b = muSingleScalarIsNaN(c2_g_x);
      if (c2_h_b) {
        guard2 = true;
      } else if ((c2_d == 0.0F) || ((c2_c_a < c2_e_b) && (c2_d < 0.0F)) ||
                 ((c2_e_b < c2_c_a) && (c2_d > 0.0F))) {
        c2_i109 = c2_angles->size[0] * c2_angles->size[1];
        c2_angles->size[0] = 1;
        c2_angles->size[1] = 0;
        c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_angles,
                             c2_i109, sizeof(real32_T), &c2_h_emlrtRTEI);
        c2_d_angles = c2_angles->size[0];
        c2_e_angles = c2_angles->size[1];
      } else {
        c2_h_x = c2_c_a;
        c2_i_b = muSingleScalarIsInf(c2_h_x);
        if (c2_i_b) {
          guard4 = true;
        } else {
          c2_i_x = c2_e_b;
          c2_j_b = muSingleScalarIsInf(c2_i_x);
          if (c2_j_b) {
            guard4 = true;
          } else {
            guard3 = true;
          }
        }
      }
    }
  }

  if (guard4) {
    c2_j_x = c2_d;
    c2_l_b = muSingleScalarIsInf(c2_j_x);
    if (c2_l_b || (c2_c_a == c2_e_b)) {
      c2_i111 = c2_angles->size[0] * c2_angles->size[1];
      c2_angles->size[0] = 1;
      c2_angles->size[1] = 1;
      c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_angles,
                           c2_i111, sizeof(real32_T), &c2_h_emlrtRTEI);
      c2_f_angles = c2_angles->size[0];
      c2_h_angles = c2_angles->size[1];
      c2_angles->data[0] = ((real32_T)rtNaN);
    } else {
      guard3 = true;
    }
  }

  if (guard3) {
    c2_k_x = c2_d;
    c2_m_b = muSingleScalarIsInf(c2_k_x);
    if (c2_m_b) {
      c2_i112 = c2_angles->size[0] * c2_angles->size[1];
      c2_angles->size[0] = 1;
      c2_angles->size[1] = 1;
      c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_angles,
                           c2_i112, sizeof(real32_T), &c2_h_emlrtRTEI);
      c2_j_angles = c2_angles->size[0];
      c2_k_angles = c2_angles->size[1];
      c2_angles->data[0] = c2_c_a;
    } else {
      c2_l_x = c2_c_a;
      c2_m_x = c2_l_x;
      c2_m_x = muSingleScalarFloor(c2_m_x);
      if (c2_m_x == c2_c_a) {
        c2_n_x = c2_d;
        c2_o_x = c2_n_x;
        c2_o_x = muSingleScalarFloor(c2_o_x);
        if (c2_o_x == c2_d) {
          c2_i114 = c2_angles->size[0] * c2_angles->size[1];
          c2_angles->size[0] = 1;
          c2_angles->size[1] = (int32_T)muSingleScalarFloor((c2_e_b - c2_c_a) /
            c2_d) + 1;
          c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_angles,
                               c2_i114, sizeof(real32_T), &c2_h_emlrtRTEI);
          c2_f_loop_ub = (int32_T)muSingleScalarFloor((c2_e_b - c2_c_a) / c2_d);
          for (c2_i115 = 0; c2_i115 <= c2_f_loop_ub; c2_i115++) {
            c2_angles->data[c2_angles->size[0] * c2_i115] = c2_c_a + c2_d *
              (real32_T)c2_i115;
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }
  }

  if (guard2) {
    c2_i108 = c2_angles->size[0] * c2_angles->size[1];
    c2_angles->size[0] = 1;
    c2_angles->size[1] = 1;
    c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_angles,
                         c2_i108, sizeof(real32_T), &c2_h_emlrtRTEI);
    c2_b_angles = c2_angles->size[0];
    c2_c_angles = c2_angles->size[1];
    c2_angles->data[0] = ((real32_T)rtNaN);
  }

  if (guard1) {
    c2_d_a = c2_c_a;
    c2_b_d = c2_d;
    c2_o_b = c2_e_b;
    c2_e_a = c2_d_a;
    c2_c_d = c2_b_d;
    c2_p_b = c2_o_b;
    c2_anew = c2_e_a;
    c2_p_x = ((real_T)c2_p_b - (real_T)c2_e_a) / (real_T)c2_c_d + 0.5;
    c2_ndbl = c2_p_x;
    c2_ndbl = muDoubleScalarFloor(c2_ndbl);
    c2_apnd = (real_T)c2_e_a + c2_ndbl * (real_T)c2_c_d;
    if (c2_c_d > 0.0F) {
      c2_cdiff = c2_apnd - (real_T)c2_p_b;
    } else {
      c2_cdiff = (real_T)c2_p_b - c2_apnd;
    }

    c2_s_x = c2_cdiff;
    c2_t_x = c2_s_x;
    c2_u_x = c2_t_x;
    c2_e_y = muDoubleScalarAbs(c2_u_x);
    c2_f_a = c2_e_a;
    c2_q_b = c2_p_b;
    c2_x_x = c2_f_a;
    c2_y_x = c2_x_x;
    c2_ab_x = c2_y_x;
    c2_absa = muDoubleScalarAbs(c2_ab_x);
    c2_bb_x = c2_q_b;
    c2_db_x = c2_bb_x;
    c2_eb_x = c2_db_x;
    c2_absb = muDoubleScalarAbs(c2_eb_x);
    c2_c_c = muDoubleScalarMax(c2_absa, c2_absb);
    c2_r_b = c2_c_c;
    c2_f_y = 2.38418579E-7F * (real32_T)c2_r_b;
    if (c2_e_y < (real_T)c2_f_y) {
      c2_ndbl++;
      c2_bnew = c2_p_b;
    } else if (c2_cdiff > 0.0) {
      c2_bnew = (real32_T)((real_T)c2_e_a + (c2_ndbl - 1.0) * (real_T)c2_c_d);
    } else {
      c2_ndbl++;
      c2_bnew = (real32_T)c2_apnd;
    }

    c2_g_a = c2_ndbl;
    c2_h_a = c2_g_a;
    c2_flt = c2_h_a;
    c2_n_too_large = (2.147483647E+9 < c2_flt);
    if (c2_ndbl >= 0.0) {
      c2_b_n = (int32_T)muDoubleScalarFloor(c2_ndbl);
    } else {
      c2_b_n = 0;
    }

    c2_b_p = !c2_n_too_large;
    if (c2_b_p) {
    } else {
      c2_g_y = NULL;
      sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_cv9, 10, 0U, 1U, 0U, 2, 1, 21),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 1U, 14, c2_g_y));
    }

    c2_i121 = c2_angles->size[0] * c2_angles->size[1];
    c2_angles->size[0] = 1;
    c2_angles->size[1] = c2_b_n;
    c2_emxEnsureCapacity(chartInstance, (c2_emxArray__common *)c2_angles,
                         c2_i121, sizeof(real32_T), &c2_m_emlrtRTEI);
    if (c2_b_n > 0) {
      c2_angles->data[0] = c2_anew;
      if (c2_b_n > 1) {
        c2_angles->data[c2_b_n - 1] = c2_bnew;
        c2_i_a = c2_b_n - 1;
        c2_nm1 = c2_i_a;
        c2_j_a = c2_nm1;
        c2_nm1d2 = c2_div_nzp_s32(chartInstance, c2_j_a, 2, 1U, 1135U, 50U);
        c2_k_a = c2_nm1d2 - 1;
        c2_i124 = c2_k_a;
        for (c2_c_k = 1; c2_c_k <= c2_i124; c2_c_k++) {
          c2_kd = (real32_T)c2_c_k * c2_b_d;
          c2_l_a = c2_c_k + 1;
          c2_e_c = c2_l_a - 1;
          c2_angles->data[c2_e_c] = c2_anew + c2_kd;
          c2_o_a = c2_b_n;
          c2_t_b = c2_c_k;
          c2_h_c = (c2_o_a - c2_t_b) - 1;
          c2_angles->data[c2_h_c] = c2_bnew - c2_kd;
        }

        c2_s_b = c2_nm1d2;
        c2_d_c = c2_s_b << 1;
        if (c2_d_c == c2_nm1) {
          c2_m_a = c2_nm1d2 + 1;
          c2_f_c = c2_m_a - 1;
          c2_angles->data[c2_f_c] = (c2_anew + c2_bnew) / 2.0F;
        } else {
          c2_kd = (real32_T)c2_nm1d2 * c2_b_d;
          c2_n_a = c2_nm1d2 + 1;
          c2_g_c = c2_n_a - 1;
          c2_angles->data[c2_g_c] = c2_anew + c2_kd;
          c2_p_a = c2_nm1d2 + 2;
          c2_i_c = c2_p_a - 1;
          c2_angles->data[c2_i_c] = c2_bnew - c2_kd;
        }
      }
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
  for (c2_i110 = 0; c2_i110 < 2; c2_i110++) {
    c2_matrixSize[c2_i110] = c2_angles->size[c2_i110];
  }

  c2_indexSize = c2_validIdx_size[0];
  c2_k_b = !(c2_matrixSize[1] != 1);
  if (c2_k_b) {
    c2_sz = c2_indexSize;
    c2_nonSingletonDimFound = false;
    if (c2_sz != 1) {
      c2_nonSingletonDimFound = true;
    }

    c2_n_b = c2_nonSingletonDimFound;
    if (c2_n_b) {
      c2_c = true;
    } else {
      c2_c = false;
    }
  } else {
    c2_c = false;
  }

  c2_b_c = c2_c;
  if (!c2_b_c) {
  } else {
    c2_d_y = NULL;
    sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_cv8, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_d_y));
  }

  c2_g_angles[0] = c2_angles->size[1];
  c2_i_angles = c2_angles->size[1];
  c2_cartAngles_size[0] = 1;
  c2_cartAngles_size[1] = c2_validIdx_size[0];
  c2_e_loop_ub = c2_validIdx_size[0] - 1;
  for (c2_i113 = 0; c2_i113 <= c2_e_loop_ub; c2_i113++) {
    c2_cartAngles_data[c2_cartAngles_size[0] * c2_i113] = c2_angles->
      data[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 1200, 16, MAX_uint32_T, (int32_T)
      c2_validIdx_data[c2_i113], 1, c2_i_angles) - 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
  c2_b_x_size[0] = 1;
  c2_b_x_size[1] = c2_cartAngles_size[1];
  c2_q_x = c2_b_x_size[0];
  c2_r_x = c2_b_x_size[1];
  c2_g_loop_ub = c2_cartAngles_size[0] * c2_cartAngles_size[1] - 1;
  for (c2_i116 = 0; c2_i116 <= c2_g_loop_ub; c2_i116++) {
    c2_b_x_data[c2_i116] = c2_cartAngles_data[c2_i116];
  }

  c2_b_nx = c2_b_x_size[1];
  for (c2_k = 0; c2_k + 1 <= c2_b_nx; c2_k++) {
    c2_v_x = c2_b_x_data[c2_k];
    c2_w_x = c2_v_x;
    c2_w_x = muSingleScalarCos(c2_w_x);
    c2_b_x_data[c2_k] = c2_w_x;
  }

  for (c2_i117 = 0; c2_i117 < 2; c2_i117++) {
    c2_iv0[c2_i117] = 1;
  }

  for (c2_i118 = 0; c2_i118 < 2; c2_i118++) {
    c2_cb_x[c2_i118] = c2_b_x_size[c2_i118];
  }

  _SFD_SIZE_EQ_CHECK_ND(c2_iv0, c2_cb_x, 2);
  c2_b_R[0] = c2_R_size[0];
  c2_b_R[1] = 1;
  c2_c_R = c2_R_size[0];
  c2_varargin_1_size[0] = c2_validIdx_size[0];
  c2_h_loop_ub = c2_validIdx_size[0] - 1;
  for (c2_i119 = 0; c2_i119 <= c2_h_loop_ub; c2_i119++) {
    c2_varargin_1_data[c2_i119] = c2_R_data[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1241, 13, MAX_uint32_T,
       (int32_T)c2_validIdx_data[c2_i119], 1, c2_c_R) - 1];
  }

  c2_iv1[0] = 1;
  c2_varargin_1[0] = c2_varargin_1_size[0];
  _SFD_SIZE_EQ_CHECK_ND(c2_iv1, c2_varargin_1, 1);
  c2_x = c2_b_x_data[0] * c2_varargin_1_data[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  c2_b_x_size[0] = 1;
  c2_b_x_size[1] = c2_cartAngles_size[1];
  c2_fb_x = c2_b_x_size[0];
  c2_gb_x = c2_b_x_size[1];
  c2_i_loop_ub = c2_cartAngles_size[0] * c2_cartAngles_size[1] - 1;
  for (c2_i120 = 0; c2_i120 <= c2_i_loop_ub; c2_i120++) {
    c2_b_x_data[c2_i120] = c2_cartAngles_data[c2_i120];
  }

  c2_c_nx = c2_b_x_size[1];
  for (c2_b_k = 0; c2_b_k + 1 <= c2_c_nx; c2_b_k++) {
    c2_hb_x = c2_b_x_data[c2_b_k];
    c2_ib_x = c2_hb_x;
    c2_ib_x = muSingleScalarSin(c2_ib_x);
    c2_b_x_data[c2_b_k] = c2_ib_x;
  }

  for (c2_i122 = 0; c2_i122 < 2; c2_i122++) {
    c2_iv2[c2_i122] = 1;
  }

  for (c2_i123 = 0; c2_i123 < 2; c2_i123++) {
    c2_jb_x[c2_i123] = c2_b_x_size[c2_i123];
  }

  _SFD_SIZE_EQ_CHECK_ND(c2_iv2, c2_jb_x, 2);
  c2_d_R[0] = c2_R_size[0];
  c2_d_R[1] = 1;
  c2_e_R = c2_R_size[0];
  c2_varargin_1_size[0] = c2_validIdx_size[0];
  c2_j_loop_ub = c2_validIdx_size[0] - 1;
  for (c2_i125 = 0; c2_i125 <= c2_j_loop_ub; c2_i125++) {
    c2_varargin_1_data[c2_i125] = c2_R_data[sf_eml_array_bounds_check
      (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 1279, 13, MAX_uint32_T,
       (int32_T)c2_validIdx_data[c2_i125], 1, c2_e_R) - 1];
  }

  c2_iv3[0] = 1;
  c2_b_varargin_1[0] = c2_varargin_1_size[0];
  _SFD_SIZE_EQ_CHECK_ND(c2_iv3, c2_b_varargin_1, 1);
  c2_y = c2_b_x_data[0] * c2_varargin_1_data[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  c2_q_a[0] = c2_x;
  c2_q_a[1] = c2_y;
  for (c2_i126 = 0; c2_i126 < 2; c2_i126++) {
    c2_cart[c2_i126] = 0.0;
  }

  c2_i127 = 0;
  for (c2_i128 = 0; c2_i128 < 2; c2_i128++) {
    c2_cart[c2_i128] = 0.0;
    for (c2_i129 = 0; c2_i129 < 2; c2_i129++) {
      c2_cart[c2_i128] += c2_q_a[c2_i129] * c2_u_b[c2_i129 + c2_i127];
    }

    c2_i127 += 2;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 40);
  c2_th = c2_pose[3] - 1.5707963267948966;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 41);
  c2_kb_x = c2_th;
  c2_lb_x = c2_kb_x;
  c2_lb_x = muDoubleScalarCos(c2_lb_x);
  c2_mb_x = c2_th;
  c2_nb_x = c2_mb_x;
  c2_nb_x = muDoubleScalarSin(c2_nb_x);
  c2_ob_x = c2_th;
  c2_pb_x = c2_ob_x;
  c2_pb_x = muDoubleScalarSin(c2_pb_x);
  c2_qb_x = c2_th;
  c2_rb_x = c2_qb_x;
  c2_rb_x = muDoubleScalarCos(c2_rb_x);
  for (c2_i130 = 0; c2_i130 < 2; c2_i130++) {
    c2_q_a[c2_i130] = c2_cart[c2_i130];
  }

  c2_v_b[0] = c2_lb_x;
  c2_v_b[2] = c2_nb_x;
  c2_v_b[1] = -c2_pb_x;
  c2_v_b[3] = c2_rb_x;
  c2_i131 = 0;
  for (c2_i132 = 0; c2_i132 < 2; c2_i132++) {
    c2_h_y[c2_i132] = 0.0;
    for (c2_i134 = 0; c2_i134 < 2; c2_i134++) {
      c2_h_y[c2_i132] += c2_q_a[c2_i134] * c2_v_b[c2_i134 + c2_i131];
    }

    c2_i131 += 2;
  }

  for (c2_i133 = 0; c2_i133 < 2; c2_i133++) {
    c2_q_a[c2_i133] = c2_pose[c2_i133];
  }

  for (c2_jcol = 0; c2_jcol + 1 < 3; c2_jcol++) {
    c2_iacol = c2_jcol;
    c2_ibmat = c2_jcol;
    c2_ibcol = c2_ibmat;
    c2_w_b[c2_ibcol] = c2_q_a[c2_iacol];
  }

  for (c2_i135 = 0; c2_i135 < 2; c2_i135++) {
    c2_dataWorld[c2_i135] = c2_h_y[c2_i135] + c2_w_b[c2_i135];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -41);
  _SFD_SYMBOL_SCOPE_POP();
  c2_emxFree_real32_T(chartInstance, &c2_angles);
}

static void c2_indexShapeCheck
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, int32_T
   c2_matrixSize, int32_T c2_indexSize[2])
{
  int32_T c2_size1;
  boolean_T c2_b;
  boolean_T c2_nonSingletonDimFound;
  boolean_T c2_c;
  boolean_T c2_b_c;
  boolean_T c2_b_b;
  const mxArray *c2_y = NULL;
  static char_T c2_cv10[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'F', 'E', ':', 'P',
    'o', 't', 'e', 'n', 't', 'i', 'a', 'l', 'V', 'e', 'c', 't', 'o', 'r', 'V',
    'e', 'c', 't', 'o', 'r' };

  (void)chartInstance;
  c2_size1 = c2_matrixSize;
  c2_b = !(c2_size1 != 1);
  if (c2_b) {
    c2_nonSingletonDimFound = false;
    if (c2_indexSize[1] != 1) {
      c2_nonSingletonDimFound = true;
    }

    c2_b_b = c2_nonSingletonDimFound;
    if (c2_b_b) {
      c2_c = true;
    } else {
      c2_c = false;
    }
  } else {
    c2_c = false;
  }

  c2_b_c = c2_c;
  if (!c2_b_c) {
  } else {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv10, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_y));
  }
}

static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_eb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i136;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i136, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i136;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_odomMsg_bus_io(void *chartInstanceVoid, void *c2_pData)
{
  const mxArray *c2_mxVal;
  int32_T c2_i137;
  c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry c2_tmp;
  int32_T c2_i138;
  int32_T c2_i139;
  int32_T c2_i140;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxVal = NULL;
  c2_mxVal = NULL;
  for (c2_i137 = 0; c2_i137 < 128; c2_i137++) {
    c2_tmp.ChildFrameId[c2_i137] = ((uint8_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
      [0])[c2_i137];
  }

  c2_tmp.ChildFrameId_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [128])[0];
  c2_tmp.ChildFrameId_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [128])[4];
  c2_tmp.Header.Seq = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [136])[0];
  for (c2_i138 = 0; c2_i138 < 128; c2_i138++) {
    c2_tmp.Header.FrameId[c2_i138] = ((uint8_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
      [136])[4])[c2_i138];
  }

  c2_tmp.Header.FrameId_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [136])[136])[0];
  c2_tmp.Header.FrameId_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [136])[136])[4];
  c2_tmp.Header.Stamp.Sec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [136])[144])[0];
  c2_tmp.Header.Stamp.Nsec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [136])[144])[8];
  for (c2_i139 = 0; c2_i139 < 36; c2_i139++) {
    c2_tmp.Pose.Covariance[c2_i139] = ((real_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T
      *)(c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)
      c2_pData)[296])[0])[c2_i139];
  }

  c2_tmp.Pose.Pose.Position.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[0])[0];
  c2_tmp.Pose.Pose.Position.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[0])[8];
  c2_tmp.Pose.Pose.Position.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Point_3iqyla *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[0])[16];
  c2_tmp.Pose.Pose.Orientation.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[24])[0];
  c2_tmp.Pose.Pose.Orientation.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[24])[8];
  c2_tmp.Pose.Pose.Orientation.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[24])[16];
  c2_tmp.Pose.Pose.Orientation.W = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Quaternion_zfsmlw *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_geometry_msgs_Pose *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_PoseWithCovariance_bliu1u *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [296])[288])[24])[24];
  for (c2_i140 = 0; c2_i140 < 36; c2_i140++) {
    c2_tmp.Twist.Covariance[c2_i140] = ((real_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)
      &((char_T *)(c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry
                   *)c2_pData)[640])[0])[c2_i140];
  }

  c2_tmp.Twist.Twist.Linear.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [640])[288])[0])[0];
  c2_tmp.Twist.Twist.Linear.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [640])[288])[0])[8];
  c2_tmp.Twist.Twist.Linear.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [640])[288])[0])[16];
  c2_tmp.Twist.Twist.Angular.X = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [640])[288])[24])[0];
  c2_tmp.Twist.Twist.Angular.Y = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [640])[288])[24])[8];
  c2_tmp.Twist.Twist.Angular.Z = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Vector3_wtscbi *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_Twist_3geo7b *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_TwistWithCovariance_v4i8v8 *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)c2_pData)
    [640])[288])[24])[16];
  sf_mex_assign(&c2_mxVal, c2_c_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static const mxArray *c2_laserMsg_bus_io(void *chartInstanceVoid, void *c2_pData)
{
  const mxArray *c2_mxVal;
  c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa c2_tmp;
  int32_T c2_i141;
  int32_T c2_i142;
  int32_T c2_i143;
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    chartInstanceVoid;
  c2_mxVal = NULL;
  c2_mxVal = NULL;
  c2_tmp.AngleMin = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[0];
  c2_tmp.AngleMax = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[4];
  c2_tmp.AngleIncrement = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[8];
  c2_tmp.TimeIncrement = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[12];
  c2_tmp.ScanTime = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16];
  c2_tmp.RangeMin = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[20];
  c2_tmp.RangeMax = *(real32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[24];
  for (c2_i141 = 0; c2_i141 < 2048; c2_i141++) {
    c2_tmp.Ranges[c2_i141] = ((real32_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[28])
      [c2_i141];
  }

  c2_tmp.Ranges_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[8224])[0];
  c2_tmp.Ranges_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[8224])[4];
  for (c2_i142 = 0; c2_i142 < 2048; c2_i142++) {
    c2_tmp.Intensities[c2_i142] = ((real32_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[8232])
      [c2_i142];
  }

  c2_tmp.Intensities_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16424])[0];
  c2_tmp.Intensities_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16424])[4];
  c2_tmp.Header.Seq = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16432])[0];
  for (c2_i143 = 0; c2_i143 < 128; c2_i143++) {
    c2_tmp.Header.FrameId[c2_i143] = ((uint8_T *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
      (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16432])
      [4])[c2_i143];
  }

  c2_tmp.Header.FrameId_SL_Info.CurrentLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16432])
    [136])[0];
  c2_tmp.Header.FrameId_SL_Info.ReceivedLength = *(uint32_T *)&((char_T *)
    (c2_SL_Bus_ROSVariableLengthArrayInfo *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16432])
    [136])[4];
  c2_tmp.Header.Stamp.Sec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16432])
    [144])[0];
  c2_tmp.Header.Stamp.Nsec = *(real_T *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_ros_time_Time *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionAvoidance_std_msgs_Header *)&((char_T *)
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)c2_pData)[16432])
    [144])[8];
  sf_mex_assign(&c2_mxVal, c2_b_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static uint8_T c2_fb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_DroneControllerCollisionAvoidance, const char_T
   *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_gb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_DroneControllerCollisionAvoidance), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_DroneControllerCollisionAvoidance);
  return c2_y;
}

static uint8_T c2_gb_emlrt_marshallIn
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u1;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u1, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_emxEnsureCapacity
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance,
   c2_emxArray__common *c2_emxArray, int32_T c2_oldNumel, uint32_T
   c2_elementSize, const emlrtRTEInfo *c2_srcLocation)
{
  int32_T c2_newNumel;
  int32_T c2_i;
  int32_T c2_newCapacity;
  void *c2_newData;
  if (c2_oldNumel < 0) {
    c2_oldNumel = 0;
  }

  c2_newNumel = 1;
  for (c2_i = 0; c2_i < c2_emxArray->numDimensions; c2_i++) {
    c2_newNumel = (int32_T)emlrtSizeMulR2012b((uint32_T)c2_newNumel, (uint32_T)
      c2_emxArray->size[c2_i], c2_srcLocation, chartInstance->c2_fEmlrtCtx);
  }

  if (c2_newNumel > c2_emxArray->allocatedSize) {
    c2_newCapacity = c2_emxArray->allocatedSize;
    if (c2_newCapacity < 16) {
      c2_newCapacity = 16;
    }

    while (c2_newCapacity < c2_newNumel) {
      if (c2_newCapacity > 1073741823) {
        c2_newCapacity = MAX_int32_T;
      } else {
        c2_newCapacity <<= 1;
      }
    }

    c2_newData = emlrtCallocMex((uint32_T)c2_newCapacity, c2_elementSize);
    if (c2_newData == NULL) {
      emlrtHeapAllocationErrorR2012b(c2_srcLocation, chartInstance->c2_fEmlrtCtx);
    }

    if (c2_emxArray->data != NULL) {
      memcpy(c2_newData, c2_emxArray->data, c2_elementSize * (uint32_T)
             c2_oldNumel);
      if (c2_emxArray->canFreeData) {
        emlrtFreeMex(c2_emxArray->data);
      }
    }

    c2_emxArray->data = c2_newData;
    c2_emxArray->allocatedSize = c2_newCapacity;
    c2_emxArray->canFreeData = true;
  }
}

static void c2_emxInit_real32_T
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance,
   c2_emxArray_real32_T **c2_pEmxArray, int32_T c2_numDimensions, const
   emlrtRTEInfo *c2_srcLocation)
{
  c2_emxArray_real32_T *c2_emxArray;
  int32_T c2_i;
  *c2_pEmxArray = (c2_emxArray_real32_T *)emlrtMallocMex(sizeof
    (c2_emxArray_real32_T));
  if ((void *)*c2_pEmxArray == NULL) {
    emlrtHeapAllocationErrorR2012b(c2_srcLocation, chartInstance->c2_fEmlrtCtx);
  }

  c2_emxArray = *c2_pEmxArray;
  c2_emxArray->data = (real32_T *)NULL;
  c2_emxArray->numDimensions = c2_numDimensions;
  c2_emxArray->size = (int32_T *)emlrtMallocMex((uint32_T)(sizeof(int32_T)
    * c2_numDimensions));
  if ((void *)c2_emxArray->size == NULL) {
    emlrtHeapAllocationErrorR2012b(c2_srcLocation, chartInstance->c2_fEmlrtCtx);
  }

  c2_emxArray->allocatedSize = 0;
  c2_emxArray->canFreeData = true;
  for (c2_i = 0; c2_i < c2_numDimensions; c2_i++) {
    c2_emxArray->size[c2_i] = 0;
  }
}

static void c2_emxFree_real32_T
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance,
   c2_emxArray_real32_T **c2_pEmxArray)
{
  (void)chartInstance;
  if (*c2_pEmxArray != (c2_emxArray_real32_T *)NULL) {
    if (((*c2_pEmxArray)->data != (real32_T *)NULL) && (*c2_pEmxArray)
        ->canFreeData) {
      emlrtFreeMex((void *)(*c2_pEmxArray)->data);
    }

    emlrtFreeMex((void *)(*c2_pEmxArray)->size);
    emlrtFreeMex((void *)*c2_pEmxArray);
    *c2_pEmxArray = (c2_emxArray_real32_T *)NULL;
  }
}

static int32_T c2_div_nzp_s32
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance, int32_T
   c2_numerator, int32_T c2_denominator, uint32_T c2_ssid_src_loc, int32_T
   c2_offset_src_loc, int32_T c2_length_src_loc)
{
  int32_T c2_quotient;
  uint32_T c2_absNumerator;
  uint32_T c2_absDenominator;
  boolean_T c2_quotientNeedsNegation;
  uint32_T c2_tempAbsQuotient;
  (void)chartInstance;
  (void)c2_ssid_src_loc;
  (void)c2_offset_src_loc;
  (void)c2_length_src_loc;
  if (c2_numerator < 0) {
    c2_absNumerator = ~(uint32_T)c2_numerator + 1U;
  } else {
    c2_absNumerator = (uint32_T)c2_numerator;
  }

  if (c2_denominator < 0) {
    c2_absDenominator = ~(uint32_T)c2_denominator + 1U;
  } else {
    c2_absDenominator = (uint32_T)c2_denominator;
  }

  c2_quotientNeedsNegation = (c2_numerator < 0 != c2_denominator < 0);
  c2_tempAbsQuotient = c2_absNumerator / c2_absDenominator;
  if (c2_quotientNeedsNegation) {
    c2_quotient = -(int32_T)c2_tempAbsQuotient;
  } else {
    c2_quotient = (int32_T)c2_tempAbsQuotient;
  }

  return c2_quotient;
}

static void init_dsm_address_info
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance)
{
  chartInstance->c2_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c2_odomMsg =
    (c2_SL_Bus_DroneControllerCollisionAvoidance_nav_msgs_Odometry *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c2_laserMsg =
    (c2_SL_Bus_DroneControllerCollisionA_LaserScan_fv06xa *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 1);
  chartInstance->c2_posePub = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_target = (real_T (*)[4])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_DroneControllerCollisionAvoidance_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3916615707U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(269938903U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(988983337U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2749206933U);
}

mxArray* sf_c2_DroneControllerCollisionAvoidance_get_post_codegen_info(void);
mxArray *sf_c2_DroneControllerCollisionAvoidance_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("LgW6OYaQmCYcveR20pwyTC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(4);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c2_DroneControllerCollisionAvoidance_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_DroneControllerCollisionAvoidance_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_DroneControllerCollisionAvoidance_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_DroneControllerCollisionAvoidance_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_DroneControllerCollisionAvoidance_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c2_DroneControllerCollisionAvoidance
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"posePub\",},{M[8],M[0],T\"is_active_c2_DroneControllerCollisionAvoidance\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_DroneControllerCollisionAvoidance_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance =
      (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _DroneControllerCollisionAvoidanceMachineNumber_,
           2,
           1,
           1,
           0,
           4,
           0,
           0,
           0,
           0,
           2,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation
          (_DroneControllerCollisionAvoidanceMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _DroneControllerCollisionAvoidanceMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _DroneControllerCollisionAvoidanceMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"odomMsg");
          _SFD_SET_DATA_PROPS(1,1,1,0,"laserMsg");
          _SFD_SET_DATA_PROPS(2,1,1,0,"target");
          _SFD_SET_DATA_PROPS(3,2,0,1,"posePub");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,3,0,2,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,858);
        _SFD_CV_INIT_EML_FCN(0,1,"findObstacle",860,-1,1454);
        _SFD_CV_INIT_EML_FCN(0,2,"readPose",1456,-1,1844);
        _SFD_CV_INIT_EML_IF(0,1,0,292,337,-1,854);
        _SFD_CV_INIT_EML_IF(0,1,1,558,610,678,721);

        {
          static int condStart[] = { 561, 570 };

          static int condEnd[] = { 566, 610 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,561,610,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,295,337,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,561,566,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,570,610,-1,2);
        _SFD_CV_INIT_SCRIPT(0,1,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"hectorQuadrotorComputePotentialField",0,-1,
          379);
        _SFD_CV_INIT_SCRIPT_IF(0,0,256,272,-1,364);
        _SFD_CV_INIT_SCRIPT_RELATIONAL(0,0,259,272,-1,3);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"getF_attractiv",0,-1,170);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_odomMsg_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_laserMsg_bus_io,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1U;
          dimVector[1]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1U;
          dimVector[1]= 4U;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _DroneControllerCollisionAvoidanceMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance =
      (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, (void *)chartInstance->c2_odomMsg);
        _SFD_SET_DATA_VALUE_PTR(1U, (void *)chartInstance->c2_laserMsg);
        _SFD_SET_DATA_VALUE_PTR(3U, (void *)chartInstance->c2_posePub);
        _SFD_SET_DATA_VALUE_PTR(2U, (void *)chartInstance->c2_target);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sQPCTrFDH12vhYZS3ieU0I";
}

static void sf_opaque_initialize_c2_DroneControllerCollisionAvoidance(void
  *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar
     )->S,0);
  initialize_params_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
  initialize_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_DroneControllerCollisionAvoidance(void
  *chartInstanceVar)
{
  enable_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_DroneControllerCollisionAvoidance(void
  *chartInstanceVar)
{
  disable_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_DroneControllerCollisionAvoidance(void
  *chartInstanceVar)
{
  sf_gateway_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c2_DroneControllerCollisionAvoidance(SimStruct* S)
{
  return get_sim_state_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_DroneControllerCollisionAvoidance
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c2_DroneControllerCollisionAvoidance(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_DroneControllerCollisionAvoidance_optimization_info();
    }

    finalize_c2_DroneControllerCollisionAvoidance
      ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_DroneControllerCollisionAvoidance
    ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_DroneControllerCollisionAvoidance(SimStruct *
  S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_DroneControllerCollisionAvoidance
      ((SFc2_DroneControllerCollisionAvoidanceInstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c2_DroneControllerCollisionAvoidance(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssSetStatesModifiedOnlyInUpdate(S, 1);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct =
      load_DroneControllerCollisionAvoidance_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 2);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1630987922U));
  ssSetChecksum1(S,(4041122397U));
  ssSetChecksum2(S,(2130836358U));
  ssSetChecksum3(S,(2288759509U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_DroneControllerCollisionAvoidance(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_DroneControllerCollisionAvoidance(SimStruct *S)
{
  SFc2_DroneControllerCollisionAvoidanceInstanceStruct *chartInstance;
  chartInstance = (SFc2_DroneControllerCollisionAvoidanceInstanceStruct *)
    utMalloc(sizeof(SFc2_DroneControllerCollisionAvoidanceInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc2_DroneControllerCollisionAvoidanceInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_DroneControllerCollisionAvoidance;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c2_DroneControllerCollisionAvoidance(chartInstance);
}

void c2_DroneControllerCollisionAvoidance_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_DroneControllerCollisionAvoidance(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_DroneControllerCollisionAvoidance(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_DroneControllerCollisionAvoidance(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_DroneControllerCollisionAvoidance_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
