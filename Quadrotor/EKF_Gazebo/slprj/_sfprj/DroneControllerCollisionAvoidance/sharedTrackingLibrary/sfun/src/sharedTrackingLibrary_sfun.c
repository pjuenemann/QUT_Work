/* Include files */

#include "sharedTrackingLibrary_sfun.h"
#include "sharedTrackingLibrary_sfun_debug_macros.h"
#include "c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary.h"
#include "c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary.h"
#include "c9_sharedTrackingLibrary.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _sharedTrackingLibraryMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void sharedTrackingLibrary_initializer(void)
{
}

void sharedTrackingLibrary_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_sharedTrackingLibrary_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==8) {
    if (!strcmp(specsCksum, "s7ItMq5ORbJIw7R5JGTHTVC")) {
      c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_method_dispatcher
        (simstructPtr, method, data);
      return 1;
    }

    if (!strcmp(specsCksum, "sFywiTZeQa1EYv2NwJ5ncFC")) {
      c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_method_dispatcher
        (simstructPtr, method, data);
      return 1;
    }

    return 0;
  }

  if (chartFileNumber==9) {
    c9_sharedTrackingLibrary_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_sharedTrackingLibrary_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>2 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"library")) {
      char machineName[100];
      mxGetString(prhs[2], machineName,sizeof(machineName)/sizeof(char));
      machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
      if (!strcmp(machineName,"sharedTrackingLibrary")) {
        if (nrhs==3) {
          ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3514910825U);
          ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(270598424U);
          ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1207393351U);
          ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1631410823U);
        } else if (nrhs==4) {
          unsigned int chartFileNumber;
          chartFileNumber = (unsigned int)mxGetScalar(prhs[3]);
          switch (chartFileNumber) {
           case 8:
            {
              extern void
                sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_get_check_sum
                (mxArray *plhs[]);
              sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_get_check_sum
                (plhs);
              break;
            }

           case 9:
            {
              extern void sf_c9_sharedTrackingLibrary_get_check_sum(mxArray
                *plhs[]);
              sf_c9_sharedTrackingLibrary_get_check_sum(plhs);
              break;
            }

           default:
            ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
            ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
            ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
            ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
          }
        } else {
          return 0;
        }
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  } else {
    return 0;
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_sharedTrackingLibrary_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 8:
      {
        if (strcmp(aiChksum, "xEs6sKgumhCSAMWWWThzt") == 0) {
          extern mxArray
            *sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_get_autoinheritance_info
            ();
          break;
        }

        if (strcmp(aiChksum, "yMteaEVEBGKUzvkUSN016C") == 0) {
          extern mxArray
            *sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 9:
      {
        if (strcmp(aiChksum, "aTczqHQqvfqMmPD2yKjjDC") == 0) {
          extern mxArray *sf_c9_sharedTrackingLibrary_get_autoinheritance_info
            (void);
          plhs[0] = sf_c9_sharedTrackingLibrary_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_sharedTrackingLibrary_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 8:
      {
        extern const mxArray
          *sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 9:
      {
        extern const mxArray
          *sf_c9_sharedTrackingLibrary_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c9_sharedTrackingLibrary_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_sharedTrackingLibrary_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 8:
      {
        if (strcmp(tpChksum, "s7ItMq5ORbJIw7R5JGTHTVC") == 0) {
          extern mxArray
            *sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_third_party_uses_info
            (void);
          plhs[0] =
            sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_third_party_uses_info
            ();
          break;
        }

        if (strcmp(tpChksum, "sFywiTZeQa1EYv2NwJ5ncFC") == 0) {
          extern mxArray
            *sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_third_party_uses_info
            (void);
          plhs[0] =
            sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_third_party_uses_info
            ();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "sH67H5rpwduK8hr62k27AJ") == 0) {
          extern mxArray *sf_c9_sharedTrackingLibrary_third_party_uses_info(void);
          plhs[0] = sf_c9_sharedTrackingLibrary_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_sharedTrackingLibrary_jit_fallback_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the jit_fallback_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 8:
      {
        if (strcmp(tpChksum, "s7ItMq5ORbJIw7R5JGTHTVC") == 0) {
          extern mxArray
            *sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_jit_fallback_info
            (void);
          plhs[0] =
            sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_jit_fallback_info
            ();
          break;
        }

        if (strcmp(tpChksum, "sFywiTZeQa1EYv2NwJ5ncFC") == 0) {
          extern mxArray
            *sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_jit_fallback_info
            (void);
          plhs[0] =
            sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_jit_fallback_info
            ();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "sH67H5rpwduK8hr62k27AJ") == 0) {
          extern mxArray *sf_c9_sharedTrackingLibrary_jit_fallback_info(void);
          plhs[0] = sf_c9_sharedTrackingLibrary_jit_fallback_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_sharedTrackingLibrary_updateBuildInfo_args_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 8:
      {
        if (strcmp(tpChksum, "s7ItMq5ORbJIw7R5JGTHTVC") == 0) {
          extern mxArray
            *sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c8_s7ItMq5ORbJIw7R5JGTHTVC_sharedTrackingLibrary_updateBuildInfo_args_info
            ();
          break;
        }

        if (strcmp(tpChksum, "sFywiTZeQa1EYv2NwJ5ncFC") == 0) {
          extern mxArray
            *sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c8_sFywiTZeQa1EYv2NwJ5ncFC_sharedTrackingLibrary_updateBuildInfo_args_info
            ();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "sH67H5rpwduK8hr62k27AJ") == 0) {
          extern mxArray *sf_c9_sharedTrackingLibrary_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c9_sharedTrackingLibrary_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void sharedTrackingLibrary_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _sharedTrackingLibraryMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"sharedTrackingLibrary","sfun",1,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _sharedTrackingLibraryMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _sharedTrackingLibraryMachineNumber_,0);
}

void sharedTrackingLibrary_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
typedef struct SfOptimizationInfoFlagsTag {
  boolean_T isRtwGen;
  boolean_T isModelRef;
  boolean_T isExternal;
} SfOptimizationInfoFlags;

static SfOptimizationInfoFlags sOptimizationInfoFlags;
void unload_sharedTrackingLibrary_optimization_info(void);
mxArray* load_sharedTrackingLibrary_optimization_info(boolean_T isRtwGen,
  boolean_T isModelRef, boolean_T isExternal)
{
  if (sOptimizationInfoFlags.isRtwGen != isRtwGen ||
      sOptimizationInfoFlags.isModelRef != isModelRef ||
      sOptimizationInfoFlags.isExternal != isExternal) {
    unload_sharedTrackingLibrary_optimization_info();
  }

  sOptimizationInfoFlags.isRtwGen = isRtwGen;
  sOptimizationInfoFlags.isModelRef = isModelRef;
  sOptimizationInfoFlags.isExternal = isExternal;
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "sharedTrackingLibrary", "DroneControllerCollisionAvoidance");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_sharedTrackingLibrary_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
