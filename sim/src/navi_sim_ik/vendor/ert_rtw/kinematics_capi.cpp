//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: kinematics_capi.cpp
//
// Code generated for Simulink model 'kinematics'.
//
// Model version                  : 2.41
// Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
// C/C++ source code generated on : Wed Sep  7 16:24:21 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "kinematics_capi_host.h"
#define sizeof(s)                      ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#ifndef SS_UINT64
#define SS_UINT64                      17
#endif

#ifndef SS_INT64
#define SS_INT64                       18
#endif

#else                                  // HOST_CAPI_BUILD
#include "builtin_typeid_types.h"
#include "kinematics.h"
#include "kinematics_capi.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((nullptr))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif                                 // HOST_CAPI_BUILD

// Individual block tuning is not valid when inline parameters is *
//  selected. An empty map is produced to provide a consistent     *
//  interface independent  of inlining parameters.                 *

static rtwCAPI_BlockParameters rtBlockParameters[]{
  // addrMapIndex, blockPath,
  //  paramName, dataTypeIndex, dimIndex, fixPtIdx

  {
    0, (nullptr), (nullptr), 0, 0, 0
  }
};

// Tunable variable parameters
static rtwCAPI_ModelParameters rtModelParameters[]{
  // addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex
  { 0, (nullptr), 0, 0, 0 }
};

// Data Type Map - use dataTypeMapIndex to access this structure
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[]{
  // cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
  //  isComplex, isPointer, enumStorageType
  {
    "", "", 0, 0, 0, 0, 0, 0, 0
  }
};

#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif

// Structure Element Map - use elemMapIndex to access this structure
static TARGET_CONST rtwCAPI_ElementMap rtElementMap[]{
  // elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex
  { (nullptr), 0, 0, 0, 0 },
};

// Dimension Map - use dimensionMapIndex to access elements of ths structure
static rtwCAPI_DimensionMap rtDimensionMap[]{
  // dataOrientation, dimArrayIndex, numDims, vardimsIndex
  {
    rtwCAPI_SCALAR, 0, 0, 0
  }
};

// Dimension Array- use dimArrayIndex to access elements of this array
static uint_T rtDimensionArray[]{ 0 };

// Fixed Point Map
static rtwCAPI_FixPtMap rtFixPtMap[]{
  // fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned
  { (nullptr), (nullptr), rtwCAPI_FIX_RESERVED, 0, 0, (boolean_T)0 },
};

// Sample Time Map - use sTimeIndex to access elements of ths structure
static rtwCAPI_SampleTimeMap rtSampleTimeMap[]{
  // samplePeriodPtr, sampleOffsetPtr, tid, samplingMode
  {
    (nullptr), (nullptr), 0, 0
  }
};

static rtwCAPI_ModelMappingStaticInfo mmiStatic{
  // Signals:{signals, numSignals,
  //            rootInputs, numRootInputs,
  //            rootOutputs, numRootOutputs},
  //  Params: {blockParameters, numBlockParameters,
  //           modelParameters, numModelParameters},
  //  States: {states, numStates},
  //  Maps:   {dataTypeMap, dimensionMap, fixPtMap,
  //           elementMap, sampleTimeMap, dimensionArray},
  //  TargetType: targetType

  { (nullptr), 0,
    (nullptr), 0,
    (nullptr), 0 },

  { rtBlockParameters, 0,
    rtModelParameters, 0 },

  { (nullptr), 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 3656232973U,
    1333837851U,
    4164911395U,
    107306688U },
  (nullptr), 0,
  (boolean_T)0
};

// Function to get C API Model Mapping Static Info
const rtwCAPI_ModelMappingStaticInfo*
  kinematics_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

// Cache pointers into DataMapInfo substructure of RTModel
#ifndef HOST_CAPI_BUILD

void kinematics_InitializeDataMapInfo(kinematics::RT_MODEL *const M)
{
  // Set C-API version
  rtwCAPI_SetVersion(M->DataMapInfo.mmi, 1);

  // Cache static C-API data into the Real-time Model Data structure
  rtwCAPI_SetStaticMap(M->DataMapInfo.mmi, &mmiStatic);

  // Cache static C-API logging data into the Real-time Model Data structure
  rtwCAPI_SetLoggingStaticMap(M->DataMapInfo.mmi, (nullptr));

  // Set Instance specific path
  rtwCAPI_SetPath(M->DataMapInfo.mmi, (nullptr));
  rtwCAPI_SetFullPath(M->DataMapInfo.mmi, (nullptr));

  // Cache the instance C-API logging pointer
  rtwCAPI_SetInstanceLoggingInfo(M->DataMapInfo.mmi, (nullptr));

  // Set reference to submodels
  rtwCAPI_SetChildMMIArray(M->DataMapInfo.mmi, (nullptr));
  rtwCAPI_SetChildMMIArrayLen(M->DataMapInfo.mmi, 0);
}

#else                                  // HOST_CAPI_BUILD
#ifdef __cplusplus

extern "C" {

#endif

  void kinematics_host_InitializeDataMapInfo(kinematics_host_DataMapInfo_T
    *dataMap, const char *path)
  {
    // Set C-API version
    rtwCAPI_SetVersion(dataMap->mmi, 1);

    // Cache static C-API data into the Real-time Model Data structure
    rtwCAPI_SetStaticMap(dataMap->mmi, &mmiStatic);

    // host data address map is NULL
    rtwCAPI_SetDataAddressMap(dataMap->mmi, (nullptr));

    // host vardims address map is NULL
    rtwCAPI_SetVarDimsAddressMap(dataMap->mmi, (nullptr));

    // Set Instance specific path
    rtwCAPI_SetPath(dataMap->mmi, path);
    rtwCAPI_SetFullPath(dataMap->mmi, (nullptr));

    // Set reference to submodels
    rtwCAPI_SetChildMMIArray(dataMap->mmi, (nullptr));
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 0);
  }

#ifdef __cplusplus

}
#endif
#endif                                 // HOST_CAPI_BUILD

//
// File trailer for generated code.
//
// [EOF]
//
