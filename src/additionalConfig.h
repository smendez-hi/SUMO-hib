#include "utils/common/SUMOTime.h"

/* default vehicle type */
//#define VEH_FAAM_ECOMILE
#define VEH_HIB_ORIGINAL // TODO mejor usar este como default
#ifdef VEH_FAAM_ECOMILE
  /* calibration case */
  #define CALIBRATION_TEST
#endif

//#define MIEV_ISTAMBUL_CALIBRATION

#define INT4SUMOTIME // Original
//#define SUMOREAL4SUMOTIME // Try (and FIXME)
#ifdef SUMOREAL4SUMOTIME
#define SUMOREAL_MAX DOUBLE_MAX
#endif

/* DELTA_T uses to value .1 secs, but expressed as millisecs (therefore 100) */
#define DELTA_T_secs ((DELTA_T) / (1000.))
#define DELTA_T_hours ((DELTA_T_secs) / (3600.))
/* CurrentTimeSteps are measured in milliseconds */
//#define SECS_IN_CURENT_TIME_STEP 1000
#define SECS_IN_CURENT_TIME_STEP 1

#define IGNORE_INTERNALS_IN_SYNTHETIC_DATA
#define IGNORE_NONFEV_SYNTH_DATA

#define BATT_TYPE_CUSTOM_MODEL
//BATT_TYPE_LI_PO
//BATT_TYPE_CUSTOM_MODEL
