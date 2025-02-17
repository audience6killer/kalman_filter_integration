#ifndef BN085_TASK_COMMON_H
#define BN085_TASK_COMMON_H

#define BNO085_STACK_SIZE       4096
#define BNO085_CORE_ID          0 
#define BNO085_TASK_PRIORITY    8 

#define BNO085_INT              4
#define BNO085_RST              17
#define BNO085_ADDR             0x4B

#define BNO085_SAMPLE_TIME      10  // 100hz @ 10ms

// HOME
#define INIT_LAT             19.5037727355f
#define INIT_LON             -99.1248474121f
#define INIT_ALT             2331.8000488f

#define R_n                     6378137.0f
#define R_e                     6356752.3142f
#define R_0                     6371000.0f
#define OMEGA_E                 7.2921159E-5
#define GRAVITY                 9.80665f

#endif