/**
  ********************************************************************************************************************************
  * @file    hardware_config.h 
  * @author  Roel Pantonial
  * @brief   Header of port and pin assignments
  * @details This file is used to align board with electromechanical parameters
  ********************************************************************************************************************************
  */

#define HARDWARE_VERSION_BULLRUNNER 0
#define HARDWARE_VERSION_1p3KW 1
#define HARDWARE_VERSION_4p5KW 2
#define HARDWARE_VERSION_8KW 3

#define HARDWARE_VERSION HARDWARE_VERSION_4p5KW // set this to choose the HW version
 

// GPIO Input Ports/Pins
#if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW
#define CLOCK_CHECK 64
#endif

#if HARDWARE_VERSION == HARDWARE_VERSION_4p5KW
#define CLOCK_CHECK 72
#endif

#if HARDWARE_VERSION == HARDWARE_VERSION_8KW
#define CLOCK_CHECK 72
#endif
