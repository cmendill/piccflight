#ifndef _ADDONS
  #define _ADDONS

  #include <phx_api.h>    /* Main Phoenix library */

  typedef enum{
    PHX_BOBCAT_BIT_DEPTH,
    PHX_BOBCAT_TAPS,
    PHX_BOBCAT_ROI
  } phxbobcatParam;

  typedef enum{
    /*PHX_BOBCAT_BIT_DEPTH_SELECT*/
    PHX_BOBCAT_8BIT,
    PHX_BOBCAT_10BIT,
    PHX_BOBCAT_12BIT,

    /* PHX_BOBCAT_TAPS */
    PHX_BOBCAT_DOUBLE_TAP,
    PHX_BOBCAT_SINGLE_TAP

  } phxbobcatParamValue;

  etStat PHX_BOBCAT_Configure(tHandle, phxbobcatParam, void*);
  
  int PHX_BOBCAT_str_to_phxbobcatParam(char*, phxbobcatParam*);
  int PHX_BOBCAT_str_to_phxbobcatParamValue(char*, phxbobcatParamValue*);

#endif   /* _ADDONS */
