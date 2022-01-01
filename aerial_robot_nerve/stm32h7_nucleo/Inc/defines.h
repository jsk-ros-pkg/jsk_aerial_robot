/* defines.h configuraion file */
 
/* Uncomment if you want to disable ID pin for USB MSC HOST library */
//#define USB_MSC_HOST_DISABLE_ID
 
/* Uncomment if you want to disable VBUS pin for USB MSC HOST library */
/* If you do this, on F4 and F429 Discovery boards USB will not work */
//#define USB_MSC_HOST_DISABLE_VBUS

#define VERSION    "1.07"

#ifndef M_PI
#define M_PI           3.141592653589793238462643383280
#endif

#ifndef FALSE
#define FALSE 0U
#endif

#ifndef TRUE
#define TRUE  1U
#endif

#ifndef PASS
#define PASS  0U
#endif

#ifndef FAIL
#define FAIL  1U
#endif
