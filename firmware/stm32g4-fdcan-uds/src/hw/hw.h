#ifndef HW_H_
#define HW_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

#include "led.h"
#include "cli.h"
#include "log.h"
#include "uart.h"
#include "button.h"
#include "swtimer.h"
#include "can.h"
// #include "gpio.h"     // Not implemented for simple version  
// #include "spi.h"      // Not needed for simple version
// #include "spi_flash.h" // Not needed for simple version
#include "usb.h"
#include "cdc.h"
#include "flash.h"
// #include "fs.h"       // File system not needed for simple version
#include "nvs.h"
#include "eeprom.h"
#include "rtc.h"
#include "reset.h"
#include "cmd.h"
#include "util.h"
#include "fault.h"


bool hwInit(void);


#ifdef __cplusplus
}
#endif

#endif