/******************************************************************************

 @file  main.c

 @brief main entry of the BLE stack sample application.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2013-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/BIOS.h>

#include "simultaneous_sensor.h"

#include <icall.h>
#include "hal_assert.h"
#include "bcomdef.h"
#ifdef PTM_MODE
#include "npi_task.h"
#endif

#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG
#include "ble_user_config.h"
icall_userCfg_t user0Cfg = BLE_USER_CFG;
#endif

#include <ti/display/Display.h>

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

extern Display_Handle display;

int main()
{
  RegisterAssertCback(AssertHandler);

  Board_initGeneral();

  VIMSConfigure(VIMS_BASE, TRUE, TRUE);
  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

#if !defined( POWER_SAVING )
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif

  user0Cfg.appServiceInfo->timerTickPeriod = Clock_tickPeriod;
  user0Cfg.appServiceInfo->timerMaxMillisecond  = ICall_getMaxMSecs();

  ICall_init();

  ICall_createRemoteTasks();

#ifdef PTM_MODE
  NPITask_createTask(ICALL_SERVICE_CLASS_BLE);
#endif

  simultaneous_sensor();

  BIOS_start();

  return 0;
}


void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  if ( !display )
  {
    display = Display_open(Display_Type_ANY, NULL);
  }

  Display_print0(display, 0, 0, ">>>STACK ASSERT");

  switch (assertCause)
  {
    case HAL_ASSERT_CAUSE_OUT_OF_MEMORY:
      Display_print0(display, 0, 0, "***ERROR***");
      Display_print0(display, 2, 0, ">> OUT OF MEMORY!");
      break;

    case HAL_ASSERT_CAUSE_INTERNAL_ERROR:
      if (assertSubcause == HAL_ASSERT_SUBCAUSE_FW_INERNAL_ERROR)
      {
        Display_print0(display, 0, 0, "***ERROR***");
        Display_print0(display, 2, 0, ">> INTERNAL FW ERROR!");
      }
      else
      {
        Display_print0(display, 0, 0, "***ERROR***");
        Display_print0(display, 2, 0, ">> INTERNAL ERROR!");
      }
      break;

    case HAL_ASSERT_CAUSE_ICALL_ABORT:
      Display_print0(display, 0, 0, "***ERROR***");
      Display_print0(display, 2, 0, ">> ICALL ABORT!");
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_ICALL_TIMEOUT:
      Display_print0(display, 0, 0, "***ERROR***");
      Display_print0(display, 2, 0, ">> ICALL TIMEOUT!");
      HAL_ASSERT_SPINLOCK;
      break;

    case HAL_ASSERT_CAUSE_WRONG_API_CALL:
      Display_print0(display, 0, 0, "***ERROR***");
      Display_print0(display, 2, 0, ">> WRONG API CALL!");
      HAL_ASSERT_SPINLOCK;
      break;

  default:
      Display_print0(display, 0, 0, "***ERROR***");
      Display_print0(display, 2, 0, ">> DEFAULT SPINLOCK!");
      HAL_ASSERT_SPINLOCK;
  }

  return;
}

void smallErrorHook(Error_Block *eb)
{
  for (;;);
}
