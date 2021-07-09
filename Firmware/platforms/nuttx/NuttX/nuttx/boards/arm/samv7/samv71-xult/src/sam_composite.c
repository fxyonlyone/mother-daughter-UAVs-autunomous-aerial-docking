/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_composite.c
 *
 *   Copyright (C) 2016, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

#include "samv71-xult.h"

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static FAR void *g_mschandle;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mscclassobject
 *
 * Description:
 *   If the mass storage class driver is part of composite device, then
 *   its instantiation and configuration is a multi-step, board-specific,
 *   process (See comments for usbmsc_configure below).  In this case,
 *   board-specific logic must provide board_mscclassobject().
 *
 *   board_mscclassobject() is called from the composite driver.  It must
 *   encapsulate the instantiation and configuration of the mass storage
 *   class and the return the mass storage device's class driver instance
 *   to the composite driver.
 *
 * Input Parameters:
 *   classdev - The location to return the mass storage class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static int board_mscclassobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  int ret;

  DEBUGASSERT(g_mschandle == NULL);

  /* Configure the mass storage device */

  uinfo("Configuring with NLUNS=1\n");
  ret = usbmsc_configure(1, &g_mschandle);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_configure failed: %d\n", -ret);
      return ret;
    }

  uinfo("MSC handle=%p\n", g_mschandle);

  /* Bind the LUN(s) */

  uinfo("Bind LUN=0 to /dev/mmcsd0\n");
  ret = usbmsc_bindlun(g_mschandle, "/dev/mmcsd0", 0, 0, 0, false);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_bindlun failed for LUN 1 at /dev/mmcsd0: %d\n",
           ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
      return ret;
    }

  /* Get the mass storage device's class object */

  ret = usbmsc_classobject(g_mschandle, devinfo, classdev);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_classobject failed: %d\n", -ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: board_mscuninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This is just an application-
 *   specific wrapper aboutn usbmsc_unitialize() that is called form the
 *   composite device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_mscclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static void board_mscuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  DEBUGASSERT(g_mschandle != NULL);
  usbmsc_uninitialize(g_mschandle);
  g_mschandle = NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_composite_initialize
 *
 * Description:
 *   Perform architecture specific initialization of a composite USB device.
 *
 ****************************************************************************/

int board_composite_initialize(int port)
{
  return OK;
}

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

FAR void *board_composite_connect(int port, int configid)
{
  /* Here we are composing the configuration of the usb composite device.
   *
   * The standard is to use one CDC/ACM and one USB mass storage device.
   */

  if (configid == 0)
    {
#ifdef CONFIG_USBMSC_COMPOSITE
      struct composite_devdesc_s dev[2];
      int ifnobase = 0;
      int strbase  = COMPOSITE_NSTRIDS;

      /* Configure the CDC/ACM device */

      /* Ask the cdcacm driver to fill in the constants we didn't
       * know here.
       */

      cdcacm_get_composite_devdesc(&dev[0]);

      /* Overwrite and correct some values... */

      /* The callback functions for the CDC/ACM class */

      dev[0].classobject  = cdcacm_classobject;
      dev[0].uninitialize = cdcacm_uninitialize;

      /* Interfaces */

      dev[0].devinfo.ifnobase = ifnobase;             /* Offset to Interface-IDs */
      dev[0].minor = 0;                               /* The minor interface number */

      /* Strings */

      dev[0].devinfo.strbase = strbase;               /* Offset to String Numbers */

      /* Endpoints */

      dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 3;
      dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 4;
      dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 5;

      /* Count up the base numbers */

      ifnobase += dev[0].devinfo.ninterfaces;
      strbase  += dev[0].devinfo.nstrings;

      /* Configure the mass storage device device */

      /* Ask the usbmsc driver to fill in the constants we didn't
       * know here.
       */

      usbmsc_get_composite_devdesc(&dev[1]);

      /* Overwrite and correct some values... */

      /* The callback functions for the USBMSC class */

      dev[1].classobject  = board_mscclassobject;
      dev[1].uninitialize = board_mscuninitialize;

      /* Interfaces */

      dev[1].devinfo.ifnobase = ifnobase;               /* Offset to Interface-IDs */
      dev[1].minor = 0;                                 /* The minor interface number */

      /* Strings */

      dev[1].devinfo.strbase = strbase;                 /* Offset to String Numbers */

      /* Endpoints */

      dev[1].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = 1;
      dev[1].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = 2;

      /* Count up the base numbers */

      ifnobase += dev[1].devinfo.ninterfaces;
      strbase  += dev[1].devinfo.nstrings;

      return composite_initialize(2, dev);
#else
      return NULL;
#endif
    }

  /* Configuration with three CDC/ACMs
   *
   * This configuration can be used e.g. on a samv71-xult. The samv71 has
   * 10 Endpoints (EPs). The EPs 0 up to 7 are DMA aware. The EPs 8 and 9
   * are not.
   *
   * In a composite device we need the EP0 as an control Endpoint.  Each
   * CDC/ACM needs one Interrupt driven and two bulk Endpoints.  This is
   * why we configure the EPs 7, 8 and 9 to be the IRQ-EPs and the
   * EP-Pairs 1/2, 3/4, 5/6 to be the bulk EPs for each device.
   *
   * This means, that
   *
   *   - the Composite device uses EP0 as the control-Endpoint,
   *   - the CDC/ACM 0 uses EP7, EP1 and EP2,
   *   - the CDC/ACM 1 uses EP8, EP3 and EP4,
   *   - the CDC/ACM 2 uses EP9, EP5 and EP6
   *
   * as its EP-Configuration.
   */

  else if (configid == 1)
    {
      struct composite_devdesc_s dev[3];
      int strbase = COMPOSITE_NSTRIDS;
      int ifnobase = 0;
      int ia;

      for (ia = 0; ia < 3; ia++)
        {
          /* Ask the cdcacm driver to fill in the constants we didn't know here */

          cdcacm_get_composite_devdesc(&dev[ia]);

          /* Overwrite and correct some values... */

          /* The callback functions for the CDC/ACM class */

          dev[ia].classobject = cdcacm_classobject;
          dev[ia].uninitialize = cdcacm_uninitialize;

          dev[ia].minor = ia;                         /* The minor interface number */

          /* Interfaces */

          dev[ia].devinfo.ifnobase = ifnobase;        /* Offset to Interface-IDs */

          /* Strings */

          dev[ia].devinfo.strbase = strbase;          /* Offset to String Numbers */

          /* Endpoints */

          dev[ia].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 7 + ia;
          dev[ia].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 1 + ia * 2;
          dev[ia].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 2 + ia * 2;

          ifnobase += dev[ia].devinfo.ninterfaces;
          strbase  += dev[ia].devinfo.nstrings;
        }

      return composite_initialize(3, dev);
    }
  else
    {
      return NULL;
    }
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
