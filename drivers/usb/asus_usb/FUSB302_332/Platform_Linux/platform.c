#include <linux/printk.h>                                                       // pr_err, printk, etc
#include "fusb30x_global.h"                                                     // Chip structure
#include "platform_helpers.h"                                                   // Implementation details
#include "../core/platform.h"
#include "../core/PD_Types.h"
/* for otg function delay */
#include<linux/delay.h>
/* for smbcharger */
#include <linux/power_supply.h>

FSC_BOOL g_vbus_enabled = FALSE;
extern SourceOrSink            sourceOrSink;       // Are we currently a source or a sink?
/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i;

    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Enable/Disable the 5V Source
        fusb_GPIO_Set_VBus5v(blnEnable == TRUE ? true : false);
        if(sourceOrSink == SOURCE) // only OTG set VBUS property
        {
                /* Only set VBUS one time in each event:
                 * [1]  turn on VBUS when VBUS is off
                 * [2]  turn off VBUS when VBUS is on
                 */
                if(g_vbus_enabled != blnEnable) // When condition is "off->on" or "on->off" , set VBUS property
                {
                        USB_FUSB302_331_INFO("set VBUS property : %d(g_vbus_enabled was %d)\n",blnEnable,g_vbus_enabled);
                        power_supply_set_usb_otg(usb_psy, blnEnable == TRUE ? 1 : 0);                           // Enable or Disable VBUS from pmic
                        power_supply_set_usb_otg(usb_parallel_psy, blnEnable == TRUE ? 1 : 0);                  // Enable or Disable VBUS from smb1351
                        g_vbus_enabled = blnEnable;
                }
                else
                {
                        USB_FUSB302_331_INFO("SKIPPING set VBUS property : %d (g_vbus_enabled is already %d)\n", blnEnable, g_vbus_enabled);
                }
        }
        break;
    case VBUS_LVL_12V:
        // Enable/Disable the 12V Source
        fusb_GPIO_Set_VBusOther(blnEnable == TRUE ? true : false);
        break;
    default:
        // Otherwise, do nothing.
        break;
    }

    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return fusb_GPIO_Get_VBus5v() ? TRUE : FALSE;

    case VBUS_LVL_12V:
        // Return the state of the 12V VBUS Source.
        return fusb_GPIO_Get_VBusOther() ? TRUE : FALSE;

    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    if (Data == NULL)
    {
        pr_err("%s - Error: Write data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (fusb_I2C_WriteData((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = TRUE;
    }
    else  // I2C Write failure
    {
        ret = FALSE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    FSC_S32 i = 0;
    FSC_U8 temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        pr_err("FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return FALSE;
    }

    if (Data == NULL)
    {
        pr_err("%s - Error: Read data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (DataLength > 1 && chip->use_i2c_blocks)    // Do block reads if able and necessary
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            ret = FALSE;
        }
        else
        {
            ret = TRUE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((FSC_U8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = TRUE;
            }
            else
            {
                ret = FALSE;
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{
    if (enable == TRUE)
    {
        fusb_StartTimers();
    }
    else
    {
        fusb_StopTimers();
    }
}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_Delay10us(delayCount);
}

/*******************************************************************************
* Function:        platform_fusb302_is_otg_present
* Input:           None
* Return:          true: otg present , flase: no otg present
* Description:     For the function of is_otg_present (qpnp-smbcharger.c)
*                  The MSM8996 original design is no type-c and with id pin.
*                  Therefore , the MSM8996 charger driver will detect the id pin
*                  to judge the otg is present or not.
*                  Now , we use type-c(fusb302) driver and with no id pin in the
*                  MSM8996. So , we need create a function to do this function.
*******************************************************************************/
bool platform_fusb302_is_otg_present(void)
{
        return flag_is_otg_present;
}
EXPORT_SYMBOL(platform_fusb302_is_otg_present);

/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/

extern FSC_U16 core_get_advertised_current(void);
//extern SourceOrSink            sourceOrSink;       // Are we currently a source or a sink?

void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
    bool test_is_otg_present = false;
    FSC_U16 currentAdvert; // for get current value
    //int otg_fail_count = 0;
    set_asus_mux();								// Set MUX orientation for usb3.0
    USB_FUSB302_331_INFO("sourceOrSink = %d\n" , sourceOrSink);


    if ((sourceOrSink == SOURCE) && (orientation != NONE))      // Enabled OTG when source attached
    {
        flag_is_otg_present = true;
        USB_FUSB302_331_INFO("flag_is_otg_present = %d(OTG Mode)\n" , flag_is_otg_present);
    }
    else if((sourceOrSink == SOURCE) && (orientation == NONE))  // Disabled OTG when source dettached
    {
        flag_is_otg_present = false;
        USB_FUSB302_331_INFO("flag_is_otg_present = %d(Dettach OTG Mode)\n" , flag_is_otg_present);
    }

    test_is_otg_present = platform_fusb302_is_otg_present();
    USB_FUSB302_331_INFO("test_is_otg_present = %d\n" , test_is_otg_present);

    // Add charger function here that calls 'core_get_advertised_current()'
    // For Debug: 
    currentAdvert = core_get_advertised_current();
    USB_FUSB302_331_INFO("[%s] Update Current: %dmA\n" , __func__, currentAdvert);

}

/*******************************************************************************
 * Function:        platform_fusb302_current
 * Input:           None
 * Return:          current value (unit = mA)
 * Description:     Charger/PMIC will call this function to know the current value
 * *******************************************************************************/
int platform_fusb302_current(void)
{
        int adv_current = 0;
        adv_current = core_get_advertised_current();
        return adv_current; // unit = mA
}
EXPORT_SYMBOL(platform_fusb302_current);

/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
*                  PDvoltage - PD contract voltage in 50mV steps
*                  PDcurrent - PD contract current in 10mA steps
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
void platform_notify_pd_contract(FSC_BOOL contract, FSC_U32 PDvoltage, FSC_U32 PDcurrent)
{
    // Optional: Notify platform of PD contract
	if (contract)
	{
		USB_FUSB302_331_INFO("[%s] PD Contract: %dmV/%dmA\n" , __func__, PDvoltage * 50, PDcurrent * 10);
	}
	else
	{
		USB_FUSB302_331_INFO("[%s] No PD Contract\n" , __func__);
	}
}

/*******************************************************************************
* Function:        platform_notify_unsupported_accessory
* Input:           None
* Return:          None
* Description:     A callback used by the core to report entry to the
*                  Unsupported Accessory state. The platform may implement
*                  USB Billboard.
*******************************************************************************/
void platform_notify_unsupported_accessory(void)
{
    // Optional: Implement USB Billboard
}

/*******************************************************************************
 * Function:        platform_fusb302_report_attached_capabilities
 * Input:           None
 * Return:          pointer to an array of capabilities of an attached device
 * Description:     The Charger/PMIC can call this to get information to
 *                  implement VBUS voltage/current.  It uses doDataObject_t object.
 * *******************************************************************************/
extern doDataObject_t CapsReceived[7];
unsigned int * platform_fusb302_report_attached_capabilities(void)
{
	int i;
	static unsigned int PDCapsTable[14];
	for (i = 0; i < 7; i++)
	{
		if (CapsReceived[i].PDO.SupplyType == pdoTypeFixed)
		{
			PDCapsTable[i * 2] = CapsReceived[i].FPDOSupply.Voltage;
			PDCapsTable[(i * 2) + 1] = CapsReceived[i].FPDOSupply.MaxCurrent;
		}
		else
		{
			PDCapsTable[i * 2] = 0;
			PDCapsTable[(i * 2) + 1] = 0;
		}
	}
        return PDCapsTable;
}
EXPORT_SYMBOL(platform_fusb302_report_attached_capabilities);

FSC_BOOL platform_check_for_connector_fault()
{
	FSC_BOOL fault_detected = FALSE;
	regMeasure_t saved_measure;
	regSwitches_t saved_switches;

	USB_FUSB302_331_INFO("[%s] Taurus is %s; testing Type-C connector for fault on %s\n", __func__, (sourceOrSink == SOURCE) ? "SOURCE" : "SINK", blnCCPinIsCC1 ? "CC2" : "CC1");

	saved_measure = Registers.Measure;
	saved_switches = Registers.Switches;

	Registers.Measure.MEAS_VBUS = 0;
	Registers.Measure.MDAC = MDAC_0P042V;
	DeviceWrite(regMeasure, 1, &Registers.Measure.byte);

        // set MDAC, measure "other CC", if CC voltage > 0V return fault detected
	if (blnCCPinIsCC1)
	{
		Registers.Switches.VCONN_CC2 = 0;                            // Disable VCONN to CC2
		Registers.Switches.PU_EN2 = 0;                               // Disable CC2 pull-up
		Registers.Switches.PDWN2 = 1;                                // Enable CC2 pull-down
		Registers.Switches.MEAS_CC1 = 0;                             // Disable CC1 measure
		Registers.Switches.MEAS_CC2 = 1;                             // Enable CC2 measure
	}
	else
	{
		Registers.Switches.VCONN_CC1 = 0;                            // Disable VCONN to CC1
		Registers.Switches.PU_EN1 = 0;                               // Disable CC1 pull-up
		Registers.Switches.PDWN1 = 1;                                // Enable CC1 pull-down
		Registers.Switches.MEAS_CC2 = 0;                             // Disable CC2 measure
		Registers.Switches.MEAS_CC1 = 1;                             // Enable CC1 measure
	}
	DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));

	platform_delay_10us(150);                                            // Delay to allow measurement to settle
	DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
	if (Registers.Status.COMP == 1)
	{
		fault_detected = TRUE;
		USB_FUSB302_331_INFO("[%s] 1st test failed: the voltage on %s is greater than 42mV with the pull-down enabled\n", __func__, blnCCPinIsCC1 ? "CC2" : "CC1");
	}
	else
        // enable pull up current source on "other CC", if CC voltage < 2.4V return fault detected
	{
		USB_FUSB302_331_INFO("[%s] 1st test passed: the voltage on %s is less than 42mV with the pull-down enabled\n", __func__, blnCCPinIsCC1 ? "CC2" : "CC1");

		Registers.Measure.MDAC = MDAC_2P436V;
		DeviceWrite(regMeasure, 1, &Registers.Measure.byte);

		Registers.Switches = saved_switches;                                 // restore register
		if (blnCCPinIsCC1)
		{
			Registers.Switches.VCONN_CC2 = 0;                            // Disable VCONN to CC2
			Registers.Switches.PDWN2 = 0;                                // Disable CC2 pull-down
			Registers.Switches.PU_EN2 = 1;                               // Enable CC2 pull-up
			Registers.Switches.MEAS_CC1 = 0;                             // Disable CC1 measure
			Registers.Switches.MEAS_CC2 = 1;                             // Enable CC2 measure
		}
		else
		{
			Registers.Switches.VCONN_CC1 = 0;                            // Disable VCONN to CC1
			Registers.Switches.PDWN1 = 0;                                // Disable CC1 pull-down
			Registers.Switches.PU_EN1 = 1;                               // Enable CC1 pull-up
			Registers.Switches.MEAS_CC2 = 0;                             // Disable CC2 measure
			Registers.Switches.MEAS_CC1 = 1;                             // Enable CC1 measure
		}
		DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));

		platform_delay_10us(150);                                            // Delay to allow measurement to settle
		DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
		if (Registers.Status.COMP == 0)
		{
			fault_detected = TRUE;
			USB_FUSB302_331_INFO("[%s] 2nd test failed: the voltage on %s is less than 2.4V with the pull-up enabled\n", __func__, blnCCPinIsCC1 ? "CC2" : "CC1");
		}
		else
		{
			USB_FUSB302_331_INFO("[%s] 2nd test passed: the voltage on %s is greater than 2.4V with the pull-up enabled\n", __func__, blnCCPinIsCC1 ? "CC2" : "CC1");
		}
	}

	Registers.Measure = saved_measure;
	DeviceWrite(regMeasure, 1, &Registers.Measure.byte);
	Registers.Switches = saved_switches;
	DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));

	USB_FUSB302_331_INFO("[%s] %sfault detected on %s\n", __func__, fault_detected ? "FAIL: " : "PASS: no ", blnCCPinIsCC1 ? "CC2" : "CC1");

//	platform_start_timer(&FaultTestTimer, 15000);                          // Start 5 second timer for Fault Test
	return fault_detected;
}
EXPORT_SYMBOL(platform_check_for_connector_fault);

///*******************************************************************************
// * Function:        platform_fusb302_update_sink_capabilities
// * Input:           Pointer to an array of new Sink PD capabilities
// * Return:          None
// * Description:     The Charger/PMIC can call this to update the PD sink
// *                  capabilities.  It will update the number of available
// *                  PD profiles based on the number of non-zero voltage values.
// *                  This function will also request to re-negotiate the
// *                  PD contract with the attached source.
// * *******************************************************************************/
//extern doDataObject_t CapsSink[7];
//extern sopMainHeader_t CapsHeaderSink;
//extern PolicyState_t PolicyState;
//void platform_fusb302_update_sink_capabilities(unsigned int * PDSinkCaps)
//{
//	int i, capsCnt;
//
//	capsCnt = 0;
//        SinkRequestMaxVoltage = 180;			// Maximum voltage that the sink will request (180 * 50mV = 9V)
//        SinkRequestMaxPower = 18000;			// Maximum power the sink will request (18000 * 0.5mW = 9W, used to calculate current as well)
//        USB_FUSB302_331_INFO("%s - charger request result : max_votlage = %d,max_power = %d\n", __func__, SinkRequestMaxVoltage, SinkRequestMaxPower);
//	for (i = 0; i < 7; i++)
//	{
//		if (PDSinkCaps[(i * 2)] > 0) {
//			CapsSink[i].FPDOSink.Voltage = PDSinkCaps[(i * 2)];
//			CapsSink[i].FPDOSink.OperationalCurrent = PDSinkCaps[(i * 2) + 1];
//			capsCnt++;
//		} else {
//			CapsSink[i].FPDOSink.Voltage = 0;
//			CapsSink[i].FPDOSink.OperationalCurrent = 0;
//		}
//                USB_FUSB302_331_INFO("%s - charger request result : CapsSink[%d].FPDOSink.Voltage = %d\n", __func__, i, CapsSink[i].FPDOSink.Voltage);
//                USB_FUSB302_331_INFO("%s - charger request result : CapsSink[%d].FPDOSink.OperationalCurrent = %d\n", __func__, i, CapsSink[i].FPDOSink.OperationalCurrent);
//	}
//	CapsHeaderSink.NumDataObjects = capsCnt;
//
//	PolicyState = peSinkEvaluateCaps;
//}
//EXPORT_SYMBOL(platform_fusb302_update_sink_capabilities);

/*******************************************************************************
* Function:        platform_notify_bist
* Input:           bistEnabled - TRUE when BIST enabled, FALSE when disabled
* Return:          None
* Description:     A callback used by the core to report the new data role after
*                  a data role swap.
*******************************************************************************/
void platform_notify_bist(FSC_BOOL bistEnabled)
{
    /* if(bistEnabled) doSomething; */
}

/*******************************************************************************
 * Function:        platform_notify_state_chaged
 * Input:           ConnectionState previous_state, ConnectionState current_tate
 * Return:          None
 * Description:     Send event to framework
 *******************************************************************************/
void platform_notify_state_chaged(ConnectionState previous_state, ConnectionState current_tate)
{
        fusb_notify_state_chaged(previous_state, current_tate);
}
