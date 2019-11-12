#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <map>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <errlog.h>
#include <iocsh.h>
#include <epicsExit.h>
#include <dbCommon.h>
#include <dbAccess.h>
#include <boRecord.h>

#include <asynPortDriver.h>
#include <asynDriver.h>
#include <asynStandardInterfaces.h>

#include <epicsExport.h>

#include "CRYOSMSDriver.h"


static const char *driverName = "CRYOSMSDriver"; ///< Name of driver for use in message printing 

CRYOSMSDriver::CRYOSMSDriver(const char *portName, std::string devPrefix)
  : asynPortDriver(portName,
	0, /* maxAddr */
	NUM_SMS_PARAMS, /* num parameters */
	asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
	asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
	ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
	1, /* Autoconnect */
	0,
	0)
{
	static const char *functionName = "asynPortDriver";
	createParam(P_deviceNameString, asynParamOctet, &P_deviceName);
	createParam(P_outputModeSetString, asynParamOctet, &P_outputModeSet);
	this->devicePrefix = devPrefix;
	
}
void CRYOSMSDriver::pollerTask()
{
}


asynStatus CRYOSMSDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	if (function == P_outputModeSet) {
		return putDb("HIDDEN:OUTPUTMODE:SP", &value);
	}
	else {
		return asynSuccess;
	}
}



asynStatus CRYOSMSDriver::putDb(std::string pvSuffix, const void *value) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		return asynError;
	}

	return (asynStatus)dbPutField(&addr, addr.dbr_field_type, value, 1);
}


extern "C"
{
	int CRYOSMSConfigure(const char *portName, std::string devPrefix)
	{
		try
		{
			new CRYOSMSDriver(portName, devPrefix);
			return asynSuccess;
		}
		catch (const std::exception &ex)
		{
			errlogSevPrintf(errlogFatal, "CRYOSMSConfigure failed: %s\n", ex.what());
			return asynError;
		}
	}
	// EPICS iocsh shell commands 

	static const iocshArg initArg0 = { "portName", iocshArgString };			///< A name for the asyn driver instance we will create - used to refer to it from EPICS DB files
	static const iocshArg initArg1 = { "devicePrefix", iocshArgString };

	static const iocshArg * const initArgs[] = { &initArg0, &initArg1 };

	static const iocshFuncDef initFuncDef = { "CRYOSMSConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs };

	static void initCallFunc(const iocshArgBuf *args)
	{
		CRYOSMSConfigure(args[0].sval, args[1].sval);
	}

	/// Register new commands with EPICS IOC shell
	static void CRYOSMSRegister(void)
	{
		iocshRegister(&initFuncDef, initCallFunc);
	}

	epicsExportRegistrar(CRYOSMSRegister);

}
