#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <cstdlib>
#include <map>
#include <string>

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

#define INIT_ROW_NUM 60


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

	createParam(P_deviceNameString, asynParamOctet, &P_deviceName);
	createParam(P_initLogicString, asynParamOctet, &P_initLogic);
	createParam(P_rateString, asynParamOctet, &P_Rate);
	createParam(P_maxTString, asynParamOctet, &P_MaxT);
	createParam(P_outputModeSetString, asynParamOctet, &P_outputModeSet);
	this->devicePrefix = devPrefix;
	this->writeDisabled = FALSE;


	pRate_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));
	pMaxT_ = (epicsFloat64 *)calloc(INIT_ROW_NUM, sizeof(epicsFloat64));
	
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
	else if (function == P_initLogic){
		return onStart();
	}
	else {
		return asynSuccess;
	}
}

asynStatus CRYOSMSDriver::onStart()
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;

	const char *tToA = std::getenv("T_TO_A");

	if (tToA == NULL) {
		const char *statMsg = "No calibration from Tesla to Amps supplied";
		this->writeDisabled = TRUE;
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}
	else {
		double teslaToAmps = std::stod(tToA);
		const char *writeUnit = std::getenv("WRITE_UNIT");
		const char *displayUnit = std::getenv("DISPlAY_UNIT");
		status = putDb("HIDDEN:CONSTANT:SP", &teslaToAmps);
		if (status != asynSuccess) {
			return status;
		}
		if (writeUnit == displayUnit && writeUnit != NULL) {
			this->writeToDispConversion = 1.0;
		}
		else if (writeUnit == "TESLA" && displayUnit == "AMPS") {
			this->writeToDispConversion = teslaToAmps;
		}
		else if (writeUnit == "AMPS" && displayUnit == "TESLA") {
			this->writeToDispConversion = 1.0 / teslaToAmps;
		}
		else if (writeUnit == "TESLA" && displayUnit == "GAUSS") {
			this->writeToDispConversion = 10000.0;
		}
		else {
			this->writeToDispConversion = 10000.0 / teslaToAmps;
		}
	}

	if (std::getenv("MAX_CURR") == NULL) {
		const char *statMsg = "No Max Current given, writes not allowed";
		this->writeDisabled = TRUE;
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
	}
	else {
		status = putDb("HIDDEN:OUTPUTMODE:SP", &falseVal);
		if (status != asynSuccess) {
			return status;
		}
		epicsFloat64 maxCurr = std::stod(std::getenv("MAX_CURR"));
		status = putDb("HIDDEN:MAX:SP", &maxCurr);
	}
	if (status != asynSuccess) {
		return status;
	}

	if (std::getenv("WRITE_UNIT") == "AMPS") {
		status = putDb("HIDDEN:OUTPUTMODE:SP", &falseVal);
	}
	else {
		status = putDb("HIDDEN:OUTPUTMODE:SP", &trueVal);
	}
	if (status != asynSuccess) {
		return status;
	}

	if (std::getenv("ALLOW_PERSIST") == "Yes") {
		if (std::getenv("FAST_FILTER_VALUE") == NULL || std::getenv("FILTER_VALUE") == NULL || std::getenv("NPP") == NULL || std::getenv("FAST_PERSISTENT_SETTLETIME") == NULL ||
			std::getenv("PERSISTENT_SETTLETIME") == NULL || std::getenv("FASTRATE") == NULL) {

			const char *statMsg = "Missing parameters to allow persistent mode to be used";
			this->writeDisabled = TRUE;
			status = putDb("STAT", &statMsg);
			status = putDb("DISABLE", &trueVal);
			if (status != asynSuccess) {
				return status;
			}
		}
		else {
			status = putDb("MAGNET:MODE.DISP", &falseVal);
			if (status != asynSuccess) {
				return status;
			}
			status = putDb("FAST:ZERO.DISP", &falseVal);
			if (status != asynSuccess) {
				return status;
			}
			status = putDb("RAMP:LEADS.DISP", &falseVal);
			if (status != asynSuccess) {
				return status;
			}
		}
	}
	else {
		const char *magMode = "Non Persistent";

		status = putDb("MAGNET:MODE", &magMode);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("FAST:ZERO", &falseVal);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("RAMP:LEADS", &falseVal);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("MAGNET:MODE.DISP", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("FAST:ZERO.DISP", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("RAMP:LEADS.DISP", &trueVal);
		if (status != asynSuccess) {
			return status;
		}

	}

	if (std::getenv("USE_SWITCH") == "Yes" && (std::getenv("SWITCH_TEMP_PV") == NULL || std::getenv("SWITCH_HIGH") == NULL || std::getenv("SWITCH_LOW") == NULL ||
		std::getenv("SWITCH_STABLE_NUMBER") == NULL || std::getenv("HEATER_TOLERANCE") == NULL || std::getenv("SWITCH_TOLERANCE") == NULL || std::getenv("SWITCH_TEMP_TOLERANCE") == NULL ||
		std::getenv("HEATER_OUT") == NULL )){

		const char *statMsg = "Missing parameters to allow a switch to be used";
		this->writeDisabled = TRUE;
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (std::getenv("HEATER_OUT") != NULL) {
		epicsFloat64 heatOut = std::stod(std::getenv("HEATER_OUT"));
		status = putDb("HIDDEN:HEATER:VOLT:SP", &heatOut);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (std::getenv("USE_MAGNET_TEMP") == "Yes" && (std::getenv("MAGNET_TEMP_PV") == NULL || std::getenv("MAX_MAGNET_TEMP") == NULL || std::getenv("MIN_MAGNET_TEMP") == NULL)) {

		const char *statMsg = "Missing parameters to allow the magnet temperature to be used";
		this->writeDisabled = TRUE;
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (std::getenv("COMP_OFF_ACT") == "Yes" && (std::getenv("NO_OF_COMP") == NULL || std::getenv("MIN_NO_OF_COMP_ON") == NULL || std::getenv("COPM_1_STAT_PV") == NULL ||
		std::getenv("COMP_2_STAT_PV") == NULL )) {
		
		const char *statMsg = "Missing parameters to allow actions on the state of the compressors";
		this->writeDisabled = TRUE;
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (std::getenv("RAMP_FILE") == NULL) {
		const char *statMsg = "Missing ramp file path";
		this->writeDisabled = TRUE;
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
	}
	else {
		status = readFile(std::getenv("RAMP_FILE"));
	}
	if (status != asynSuccess) {
		return status;
	}

	double currT;
	double initRate;
	int i;
	status = getDb("OUTPUT:FIELD:TESLA", &currT);
	for (i = 0; i <= sizeof(pMaxT_); i++) {
		if (pMaxT_[i] > currT) {
			break;
		}
	}
	if (i == sizeof(pMaxT_)) {
		initRate = 0.0;
	}
	else {
		initRate = pRate_[i];
	}
	status = putDb("HIDDEN:RAMP:RATE:SP", &initRate);

	status = procDb("PAUSE");
	if (status != asynSuccess) {
		return status;
	}

	std::string isPaused;
	status = getDb("PAUSE", &isPaused);
	if (status != asynSuccess) {
		return status;
	}
	else if (isPaused == "ON"){
		status = putDb("PAUSE:QUEUE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}
	
	status = procDb("FAN:INIT");
	if (status != asynSuccess) {
		return status;
	}

	if (this->writeDisabled == FALSE) {
		float targetVal;
		status = getDb("RAMP:TARGET:DISPLAY", &targetVal);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("TARGET:SP", &targetVal);
		if (status != asynSuccess) {
			return status;
		}

		double midTarget;
		status = getDb("MID", &midTarget);
		if (status != asynSuccess) {
			return status;
		}
		else {
			midTarget *= this->writeToDispConversion;
		}
		status = putDb("MID:SP", &midTarget);
		if (status != asynSuccess) {
			return status;
		}
	}
	
	return status;
}

asynStatus CRYOSMSDriver::procDb(std::string pvSuffix) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		return asynError;
	}
	dbCommon *precord = addr.precord;
	return (asynStatus)dbProcess(precord);
}

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, void *pbuffer) {
	DBADDR addr;
	long numReq = 1;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		return asynError;
	}
	return (asynStatus)dbGetField(&addr, addr.dbr_field_type, &pbuffer, NULL, &numReq, NULL);
}
asynStatus CRYOSMSDriver::putDb(std::string pvSuffix, const void *value) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		return asynError;
	}

	return (asynStatus)dbPutField(&addr, addr.dbr_field_type, value, 1);
}

asynStatus CRYOSMSDriver::readFile(const char *dir)
{
	//Reads ramp rates from a file and places them in an array
	float rate, maxT;
	int ind = 0;
	FILE *fp;
	int rowNum = 0;

	if (NULL != (fp = fopen(dir, "rt"))) {

		//ignore first line
		fscanf(fp, "%*[^\n]\n");

		int result = fscanf(fp, "%f %f", &rate, &maxT);

		//check for incorrect format
		if (result < 2) {
			std::cerr << "ReadASCII: File format incorrect: " << dir << std::endl;
			fclose(fp);
			return asynError;
		}

		do {
			pRate_[ind] = rate;
			pMaxT_[ind] = maxT;

			ind++;

		} while (fscanf(fp, "%f %f", &rate, &maxT) != EOF && ind < INIT_ROW_NUM);

		fclose(fp);

		rowNum = ind;

		doCallbacksFloat64Array(pRate_, rowNum, P_Rate, 0);
		doCallbacksFloat64Array(pMaxT_, rowNum, P_MaxT, 0);
		std::cerr << "ReadASCII: read " << rowNum << " lines from file: " << dir << std::endl;

	}
	else {
		//send a file not found error
		return asynError;
	}

	return asynSuccess;

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

	static const iocshArg initArg0 = { "portName", iocshArgString };			///< Port to connect to
	static const iocshArg initArg1 = { "devicePrefix", iocshArgString };		///< PV Prefix for device

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
