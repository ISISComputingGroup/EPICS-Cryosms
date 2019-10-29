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

#define INIT_ROW_NUM 60


static const char *driverName = "CRYOSMSDriver"; ///< Name of driver for use in message printing 

CRYOSMSDriver::CRYOSMSDriver(const char *portName, std::string devPrefix, const char *tToA, const char *maxCurr, const char *writeUnit, const char *allowPersist)
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
	createParam(P_initLogicString, asynParamOctet, &P_initLogic);
	createParam(P_rateString, asynParamOctet, &P_Rate);
	createParam(P_maxTString, asynParamOctet, &P_MaxT);
	createParam(P_outputModeSetString, asynParamOctet, &P_outputModeSet);
	this->devicePrefix = devPrefix;
	this->teslaToAmps = std::atof(tToA);
	this->maxCurrent = std::atof(maxCurr);
	this->writeUnit = writeUnit;
	this->allowPersist = allowPersist;

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

	if (this->teslaToAmps == 0) {
		std::string statMsg = "No calibration from Tesla to Amps supplied";
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}
	else {
		status = putDb("HIDDEN:CONSTANT:SP", &this->teslaToAmps);
		if (status != asynSuccess) {
			return status;
		}
	}


	if (this->maxCurrent == 0) {
		std::string statMsg = "No Max Current given, writes not allowed";
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
	}
	else {
		status = putDb("HIDDEN:OUTPUTMODE:SP", &falseVal);
		if (status != asynSuccess) {
			return status;
		}
		status = putDb("HIDDEN:MAX:SP", &this->maxCurrent);
	}
	if (status != asynSuccess) {
		return status;
	}

	if (this->writeUnit == "Amps") {
		status = putDb("HIDDEN:OUTPUTMODE:SP", &falseVal);
	}
	else {
		status = putDb("HIDDEN:OUTPUTMODE:SP", &trueVal);
	}
	if (status != asynSuccess) {
		return status;
	}


	if (this->allowPersist == "No") {
		std::string magMode = "Non Persistent";

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

	status = procDb("PAUSE");
	if (status != asynSuccess) {
		return status;
	}

	int isPaused;
	status = getDb("PAUSE", &isPaused);
	if (status != asynSuccess) {
		return status;
	}
	else if (isPaused = 1){
		status = putDb("PAUSE:QUEUE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}
	
	status = procDb("FAN:INIT");
	if (status != asynSuccess) {
		return status;
	}

	double writeToDispConversion;
	std::string writeUnit = std::getenv("WRITE_UNIT");
	std::string displayUnit = std::getenv("DISPLAY_UNIT");
	if (writeUnit == displayUnit) {
		writeToDispConversion = 1.0;
	}
	else if (writeUnit == "TESLA" && displayUnit == "AMPS") {
		writeToDispConversion = this->teslaToAmps;
	}
	else if (writeUnit == "AMPS" && displayUnit == "TESLA") {
		writeToDispConversion = 1.0 / this->teslaToAmps;
	}
	else {
		return asynError;
	}

	float targetVal;
	status = getDb("RAMP:TARGET:DISPLAY", &targetVal);
	if (status != asynSuccess) {
		return status;
	}
	status = putDb("TARGET:SP", &targetVal);
	if (status != asynSuccess) {
		return status;
	}

	float midTarget;
	status = getDb("MID", &midTarget);
	if (status != asynSuccess) {
		return status;
	}
	else {
		midTarget *= writeToDispConversion;
	}
	status = putDb("MID:SP", &midTarget);

	return status;
}

asynStatus CRYOSMSDriver::procDb(std::string pvSuffix) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		throw std::runtime_error("PV not found: " + fullPV);
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
		throw std::runtime_error("PV not found: " + fullPV);
		return asynError;
	}
	return (asynStatus)dbGetField(&addr, addr.dbr_field_type, &pbuffer, NULL, &numReq, NULL);
}
asynStatus CRYOSMSDriver::putDb(std::string pvSuffix, const void *value) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		throw std::runtime_error("PV not found: " + fullPV);
		return asynError;
	}

	return (asynStatus)dbPutField(&addr, addr.dbr_field_type, value, 1);
}

asynStatus CRYOSMSDriver::readFile(const char *dir)
{
	//Reads PID values from a file and places them in an array
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

	int CRYOSMSConfigure(const char *portName, std::string devPrefix, const char *tToA, const char *maxCurr, const char *writeUnit, const char *allowPersist)
	{
		try
		{
			new CRYOSMSDriver(portName, devPrefix, tToA, maxCurr, writeUnit, allowPersist);
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
	static const iocshArg initArg2 = { "tToA", iocshArgString };				///< Tesla - Amp calibration factor
	static const iocshArg initArg3 = { "maxCurr", iocshArgString };				///< Max current permittable
	static const iocshArg initArg4 = { "writeUnit", iocshArgString };			///< Unit to write to PSU in
	static const iocshArg initArg5 = { "allowPersist", iocshArgString };		///< Whether or not to allow persistant mode

	static const iocshArg * const initArgs[] = { &initArg0, &initArg1, &initArg2, &initArg3, &initArg4, &initArg5 };

	static const iocshFuncDef initFuncDef = { "CRYOSMSConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs };

	static void initCallFunc(const iocshArgBuf *args)
	{
		CRYOSMSConfigure(args[0].sval, args[1].sval, args[2].sval, args[3].sval, args[4].sval, args[5].sval);
	}

	/// Register new commands with EPICS IOC shell
	static void CRYOSMSRegister(void)
	{
		iocshRegister(&initFuncDef, initCallFunc);
	}

	epicsExportRegistrar(CRYOSMSRegister);

}
