#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <cstdlib>
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
	createParam(P_initLogicString, asynParamOctet, &P_initLogic);
	createParam(P_rateString, asynParamOctet, &P_Rate);
	createParam(P_maxTString, asynParamOctet, &P_MaxT);
	createParam(P_outputModeSetString, asynParamOctet, &P_outputModeSet);
	this->devicePrefix = devPrefix;
	this->maxCurrent = std::atof(std::getenv("MAX_CURR"));
	this->teslaToAmps = std::atof(std::getenv("T_TO_A"));
	this->maxVolt = std::atof(std::getenv("MAX_CURR"));
	this->allowPersist = std::getenv("ALLOW_PERSIST");
	this->useSwitch = std::getenv("USE_SWITCH");
	this->switchTempPV = std::getenv("SWITVH_TEMP_PV");
	this->switchHigh = std::atof(std::getenv("SWITCH_HIGH"));
	this->switchLow = std::atof(std::getenv("SWITCH_LOW"));
	this->switchStableNumber = std::stoi(std::getenv("SWITCH_STABLE_NUMBER"));
	this->heaterTolerance = std::atof(std::getenv("HEATER_TOLERANCE"));
	this->switchTimeout = std::atof(std::getenv("SWITCH_TIMEOUT"));
	this->switchTempTolerance = std::atof(std::getenv("SWITCH_TEMP_TOLERANCE"));
	this->heaterOut = std::getenv("HEATER_OUT");
	this->useMagnetTemp = std::getenv("USE_MAGNET_TEMP");
	this->magnetTempPV = std::getenv("MAGNET_TEMP_PV");
	this->maxMagnetTemp = std::atof(std::getenv("MAX_MAGNET_TEMP"));
	this->minMagnetTemp = std::atof(std::getenv("MIN_MAGNET_TEMP"));
	this->compOffAct = std::getenv("COMP_OFF_ACT");
	this->noOfComp = std::stoi(std::getenv("NO_OF_COMP"));
	this->minNoOfCompOn = std::stoi(std::getenv("MIN_NO_OF_COMP_ON"));
	this->comp1StatPV = std::getenv("COMP_1_STAT_PV");
	this->comp2StatPV = std::getenv("COMP_2_STAT_PV");
	this->fastRate = std::atof(std::getenv("FAST_RATE"));
	this->fastPersistentSettleTime = std::atof(std::getenv("FAST_PERSISTANT_SETTLE_TIME"));
	this->persistentSettleTime = std::atof(std::getenv("PERSISTANT_SETTLE_TIME"));
	this->filterValue = std::atof(std::getenv("FILTER_VALUE"));
	this->fastFilterValue = std::atof(std::getenv("FAST_FILTER_VALUE"));
	this->npp = std::atof(std::getenv("NPP"));

	std::string writeUnit = std::getenv("WRITE_UNIT");
	std::string displayUnit = std::getenv("DISPLAY_UNIT");

	if (writeUnit == displayUnit && writeUnit != NULL) {
		this->writeToDispConversion = 1.0;
	}
	else if (writeUnit == "TESLA" && displayUnit == "AMPS") {
		this->writeToDispConversion = this->teslaToAmps;
	}
	else {
		this->writeToDispConversion = 1.0 / this->teslaToAmps;
	}

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


	if (this->allowPersist == "Yes") {
		if (this->fastFilterValue == 0 || this->filterValue == 0 || this->npp == 0 || this->fastPersistentSettleTime == 0 ||
			this->persistentSettleTime == 0 || this->fastRate == 0) {

			std::string statMsg = "Missing parameters to allow persistent mode to be used";
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

	if (this->useSwitch == "Yes" && (this->switchTempPV == "" || this->switchHigh == 0 || this->switchLow == 0 || this->switchStableNumber == 0 ||
		this->heaterTolerance == 0 || this->switchTimeout == 0 || this->switchTempTolerance == 0 || this->heaterOut == "")) {

		std::string statMsg = "Missing parameters to allow a switch to be used";
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (this->heaterOut != "") {
		status = putDb("HIDDEN:HEATER:VOLT:SP", &this->heaterOut);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (this->useMagnetTemp == "Yes" && (this->magnetTempPV == "" || this->maxMagnetTemp == 0 || this->minMagnetTemp == 0)) {

		std::string statMsg = "Missing parameters to allow the magnet temperature to be used";
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
		if (status != asynSuccess) {
			return status;
		}
	}

	if (this->compOffAct == "Yes" && (this->noOfComp == 0 || this->minNoOfCompOn == 0 || this->comp1StatPV == "" || this->comp2StatPV == "")) {
		
		std::string statMsg = "Missing parameters to allow actions on the state of the compressors";
		status = putDb("STAT", &statMsg);
		status = putDb("DISABLE", &trueVal);
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
