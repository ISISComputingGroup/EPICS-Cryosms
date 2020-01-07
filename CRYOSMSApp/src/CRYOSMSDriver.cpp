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

#include <boost/msm/back/state_machine.hpp>
#include <QueuedStateMachine.h>
#include <StateMachineDriver.h>

#include <epicsExport.h>

#include "CRYOSMSDriver.h"

#define INIT_ROW_NUM 60

#define RETURN_IF_ASYNERROR(func, ...) status = (func)(__VA_ARGS__); \
if (status != asynSuccess)\
{\
return status; \
}

static const char *driverName = "CRYOSMSDriver"; ///< Name of driver for use in message printing 

static void eventQueueThread(CRYOSMSDriver* drv);

CRYOSMSDriver::CRYOSMSDriver(const char *portName, std::string devPrefix)
  : asynPortDriver(portName,
	0, /* maxAddr */
	NUM_SMS_PARAMS, /* num parameters */
	asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
	asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
	ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
	1, /* Autoconnect */
	0,
	0), qsm(static_cast<SMDriver*>(this))
{
	createParam(P_deviceNameString, asynParamOctet, &P_deviceName);
	createParam(P_initLogicString, asynParamOctet, &P_initLogic);
	createParam(P_rateString, asynParamOctet, &P_Rate);
	createParam(P_maxTString, asynParamOctet, &P_MaxT);
	createParam(P_startRampString, asynParamOctet, &P_startRamp);
	createParam(P_pauseRampString, asynParamOctet, &P_pauseRamp);
	createParam(P_abortRampString, asynParamOctet, &P_abortRamp);
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
	int falseVal = 0;
	int trueVal = 1;
	if (function == P_outputModeSet) {
		return putDb("OUTPUTMODE:_SP", &value);
	}
	else if (function == P_initLogic){
		return onStart();
	}
	else if (function == P_startRamp && value == 1) {
		eventQueue.push_back(&startRampEv);
		eventQueue.push_back(&targetReachedEv);
		return putDb("START:SP", &falseVal);
	}
	else if (function == P_pauseRamp) {
		if (value == 0) {
			qsm.process_event(resumeRampEvent{this});
		}
		else {
			queuePaused = true;
		}
		return asynSuccess;
	}
	else if (function == P_abortRamp && value != 0) {
		eventQueue.push_front(&abortRampEv);
		epicsThreadResume(queueThreadId);
		return asynSuccess;
	}
	else {
		return asynSuccess;
	}
}

asynStatus CRYOSMSDriver::checkTToA()
{
	asynStatus status;
	int trueVal = 1;

	if (envVarMap.at("T_TO_A") == NULL) {
		const char *statMsg = "No calibration from Tesla to Amps supplied";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		double teslaToAmps = std::stod(envVarMap.at("T_TO_A"));
		if (! std::strcmp(envVarMap.at("WRITE_UNIT"), envVarMap.at("DISPLAY_UNIT")) && envVarMap.at("WRITE_UNIT") != NULL) {
			this->writeToDispConversion = 1.0;
		}
		else if (!std::strcmp(envVarMap.at("WRITE_UNIT"), "TESLA") && !std::strcmp(envVarMap.at("DISPLAY_UNIT"), "AMPS")) {
			this->writeToDispConversion = teslaToAmps;
		}
		else if (!std::strcmp(envVarMap.at("WRITE_UNIT"), "AMPS") && !std::strcmp(envVarMap.at("DISPLAY_UNIT"), "TESLA")) {
			this->writeToDispConversion = 1.0 / teslaToAmps;
		}
		else if (!std::strcmp(envVarMap.at("WRITE_UNIT"), "TESLA") && !std::strcmp(envVarMap.at("DISPLAY_UNIT"), "GAUSS")) {
			this->writeToDispConversion = 10000.0;
		}
		else {
			this->writeToDispConversion = 10000.0 / teslaToAmps;
		}
		RETURN_IF_ASYNERROR(putDb, "CONSTANT:_SP", &teslaToAmps);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkMaxCurr()
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;
	if (envVarMap.at("MAX_CURR") == NULL) {
		const char *statMsg = "No Max Current given, writes not allowed";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		epicsFloat64 maxCurr = std::stod(envVarMap.at("MAX_CURR"));
		RETURN_IF_ASYNERROR(putDb, "OUTPUTMODE:_SP", &falseVal);
		RETURN_IF_ASYNERROR(putDb, "MAX:_SP", &maxCurr);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkMaxVolt()
{
	asynStatus status;
	if (envVarMap.at("MAX_VOLT") != NULL) {
		epicsFloat64 maxVolt = std::stod(envVarMap.at("MAX_VOLT"));
		RETURN_IF_ASYNERROR(putDb, "MAXVOLT:_SP", &maxVolt);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkWriteUnit()
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;

	if (!std::strcmp(envVarMap.at("WRITE_UNIT"), "AMPS")) {
		RETURN_IF_ASYNERROR(putDb, "OUTPUTMODE:_SP", &falseVal);
	}
	else {
		RETURN_IF_ASYNERROR(putDb, "OUTPUTMODE:_SP", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkAllowPersist()
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;
	if (!std::strcmp(envVarMap.at("ALLOW_PERSIST"), "Yes")) {
		if (envVarMap.at("FAST_FILTER_VALUE") == NULL || envVarMap.at("FILTER_VALUE") == NULL || envVarMap.at("NPP") == NULL || envVarMap.at("FAST_PERSISTENT_SETTLETIME") == NULL ||
			envVarMap.at("PERSISTENT_SETTLETIME") == NULL || envVarMap.at("FASTRATE") == NULL) {

			const char *statMsg = "Missing parameters to allow persistent mode to be used";
			this->writeDisabled = TRUE;
			RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
			RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
		}
		else {
			RETURN_IF_ASYNERROR(putDb, "MAGNET:MODE.DISP", &falseVal);
			RETURN_IF_ASYNERROR(putDb, "FAST:ZERO.DISP", &falseVal);
			RETURN_IF_ASYNERROR(putDb, "RAMP:LEADS.DISP", &falseVal);
		}
	}
	else {
		RETURN_IF_ASYNERROR(putDb, "MAGNET:MODE", &falseVal);
		RETURN_IF_ASYNERROR(putDb, "FAST:ZERO", &falseVal);
		RETURN_IF_ASYNERROR(putDb, "RAMP:LEADS", &falseVal);
		RETURN_IF_ASYNERROR(putDb, "MAGNET:MODE.DISP", &trueVal);
		RETURN_IF_ASYNERROR(putDb, "FAST:ZERO.DISP", &trueVal);
		RETURN_IF_ASYNERROR(putDb, "RAMP:LEADS.DISP", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkUseSwitch()
{
	asynStatus status = asynSuccess;
	int trueVal = 1;

	if (!std::strcmp(envVarMap.at("USE_SWITCH"), "Yes") && (envVarMap.at("SWITCH_TEMP_PV") == NULL || envVarMap.at("SWITCH_HIGH") == NULL || envVarMap.at("SWITCH_LOW") == NULL ||
		envVarMap.at("SWITCH_STABLE_NUMBER") == NULL || envVarMap.at("HEATER_TOLERANCE") == NULL || envVarMap.at("SWITCH_TIMEOUT") == NULL || envVarMap.at("SWITCH_TEMP_TOLERANCE") == NULL ||
		envVarMap.at("HEATER_OUT") == NULL)) 
	{
		const char *statMsg = "Missing parameters to allow a switch to be used";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkHeaterOut()
{
	asynStatus status = asynSuccess;

	if (envVarMap.at("HEATER_OUT") != NULL) {
		epicsFloat64 heatOut = std::stod(envVarMap.at("HEATER_OUT"));
		RETURN_IF_ASYNERROR(putDb, "HEATER:VOLT:_SP", &heatOut);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkUseMagnetTemp()
{
	asynStatus status = asynSuccess;
	int trueVal = 1;

	if (!std::strcmp(envVarMap.at("USE_MAGNET_TEMP"),  "Yes") && (envVarMap.at("MAGNET_TEMP_PV") == NULL || envVarMap.at("MAX_MAGNET_TEMP") == NULL || envVarMap.at("MIN_MAGNET_TEMP") == NULL)) {

		const char *statMsg = "Missing parameters to allow the magnet temperature to be used";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkCompOffAct()
{
	asynStatus status = asynSuccess;
	int trueVal = 1;
	if (!std::strcmp(envVarMap.at("COMP_OFF_ACT"), "Yes") && (envVarMap.at("NO_OF_COMP") == NULL || envVarMap.at("MIN_NO_OF_COMP_ON") == NULL || envVarMap.at("COPM_1_STAT_PV") == NULL ||
		envVarMap.at("COMP_2_STAT_PV") == NULL)) {

		const char *statMsg = "Missing parameters to allow actions on the state of the compressors";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkRampFile()
{
	asynStatus status;
	int trueVal = 1;
	if (envVarMap.at("RAMP_FILE") == NULL) {
		const char *statMsg = "Missing ramp file path";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		status = readFile(envVarMap.at("RAMP_FILE"));
		if (status != asynSuccess) {
			this->writeDisabled = TRUE;
			return status;
		}
	}
	double currT;
	double initRate;
	int i;
	RETURN_IF_ASYNERROR(getDb, "OUTPUT:FIELD:TESLA", &currT);
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
	RETURN_IF_ASYNERROR(putDb, "RAMP:RATE:_SP", &initRate);
	return status;
}

asynStatus CRYOSMSDriver::onStart()
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;
	std::vector<std::string> envVarsNames = {
		"T_TO_A", "WRITE_UNIT", "DISPLAY_UNIT", "MAX_CURR", "MAX_VOLT", "ALLOW_PERSIST", "FAST_FILTER_VALUE", "FILTER_VALUE", "NPP", "FAST_PERSISTANT_SETTLETIME", "PERSISTNENT_SETTLETIME",
		"FASTRATE", "USE_SWITCH", "SWITCH_TEMP_PV", "SWITCH_HIGH", "SWITCH_LOW", "SWITCH_STABLE_NUMBER", "HEATER_TOLERANCE", "SWITHC_TOLERANCE", "SWITCH_TEMP_tOLERANCE", "HEATER_OUT",
		"USE_MAGNET_TEMP", "MAGNET_TEMP_PV", "MAX_MAGNET_TEMP", "MIN_MAGNET_TEMP", "COMP_OFF_ACT", "NO_OF_COMP", "MIN_NO_OF_COMP_ON", "COMP_1_STAT_PV", "COMP_2_STAT_PV", "RAMP_FILE" };
	for (std::string envVar : envVarsNames)
	{
		envVarMap.insert(std::pair<std::string, const char* >(envVar, std::getenv(envVar.c_str())));
	}

	RETURN_IF_ASYNERROR(checkTToA);

	RETURN_IF_ASYNERROR(checkMaxCurr);

	RETURN_IF_ASYNERROR(checkMaxVolt);

	RETURN_IF_ASYNERROR(checkWriteUnit);

	RETURN_IF_ASYNERROR(checkAllowPersist);

	RETURN_IF_ASYNERROR(checkUseSwitch);

	RETURN_IF_ASYNERROR(checkHeaterOut);

	RETURN_IF_ASYNERROR(checkUseMagnetTemp);

	RETURN_IF_ASYNERROR(checkCompOffAct);
	epicsThreadSleep(0.2);//ramp not set correctly unless we wait briefly here
	RETURN_IF_ASYNERROR(checkRampFile);

	RETURN_IF_ASYNERROR(procDb, "PAUSE");

	std::string isPaused;
	RETURN_IF_ASYNERROR(getDb, "PAUSE", &isPaused);
	if (isPaused == "ON"){
		RETURN_IF_ASYNERROR(putDb, "PAUSE:QUEUE", &trueVal);
	}
	
	RETURN_IF_ASYNERROR(procDb, "FAN:INIT");

	if (this->writeDisabled == FALSE) {
		float targetVal;
		RETURN_IF_ASYNERROR(getDb, "RAMP:TARGET:DISPLAY", &targetVal);
		RETURN_IF_ASYNERROR(putDb, "TARGET:SP", &targetVal);

		double midTarget;
		RETURN_IF_ASYNERROR(getDb, "MID", &midTarget);
		midTarget *= this->writeToDispConversion;
		RETURN_IF_ASYNERROR(putDb, "MID:SP", &midTarget);
	}

	qsm.start();
	queueThreadId = epicsThreadCreate("Event Queue", epicsThreadPriorityHigh, epicsThreadStackMedium, (EPICSTHREADFUNC)::eventQueueThread, this);
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

static void eventQueueThread(CRYOSMSDriver* drv)
{
	while (1)
	{
		if (drv->eventQueue.empty()) {
			epicsThreadSleep(1);
			continue;
		}
		void* drvEv = drv->eventQueue.front();
		drv->eventQueue.pop_front();
		drv->qsm.process_event(drvEv);
		while (!drv->atTarget) {
			if (drv->queuePaused) {
				drv->qsm.process_event(pauseRampEvent{ drv });
				continue;
			}
			drv->checkForTarget();
		}
	}
}

void CRYOSMSDriver::checkForTarget()
{
	epicsThreadSleep(1);
	simulatedRampIncrement++;
	if (simulatedRampIncrement > 5) {
		atTarget = true;
	}
}

void CRYOSMSDriver::resumeRamp()
{
	queuePaused = false;
	epicsThreadResume(queueThreadId);
}

void CRYOSMSDriver::abortRamp()
{
	std::deque<void*> emptyQueue;
	std::swap(eventQueue, emptyQueue);
	eventQueue.push_back(&abortRampEv);
	eventQueue.push_back(&targetReachedEv);
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
