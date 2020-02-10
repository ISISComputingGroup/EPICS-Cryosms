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
#include <recGbl.h>
#include <alarm.h>

#include <asynPortDriver.h>
#include <asynDriver.h>
#include <asynStandardInterfaces.h>

#include <boost/msm/back/state_machine.hpp>
#include <QueuedStateMachine.h>
#include <StateMachineDriver.h>

#include <epicsExport.h>

#include "CRYOSMSDriver.h"


#define RETURN_IF_ASYNERROR(func, ...) status = (func)(__VA_ARGS__); \
if (status != asynSuccess)\
{\
errlogSevPrintf(errlogMajor, "Error returned when calling %s with arguments %s", #func, __VA_ARGS__);\
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
	0), qsm(this)
{
	createParam(P_deviceNameString, asynParamOctet, &P_deviceName);
	createParam(P_initLogicString, asynParamInt32, &P_initLogic);
	createParam(P_rateString, asynParamOctet, &P_Rate);
	createParam(P_maxTString, asynParamOctet, &P_MaxT);
	createParam(P_startRampString, asynParamInt32, &P_startRamp);
	createParam(P_pauseRampString, asynParamInt32, &P_pauseRamp);
	createParam(P_abortRampString, asynParamInt32, &P_abortRamp);
	createParam(P_outputModeSetString, asynParamInt32, &P_outputModeSet);
	this->devicePrefix = devPrefix;
	this->writeDisabled = FALSE;

	std::vector<epicsFloat64*> pRate_; //variables which store the data read from the ramp rate file
	std::vector<epicsFloat64*> pMaxT_;
	
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
		eventQueue.push_back(startRampEvent{ this });
		eventQueue.push_back(targetReachedEvent{ this });
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
		queuePaused = false;
		epicsThreadResume(queueThreadId);
		qsm.process_event(abortRampEvent{ this });
		return asynSuccess;
	}
	else {
		return asynSuccess;
	}
}

asynStatus CRYOSMSDriver::checkTToA()
/*  Checks whether the conversion factor from tesla to amps has been provided, and if so, calculates the conversion
    factor from write units to display units. If no conversion factor is provided, disables all writes and posts
	relevant status message.
	Possible Write units: Amps, Tesla     Possible display units: Amps, Tesla, Gauss
*/
{
	asynStatus status = asynSuccess;
	int trueVal = 1;

	if (envVarMap.at("T_TO_A") == NULL) {
		errlogSevPrintf(errlogMajor, "T_TO_A not provided, check macros are correct");
		const char *statMsg = "No calibration from Tesla to Amps supplied";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else try {
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
			this->writeToDispConversion = 10000.0; // 1 Tesla = 10^4 Gauss
		}
		else {
			this->writeToDispConversion = 10000.0 / teslaToAmps;
		}
		RETURN_IF_ASYNERROR(putDb, "CONSTANT:_SP", &teslaToAmps);
	}
	catch (std::exception &e) {
		errlogSevPrintf(errlogMajor, "Invalid value of T_TO_A provided");
	}
	return status;
}

asynStatus CRYOSMSDriver::checkMaxCurr()
/*	Checks whether a maximum allowed current has been supplied. If so, sends the value to the PSU in the "amps" mode
	If not supplied, disables puts and posts relevant status message
*/
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;
	if (envVarMap.at("MAX_CURR") == NULL) {
		errlogSevPrintf(errlogMajor, "MAX_CURR not provided, check macros are correct");
		const char *statMsg = "No Max Current given, writes not allowed";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		testVar = 1;
		epicsFloat64 maxCurr = std::stod(envVarMap.at("MAX_CURR"));
		RETURN_IF_ASYNERROR(putDb, "OUTPUTMODE:_SP", &falseVal);
		RETURN_IF_ASYNERROR(putDb, "MAX:_SP", &maxCurr);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkMaxVolt()
{
/*	Checks whether a voltage limit has been supplied. Sends the value to the PSU if so.
*/
	asynStatus status = asynSuccess;
	testVar = 1;
	if (envVarMap.at("MAX_VOLT") != NULL) {
		try {
			epicsFloat64 maxVolt = std::stod(envVarMap.at("MAX_VOLT"));
			testVar = 2;
			RETURN_IF_ASYNERROR(putDb, "MAXVOLT:_SP", &maxVolt);
		}
		catch (std::exception &e) {
			errlogSevPrintf(errlogMajor, "Invalid value of MAX_VOLT provided");
		}
	}
	return status;
}

asynStatus CRYOSMSDriver::checkWriteUnit()
{
/* Checks if the user wants to send data to the PSU in units of amps. Sends this choice to the machine if so, otherwise defaults to tesla.
*/
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;

	if (!std::strcmp(envVarMap.at("WRITE_UNIT"), "AMPS")) {
		testVar = 1;
		RETURN_IF_ASYNERROR(putDb, "OUTPUTMODE:_SP", &falseVal);
	}
	else {
		testVar = 2;
		RETURN_IF_ASYNERROR(putDb, "OUTPUTMODE:_SP", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkAllowPersist()
/*	Check if the user has specified that persistent mode should be allowed. If so, enable MAGNET:MODE, FAST:ZERO and RAMP:LEADS if all values for persistent mode have been provided.
	If required values have not been provided, disable writes and post a relevant stat message. If the user does not specify that persistent mode should be on, set MAGNET:MODE,
	FAST:ZERO and RAMP:LEADS to 0 and disable them.
*/
{
	asynStatus status;
	int trueVal = 1;
	int falseVal = 0;
	if (!std::strcmp(envVarMap.at("ALLOW_PERSIST"), "Yes")) {
		if (envVarMap.at("FAST_FILTER_VALUE") == NULL || envVarMap.at("FILTER_VALUE") == NULL || envVarMap.at("NPP") == NULL || envVarMap.at("FAST_PERSISTENT_SETTLETIME") == NULL ||
			envVarMap.at("PERSISTENT_SETTLETIME") == NULL || envVarMap.at("FAST_RATE") == NULL) {

			errlogSevPrintf(errlogMajor, "ALLOW_PERSIST set to yes but other values required for this mode not provided, check macros are correct");
			const char *statMsg = "Missing parameters to allow persistent mode to be used";
			this->writeDisabled = TRUE;
			RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
			RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
		}
		else {
			testVar = 1;
			RETURN_IF_ASYNERROR(putDb, "MAGNET:MODE.DISP", &falseVal);
			RETURN_IF_ASYNERROR(putDb, "FAST:ZERO.DISP", &falseVal);
			RETURN_IF_ASYNERROR(putDb, "RAMP:LEADS.DISP", &falseVal);
		}
	}
	else {
		testVar = 2;
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
/*	If the user has specified that the PSU should monitor and use switches, but has not provided the required information for this, disable puts and post a relevant status message
*/
	asynStatus status = asynSuccess;
	int trueVal = 1;

	if (!std::strcmp(envVarMap.at("USE_SWITCH"), "Yes") && (envVarMap.at("SWITCH_TEMP_PV") == NULL || envVarMap.at("SWITCH_HIGH") == NULL || envVarMap.at("SWITCH_LOW") == NULL ||
		envVarMap.at("SWITCH_STABLE_NUMBER") == NULL || envVarMap.at("HEATER_TOLERANCE") == NULL || envVarMap.at("SWITCH_TIMEOUT") == NULL || envVarMap.at("SWITCH_TEMP_TOLERANCE") == NULL ||
		envVarMap.at("HEATER_OUT") == NULL)) 
	{
		errlogSevPrintf(errlogMajor, "USE_SWITCH set to yes but other values required for this mode not provided, check macros are correct");
		const char *statMsg = "Missing parameters to allow a switch to be used";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else
	{
		testVar = 1;
	}
	return status;
}

asynStatus CRYOSMSDriver::checkHeaterOut()
/*	If the user has supplied a heater output, send this to the PSU
*/
{
	asynStatus status = asynSuccess;

	if (envVarMap.at("HEATER_OUT") != NULL) {
		epicsFloat64 heatOut = std::stod(envVarMap.at("HEATER_OUT"));
		RETURN_IF_ASYNERROR(putDb, "HEATER:VOLT:_SP", &heatOut);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkUseMagnetTemp()
/*	If the user would like the driver to act when the magnet temperature goes out of range, but has not provided required information, disables writes and posts a relevant status message.
*/
{
	asynStatus status = asynSuccess;
	int trueVal = 1;

	if (!std::strcmp(envVarMap.at("USE_MAGNET_TEMP"),  "Yes") && (envVarMap.at("MAGNET_TEMP_PV") == NULL || envVarMap.at("MAX_MAGNET_TEMP") == NULL || envVarMap.at("MIN_MAGNET_TEMP") == NULL)) {

		errlogSevPrintf(errlogMajor, "USE_MAGNET_TEMP set to yes but other values required for this mode not provided, check macros are correct");
		const char *statMsg = "Missing parameters to allow the magnet temperature to be used";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		testVar = 1;
	}
	return status;
}

asynStatus CRYOSMSDriver::checkCompOffAct()
/*	If the user would like the driver to act when the compressors turn off, but has not provided the required information, disable writes and post a relevant status message.
*/
{
	asynStatus status = asynSuccess;
	int trueVal = 1;
	if (!std::strcmp(envVarMap.at("COMP_OFF_ACT"), "Yes") && (envVarMap.at("NO_OF_COMP") == NULL || envVarMap.at("MIN_NO_OF_COMP_ON") == NULL || envVarMap.at("COMP_1_STAT_PV") == NULL ||
		envVarMap.at("COMP_2_STAT_PV") == NULL)) {

		errlogSevPrintf(errlogMajor, "COMP_OFF_ACT set to yes but other values required for this mode not provided, check macros are correct");
		const char *statMsg = "Missing parameters to allow actions on the state of the compressors";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		testVar = 1;
	}
	return status;
}

asynStatus CRYOSMSDriver::checkRampFile()
/*	Reads ramp rates from a specified file. If no file path has been given, disable writes and post and post a relevant status message. Otherwise, read the rows of the file
	into memory, then read the current field value from the device and send back an appropriate ramp rate based on the ramp table.
*/
{
	asynStatus status;
	int trueVal = 1;
	testVar = 1;
	if (envVarMap.at("RAMP_FILE") == NULL) {
		errlogSevPrintf(errlogMajor, "Missing ramp file path, check macros are correct");
		const char *statMsg = "Missing ramp file path";
		this->writeDisabled = TRUE;
		testVar = 0;
		RETURN_IF_ASYNERROR(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR(putDb, "DISABLE", &trueVal);
	}
	else {
		status = readFile(envVarMap.at("RAMP_FILE"));
		if (status != asynSuccess) {
			this->writeDisabled = TRUE;
			testVar = 0;
			return status;
		}
	}
	double currT;
	double initRate;
	int i;
	RETURN_IF_ASYNERROR(getDb, "OUTPUT:FIELD:TESLA", currT);
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
/*	Startup procedure for this driver. First of all, all macros are declared in a vector, which is then iterated over, pulling the values out of their environment variables and storing them in a map.
	These variables are then subject to various checks to make sure that the driver initialises into a valid arrangement. Full details of each check is provided in the individual functions being called
	through RETURN_IF_ASYNERROR. After these check have taken place, the PSU is checke to see if it is paused, if so the state queue will be paused. Next, FAN:INIT is processed to make sure relevant
	records have been initialised at the correct time (need to ensure this happens after rest of setup so we know which units device is using to communicate etc.). Finally, if at any point writes 
	have been disabled set the target and mid setpoints to their readback values, to avoid accidentally starting a ramp to 0.
*/
{
	asynStatus status = asynSuccess;
	if (started)
	{
		return status;
	}
	started = true;
	int trueVal = 1;
	int falseVal = 0;
	std::vector<std::string> envVarsNames = {
		"T_TO_A", "WRITE_UNIT", "DISPLAY_UNIT", "MAX_CURR", "MAX_VOLT", "ALLOW_PERSIST", "FAST_FILTER_VALUE", "FILTER_VALUE", "NPP", "FAST_PERSISTANT_SETTLETIME", "PERSISTNENT_SETTLETIME",
		"FASTRATE", "USE_SWITCH", "SWITCH_TEMP_PV", "SWITCH_HIGH", "SWITCH_LOW", "SWITCH_STABLE_NUMBER", "HEATER_TOLERANCE", "SWITCH_TOLERANCE", "SWITCH_TEMP_TOLERANCE", "HEATER_OUT",
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

	RETURN_IF_ASYNERROR(checkRampFile);

	RETURN_IF_ASYNERROR(procDb, "PAUSE");

	int isPaused;
	RETURN_IF_ASYNERROR(getDb, "PAUSE", isPaused);
	if (isPaused == 1){
		RETURN_IF_ASYNERROR(putDb, "PAUSE:QUEUE", &trueVal);
	}
	
	RETURN_IF_ASYNERROR(procDb, "FAN:INIT");

	if (this->writeDisabled == FALSE) {
		double targetVal;
		RETURN_IF_ASYNERROR(getDb, "RAMP:TARGET:DISPLAY", targetVal);
		RETURN_IF_ASYNERROR(putDb, "TARGET:SP", &targetVal);

		double midTarget;
		RETURN_IF_ASYNERROR(getDb, "MID", midTarget);
		midTarget *= this->writeToDispConversion;
		RETURN_IF_ASYNERROR(putDb, "MID:SP", &midTarget);
	}

	qsm.start();
	queueThreadId = epicsThreadCreate("Event Queue", epicsThreadPriorityHigh, epicsThreadStackMedium, (EPICSTHREADFUNC)::eventQueueThread, this);

	RETURN_IF_ASYNERROR(putDb, "INIT", &trueVal);
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

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, int &pbuffer) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix);
		return asynError;
	}
	if (!(addr.dbr_field_type == DBR_INT64 || addr.dbr_field_type == DBR_ENUM)) {
		errlogSevPrintf(errlogFatal, "Attempting to read field of incorrect data type: %s is not type int", fullPV.c_str());
		return asynError;
	}
	int* val = (int*)addr.pfield;
	pbuffer = *val;
	return asynSuccess;
}

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, double &pbuffer) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix);
		return asynError;
	}
	if (addr.dbr_field_type != DBR_DOUBLE) {
		errlogSevPrintf(errlogFatal, "Attempting to read field of incorrect data type: %s is not type double", fullPV.c_str());
		return asynError;
	}
	double* val = (double*)addr.pfield;
	pbuffer = *val;
	return asynSuccess;
}

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, std::string &pbuffer) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix);
		return asynError;
	}
	if (addr.dbr_field_type != DBR_STRING) {
		errlogSevPrintf(errlogFatal, "Attempting to read field of incorrect data type: %s is not type string", fullPV.c_str());
		return asynError;
	}
	std::string* val = (std::string*)addr.pfield;
	pbuffer = *val;
	return asynSuccess;
}

asynStatus CRYOSMSDriver::putDb(std::string pvSuffix, const void *value) {
	DBADDR addr;
	std::string fullPV = this->devicePrefix + pvSuffix;
	asynStatus status;
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		errlogSevPrintf(errlogMajor, "Invalid PV for putDb: %s", pvSuffix);
		return asynError;
	}
	status = (asynStatus)dbPutField(&addr, addr.dbr_field_type, value, 1);
	if (status) {
		dbCommon *precord = addr.precord;
		recGblSetSevr(precord, READ_ACCESS_ALARM, INVALID_ALARM);
		errlogSevPrintf(errlogMajor, "Error returned when attenpting to set %s to %s", pvSuffix, value);
	}
	return status;
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
			pRate_.push_back(rate);
			pMaxT_.push_back(maxT);

			ind++;

		} while (fscanf(fp, "%f %f", &rate, &maxT) != EOF);

		fclose(fp);

	}
	else {
		//send a file not found error
		return asynError;
	}

	return asynSuccess;

}

static void eventQueueThread(CRYOSMSDriver* drv)
/*	Function run by the event queue thread. Will continually process whichever is the next event in the queue before removing it. Will be suspended when
	the queue is paused, and will not process new events while waiting to reach a target (pauses and aborts processed elsewhere)
*/
{
	while (1)
	{
		if (drv->eventQueue.empty()) {
			epicsThreadSleep(1);
			continue;
		}
		boost::apply_visitor(processEventVisitor(drv->qsm, drv->eventQueue), drv->eventQueue.front());
		while (!drv->atTarget) {
			epicsThreadSleep(1);
			/*  Let the IOC update status from the machine, being over-cautious here as c and the db seem to disagree on the duration of "0.1 seconds". Need to wait otherwise it will think
				ramp has completed before it has started. This is a temporary solution, in a future ticket more rigorous checks for target will be implemented and waiting here won't be needed.*/
			if (drv->queuePaused) {
				drv->qsm.process_event(pauseRampEvent{ drv });
				if (!drv->queuePaused) continue;
				epicsThreadSuspendSelf();
				continue;
			}
			drv->checkForTarget();
		}
	}
}

void CRYOSMSDriver::checkForTarget()
/*	Check if ramp status is "holding on target"
*/
{
	int stat;
	getDb("RAMP:STAT", stat);
	if (stat == 1) {
		atTarget = true;
	}
}

void CRYOSMSDriver::pauseRamp()
/* tells PSU to pause
*/
{
	int trueVal = 1;
	putDb("PAUSE:_SP", &trueVal);
}

void CRYOSMSDriver::resumeRamp()
/* tells PSU to resume and unpauses the queue
*/
{
	queuePaused = false;
	epicsThreadResume(queueThreadId);
	int falseVal = 0;
	putDb("PAUSE:_SP", &falseVal);
}

void CRYOSMSDriver::startRamping()
/* tells PSU to start its ramp
*/
{
	atTarget = false;
	int trueVal = 1;
	double currVal = 0;
	getDb("OUTPUT:RAW", currVal);
	currVal += 30;
	putDb("MID:_SP", &currVal);
	putDb("START:_SP", &trueVal);
}

void CRYOSMSDriver::abortRamp()
/* Empties the queue, tells PSU to take the current location as its target
*/
{
	std::deque<eventVariant> emptyQueue;
	std::swap(eventQueue, emptyQueue);
	eventQueue.push_back(targetReachedEvent{ this });
	int trueVal = 1;
	int falseVal = 0;
	putDb("PAUSE:_SP", &trueVal);

	double currVal;
	getDb("OUTPUT:RAW", currVal);
	putDb("MID:_SP", &currVal);
	putDb("PAUSE:_SP", &falseVal);
	putDb("PAUSE:SP", &falseVal);

}

void CRYOSMSDriver::reachTarget()
{
}

void CRYOSMSDriver::continueAbort()
{
	queuePaused = false;
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
