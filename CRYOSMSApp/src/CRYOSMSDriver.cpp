#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <cstdlib>
#include <cstring>
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


#define RETURN_IF_ASYNERROR0(func) status = (func)(); \
if (status != asynSuccess)\
{\
errlogSevPrintf(errlogMajor, "Error returned when calling %s", #func);\
return status; \
}

#define RETURN_IF_ASYNERROR1(func, arg) status = (func)(arg); \
if (status != asynSuccess)\
{\
errlogSevPrintf(errlogMajor, "Error returned when calling %s with arguments %s", #func, arg);\
return status; \
}

#define RETURN_IF_ASYNERROR2(func, arg1, arg2) status = (func)(arg1, arg2); \
if (status != asynSuccess)\
{\
errlogSevPrintf(errlogMajor, "Error returned when calling %s with arguments %s", #func, arg1);\
return status; \
}


static const char *driverName = "CRYOSMSDriver"; ///< Name of driver for use in message printing 

static void eventQueueThread(CRYOSMSDriver* drv);

CRYOSMSDriver::CRYOSMSDriver(const char *portName, std::string devPrefix)
  : asynPortDriver(portName,
	0, /* maxAddr */
	static_cast<int>NUM_SMS_PARAMS, /* num parameters */
	asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
	asynInt32Mask | asynInt32ArrayMask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
	ASYN_CANBLOCK, /* asynFlags.  This driver can block but it is not multi-device */
	1, /* Autoconnect */
	0,
	0), qsm(this), started(false), devicePrefix(devPrefix), writeDisabled(FALSE), atTarget(true), abortQueue(true)

{
	createParam(P_deviceNameString, asynParamOctet, &P_deviceName);
	createParam(P_initLogicString, asynParamInt32, &P_initLogic);
	createParam(P_rateString, asynParamOctet, &P_Rate);
	createParam(P_maxTString, asynParamOctet, &P_MaxT);
	createParam(P_startRampString, asynParamInt32, &P_startRamp);
	createParam(P_pauseRampString, asynParamInt32, &P_pauseRamp);
	createParam(P_abortRampString, asynParamInt32, &P_abortRamp);
	createParam(P_outputModeSetString, asynParamInt32, &P_outputModeSet);

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
		setupRamp();
		return putDb("START:SP", &falseVal);
	}
	else if (function == P_pauseRamp) {
		// 0 = paused off (running)
		// 1 = paused on (paused)
		if (value == 0) {
			qsm.process_event(resumeRampEvent(this));
		}
		else {
			queuePaused = true;
		}
		return asynSuccess;
	}
	else if (function == P_abortRamp && value != 0) {
		qsm.process_event(abortRampEvent(this));
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
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
	}
	else try {
		double teslaToAmps = std::stod(envVarMap.at("T_TO_A"));
		this->writeToDispConversion = unitConversion(1.0, envVarMap.at("WRITE_UNIT"), envVarMap.at("DISPLAY_UNIT"));
		RETURN_IF_ASYNERROR2(putDb, "CONSTANT:_SP", &teslaToAmps);
	}
	catch (std::exception &e) {
		errlogSevPrintf(errlogMajor, "Invalid value of T_TO_A provided");
		const char *statMsg = "Invalid calibration from Tesla to Amps supplied";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkMaxCurr()
/*	Checks whether a maximum allowed current has been supplied. If so, sends the value to the PSU in the "amps" mode
	If not supplied, disables puts and posts relevant status message
*/
{
	asynStatus status = asynSuccess;
	int trueVal = 1;
	int falseVal = 0;
	if (envVarMap.at("MAX_CURR") == NULL) {
		errlogSevPrintf(errlogMajor, "MAX_CURR not provided, check macros are correct");
		const char *statMsg = "No Max Current given, writes not allowed";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
	}
	else {
		testVar = 1;
		epicsFloat64 maxCurr = std::stod(envVarMap.at("MAX_CURR"));
		RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &falseVal);
		RETURN_IF_ASYNERROR2(putDb, "MAX:_SP", &maxCurr);
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
			RETURN_IF_ASYNERROR2(putDb, "MAXVOLT:_SP", &maxVolt);
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
	asynStatus status = asynSuccess;
	int trueVal = 1;
	int falseVal = 0;

	if (!std::strcmp(envVarMap.at("WRITE_UNIT"), "AMPS")) {
		testVar = 1;
		RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &falseVal);
	}
	else {
		testVar = 2;
		RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &trueVal);
	}
	return status;
}

asynStatus CRYOSMSDriver::checkAllowPersist()
/*	Check if the user has specified that persistent mode should be allowed. If so, enable MAGNET:MODE, FAST:ZERO and RAMP:LEADS if all values for persistent mode have been provided.
	If required values have not been provided, disable writes and post a relevant stat message. If the user does not specify that persistent mode should be on, set MAGNET:MODE,
	FAST:ZERO and RAMP:LEADS to 0 and disable them.
*/
{
	asynStatus status = asynSuccess;
	int trueVal = 1;
	int falseVal = 0;
	if (!std::strcmp(envVarMap.at("ALLOW_PERSIST"), "Yes")) {
		if (envVarMap.at("FAST_FILTER_VALUE") == NULL || envVarMap.at("FILTER_VALUE") == NULL || envVarMap.at("NPP") == NULL || envVarMap.at("FAST_PERSISTENT_SETTLETIME") == NULL ||
			envVarMap.at("PERSISTENT_SETTLETIME") == NULL || envVarMap.at("FAST_RATE") == NULL) {

			errlogSevPrintf(errlogMajor, "ALLOW_PERSIST set to yes but other values required for this mode not provided, check macros are correct");
			const char *statMsg = "Missing parameters to allow persistent mode to be used";
			this->writeDisabled = TRUE;
			RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
			RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
		}
		else {
			testVar = 1;
			RETURN_IF_ASYNERROR2(putDb, "MAGNET:MODE.DISP", &falseVal);
			RETURN_IF_ASYNERROR2(putDb, "FAST:ZERO.DISP", &falseVal);
			RETURN_IF_ASYNERROR2(putDb, "RAMP:LEADS.DISP", &falseVal);
		}
	}
	else {
		testVar = 2;
		RETURN_IF_ASYNERROR2(putDb, "MAGNET:MODE", &falseVal);
		RETURN_IF_ASYNERROR2(putDb, "FAST:ZERO", &falseVal);
		RETURN_IF_ASYNERROR2(putDb, "RAMP:LEADS", &falseVal);
		RETURN_IF_ASYNERROR2(putDb, "MAGNET:MODE.DISP", &trueVal);
		RETURN_IF_ASYNERROR2(putDb, "FAST:ZERO.DISP", &trueVal);
		RETURN_IF_ASYNERROR2(putDb, "RAMP:LEADS.DISP", &trueVal);
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
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
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
		RETURN_IF_ASYNERROR2(putDb, "HEATER:VOLT:_SP", &heatOut);
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
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
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
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
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
	asynStatus status = asynSuccess;
	int trueVal = 1;
	testVar = 1;
	if (envVarMap.at("RAMP_FILE") == NULL) {
		errlogSevPrintf(errlogMajor, "Missing ramp file path, check macros are correct");
		const char *statMsg = "Missing ramp file path";
		this->writeDisabled = TRUE;
		testVar = 0;
		RETURN_IF_ASYNERROR2(putDb, "STAT", &statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
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
	RETURN_IF_ASYNERROR2(getDb, "OUTPUT:FIELD:TESLA", currT);
	for (i = 0; i <= static_cast<int>(pMaxT_.size()); i++) {
		if (pMaxT_[i] > abs(currT)) {
			break;
		}
	}
	if (i == static_cast<int>(pMaxT_.size())) {
		initRate = 0.0;
	}
	else {
		initRate = pRate_[i];
	}
	RETURN_IF_ASYNERROR2(putDb, "RAMP:RATE:_SP", &initRate);
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
	const char* envVarsNames[] = {
		"T_TO_A", "WRITE_UNIT", "DISPLAY_UNIT", "MAX_CURR", "MAX_VOLT", "ALLOW_PERSIST", "FAST_FILTER_VALUE", "FILTER_VALUE", "NPP", "FAST_PERSISTANT_SETTLETIME", "PERSISTNENT_SETTLETIME",
		"FASTRATE", "USE_SWITCH", "SWITCH_TEMP_PV", "SWITCH_HIGH", "SWITCH_LOW", "SWITCH_STABLE_NUMBER", "HEATER_TOLERANCE", "SWITCH_TOLERANCE", "SWITCH_TEMP_TOLERANCE", "HEATER_OUT",
		"USE_MAGNET_TEMP", "MAGNET_TEMP_PV", "MAX_MAGNET_TEMP", "MIN_MAGNET_TEMP", "COMP_OFF_ACT", "NO_OF_COMP", "MIN_NO_OF_COMP_ON", "COMP_1_STAT_PV", "COMP_2_STAT_PV", "RAMP_FILE" };
	
	for(int i=0; i<sizeof(envVarsNames)/ sizeof(const char*); ++i)
	{
		envVarMap.insert(std::pair<std::string, const char* >(envVarsNames[i], std::getenv(envVarsNames[i])));
	}

	RETURN_IF_ASYNERROR0(checkTToA);

	RETURN_IF_ASYNERROR0(checkMaxCurr);

	RETURN_IF_ASYNERROR0(checkMaxVolt);

	RETURN_IF_ASYNERROR0(checkWriteUnit);

	RETURN_IF_ASYNERROR0(checkAllowPersist);

	RETURN_IF_ASYNERROR0(checkUseSwitch);

	RETURN_IF_ASYNERROR0(checkHeaterOut);

	RETURN_IF_ASYNERROR0(checkUseMagnetTemp);

	RETURN_IF_ASYNERROR0(checkCompOffAct);

	RETURN_IF_ASYNERROR0(checkRampFile);

	RETURN_IF_ASYNERROR1(procDb, "PAUSE");

	int isPaused;
	RETURN_IF_ASYNERROR2(getDb, "PAUSE", isPaused);
	if (isPaused){
		RETURN_IF_ASYNERROR2(putDb, "PAUSE:QUEUE", &trueVal);
	}
	
	RETURN_IF_ASYNERROR1(procDb, "FAN:INIT");

	if (this->writeDisabled == FALSE) {
		double targetVal;
		RETURN_IF_ASYNERROR2(getDb, "RAMP:TARGET:DISPLAY", targetVal);
		RETURN_IF_ASYNERROR2(putDb, "TARGET:SP", &targetVal);

		double midTarget;
		RETURN_IF_ASYNERROR2(getDb, "MID", midTarget);
		midTarget *= this->writeToDispConversion;
		RETURN_IF_ASYNERROR2(putDb, "MID:SP", &midTarget);
	}

	qsm.start();
	queueThreadId = epicsThreadCreate("Event Queue", epicsThreadPriorityHigh, epicsThreadStackMedium, (EPICSTHREADFUNC)::eventQueueThread, this);

	RETURN_IF_ASYNERROR2(putDb, "INIT", &trueVal);
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
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix.c_str());
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
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix.c_str());
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
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix.c_str());
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
		errlogSevPrintf(errlogMajor, "Invalid PV for putDb: %s", pvSuffix.c_str());
		return asynError;
	}
	status = (asynStatus)dbPutField(&addr, addr.dbr_field_type, value, 1);
	if (status) {
		dbCommon *precord = addr.precord;
		recGblSetSevr(precord, READ_ACCESS_ALARM, INVALID_ALARM);
		errlogSevPrintf(errlogMajor, "Error returned when attempting to set %s", pvSuffix.c_str());
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

double CRYOSMSDriver::unitConversion(double value, const char* startUnit, const char* endUnit)
/*   Function to convert any value between tesla, amps and gauss.
*/
{
	double teslaPerAmp = std::stod(envVarMap.at("T_TO_A"));
	if (std::strcmp(startUnit, endUnit) == 0 && startUnit != NULL) {
		return value;
	}
	else if (std::strcmp(startUnit, "TESLA") == 0 && std::strcmp(endUnit, "AMPS") == 0) {
		return value / teslaPerAmp;
	}
	else if (std::strcmp(startUnit, "AMPS") == 0 && std::strcmp(endUnit, "TESLA") == 0) {
		return value * teslaPerAmp;
	}
	else if (std::strcmp(startUnit, "TESLA") == 0 && std::strcmp(endUnit, "GAUSS") == 0) {
		return value * 10000.0; // 1 Tesla = 10^4 Gauss
	}
	else if (std::strcmp(startUnit, "AMPS") == 0 && std::strcmp(endUnit, "GAUSS") == 0) {
		return value * 10000.0 * teslaPerAmp;
	}
	else if (std::strcmp(startUnit, "GAUSS") == 0 && std::strcmp(endUnit, "TESLA") == 0) {
		return value / 10000.0;
	}
	else if (std::strcmp(startUnit, "GAUSS") == 0 && std::strcmp(endUnit, "TESLA") == 0) {
		return value / (10000.0 * teslaPerAmp);
	}
    errlogSevPrintf(errlogMajor, "Error: Units not converted for %f, %s to %s", value, startUnit, endUnit);
    return 0;
}

static void eventQueueThread(CRYOSMSDriver* drv)
/*	Function run by the event queue thread. Will continually process whichever is the next event in the queue before removing it. Will be suspended when
	the queue is paused, and will not process new events while waiting to reach a target (pauses and aborts processed elsewhere)
*/
{
	while (true)
	{
		if (drv->eventQueue.empty()) {
			epicsThreadSleep(0.1);
			continue;
		}
		boost::apply_visitor(processEventVisitor(drv->qsm, drv->eventQueue), drv->eventQueue.front());
		while (!drv->atTarget) {
			epicsThreadSleep(0.1);
			drv->checkIfPaused();
			drv->checkForTarget();
		}
	}
}

void CRYOSMSDriver::checkIfPaused()
{
	if (queuePaused)
	{
		qsm.process_event(pauseRampEvent(this));
		if (!queuePaused) return;
		epicsThreadSuspendSelf();
	}
}

void CRYOSMSDriver::checkForTarget()
/*	Check if ramp status is "holding on target"
*/
{
	int rampStatus;
	int holdingOnTarget = 1;
	getDb("RAMP:STAT", rampStatus);
	if (rampStatus == holdingOnTarget) {
		atTarget = true;
	}
}

void CRYOSMSDriver::pauseRamp()
/**
 * Tells PSU to pause
 */
{
	int trueVal = 1;
	putDb("PAUSE:_SP", &trueVal);
}

void CRYOSMSDriver::resumeRamp()
/**
 * Tells PSU to resume and unpauses the queue
 */
{
	queuePaused = false;
	epicsThreadResume(queueThreadId);
	int falseVal = 0;
	putDb("PAUSE:_SP", &falseVal);
}

asynStatus CRYOSMSDriver::setupRamp()
{
	asynStatus status;
	double startVal = 0;
	double targetVal = 0;
	RETURN_IF_ASYNERROR2(getDb, "OUTPUT:RAW", startVal);
	RETURN_IF_ASYNERROR2(getDb, "MID:SP", targetVal);

	//The ramp file stores boundaries in Tesla, and the ramp rates to use up to those boundaries in Amps/second.
	//To start, we therefore first convert the current device output (startVal) and target output (targetVal) into tesla.

	targetVal = unitConversion(targetVal, envVarMap.at("DISPLAY_UNIT"), "TESLA");
	
	startVal = unitConversion(startVal, envVarMap.at("WRITE_UNIT"), "TESLA");
	
	//Next, find out if the device starts in +ve or -ve mode
	int sign = (startVal >= 0) ? 1 : -1;

	//Now there are a number of different ways we might need to navigate the ramp table:
	if ((startVal >= 0 && targetVal >= 0) || (startVal < 0 && targetVal < 0))  //1 start and target value are the same sign...
	{
		if (abs(startVal) < abs(targetVal)) //1.1 ...with start being closer to 0:
		{
			//step upwards through the table,
			for (int i = 0; i <= static_cast<int>(pMaxT_.size()) - 1; i++)
			{
				//once startVal has been passed, add a "start ramp" and "end ramp" event to the queue, with the current sign and the rate and boundary of each row  of the table,
				if (pMaxT_[i] > abs(startVal) && pMaxT_[i] < abs(targetVal)) 
				{
					eventQueue.push_back(startRampEvent(this, pRate_[i], pMaxT_[i], sign));
					eventQueue.push_back(targetReachedEvent( this ));
				}
				//until the target would be before the next boundary, so we replace the boundary in the argument for the start event with the target,
				else if (pMaxT_[i] > abs(startVal) && pMaxT_[i] >= abs(targetVal))
				{
					eventQueue.push_back(startRampEvent( this, pRate_[i], targetVal, sign ));
					eventQueue.push_back(targetReachedEvent( this ));
					//and we stop stepping through the table.
					break;
				}
			}
		}
		else //1.2 ...with target being closer to zero than start
		{
			//step downwards through the table,
			for (int i = static_cast<int>(pMaxT_.size()) - 1; i >= 0; i--)
			{
				//as we are going down in intensity, the bopundary for each ramp rate is the boundary listed in the previous row in the table, except for the first row where it is 0,
				double boundary = (i == 0) ? 0 : pMaxT_[i - 1];
				
				//once we pass the startVal, add "start ramp" and "end ramp" events to queue with the current sign, rate for current row of table and boundary found in previous step
				if (boundary < abs(startVal) && boundary > abs(targetVal))
				{
					eventQueue.push_back(startRampEvent( this, pRate_[i], boundary, sign ));
					eventQueue.push_back(targetReachedEvent( this ));
				}
				//if target is before the next boundary, go to target instead,
				else if (boundary < abs(startVal) && boundary <= abs(targetVal))
				{
					eventQueue.push_back(startRampEvent( this, pRate_[i], abs(targetVal), sign ));
					eventQueue.push_back(targetReachedEvent( this ));
					//then stop.
					break;
				}
			}
		}
	}
	else //2 start and target are opposite signs
	{
		//We have already found the initial sign (before this logic block), so start by simply stepping downwards through the table
		for (int i = static_cast<int>(pMaxT_.size()) - 1; i >= 0; i--)
		{
			//when we get to the bottom of the table, schedule a ramp to zero with the ramp rate from the lowest row,
			if (i == 0)
			{
				eventQueue.push_back(startRampEvent(this, pRate_[i], 0, sign));
				eventQueue.push_back(targetReachedEvent(this ));
			}
			//until then, schedule ramps from highest row to loest, for all rows below the start val
			else if (pMaxT_[i - 1] < abs(startVal))
			{
				eventQueue.push_back(startRampEvent( this, pRate_[i], pMaxT_[i - 1], sign ));
				eventQueue.push_back(targetReachedEvent( this ));
			}
		}
		//after we reach zero, flip the sign
		sign = -1 * sign;
		//now ramp upwards to target val:
		for (int i = 0; i <= static_cast<int>(pMaxT_.size()) - 1; i++)
		{
			//If the target is beffore the next boundary, schedule a ramp to the target with the ramp rate of that boundary,
			if (pMaxT_[i] >= abs(targetVal))
			{
				eventQueue.push_back(startRampEvent(this, pRate_[i], abs(targetVal), sign ));
				eventQueue.push_back(targetReachedEvent( this ));
				//and stop steppping through the table
				break;
			}
			//until then, schedule ramps to each boundary from low to high.
			eventQueue.push_back(startRampEvent( this, pRate_[i], pMaxT_[i], sign ));
			eventQueue.push_back(targetReachedEvent( this ));
		}
	}
	return status;
}

void CRYOSMSDriver::startRamping(double rate, double target, int sign)
/* tells PSU to start its ramp
	rate: ramp rate in A/s
	target: ramp target in T. To avoid messy conversions all over the startup logic, conversion is handled here in a single place
	sign: whether the output should be positive or negative.
*/
{
	int trueVal = 1; //need a pointer to "1" later
	int signString = (sign == 1) ? 2 : 1; //sign is handled by mbbo with 0 = 0, 1 = negative, 2 = positive
	putDb("DIRECTION:_SP", &signString);

	target = unitConversion(target, "TESLA", envVarMap.at("WRITE_UNIT"));
	//put values in correct PVs, to be sent to device
	putDb("MID:_SP", &target);
	putDb("RAMP:RATE:_SP", &rate);
	putDb("START:_SP", &trueVal);

	//check that the ramp has started before going back to the event queue loop
	int rampStatus;
	int ramping = 0; //ramp status is mbbi,  0 = ramping
	int numAttempts = 0;
	getDb("RAMP:STAT", rampStatus);
	while (rampStatus != ramping)
	{
		getDb("RAMP:STAT", rampStatus);
		epicsThreadSleep(0.1);
		numAttempts++;
		if (numAttempts > 50) //5 seconds arbitrarily chosen
		{
			double output;
			getDb("OUTPUT:RAW", output);
			if (unitConversion(output, envVarMap.at("DISPLAY_UNIT"), envVarMap.at("WRITE_UNIT")) != target)
			{
				errlogSevPrintf(errlogMajor, "Ramp failing to initialise after 5 seconds, aborting queue");
				const char *statMsg = "Ramp Failing to initialise";
				putDb("STAT", &statMsg);
				eventQueue.push_front(abortRampEvent( this ));
				atTarget = true;
			}
			return;
		}
	}
	atTarget = false;
}

void CRYOSMSDriver::abortRamp()
/**
 * Empties the queue, pauses the device, reads current output, tells the device that this is the new midpoint,
 * unpauses the device and sets the user-facing pause value to unpaused.
 */
{
	std::deque<eventVariant> emptyQueue;
	std::swap(eventQueue, emptyQueue);
	eventQueue.push_back(targetReachedEvent( this ));
	queuePaused = false;
	epicsThreadResume(queueThreadId);
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
