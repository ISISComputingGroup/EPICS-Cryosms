#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <exception>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cmath>
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

#define RETURN_IF_ASYNERROR3(func, arg1, arg2, arg3) status = (func)(arg1, arg2, arg3); \
if (status != asynSuccess)\
{\
errlogSevPrintf(errlogMajor, "Error returned when calling %s with arguments %s", #func, arg1);\
return status; \
}

static const char *driverName = "CRYOSMSDriver"; ///< Name of driver for use in message printing 

static void eventQueueThread(CRYOSMSDriver* drv);

CRYOSMSDriver::CRYOSMSDriver(const char *portName, std::string devPrefix, const char *TToA, const char *writeUnit, const char *displayUnit, const char *maxCurr, const char *maxVolt,
	const char *allowPersist, const char *fastFilterValue, const char *filterValue, const char *npp, const char *fastPersistentSettletime, const char *persistentSettletime,
	const char *fastRate, const char *useSwitch, const char *switchTempPv, const char *switchHigh, const char *switchLow, const char *switchStableNumber, const char *heaterTolerance,
	const char *switchTimeout, const char *heaterOut, const char *useMagnetTemp, const char *magnetTempPv, const char *maxMagnetTemp,
	const char *minMagnetTemp, const char *compOffAct, const char *noOfComp, const char *minNoOfComp, const char *comp1StatPv, const char *comp2StatPv, const char *rampFile,
	const char *cryomagnet, const char *voltTolerance, const char *voltStabilityDuration, const char *midTolerance)
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
	createParam(P_calcHeaterString, asynParamInt32, &P_calcHeater);

	std::vector<epicsFloat64*> pRate_; //variables which store the data read from the ramp rate file
	std::vector<epicsFloat64*> pMaxT_;

	const char* envVarsNames[] = {
		"T_TO_A", "WRITE_UNIT", "DISPLAY_UNIT", "MAX_CURR", "MAX_VOLT", "ALLOW_PERSIST", "FAST_FILTER_VALUE", "FILTER_VALUE", "NPP", "FAST_PERSISTENT_SETTLETIME", "PERSISTENT_SETTLETIME",
		"FAST_RATE", "USE_SWITCH", "SWITCH_TEMP_PV", "SWITCH_HIGH", "SWITCH_LOW", "SWITCH_STABLE_NUMBER", "HEATER_TOLERANCE", "SWITCH_TIMEOUT", "HEATER_OUT",
		"USE_MAGNET_TEMP", "MAGNET_TEMP_PV", "MAX_MAGNET_TEMP", "MIN_MAGNET_TEMP", "COMP_OFF_ACT", "NO_OF_COMP", "MIN_NO_OF_COMP_ON", "COMP_1_STAT_PV", "COMP_2_STAT_PV", "RAMP_FILE",
		"CRYOMAGNET", "VOLT_TOLERANCE", "VOLT_STABILITY_DURATION", "MID_TOLERANCE"};

	const char* envVarVals[] = { TToA, writeUnit, displayUnit, maxCurr, maxVolt, allowPersist, fastFilterValue, filterValue, npp, fastPersistentSettletime, persistentSettletime,
				fastRate, useSwitch, switchTempPv, switchHigh, switchLow, switchStableNumber, heaterTolerance, switchTimeout, heaterOut,
				useMagnetTemp, magnetTempPv, maxMagnetTemp, minMagnetTemp, compOffAct, noOfComp, minNoOfComp, comp1StatPv, comp2StatPv, rampFile,
				cryomagnet, voltTolerance, voltStabilityDuration, midTolerance};
	for (int i = 0; i < sizeof(envVarsNames) / sizeof(const char*); ++i)
	{
		std::pair<std::string, std::string > newPair(envVarsNames[i], envVarVals[i]);
		envVarMap.insert(newPair);
	}
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
		if (value == 0)
		{
			envVarMap.at("WRITE_UNIT") = "AMPS";
		}
		else 
		{
			envVarMap.at("WRITE_UNIT") = "TESLA";
		}
		return putDb("OUTPUTMODE:_SP", &value);
	}
	else if (function == P_initLogic){
		return onStart();
	}
	else if (function == P_startRamp && value == 1) {
		setupRamp();
		return putDb("START:SP", &falseVal);
	}
	else if (function == P_calcHeater) {
		// Deconstructing the "HEATER" command is a little too complicated for streamDevice so we do it here. There are 2 types of readback:
		// ........ HEATER STATUS: {OFF|ON}
		// ........ HEATER STATUS: OFF AT {value} {units}
		// Due to string size constraints we get stream device to cut off everything before the OFF/ON
		asynStatus status = asynSuccess;
		std::string heater_resp;
		// Extract from db
		RETURN_IF_ASYNERROR2(getDb, "HEATER:STAT:RAW", heater_resp);
		// Handle simple on/off case
		if (heater_resp.find("ON") != std::string::npos) {
			return putDb("HEATER:STAT", &trueVal);
		}
		else if (heater_resp.find("OFF") != std::string::npos && heater_resp.find("AT") == std::string::npos)
		{
			return putDb("HEATER:STAT", &falseVal);
		}
		// If we find "AT"  then we know to expect numbers
		else if (heater_resp.find("OFF") != std::string::npos && heater_resp.find("AT") != std::string::npos) {
			RETURN_IF_ASYNERROR2(putDb, "HEATER:STAT", &falseVal);
			// Check units. Can only be TESLA or AMPS so we just use if / else instead of if / else if / else
			if (heater_resp.find("TESLA") != std::string::npos) {
				RETURN_IF_ASYNERROR2(putDb, "OUTPUT:PERSIST:RAW:UNIT", &trueVal);
			}
			else {
				RETURN_IF_ASYNERROR2(putDb, "OUTPUT:PERSIST:RAW:UNIT", &falseVal);
			}
			// Find first  and last digits of value
			int firstNum = heater_resp.find_first_of("1234567890");
			int lastNum = heater_resp.find_last_of("1234567890");
			// And snip the middle to another string
			std::string persistStr(heater_resp, firstNum, lastNum);
			double persistVal = std::stod(persistStr);
			// Then put it in the persist record
			return putDb("OUTPUT:PERSIST:RAW", &persistVal);
		}
		else {
			return status;
		}
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
		warming = 0;
		cooling = 0;
		return asynSuccess;
	}
	else {
		return asynSuccess;
	}
}

asynStatus CRYOSMSDriver::checkTToA()
/*  Checks whether the conversion factor from tesla to amps has been provided, and if so, calculates the conversion
    factor from write units to display units. If no conversion factor is provided, disables all writes and posts
	relevant status message. Also ensure that device is initially communicating in correct units.
	Possible Write units: Amps, Tesla     Possible display units: Amps, Tesla, Gauss
*/
{
	asynStatus status = asynSuccess;
	int trueVal = 1;
	int falseVal = 0;

	if (envVarMap.at("T_TO_A") == "NULL") {
		errlogSevPrintf(errlogMajor, "T_TO_A not provided, check macros are correct");
		const char *statMsg = "No calibration from Tesla to Amps supplied";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
	}
	else try {
		double teslaToAmps = std::stod(envVarMap.at("T_TO_A"));
		this->writeToDispConversion = unitConversion(1.0, envVarMap.at("WRITE_UNIT"), envVarMap.at("DISPLAY_UNIT"));
		RETURN_IF_ASYNERROR2(putDb, "CONSTANT:_SP", &teslaToAmps);
		if (envVarMap.at("DISPLAY_UNIT").compare("TESLA") == 0) {
			RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &trueVal);
		}
		else {
			RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &falseVal);
		}
	}
	catch (std::exception &e) {
		errlogSevPrintf(errlogMajor, "Invalid value of T_TO_A provided");
		const char *statMsg = "Invalid calibration from Tesla to Amps supplied";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
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
	if (envVarMap.at("MAX_CURR") == "NULL") {
		errlogSevPrintf(errlogMajor, "MAX_CURR not provided, check macros are correct");
		const char *statMsg = "No Max Current given, writes not allowed";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
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
	if (envVarMap.at("MAX_VOLT") != "NULL") {
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

	if (!envVarMap.at("WRITE_UNIT").compare("AMPS")) {
		testVar = 1;
		RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &falseVal);
		correctWriteUnit = "AMPS";
	}
	else {
		testVar = 2;
		RETURN_IF_ASYNERROR2(putDb, "OUTPUTMODE:_SP", &trueVal);
		correctWriteUnit = "TESLA";
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
	if (!envVarMap.at("ALLOW_PERSIST").compare("Yes")) {
		if (envVarMap.at("FAST_FILTER_VALUE") == "NULL" || envVarMap.at("FILTER_VALUE") == "NULL" || envVarMap.at("NPP") == "NULL" || envVarMap.at("FAST_PERSISTENT_SETTLETIME") == "NULL" ||
			envVarMap.at("PERSISTENT_SETTLETIME") == "NULL" || envVarMap.at("FAST_RATE") == "NULL") {

			errlogSevPrintf(errlogMajor, "ALLOW_PERSIST set to yes but other values required for this mode not provided, check macros are correct");
			const char *statMsg = "Missing parameters to allow persistent mode to be used";
			this->writeDisabled = TRUE;
			RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
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

	if (!envVarMap.at("USE_SWITCH").compare("Yes") && (envVarMap.at("SWITCH_TEMP_PV") == "NULL" || envVarMap.at("SWITCH_HIGH") == "NULL" || envVarMap.at("SWITCH_LOW") == "NULL" ||
		envVarMap.at("SWITCH_STABLE_NUMBER") == "NULL" || envVarMap.at("HEATER_TOLERANCE") == "NULL" || envVarMap.at("SWITCH_TIMEOUT") == "NULL" ||
		envVarMap.at("HEATER_OUT") == "NULL")) 
	{
		errlogSevPrintf(errlogMajor, "USE_SWITCH set to yes but other values required for this mode not provided, check macros are correct");
		const char *statMsg = "Missing parameters to allow a switch to be used";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
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

	if (envVarMap.at("HEATER_OUT") != "NULL") {
		epicsFloat64 heatOut;
		RETURN_IF_ASYNERROR3(getDb, envVarMap.at("HEATER_OUT"), heatOut, true);
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

	if (!envVarMap.at("USE_MAGNET_TEMP").compare("Yes") && (envVarMap.at("MAGNET_TEMP_PV") == "NULL" || envVarMap.at("MAX_MAGNET_TEMP") == "NULL" || envVarMap.at("MIN_MAGNET_TEMP") == "NULL")) {

		errlogSevPrintf(errlogMajor, "USE_MAGNET_TEMP set to yes but other values required for this mode not provided, check macros are correct");
		const char *statMsg = "Missing parameters to allow the magnet temperature to be used";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
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
	if (!envVarMap.at("COMP_OFF_ACT").compare("Yes") && (envVarMap.at("NO_OF_COMP") == "NULL" || envVarMap.at("MIN_NO_OF_COMP_ON") == "NULL" || envVarMap.at("COMP_1_STAT_PV") == "NULL" ||
		envVarMap.at("COMP_2_STAT_PV") == "NULL")) {

		errlogSevPrintf(errlogMajor, "COMP_OFF_ACT set to yes but other values required for this mode not provided, check macros are correct");
		const char *statMsg = "Missing parameters to allow actions on the state of the compressors";
		this->writeDisabled = TRUE;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
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
	if (envVarMap.at("RAMP_FILE") == "NULL") {
		errlogSevPrintf(errlogMajor, "Missing ramp file path, check macros are correct");
		const char *statMsg = "Missing ramp file path";
		this->writeDisabled = TRUE;
		testVar = 0;
		RETURN_IF_ASYNERROR2(putDb, "STAT", statMsg);
		RETURN_IF_ASYNERROR2(putDb, "DISABLE", &trueVal);
	}
	else {
		status = readFile(envVarMap.at("RAMP_FILE"));
		if (status != asynSuccess) {
			errlogSevPrintf(errlogMajor, "Unable to read ramp file");
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
	checkThreadId = epicsThreadCreate("Checks Queue", epicsThreadPriorityHigh, epicsThreadStackMedium, (EPICSTHREADFUNC)::eventQueueThread, this);

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

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, int &pbuffer, bool isExternal) {
	DBADDR addr;
	std::string fullPV;
	if (!isExternal) {
		fullPV = this->devicePrefix + pvSuffix;
	}
	else {
		fullPV = pvSuffix;
	}
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

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, double &pbuffer, bool isExternal) {
	DBADDR addr;
	std::string fullPV;
	if (!isExternal) {
		fullPV = this->devicePrefix + pvSuffix;
	}
	else {
		fullPV = pvSuffix;
	}
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

asynStatus CRYOSMSDriver::getDb(std::string pvSuffix, std::string &pbuffer, bool isExternal) {
	DBADDR addr;
	std::string fullPV;
	if (!isExternal) {
		fullPV = this->devicePrefix + pvSuffix;
	}
	else {
		fullPV = pvSuffix;
	}
	if (dbNameToAddr(fullPV.c_str(), &addr)) {
		errlogSevPrintf(errlogMajor, "Invalid PV for getDb: %s", pvSuffix.c_str());
		return asynError;
	}
	if (addr.dbr_field_type != DBR_STRING) {
		errlogSevPrintf(errlogFatal, "Attempting to read field of incorrect data type: %s is not type string", fullPV.c_str());
		return asynError;
	}
	void * pf = addr.pfield;
	const char *val[64] = { (const char*)pf };
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
	epicsThreadSleep(0.01);
	if (status) {
		dbCommon *precord = addr.precord;
		recGblSetSevr(precord, READ_ACCESS_ALARM, INVALID_ALARM);
		errlogSevPrintf(errlogMajor, "Error returned when attempting to set %s", pvSuffix.c_str());
	}
	return status;
}

asynStatus CRYOSMSDriver::readFile(std::string str_dir)
{
	//Reads ramp rates from a file and places them in an array
	float rate, maxT;
	int ind = 0;
	FILE *fp;
	int rowNum = 0;
	const char *dir = str_dir.c_str();

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

double CRYOSMSDriver::unitConversion(double value, std::string startUnit, std::string endUnit)
/*   Function to convert any value between tesla, amps and gauss.
*/
{
	double teslaPerAmp = std::stod(envVarMap.at("T_TO_A"));
	if (startUnit.compare(endUnit) == 0 && startUnit != "NULL") {
		return value;
	}
	else if (startUnit.compare("TESLA") == 0 && endUnit.compare("AMPS") == 0) {
		return value / teslaPerAmp;
	}
	else if (startUnit.compare("AMPS") == 0 && endUnit.compare("TESLA") == 0) {
		return value * teslaPerAmp;
	}
	else if (startUnit.compare("TESLA") == 0 && endUnit.compare("GAUSS") == 0) {
		return value * 10000.0; // 1 Tesla = 10^4 Gauss
	}
	else if (startUnit.compare("AMPS") == 0 && endUnit.compare("GAUSS") == 0) {
		return value * 10000.0 * teslaPerAmp;
	}
	else if (startUnit.compare("GAUSS") == 0 && endUnit.compare("TESLA") == 0) {
		return value / 10000.0;
	}
	else if (startUnit.compare("GAUSS") == 0 && endUnit.compare("AMPS") == 0) {
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
		while (drv->cooling || drv->warming)
		{
			epicsThreadSleep(0.1);
			drv->checkHeaterDone();
		}
	}
}

static void checksThread(CRYOSMSDriver* drv)
/*  Function which performs various periodic checks
*/
{
	int writeUnitInc;
	std::deque<double> voltReadings;
	int falseVal = 0;
	int trueVal = 1;

	while (true)
	{
		epicsThreadSleep(0.5);
		// Check compressor status if applicable
		if (!drv->envVarMap.at("COMP_OFF_ACT").compare("Yes")) 
		{
			int comp1stat;
			int comp2stat;
			int compStatMsg;

			drv->getDb(drv->envVarMap.at("COMP_1_STAT_PV"), comp1stat, true);
			drv->getDb(drv->envVarMap.at("COMP_2_STAT_PV"), comp2stat, true);

			if (comp1stat + comp2stat < std::stod(drv->envVarMap.at("MIN_NO_OF_COMP_ON")))
			{
				compStatMsg = 2;
				const char *statMsg = "Paused: not enough compressors on";
				drv->putDb("COMP:STAT", &compStatMsg);
				drv->putDb("STAT", statMsg);
				drv->queuePaused = true;
			}
			else if (comp1stat + comp2stat < std::stod(drv->envVarMap.at("NO_OF_COMP")))
			{
				compStatMsg = 1;
				drv->putDb("COMP:STAT", &compStatMsg);
			}
			else
			{
				compStatMsg = 0;
				drv->putDb("COMP:STAT", &compStatMsg);
			}
		}

		// Checks whether the write unit has been changed, if so restores it after a set period
		if (drv->correctWriteUnit.compare(drv->envVarMap.at("WRITE_UNIT"))) //if write unit stored on init differs from currecnt write unit
		{
			if (writeUnitInc >= 2*  std::stod(drv->envVarMap.at("WRITE_UNIT_TIMEOUT"))) // *2 because this thread polls every half second
			{
				writeUnitInc = 0;
				drv->envVarMap.at("WRITE_UNIT") = drv->correctWriteUnit;
				if (!drv->correctWriteUnit.compare("AMPS"))
				{
					drv->putDb("OUTPUTMODE:_SP", &falseVal);
				}
				else
				{
					drv->putDb("OUTPUTMODE:_SP", &trueVal);
				}
			}
			else
			{
				writeUnitInc++;
			}
		}
		else
		{
			writeUnitInc = 0;
		}

		// Check on the magnet temperature
		if (!drv->envVarMap.at("USE_MAGNET_TEMP").compare("Yes"))
		{
			double magTemp;
			int oldInRange;
			int magTempPause;
			bool inRange = true;

			drv->getDb("MAGNET:TEMP:INRANGE", oldInRange);
			drv->getDb("MAGNET:TEMP:PAUSE", magTempPause);
			drv->getDb(drv->envVarMap.at("MAGNET_TEMP_PV"), magTemp, true);
			if (oldInRange)
			{
				if (magTemp > std::stod(drv->envVarMap.at("MAX_MAGNET_TEMP")))
				{
					drv->putDb("MAGNET:TEMP:INRAGE", &falseVal);
					drv->putDb("MAGNET:TEMP:TOOHOT", &trueVal);
					inRange = false;
				}
				else if (magTemp < std::stod(drv->envVarMap.at("MIN_MAGNET_TEMP")))
				{
					drv->putDb("MAGNET:TEMP:INRANGE", &falseVal);
					inRange = false;
				}
			}
			else if (magTemp >= std::stod(drv->envVarMap.at("MIN_MAGNET_TEMP")) && magTemp <= std::stod(drv->envVarMap.at("MAX_MAGNET_TEMP")))
			{
				drv->putDb("MAGNET:TEMP:INRAGE", &trueVal);
			}
			if (oldInRange == 1 && ! inRange)
			{ 
				const char *statMsg = "Paused: Magnet temperature out of range";
				drv->putDb("MAGNET:TEMP:PAUSE", &trueVal);
				drv->putDb("STAT", statMsg);
				drv->queuePaused = true;
			}
			else if (oldInRange == 0 && inRange)
			{
				drv->putDb("MAGNET:TEMP:PAUSE", &falseVal);
				drv->qsm.process_event(resumeRampEvent(drv));
			}
		}
		// Check Voltage stability
		
		double newVoltage;
		drv->getDb("OUTPUT:VOLT", newVoltage);
		voltReadings.push_back(newVoltage);
		if (voltReadings.size() > std::stod(drv->envVarMap.at("VOLT_STABILITY_DURATION")) * 2)
		{
			voltReadings.pop_front();
		}
		std::deque<double>::iterator maxVItt = std::max_element(voltReadings.begin(), voltReadings.end());
		double maxV = *maxVItt;
		std::deque<double>::iterator minVItt = std::min_element(voltReadings.begin(), voltReadings.end());
		double minV = *minVItt;
		if (abs(maxV - minV) < std::stod(drv->envVarMap.at("VOLT_TOLERANCE"))) // Fairly certain volt can only be +ve, but abs here just in case
		{//within tolerance
			drv->putDb("VOLT:STAT", &trueVal);
		}
		else
		{//not within tolerance
			drv->putDb("VOLT:STAT", &falseVal);
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
	double outputCurr;
	double target;

	getDb("MID:_SP", target);
	getDb("OUTPUT:CURR", outputCurr);

	target = unitConversion(target, envVarMap.at("WRITE_UNIT"), "AMPS");

	getDb("RAMP:STAT", rampStatus);

	if (!envVarMap.at("CRYOMAGNET").compare("Yes"))
	{
		double rampRate;
		double endPoint;
		getDb("RAMP:RATE", rampRate);
		getDb("MID:SP", endPoint);

		double npp = std::stod(envVarMap.at("NPP"));
		double tolerance = std::max(1e-3, (2e-2)*rampRate);
		double currVel = outputCurr - oldCurr;

		double cin = (fastRampZero) ? 1 : 0.1;
		double c = (abs((currVel - oldCurrVel)/ npp) < 1) ? tanh((currVel - oldCurrVel)/ npp) : cin;

		double rateOfChange = currVel * c + oldCurrVel * (1 - c);

		if (target == 0 && endPoint == 0)
		{
			double volt;
			getDb("OUTPUT:VOLT", volt);

			if (abs(rateOfChange) < tolerance && abs(volt) < std::stod(envVarMap.at("VOLT_TOLERANCE")))
			{
				atTarget = true;
			}
		}
		else if (target == 0)
		{
			if (abs(outputCurr) < tolerance)
			{
				atTarget = true;
			}
		}
		else
		{
			int voltStable;
			getDb("VOLT:STAT", voltStable);
			if (voltStable && abs(rateOfChange) < tolerance)
			{
				atTarget = true;
			}
		}

		oldCurr = outputCurr;
		oldCurrVel = currVel;
	}
	else
	{
		if (rampStatus == holdingOnTarget && abs(outputCurr - target) <= std::stod(envVarMap.at("MID_TOLERANCE"))) {
			atTarget = true;
		}
	}
}

void CRYOSMSDriver::checkHeaterDone()
/*	Check if the heater has reached sorrect temperature
*/
{
	int switchStat;

	getDb("SWITCH:STAT", switchStat);

	if (cooling && switchStat == 0)
	{
		cooling = 0;
	}
	if (warming && switchStat == 3)
	{
		warming = 0;
	}
}

void CRYOSMSDriver::pauseRamp()
/**
 * Tells PSU to pause
 */
{
	int trueVal = 1;
	putDb("PAUSE:_SP", &trueVal);
	const char *statMsg = "Paused";
	putDb("STAT", statMsg);
}

void CRYOSMSDriver::resumeRamp()
/**
 * Tells PSU to resume and unpauses the queue
 */
{
	const char *statMsg;
	if (atTarget)
	{
		statMsg = "Ready";
	}
	else if (fastRamp)
	{
		statMsg = "Ramping fast";
	}
	else if (fastRampZero)
	{
		statMsg = "Ramping fast to zero";
	}
	else
	{
		statMsg = "Ramping";
	}
	putDb("STAT", statMsg);
	queuePaused = false;
	epicsThreadResume(queueThreadId);
	int falseVal = 0;
	putDb("PAUSE:_SP", &falseVal);
}

asynStatus CRYOSMSDriver::setupRamp()
/*The logic for setting up ramp events in the state machine, based on rates in the ramp table
*/
{
	asynStatus status;
	double startVal = 0;
	double targetVal = 0;
	int magMode = 0;
	RETURN_IF_ASYNERROR2(getDb, "OUTPUT:RAW", startVal);
	RETURN_IF_ASYNERROR2(getDb, "MID:SP", targetVal);
	RETURN_IF_ASYNERROR2(getDb, "MAGNET:MODE", magMode);

	//First, check if magnet is in persistent mode, then execute relevant commands (moved to other functions for tidiness)

	if (magMode == 1)
	{
		RETURN_IF_ASYNERROR0(setupPersistOn);
		RETURN_IF_ASYNERROR2(getDb, "OUTPUT:PERSIST:RAW", startVal);  // Also consider that we will be starting where the fast ramp stops
	}

	//The ramp file stores boundaries in Tesla, and the ramp rates to use up to those boundaries in Amps/second.
	//To start, we therefore first convert the current device output (startVal) and target output (targetVal) into tesla.

	targetVal = unitConversion(targetVal, envVarMap.at("DISPLAY_UNIT"), "TESLA");

	startVal = unitConversion(startVal, envVarMap.at("WRITE_UNIT"), "TESLA");

	//Also make sure that C++ doesn't try to add ramps from 2.0000000001 to 2, by rounding after unit conversion

	targetVal = (targetVal >= 0 ? floor(1000.0*targetVal + 0.5) : ceil(1000.0*targetVal - 0.5)) / 1000.0;
	startVal = (startVal >= 0 ? floor(1000.0*startVal + 0.5) : ceil(1000.0*startVal - 0.5)) / 1000.0;

	//Next, set the state machine up so that heater status is checked before ramping starts but AFTER any persistent mode events have been processed

	if (!envVarMap.at("CRYOMAGNET").compare("Yes"))
	{
		eventQueue.push_back(checkHeaterEvent(this));
	}
	//Next, find out if the device starts in +ve or -ve mode
	int sign = (startVal >= 0) ? 1 : -1;
	//And set ramp type to standard (for STAT messages)
	RampType rType = RampType::standardRampType;

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
					eventQueue.push_back(startRampEvent(this, pRate_[i], pMaxT_[i], sign, rType));
					eventQueue.push_back(targetReachedEvent( this ));
				}
				//until the target would be before the next boundary, so we replace the boundary in the argument for the start event with the target,
				else if (pMaxT_[i] > abs(startVal) && pMaxT_[i] >= abs(targetVal))
				{
					eventQueue.push_back(startRampEvent( this, pRate_[i], targetVal, sign, rType));
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
				//as we are going down in intensity, the boundary for each ramp rate is the boundary listed in the previous row in the table, except for the first row where it is 0,
				double boundary = (i == 0) ? 0 : pMaxT_[i - 1];
				
				//once we pass the startVal, add "start ramp" and "end ramp" events to queue with the current sign, rate for current row of table and boundary found in previous step
				if (boundary < abs(startVal) && boundary > abs(targetVal))
				{
					eventQueue.push_back(startRampEvent( this, pRate_[i], boundary, sign, rType));
					eventQueue.push_back(targetReachedEvent( this ));
				}
				//if target is before the next boundary, go to target instead,
				else if (boundary < abs(startVal) && boundary <= abs(targetVal))
				{
					eventQueue.push_back(startRampEvent( this, pRate_[i], abs(targetVal), sign, rType));
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
				eventQueue.push_back(startRampEvent(this, pRate_[i], 0, sign, rType));
				eventQueue.push_back(targetReachedEvent(this ));
			}
			//until then, schedule ramps from highest row to loest, for all rows below the start val
			else if (pMaxT_[i - 1] < abs(startVal))
			{
				eventQueue.push_back(startRampEvent( this, pRate_[i], pMaxT_[i - 1], sign, rType));
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
				eventQueue.push_back(startRampEvent(this, pRate_[i], abs(targetVal), sign, rType ));
				eventQueue.push_back(targetReachedEvent( this ));
				//and stop steppping through the table
				break;
			}
			//until then, schedule ramps to each boundary from low to high.
			eventQueue.push_back(startRampEvent( this, pRate_[i], pMaxT_[i], sign, rType ));
			eventQueue.push_back(targetReachedEvent( this ));
		}
	}

	if (magMode == 1)
	{
		//all persistent mode ramps end with cooling back down

		eventQueue.push_back(startCoolEvent(this));
		eventQueue.push_back(tempReachedEvent(this));
	}
	return asynSuccess;
}

asynStatus CRYOSMSDriver::setupPersistOn()
/* Procedures to follow when starting a ramp in persistent mode
*/
{
	asynStatus status;

	int switchStat;
	double persistCurr;

	//Switch stat = 0 if device is cold, 3 if device is warm, 1 or 2 if changing
	RETURN_IF_ASYNERROR2(getDb, "SWITCH:STAT", switchStat);
	RETURN_IF_ASYNERROR2(getDb, "OUTPUT:PERSIST:RAW", persistCurr);

	cooling = (switchStat == 1 ? 1 : 0);
	warming = (switchStat == 2 ? 1 : 0);

	while (switchStat == 1 || switchStat == 2)
	{
		epicsThreadSleep(1); //1s between reads, pointless to poll quicker here
		RETURN_IF_ASYNERROR2(getDb, "SWITCH:STAT", switchStat);
		if (!warming && !cooling) {
			return status; //allows aborting to break this loop (aborting sets both to 0)
		}
	}

	if (switchStat == 0)
	{
		eventQueue.push_back(startCoolEvent(this));
		eventQueue.push_back(tempReachedEvent(this));
	}
	
	persistCurr = unitConversion(persistCurr, envVarMap.at("WRITE_UNIT"), "TESLA"); //ramp targets are all supplied in tesla

	RETURN_IF_ASYNERROR1(setupFastRamp, persistCurr);

	return asynSuccess;
}

asynStatus CRYOSMSDriver::setupFastRamp(double targetVal)
/*set up a fast ramp. Fast ramps do not use the ramping table, instead they use a ramp rate supplied by the FASTRATE macro. They also have unique status messages
 */
{
	asynStatus status;

	double startVal;
	double fastRate = std::stod(envVarMap.at("FAST_RATE"));

	RETURN_IF_ASYNERROR2(getDb, "OUTPUT:RAW", startVal);

	int sign = (startVal >= 0) ? 1 : -1;
	RampType rType;

	//If the target is 0, ramp fast to 0
	if (targetVal == 0)
	{ 
		rType = RampType::fastZeroRampType;

		eventQueue.push_back(startRampEvent(this, fastRate, 0.0, sign, rType));
		eventQueue.push_back(targetReachedEvent(this));
	}
	//when 0 is not the target and the start and target values are the same sign, or the start is 0, simply ramp fast to target
	else if ((startVal <= 0 && targetVal < 0) || (startVal >= 0 && targetVal > 0)) 
	{
		if (startVal == 0)
		{//Make sure we go in the right direction if start is 0. Don't need to worry if target is also 0 as 1- this code won't be reached (handled above) and 2- no ramps will actually happen
			sign = (targetVal >= 0) ? 1 : -1; 
		}
		rType = RampType::fastRampType;

		eventQueue.push_back(startRampEvent(this, fastRate, abs(targetVal), sign, rType));
		eventQueue.push_back(targetReachedEvent(this));
	} //otherwise, ramp to 0 first, then to the target
	else if ((startVal < 0 && targetVal > 0) || (startVal > 0 && targetVal < 0))
	{
		rType = RampType::fastZeroRampType;

		eventQueue.push_back(startRampEvent(this, fastRate, 0.0, sign, rType));
		eventQueue.push_back(targetReachedEvent(this));

		sign = -1 * sign; //going past zero, so flip sign
		rType = RampType::fastRampType;

		eventQueue.push_back(startRampEvent(this, fastRate, abs(targetVal), sign, rType));
		eventQueue.push_back(targetReachedEvent(this));		
	}
	else
	{
		//should never get here, if we have feed back up that something bad happened
		return asynError;
	}
	return asynSuccess;
}

void CRYOSMSDriver::startCooling()
{//Tell device to start cooling, called from state machine
	int falseVal = 0;
	const char* statMsg = "Cooling";
	cooling = 1;

	putDb("HEATER:STAT:_SP", &falseVal);
	putDb("STAT", statMsg);

	//If  RAMP:LEADS set to "Ramp", ramp fast to zero when done
	int rampLeads;
	getDb("RAMP:LEADS", rampLeads);
	if (rampLeads == 1) {
		setupFastRamp(0.0);
	}

}

void CRYOSMSDriver::startWarming()
{//Tell device to start warming, called from state machine
	int trueVal = 1;
	const char* statMsg = "Warming";
	double persistCurr;
	double outputCurr;
	double heaterTolerence = std::stod(envVarMap.at("HEATER_TOLERANCE"));
	warming = 1;

	getDb("OUTPUT:PERSIST:RAW", persistCurr);
	getDb("OUTPUT:RAW", outputCurr);

	while (abs(persistCurr - outputCurr) > heaterTolerence)
	{
		epicsThreadSleep(0.1);
		getDb("OUTPUT:PERSIST:RAW", persistCurr);
		getDb("OUTPUT:RAW", outputCurr);
		if (!warming) {
			return;
		}
	}

	putDb("HEATER:STAT:_SP", &trueVal);
	putDb("STAT", statMsg);
}

void CRYOSMSDriver::reachTemp()
{//Called from state machine when correct temperature reached
	const char* statMsg = "Ready";

	putDb("STAT", statMsg);
}

void CRYOSMSDriver::preRampHeaterCheck()
/* Called from state machine, checks heater is on and warm before continuing with ramps
*/
{
	int heaterStat;
	getDb("HEATER:STAT", heaterStat);
	
	//If heater is off, warm up
	if (heaterStat == 0)
	{
		eventQueue.push_front(tempReachedEvent(this));
		eventQueue.push_front(startWarmEvent(this));
	}
}

void CRYOSMSDriver::startRamping(double rate, double target, int sign, RampType rampType)
/* tells PSU to start its ramp
	rate: ramp rate in A/s
	target: ramp target in T. To avoid messy conversions all over the startup logic, conversion is handled here in a single place
	sign: whether the output should be positive or negative.
	rampType: enum of the sifferent ramp types, standard, fast and fastZero. Used for setting STAT msg
*/
{
	int trueVal = 1; //need a pointer to "1" to set PVs to "true" (can't just send "1")

	const char *statMsg;
	switch (rampType)
	{//set whether ramping is fast/fast zero for future ramps. Done here so that correct status messages are preserved if a puase happens. Cleared upon target reached.
	case standardRampType:
		statMsg = "Ramping";
		break;
	case fastRampType:
		fastRamp = true;
		statMsg = "Ramping fast";
		break;
	case fastZeroRampType:
		fastRampZero = true;
		statMsg = "Ramping fast to zero";
		putDb("FAST:ZERO", &trueVal);
		break;
	}

	int newSign = (sign == 1) ? 2 : 1; //sign is handled by mbbo with 0 = 0, 1 = negative, 2 = positive
	int oldSign;
	double output;

	getDb("OUTPUT", output);
	getDb("DIRECTION", oldSign);

	if (oldSign != newSign && abs(output) <= std::stod(envVarMap.at("MID_TOLERANCE")))
	{
		putDb("DIRECTION:_SP", &newSign);
	}
	else if (oldSign != newSign)
	{
		errlogSevPrintf(errlogMajor, "Cannot change direction when not at 0");
	}


	target = unitConversion(target, "TESLA", envVarMap.at("WRITE_UNIT"));
	//put values in correct PVs, to be sent to device
	putDb("MID:_SP", &target);
	putDb("RAMP:RATE:_SP", &rate);
	putDb("START:_SP", &trueVal);
	putDb("STAT", statMsg);
	atTarget = false;
}

void CRYOSMSDriver::abortRamp()
/*
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

void CRYOSMSDriver::abortBasic()
/* Abort event for when device is not ramping, so only empties the queue and ensures any variables not handled by reachTarget() are reset correctly
*/
{
	std::deque<eventVariant> emptyQueue;
	std::swap(eventQueue, emptyQueue);
	eventQueue.push_back(targetReachedEvent(this));
}

void CRYOSMSDriver::reachTarget()
{
	fastRamp = false;
	fastRampZero = false;
	const char *statMsg = "Ready";
	putDb("STAT", statMsg);
}

void CRYOSMSDriver::continueAbort()
{
	queuePaused = false;
}

extern "C"
{

	int CRYOSMSConfigure(const char *portName, std::string devPrefix, const char *TToA, const char *writeUnit, const char *displayUnit, const char *maxCurr, const char *maxVolt,
		const char *allowPersist, const char *fastFilterValue, const char *filterValue, const char *npp, const char *fastPersistentSettletime, const char *persistentSettletime,
		const char *fastRate, const char *useSwitch, const char *switchTempPv, const char *switchHigh, const char *switchLow, const char *switchStableNumber, const char *heaterTolerance,
		const char *switchTimeout, const char *heaterOut, const char *useMagnetTemp, const char *magnetTempPv, const char *maxMagnetTemp,
		const char *minMagnetTemp, const char *compOffAct, const char *noOfComp, const char *minNoOfComp, const char *comp1StatPv, const char *comp2StatPv, const char *rampFile,
		const char *cryomagnet, const char *voltTolerance, const char *voltStabilityDuration, const char *midTolerance)

	{
		try
		{
			new CRYOSMSDriver(portName, devPrefix, TToA, writeUnit, displayUnit, maxCurr, maxVolt,
				allowPersist, fastFilterValue, filterValue, npp, fastPersistentSettletime, persistentSettletime,
				fastRate, useSwitch, switchTempPv, switchHigh, switchLow, switchStableNumber, heaterTolerance,
				switchTimeout, heaterOut, useMagnetTemp, magnetTempPv, maxMagnetTemp,
				minMagnetTemp, compOffAct, noOfComp, minNoOfComp, comp1StatPv, comp2StatPv, rampFile,
				cryomagnet, voltTolerance, voltStabilityDuration, midTolerance);
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
	static const iocshArg initArg2 = { "TToA", iocshArgString };		///< Tesla/ Amps conversion rate
	static const iocshArg initArg3 = { "writeUnit", iocshArgString };		///< unit to write to device in
	static const iocshArg initArg4 = { "displayUnit", iocshArgString };		///< unit to display values to user in
	static const iocshArg initArg5 = { "maxCurr", iocshArgString };		///< max current for ramping
	static const iocshArg initArg6 = { "maxVolt", iocshArgString };		///< max voltage for device
	static const iocshArg initArg7 = { "allowPersist", iocshArgString };		///< Whether or not to allow persistent mode
	static const iocshArg initArg8 = { "fastFilterValue", iocshArgString };		///< Filter value, fast ramps
	static const iocshArg initArg9 = { "filterValue", iocshArgString };		///< Filter value
	static const iocshArg initArg10 = { "npp", iocshArgString };		///< value used in at-target checks
	static const iocshArg initArg11 = { "fastPersistentSettletime", iocshArgString };		///< time to wait after fast ramp to persist
	static const iocshArg initArg12 = { "persistentSettletime", iocshArgString };		///< time to wait after ramping to persist
	static const iocshArg initArg13 = { "fastRate", iocshArgString };		///< ramp rate to use in fast ramps
	static const iocshArg initArg14 = { "useSwitch", iocshArgString };		///< Whether or not to use switch temperature
	static const iocshArg initArg15 = { "switchTempPv", iocshArgString };		///< PV for switch temp
	static const iocshArg initArg16 = { "switchHigh", iocshArgString };		///< high limit of switch temp
	static const iocshArg initArg17 = { "switchLow", iocshArgString };		///< high limit of switch temp
	static const iocshArg initArg18 = { "switchStableNumber", iocshArgString };		///< number of measurements before switch temp is said to be stable
	static const iocshArg initArg19 = { "heaterTolerance", iocshArgString };		///< max deviation of heater temp
	static const iocshArg initArg20 = { "switchTimeout", iocshArgString };		///< timeout for switch readings
	static const iocshArg initArg21 = { "heaterOut", iocshArgString };		///< PV for heater temp
	static const iocshArg initArg22 = { "useMagnetTemp", iocshArgString };		///< whether to use magnet temperatures
	static const iocshArg initArg23 = { "magnetTempPv", iocshArgString };		///< PV for magnet temp
	static const iocshArg initArg24 = { "maxMagnetTemp", iocshArgString };		///< Max temp of magnet
	static const iocshArg initArg25 = { "minMagnetTemp", iocshArgString };		///< Min temp for magnet
	static const iocshArg initArg26 = { "compOffAct", iocshArgString };		///< Whether to act if compressors turn off
	static const iocshArg initArg27 = { "noOfComp", iocshArgString };		///< Number of connected compressors
	static const iocshArg initArg28 = { "minNoOfComp", iocshArgString };		///< Min number of active comps
	static const iocshArg initArg29 = { "comp1StatPv", iocshArgString };		///< PV for compressor 1 status
	static const iocshArg initArg30 = { "comp2StatPv", iocshArgString };		///< PV for compressor 2 status
	static const iocshArg initArg31 = { "rampFile", iocshArgString };		///< file path for ramp table
	static const iocshArg initArg32 = { "cryomagnet", iocshArgString };		///< whether this is a cryomagnet
	static const iocshArg initArg33 = { "voltTolerance", iocshArgString };		///< Tolerance for volt stability
	static const iocshArg initArg34 = { "voltStabilityDuration", iocshArgString };		///< how long to measure volt stability over
	static const iocshArg initArg35 = { "midTolerance", iocshArgString };		///< Tolerance for checking if midpoing  is reached

	static const iocshArg * const initArgs[] = { &initArg0, &initArg1, &initArg2, &initArg3, &initArg4, &initArg5, &initArg6, &initArg7, &initArg8, &initArg9, &initArg10, &initArg11, 
		&initArg12, &initArg13, &initArg14, &initArg15, &initArg16, &initArg17, &initArg18, &initArg19, &initArg20, &initArg21, &initArg22, &initArg23, &initArg24, &initArg25, &initArg26,
		&initArg27, &initArg28, &initArg29, &initArg30, &initArg31, &initArg32, &initArg33, &initArg34, &initArg35 };

	static const iocshFuncDef initFuncDef = { "CRYOSMSConfigure", sizeof(initArgs) / sizeof(iocshArg*), initArgs };

	static void initCallFunc(const iocshArgBuf *args)
	{
		CRYOSMSConfigure(args[0].sval, args[1].sval, args[2].sval, args[3].sval, args[4].sval, args[5].sval, args[6].sval, args[7].sval, args[8].sval, args[9].sval, args[10].sval, args[11].sval,
			args[12].sval, args[13].sval, args[14].sval, args[15].sval, args[16].sval, args[17].sval, args[18].sval, args[19].sval, args[20].sval, args[21].sval, args[22].sval, args[23].sval,
			args[24].sval, args[25].sval, args[26].sval, args[27].sval, args[28].sval, args[29].sval, args[30].sval, args[31].sval, args[32].sval, args[33].sval, args[34].sval, args[35].sval);
	}

	/// Register new commands with EPICS IOC shell
	static void CRYOSMSRegister(void)
	{
		iocshRegister(&initFuncDef, initCallFunc);
	}

	epicsExportRegistrar(CRYOSMSRegister);

}
