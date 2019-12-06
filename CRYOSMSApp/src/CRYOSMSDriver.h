#ifndef CRYOSMSDRIVER_H
#define CRYOSMSDRIVER_H

#include <asynPortDriver.h>

/// EPICS Asyn port driver class. 
class CRYOSMSDriver : public asynPortDriver
{
public:
	CRYOSMSDriver(const char *portName, std::string devPrefix);
	virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	asynStatus checkTToA();
	asynStatus checkMaxCurr();
	asynStatus checkMaxVolt();
	asynStatus checkWriteUnit();
	asynStatus checkAllowPersist();
	asynStatus checkUseSwitch();
	asynStatus checkHeaterOut();
	asynStatus checkUseMagnetTemp();
	asynStatus checkCompOffAct();
	asynStatus checkRampFile();
	std::map<std::string,const char*> envVarMap;
	double writeToDispConversion;
	bool writeDisabled;
	asynStatus procDb(std::string pvSuffix);
	asynStatus getDb(std::string pvSuffix, void *pbuffer);
	asynStatus putDb(std::string pvSuffix, const void *value);
private:
	std::string devicePrefix;

#define FIRST_SMS_PARAM P_deviceName

	int P_deviceName; // string
	int P_initLogic;
	int P_Rate; //float
	int P_MaxT; //float
	int P_outputModeSet; //int as above

#define LAST_SMS_PARAM 	P_outputModeSet
#define NUM_SMS_PARAMS	(&LAST_SMS_PARAM - &FIRST_SMS_PARAM + 1)


	epicsFloat64 *pRate_;
	epicsFloat64 *pMaxT_;
	asynStatus onStart();
	asynStatus readFile(const char *dir);
	static void pollerTaskC(void* arg)
	{
		CRYOSMSDriver* driver = static_cast<CRYOSMSDriver*>(arg);
		driver->pollerTask();
	}
	void pollerTask();
};

#define P_deviceNameString "DEVICE"
#define P_initLogicString "INIT_LOGIC"
#define P_rateString "Rate"
#define P_maxTString "MaxT"
#define P_outputModeSetString "OUTPUTMODE_SET"

#endif /* CRYOSMSDRIVER_H */
