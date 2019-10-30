#ifndef CRYOSMSDRIVER_H
#define CRYOSMSDRIVER_H

/// EPICS Asyn port driver class. 
class epicsShareClass CRYOSMSDriver : public asynPortDriver
{
public:
	CRYOSMSDriver(const char *portName, std::string devPrefix);
	virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
private:
	std::string devicePrefix;
	double maxCurrent;
	double teslaToAmps;
	double maxVolt;
	std::string writeUnit;
	std::string displayUnit;
	std::string rampFile;
	std::string allowPersist;
	std::string useSwitch;
	std::string switchTempPV;
	double switchHigh;
	double switchLow;
	int switchStableNumber;
	double heaterTolerance;
	double switchTimeout;
	double switchTempTolerance;
	std::string heaterOut;
	std::string useMagnetTemp;
	std::string magnetTempPV;
	double maxMagnetTemp;
	double minMagnetTemp;
	std::string compOffAct;
	int noOfComp;
	int minNoOfCompOn;
	std::string comp1StatPV;
	std::string comp2StatPV;
	double fastRate;
	double fastPersistentSettleTime;
	double persistentSettleTime;
	double filterValue;
	double fastFilterValue;
	double npp;
	double writeToDispConversion;

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
	asynStatus procDb(std::string pvSuffix);
	asynStatus getDb(std::string pvSuffix, void *pbuffer);
	asynStatus putDb(std::string pvSuffix, const void *value);
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
#define P_devicePrefixString "IN:NXHIFI:CRYOSMS_01"
#define P_outputModeSetString "OUTPUTMODE_SET"

#endif /* CRYOSMSDRIVER_H */