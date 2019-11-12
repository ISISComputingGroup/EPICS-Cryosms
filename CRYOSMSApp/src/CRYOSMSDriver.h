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

#define FIRST_SMS_PARAM P_deviceName

	int P_deviceName; // string
	int P_outputModeSet; //int as above

#define LAST_SMS_PARAM 	P_outputModeSet
#define NUM_SMS_PARAMS	(&LAST_SMS_PARAM - &FIRST_SMS_PARAM + 1)

	asynStatus putDb(std::string pvSuffix, const void *value);
	static void pollerTaskC(void* arg)
	{
		CRYOSMSDriver* driver = static_cast<CRYOSMSDriver*>(arg);
		driver->pollerTask();
	}
	void pollerTask();
};

#define P_deviceNameString "DEVICE"
#define P_outputModeSetString "OUTPUTMODE_SET"

#endif /* CRYOSMSDRIVER_H */
