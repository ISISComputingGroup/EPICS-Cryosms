#ifndef CRYOSMSDRIVER_H
#define CRYOSMSDRIVER_H

#include <asynPortDriver.h>
#include <QueuedStateMachine.h>
#include <StateMachineDriver.h>
#include <boost/msm/back/state_machine.hpp>

/// EPICS Asyn port driver class. 
class CRYOSMSDriver : public asynPortDriver, public SMDriver
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
	std::deque<void*> eventQueue;
	epicsThreadId queueThreadId;
	bool atTarget = true;
	bool abortQueue = true;
	void checkForTarget();
	int simulatedRampIncrement;
	boost::msm::back::state_machine<cryosmsStateMachine> qsm;
	void resumeRamp() override;
	void abortRamp() override;
	startRampEvent startRampEv = startRampEvent{ this };
	abortRampEvent abortRampEv = abortRampEvent{ this };
	pauseRampEvent pauseRampEv = pauseRampEvent{ this };
	resumeRampEvent resumeRampEv = resumeRampEvent{ this };
	targetReachedEvent targetReachedEv = targetReachedEvent{ this };
private:
	std::string devicePrefix;

#define FIRST_SMS_PARAM P_deviceName

	int P_deviceName; // string
	int P_initLogic;
	int P_Rate; //float
	int P_MaxT; //float
	int P_startRamp;
	int P_pauseRamp;
	int P_abortRamp;
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
#define P_startRampString "RAMP_START"
#define P_pauseRampString "RAMP_PAUSE"
#define P_abortRampString "RAMP_ABORT"
#define P_outputModeSetString "OUTPUTMODE_SET"

#endif /* CRYOSMSDRIVER_H */
