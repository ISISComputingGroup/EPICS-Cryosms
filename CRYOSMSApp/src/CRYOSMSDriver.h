#ifndef CRYOSMSDRIVER_H
#define CRYOSMSDRIVER_H

#include <asynPortDriver.h>
#include <QueuedStateMachine.h>
#include <StateMachineDriver.h>
#include <boost/msm/back/state_machine.hpp>
#include <boost/variant/variant.hpp>

typedef boost::variant<startRampEvent, abortRampEvent, pauseRampEvent, resumeRampEvent, targetReachedEvent, startCoolEvent, startWarmEvent, tempReachedEvent, checkHeaterEvent> eventVariant;

struct processEventVisitor : boost::static_visitor<>
{
	processEventVisitor(boost::msm::back::state_machine<cryosmsStateMachine>& qsm, std::deque<eventVariant>& eventQueue) : _qsm(qsm), _eventQueue(eventQueue) {}
	boost::msm::back::state_machine<cryosmsStateMachine>& _qsm;
	std::deque<eventVariant>& _eventQueue;

	template <typename qsmEventType>
	void operator()(qsmEventType const& qsmEvent) const
	{
		_eventQueue.pop_front();
		_qsm.process_event(qsmEvent);
	}
};
/// EPICS Asyn port driver class. 
class CRYOSMSDriver : public asynPortDriver, public SMDriver
{
public:
	CRYOSMSDriver(const char *portName, std::string devPrefix, std::map<std::string, std::string> argMap);
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
	asynStatus setupRamp();
	asynStatus setupPersistOn();
	asynStatus setupFastRamp(double target);
	std::map<std::string,std::string> envVarMap;
	double writeToDispConversion;
	double unitConversion(double value, std::string startUnit, std::string endUnit);
	bool writeDisabled;
	int testVar; //for use in google tests where functionality can not be tested with PV values
	bool started;
	bool fastRamp; //whether or not device is in "fast" mode, used exclusively to update "STAT" PV correctly
	bool fastRampZero; //whether or not device is in "fast zero" mode, used exclusively to update "STAT" PV correctly
	bool cooling;//whether heater is cooling down
	bool warming;//whether heater is warming up
	int trueVal = 1; //Used in dbputs, as it needs to be passed ref to int
	int falseVal = 0;//Used in dbputs, as it needs to be passed ref to int
	asynStatus procDb(std::string pvSuffix);
	asynStatus getDb(std::string pvSuffix, int &pbuffer);
	asynStatus getDb(std::string pvSuffix, double &pbuffer);
	asynStatus getDb(std::string pvSuffix, std::string &pbuffer);
	asynStatus putDb(std::string pvSuffix, const void *value);
	asynStatus putDbAndWait(const std::string& pvSuffix, const void *value, double timeout);
	std::deque<eventVariant> eventQueue;
	epicsThreadId queueThreadId;
	bool atTarget;
	bool abortQueue;
	void checkForTarget();
	void checkIfPaused();
	void checkHeaterDone();
	void resumeRamp() override;
	void pauseRamp() override;
	void startRamping(double rate, double target, int rampDir, RampType rampType) override;
	void abortRamp() override;
	void reachTarget() override;
	void continueAbort() override;
	void abortBasic() override;
	void startCooling() override;
	void startWarming() override;
	void reachTemp() override;
	void preRampHeaterCheck() override;
	boost::msm::back::state_machine<cryosmsStateMachine> qsm;
private:
	std::string devicePrefix;
	int P_deviceName; // string
	int P_initLogic;
	int P_Rate; //float
	int P_MaxT; //float
	int P_startRamp;
	int P_pauseRamp;
	int P_abortRamp;
	int P_outputModeSet;
	int P_calcHeater; //int as above


	std::vector<double> pRate_;
	std::vector<double> pMaxT_;
	asynStatus onStart();
	asynStatus readFile(std::string dir);
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
#define P_calcHeaterString "CALC_HEATER"

#endif /* CRYOSMSDRIVER_H */
