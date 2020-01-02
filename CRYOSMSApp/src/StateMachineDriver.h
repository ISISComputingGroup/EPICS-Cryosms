#ifndef STATEMACHINEDRIVER_H
#define STATEMACHINEDRIVER_H

class SMDriver {
public:
	virtual ~SMDriver() = default;
	virtual void startRamp() = 0;
	virtual void pauseRamp() = 0;
	virtual void resumeRamp() = 0;
	virtual void coolSwitchLogic() = 0;
	virtual void warmSwitchLogic() = 0;
	virtual void rampFastLogic() = 0;
	virtual void rampZeroLogic() = 0;
	virtual void rampPersistLogic() = 0;
	virtual void waitFastPersist() = 0;
	virtual void waitNonPersist() = 0;
};

#endif // STATEMACHINEDRIVER_H