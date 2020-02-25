#ifndef STATEMACHINEDRIVER_H
#define STATEMACHINEDRIVER_H

class SMDriver {
	/// class to pass to the queued state machine to avoid circular dependancies
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
	virtual void startRamping() = 0;
	virtual void abortRamp() = 0;
	virtual void reachTarget() = 0;
	virtual void continueAbort() = 0;
	bool queuePaused = false;
	bool atTarget = true;
};

#endif // STATEMACHINEDRIVER_H