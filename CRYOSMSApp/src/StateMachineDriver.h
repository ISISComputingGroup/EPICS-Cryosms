#ifndef STATEMACHINEDRIVER_H
#define STATEMACHINEDRIVER_H

class SMDriver {
	/// class to pass to the queued state machine to avoid circular dependancies
public:
	virtual ~SMDriver() { }
	virtual void resumeRamp() = 0;
	virtual void pauseRamp() = 0;
	virtual void startRamping(double rate, double target, int rampDir) = 0;
	virtual void abortRamp() = 0;
	virtual void reachTarget() = 0;
	virtual void continueAbort() = 0;
	bool queuePaused;
	bool atTarget;
    SMDriver() : queuePaused(false), atTarget(true) { }
};

#endif // STATEMACHINEDRIVER_H
