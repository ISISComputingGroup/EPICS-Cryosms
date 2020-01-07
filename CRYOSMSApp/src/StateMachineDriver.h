#ifndef STATEMACHINEDRIVER_H
#define STATEMACHINEDRIVER_H

class SMDriver {
public:
	virtual ~SMDriver() = default;
	virtual void resumeRamp() = 0;
	virtual void abortRamp() = 0;
	bool queuePaused = false;
	bool atTarget = true;
};

#endif // STATEMACHINEDRIVER_H