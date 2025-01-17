#ifndef QUEUEDSTATEMACHINE_H
#define QUEUEDSTATEMACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <StateMachineDriver.h>
#include <epicsThread.h>
#include <errlog.h>

namespace mpl = boost::mpl;
namespace msm = boost::msm;

///Events (trigger state transitions when sent to qsm):
struct driverEvent {
	driverEvent(SMDriver* _dvr) : dvr(_dvr) {}
	SMDriver* dvr;
};
struct startRampEvent {
	/*	rate: ramp rate for ramp to start
		target: absolute value to ramp to
		rampDir: direction of ramp
		rampType: whether this ramp is fast/fastZero/normal
	*/
	startRampEvent(SMDriver* _dvr, double _rate, double _target, int _rampDir, RampType _rampType) : dvr(_dvr), rate(_rate), target(_target), rampDir(_rampDir), rampType(_rampType) {}
	SMDriver* dvr;
	double rate;
	double target;
	int rampDir;
	RampType rampType;
};
struct pauseRampEvent : driverEvent {
	pauseRampEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct abortRampEvent : driverEvent {
	abortRampEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct resumeRampEvent : driverEvent {
	resumeRampEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct targetReachedEvent : driverEvent {
	targetReachedEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct startCoolEvent : driverEvent {
	startCoolEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct startWarmEvent : driverEvent {
	startWarmEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct tempReachedEvent : driverEvent {
	tempReachedEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};
struct checkHeaterEvent : driverEvent {
	checkHeaterEvent(SMDriver* _dvr) : driverEvent(_dvr) { }
};

///States (have code that executes on entrance/exit, shown here for proof of concept/debugging):
struct ready : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
	}
};
struct ramping : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
	}
};
struct paused : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
	}
};
struct aborting : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
	}
};
struct cooling : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
	}
};
struct warming : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
	}
};
struct cryosmsStateMachine : public msm::front::state_machine_def<cryosmsStateMachine>
{
	explicit cryosmsStateMachine(SMDriver* drv) : drv_(*drv) {}
	SMDriver& drv_;
	
	///Actions (executed when a state transition occurs, gets passed the event which triggers the transition): 
	//Putting as much of the code as possible in the driver to keep this file as legible as possible
	void startNewRamp(startRampEvent const& evt) {
		drv_.startRamping(evt.rate, evt.target, evt.rampDir, evt.rampType); 
	}
	void pauseInRamp(pauseRampEvent const&) {
		drv_.pauseRamp();
	}
	void resumeRampFromPause(resumeRampEvent const&) {
		drv_.resumeRamp();
	}
	void abortRamp(abortRampEvent const&) {
		drv_.abortRamp();
	}
	void reachTarget(targetReachedEvent const&) {
		drv_.reachTarget();
	}
	void continueAbort(pauseRampEvent const&) {
		drv_.continueAbort();
	}
	void startCool(startCoolEvent const&) {
		drv_.startCooling();
	}
	void startWarm(startWarmEvent const&) {
		drv_.startWarming();
	}
	void tempReached(tempReachedEvent const&) {
		drv_.reachTemp();
	}
	void checkHeater(checkHeaterEvent const&) {
		drv_.preRampHeaterCheck();
	}
	void abortBasic(abortRampEvent const&) {
		//different from abort ramp: doesn't do any field setting, just clears queue
		drv_.abortBasic();
	}
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table

	///Table of valid state transitions:
	struct transition_table : mpl::vector<
		//	   Start		Event				Target		Action					
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< ready,		startRampEvent,		ramping,	&csm::startNewRamp			>,	
		a_row< ready,		resumeRampEvent,	ready,		&csm::resumeRampFromPause	>,
		a_row< ready,		startCoolEvent,		cooling,	&csm::startCool				>,
		a_row< ready,		startWarmEvent,		warming,	&csm::startWarm				>,
		a_row< ready,		checkHeaterEvent,	ready,		&csm::checkHeater			>,
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< ramping,		pauseRampEvent,		paused,		&csm::pauseInRamp			>,
		a_row< ramping,		targetReachedEvent,	ready,		&csm::reachTarget			>,
		a_row< ramping,		abortRampEvent,		aborting,	&csm::abortRamp				>,
		a_row< ramping,		startRampEvent,		ramping,	&csm::startNewRamp			>,
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< paused,		resumeRampEvent,	ramping,	&csm::resumeRampFromPause	>,
		a_row< paused,		abortRampEvent,		aborting,	&csm::abortRamp				>,
		_row<  paused,		pauseRampEvent,		paused									>,
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< aborting,	targetReachedEvent, ready,		&csm::reachTarget			>,
		a_row< aborting,	pauseRampEvent,		aborting,	&csm::continueAbort			>,
		_row<  aborting,	resumeRampEvent,	aborting								>,
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< cooling,		abortRampEvent,		aborting,	&csm::abortBasic			>,
		a_row< cooling,		tempReachedEvent,	ready,		&csm::tempReached			>,
		a_row< warming,		abortRampEvent,		aborting,	&csm::abortBasic			>,
		a_row< warming,		tempReachedEvent,	ready,		&csm::tempReached			>
	> {};
	/// For debugging purposes, prints whenever an invalid state transition is requested
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::string stateList[6] = { "ready", "ramping", "paused", "aborting", "cooling", "warming" };
		errlogSevPrintf(errlogMajor, "no transition from state %s on event %s", stateList[state].c_str(), typeid(e).name());
	}
};
#endif /* !QUEUEDSTATEMACHINE_H */
