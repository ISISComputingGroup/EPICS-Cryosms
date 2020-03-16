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
		target: absolute value to ramp to, direction decided elsewhere
	*/
	startRampEvent(SMDriver* _dvr, double _rate, double _target, int _rampDir) : dvr(_dvr), rate(_rate), target(_target), rampDir(_rampDir) {}
	startRampEvent(SMDriver* _dvr, double _rate, double _target) : dvr(_dvr), rate(_rate), target(_target), rampDir(0) {}
	SMDriver* dvr;
	double rate;
	double target;
	int rampDir;
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
struct cryosmsStateMachine : public msm::front::state_machine_def<cryosmsStateMachine>
{
	explicit cryosmsStateMachine(SMDriver* drv) : drv_(*drv) {}
	SMDriver& drv_;
	
	///Actions (executed when a state transition occurs, gets passed the event which triggers the transition): 
	//Putting as much of the code as possible in the driver to keep this file as legible as possible
	void startNewRamp(startRampEvent const& evt) {
		drv_.startRamping(evt.rate, evt.target, evt.rampDir); 
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
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table

	///Table of valid state transitions:
	struct transition_table : mpl::vector<
		//	   Start		Event				Target		Action					
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< ready,		startRampEvent,		ramping,	&csm::startNewRamp			>,	
		a_row< ready,		resumeRampEvent,	ready,		&csm::resumeRampFromPause	>,
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
		_row<  aborting,	resumeRampEvent,	aborting								>
	> {};
	/// For debugging purposes, prints whenever an invalid state transition is requested
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::string stateList[4] = { "ready", "ramping", "paused", "aborting" };
		errlogSevPrintf(errlogMajor, "no transition from state %s on event %s", stateList[state].c_str(), typeid(e).name());
	}
};
#endif !QUEUEDSTATEMACHINE_H
