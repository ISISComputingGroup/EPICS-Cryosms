#ifndef QUEUEDSTATEMACHINE_H
#define QUEUEDSTATEMACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <StateMachineDriver.h>
#include <epicsThread.h>

namespace mpl = boost::mpl;
namespace msm = boost::msm;

///Events (trigger state transitions when sent to qsm):
struct driverEvent {
	driverEvent(SMDriver* dvr) : dvr(dvr) {}
	SMDriver* dvr;
};
struct startRampEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct pauseRampEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct abortRampEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct resumeRampEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct targetReachedEvent : driverEvent {
	using driverEvent::driverEvent;
};

///States (have code that executes on entrance/exit, shown here for proof of concept/debugging):
struct ready : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering ready" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving ready" << std::endl;
	}
};
struct ramping : public msm::front::state<>
{
};
struct paused : public msm::front::state<>
{
};
struct cryosmsStateMachine : public msm::front::state_machine_def<cryosmsStateMachine>
{
	explicit cryosmsStateMachine(SMDriver* drv) : drv_(*drv) {}
	SMDriver& drv_;
	
	///Actions (executed when a state transition occurs, passed the event which triggers the transition): 
	void startNewRamp(startRampEvent const&) {
		drv_.startRamping();
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
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table

	///Table of valid state transitions:
	struct transition_table : mpl::vector<
		//	   Start		Event				Target		Action					
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< ready,		startRampEvent,		ramping,	&csm::startNewRamp			>,
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< ramping,		pauseRampEvent,		paused,		&csm::pauseInRamp			>,
		a_row< ramping,		abortRampEvent,		ready,		&csm::abortRamp				>,
		a_row< ramping,		targetReachedEvent,	ready,		&csm::reachTarget			>,
		//	 +-------------+-------------------+-----------+-----------------------------+
		a_row< paused,		resumeRampEvent,	ramping,	&csm::resumeRampFromPause	>,
		a_row< paused,		abortRampEvent,		ready,		&csm::abortRamp				>
		//	 +-------------+-------------------+-----------+-----------------------------+
	> {};
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::cout << "no transition from state " << state
			<< " on event " << typeid(e).name() << std::endl;
	}
};
#endif // !QUEUEDSTATEMACHINE_H
