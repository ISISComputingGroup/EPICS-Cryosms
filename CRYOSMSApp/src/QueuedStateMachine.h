#ifndef QUEUEDSTATEMACHINE_H
#define QUEUEDSTATEMACHINE_H

//allow transition tables of more than 20 transitions:
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50                
#define BOOST_MPL_LIMIT_MAP_SIZE 50  

#include <boost/msm/front/state_machine_def.hpp>
#include <StateMachineDriver.h>
#include <epicsThread.h>
#include <errlog.h>
#include <boost/mpl/vector/vector20.hpp>

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
struct warmSwitchEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct coolSwitchEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct goodSwitchTempEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct startRampZeroEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct rampPersistEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct rampFastEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct rampMidEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct rampMidZeroEvent : driverEvent {
	using driverEvent::driverEvent;
};
struct reachMidEvent : driverEvent {
	using driverEvent::driverEvent;
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
struct starting : public msm::front::state<> {};
struct rampingZero : public msm::front::state<> {};
struct rampingPersist : public msm::front::state<> {};
struct aborting : public msm::front::state<> {};
struct coolingSwitch : public msm::front::state<> {};
struct warmingSwitch : public msm::front::state<> {};
struct rampFastStartup : public msm::front::state<> {};

struct cryosmsStateMachine : public msm::front::state_machine_def<cryosmsStateMachine>
{
	explicit cryosmsStateMachine(SMDriver* drv) : drv_(*drv) {}
	SMDriver& drv_;

	///Actions (executed when a state transition occurs, gets passed the event which triggers the transition): 
	void startNewRamp(startRampEvent const&) {
		drv_.startRamping(); //Putting as much of the ode as possible in the driver to keep this file as legible as possible
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
	void startNewRampZero(startRampZeroEvent const& evt) {
		drv_.rampZeroLogic();
	}
	void startPersistRamp(rampPersistEvent const&) {
		drv_.rampPersistLogic();
	}
	void startFastRamp(rampFastEvent const&) {
		drv_.rampFastLogic();
	}
	void abortRampFromRamping(abortRampEvent const&) {}
	void abortRampFromPaused(abortRampEvent const&) {}
	void goodSwitchTemp(goodSwitchTempEvent const&) {}
	void coolSwitch(coolSwitchEvent const&) {
		drv_.coolSwitchLogic();
	}
	void warmSwitch(warmSwitchEvent const&) {
		drv_.warmSwitchLogic();
	}
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table
	///Table of valid state transitions:
	struct transition_table : mpl::vector15 <
		//	   Start			Event					Target			Action					
		a_row< ready,			startRampEvent,			starting,		&csm::startNewRamp			>,
		a_row< starting,		coolSwitchEvent,		coolingSwitch,	&csm::coolSwitch			>,
		a_row< starting,		warmSwitchEvent,		warmingSwitch,	&csm::warmSwitch			>,
		a_row< starting,		rampFastEvent,			starting,		&csm::startFastRamp			>,
		a_row< starting,		startRampZeroEvent,		ramping,		&csm::startNewRampZero		>,
		a_row< starting,		rampPersistEvent,		ramping,		&csm::startPersistRamp		>,
		a_row< ramping,			pauseRampEvent,			paused,			&csm::pauseInRamp			>,
		a_row< ramping,			abortRampEvent,			aborting,		&csm::abortRampFromRamping	>,
		a_row< ramping,			targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< paused,			resumeRampEvent,		ramping,		&csm::resumeRampFromPause	>,
		a_row< paused,			abortRampEvent,			aborting,		&csm::abortRampFromPaused	>,
		a_row< aborting,		targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< coolingSwitch,	goodSwitchTempEvent,	ready,			&csm::goodSwitchTemp		>,
		a_row< aborting,		targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< aborting,		pauseRampEvent,			aborting,		&csm::continueAbort			>
	> {};
	/// For debugging purposes, prints whenever an invalid state transition is requested
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::string stateList[4] = { "ready", "ramping", "paused", "aborting" };
		errlogSevPrintf(errlogMajor, "no transition from state %s on event %s", stateList[state].c_str(), typeid(e).name());
	}
};
#endif // !QUEUEDSTATEMACHINE_H
