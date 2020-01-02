#ifndef QUEUEDSTATEMACHINE_H
#define QUEUEDSTATEMACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <StateMachineDriver.h>

namespace mpl = boost::mpl;
namespace msm = boost::msm;

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
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering ramping" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving ramping" << std::endl;
	}
};
struct rampingZero : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering ramping 0" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving ramping 0" << std::endl;
		int falseVal = 0;
		Event.dvr->putDbNoReturn("FAST:ZERO", &falseVal);
	}
};
struct rampingPersist : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering ramping persist" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving ramping persist" << std::endl;
		Event.dvr->waitFastPersist();
	}
};
struct paused : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering paused" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving paused" << std::endl;
	}
};
struct aborting : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering aborting" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving aborting" << std::endl;
	}
};
struct coolingSwitch : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering cooling switch" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving cooling switch" << std::endl;
	}
};
struct warmingSwitch : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering warming switch" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving warming switch" << std::endl;
	}
};
struct rampFastStartup : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "Performing ramp fast setup" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "Ramp Fast setup done" << std::endl;
	}
};
struct cryosmsStateMachine : public msm::front::state_machine_def<cryosmsStateMachine>
{
	explicit cryosmsStateMachine(SMDriver* drv) : drv_(drv) {}
	SMDriver* drv_;
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "entering QSM" << std::endl;
	}
	template <class Event, class QSM>
	void on_exit(Event const&, QSM&)
	{
		std::cout << "leaving QSM" << std::endl;
	}
	void startNewRamp(startRampEvent const& evt) {
		drv_->startRamp();
	}
	void startNewRampZero(startRampZeroEvent const& evt) {
		drv_->rampZeroLogic();
	}
	void startPersistRamp(rampPersistEvent const&) {
		drv_->rampPersistLogic();
	}
	void startFastRamp(rampFastEvent const&) {
		drv_->rampFastLogic();
	}
	void pauseInRamp(pauseRampEvent const&) {}
	void resumeRampFromPause(resumeRampEvent const&) {}
	void abortRampFromRamping(abortRampEvent const&) {}
	void abortRampFromPaused(abortRampEvent const&) {}
	void reachTarget(targetReachedEvent const&) {}
	void goodSwitchTemp(goodSwitchTempEvent const&) {}
	void coolSwitch(coolSwitchEvent const&) {
		drv_->coolSwitchLogic();
	}
	void warmSwitch(warmSwitchEvent const&) {
		drv_->warmSwitchLogic();
	}
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table

	struct transition_table : mpl::vector<
		//	   Start			Event					Target			Action					
		a_row< ready,			startRampEvent,			ramping,		&csm::startNewRamp			>,
		a_row< ready,			coolSwitchEvent,		coolingSwitch,	&csm::coolSwitch			>,
		a_row< ready,			warmSwitchEvent,		warmingSwitch,	&csm::warmSwitch			>,
		a_row< ready,			rampFastEvent,			rampFastStartup,&csm::startFastRamp			>,
		a_row< rampFastStartup,	startRampZeroEvent,		rampingZero,	&csm::startNewRampZero		>,
		a_row< rampFastStartup,	rampPersistEvent,		rampingPersist,	&csm::startPersistRamp		>,
		a_row< ramping,			pauseRampEvent,			paused,			&csm::pauseInRamp			>,
		a_row< ramping,			abortRampEvent,			aborting,		&csm::abortRampFromRamping	>,
		a_row< ramping,			targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< rampingZero,		targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< rampingZero,		pauseRampEvent,			paused,			&csm::pauseInRamp			>,
		a_row< rampingZero,		abortRampEvent,			aborting,		&csm::abortRampFromRamping	>,
		a_row< rampingPersist,	targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< rampingPersist,	pauseRampEvent,			paused,			&csm::pauseInRamp			>,
		a_row< rampingPersist,	abortRampEvent,			aborting,		&csm::abortRampFromRamping	>,
		a_row< paused,			resumeRampEvent,		ramping,		&csm::resumeRampFromPause	>,
		a_row< paused,			abortRampEvent,			aborting,		&csm::abortRampFromPaused	>,
		a_row< aborting,		targetReachedEvent,		ready,			&csm::reachTarget			>,
		a_row< coolingSwitch,	goodSwitchTempEvent,	ready,			&csm::goodSwitchTemp		>
	> {};
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::cout << "no transition from state " << state
			<< " on event " << typeid(e).name() << std::endl;
	}
};
#endif // !QUEUEDSTATEMACHINE_H
