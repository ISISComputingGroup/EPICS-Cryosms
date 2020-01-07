#ifndef QUEUEDSTATEMACHINE_H
#define QUEUEDSTATEMACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <StateMachineDriver.h>
#include <epicsThread.h>

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
struct wrong : public msm::front::state<>
{
	template <class Event, class QSM>
	void on_entry(Event const&, QSM&)
	{
		std::cout << "something is wrong" << std::endl;
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
	void startNewRamp(startRampEvent const&) {
		drv_->atTarget = false;
	}
	void pauseInRamp(pauseRampEvent const&) {
		epicsThreadSuspendSelf();
	}
	void resumeRampFromPause(resumeRampEvent const&) {
		drv_->resumeRamp();
	}
	void abortRampFromRamping(abortRampEvent const&) {
		drv_->abortRamp();
	}
	void abortRampFromPaused(abortRampEvent const&) {}
	void reachTarget(targetReachedEvent const&) {}
	void somethingwrong(driverEvent const&) {}
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table

	struct transition_table : mpl::vector<
		//	   Start		Event			Target		Action					Guard
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< ready,		startRampEvent,		ramping,	&csm::startNewRamp					>,
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< ramping,		pauseRampEvent,		paused,		&csm::pauseInRamp					>,
		a_row< ramping,		abortRampEvent,		aborting,	&csm::abortRampFromRamping			>,
		a_row< ramping,		targetReachedEvent,	ready,		&csm::reachTarget					>,
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< paused,		resumeRampEvent,		ramping,	&csm::resumeRampFromPause			>,
		a_row< paused,		abortRampEvent,		aborting,	&csm::abortRampFromPaused			>,
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< aborting,	targetReachedEvent,	ready,		&csm::reachTarget					>
	> {};
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::cout << "no transition from state " << state
			<< " on event " << typeid(e).name() << std::endl;
	}
};
#endif // !QUEUEDSTATEMACHINE_H
