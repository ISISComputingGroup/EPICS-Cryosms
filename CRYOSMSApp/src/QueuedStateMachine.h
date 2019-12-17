#ifndef QUEUEDSTATEMACHINE_H
#define QUEUEDSTATEMACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <StateMachineDriver.h>

namespace mpl = boost::mpl;
namespace msm = boost::msm;
struct startRamp {};
struct pauseRamp {};
struct abortRamp {};
struct resumeRamp {};
struct targetReached {};


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
	void startNewRamp(startRamp const&) {}
	void pauseInRamp(pauseRamp const&) {}
	void resumeRampFromPause(resumeRamp const&) {}
	void abortRampFromRamping(abortRamp const&) {}
	void abortRampFromPaused(abortRamp const&) {}
	void reachTarget(targetReached const&) {}
	typedef mpl::vector<ready> initial_state;
	typedef cryosmsStateMachine csm; //declutters transition table

	struct transition_table : mpl::vector<
		//	   Start		Event			Target		Action					Guard
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< ready,		startRamp,		ramping,	&csm::startNewRamp					>,
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< ramping,		pauseRamp,		paused,		&csm::pauseInRamp					>,
		a_row< ramping,		abortRamp,		aborting,	&csm::abortRampFromRamping			>,
		a_row< ramping,		targetReached,	ready,		&csm::reachTarget					>,
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< paused,		resumeRamp,		ramping,	&csm::resumeRampFromPause			>,
		a_row< paused,		abortRamp,		aborting,	&csm::abortRampFromPaused			>,
		//	 +-------------+---------------+-----------+--------------------------+----------+
		a_row< aborting,	targetReached,	ready,		&csm::reachTarget					>
	> {};
	template <class QSM, class Event>
	void no_transition(Event const& e, QSM&, int state)
	{
		std::cout << "no transition from state " << state
			<< " on event " << typeid(e).name() << std::endl;
	}
};
#endif // !QUEUEDSTATEMACHINE_H
