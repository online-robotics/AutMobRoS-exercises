#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "AutMobRoSSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>
#include "customSteps/moveServoTo.hpp"
#include "customSequences/orientationException.hpp"
#include <eeros/sequencer/Monitor.hpp>

class MainSequence : public eeros::sequencer::Sequence
{
public:
    MainSequence(std::string name, eeros::sequencer::Sequencer &seq,
                 eeros::safety::SafetySystem &ss,
                 AutMobRoSSafetyProperties &sp, ControlSystem &cs)
        : eeros::sequencer::Sequence(name, seq),
          ss(ss),
          sp(sp),
          cs(cs),

          sleep("Sleep", this),
          moveServoTo("moveServoTo", this, cs),

          checkOrientation(0.1, cs),
          orientationException("Orientation exception", this, cs, checkOrientation),
          orientationMonitor("Orientation monitor", this, checkOrientation, eeros::sequencer::SequenceProp::resume, &orientationException)
    {
        addMonitor(&orientationMonitor);
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running)
        {
            moveServoTo(-0.5);
            sleep(1.0);
            moveServoTo(0.5);
            sleep(1.0);
        }
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    AutMobRoSSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
    MoveServoTo moveServoTo;
    CheckOrientation checkOrientation;
    OrientationException orientationException;
    eeros::sequencer::Monitor orientationMonitor;
};

#endif // MAINSEQUENCE_HPP_