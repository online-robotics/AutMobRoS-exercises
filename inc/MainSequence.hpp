#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "AutMobRoSSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>
#include <assert.h>

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

          sleep("Sleep", this)
    {
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running)
        {
            log.info() << "Setting RvRx = 1.0, omegaR = 0.0";
            log.info() << "Expecting wheel velocities to be vWl = 1.0, vWr = 1.0";
            cs.RvRx.setValue(1.0);
            cs.omegaR.setValue(0.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            // Note: you could also use assert, so you do not have to compare the values urself
            eeros::math::Vector2 v = {1.0,1.0};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            log.info() << "Setting RvRx = -1.0, omegaR = 0.0";
            log.info() << "Expecting wheel velocities to be vWl = -1.0, vWr = -1.0";
            cs.RvRx.setValue(-1.0);
            cs.omegaR.setValue(0.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            v = {-1.0, -1.0};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            log.info() << "Setting RvRx = 2.0, omegaR = 0.0";
            log.info() << "Expecting wheel velocities to be vWl = 2.0, vWr = 2.0";
            cs.RvRx.setValue(2.0);
            cs.omegaR.setValue(0.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            v = {2.0, 2.0};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            log.info() << "Setting RvRx = 0.0, omegaR = 1.0";
            log.info() << "Expecting wheel velocities to be vWl = -0.075, vWr = 0.075";
            cs.RvRx.setValue(0.0);
            cs.omegaR.setValue(1.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            v = {-0.075, 0.075};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            log.info() << "Setting RvRx = 0.0, omegaR = -1.0";
            log.info() << "Expecting wheel velocities to be vWl = 0.075, vWr = -0.075";
            cs.RvRx.setValue(0.0);
            cs.omegaR.setValue(-1.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            v = {0.075, -0.075};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            log.info() << "Setting RvRx = 0.0, omegaR = 2.0";
            log.info() << "Expecting wheel velocities to be vWl = -0.15, vWr = 0.15";
            cs.RvRx.setValue(0.0);
            cs.omegaR.setValue(2.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            v = {-0.15, 0.15};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            log.info() << "Setting RvRx = 1.0, omegaR = 1.0";
            log.info() << "Expecting wheel velocities to be vWl = 0.925, vWr = 1.075";
            cs.RvRx.setValue(1.0);
            cs.omegaR.setValue(1.0);
            sleep(1.0);
            log.info() << cs.invKin.getOut().getSignal();
            v = {0.925, 1.075};
            assert(cs.invKin.getOut().getSignal().getValue() == v);

            ss.triggerEvent(sp.abort);
            break;
        }
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    AutMobRoSSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
};

#endif // MAINSEQUENCE_HPP_