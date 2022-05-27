#ifndef MOVESERVOTO_HPP_
#define MOVESERVOTO_HPP_

#include <eeros/sequencer/Step.hpp>
#include "ControlSystem.hpp"

class MoveServoTo : public eeros::sequencer::Step
{
public:
    MoveServoTo(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs)
        : cs(cs), eeros::sequencer::Step(name, caller)
    {
        log.info() << "Step created: " << name;
    }

    int operator() (double c)
    {
        this->c = c;
        return start();
    }

    int action()
    {
        log.info() << "Moving to " << c << " rad.";
        cs.c.setValue(c);
        return 0;
    }

private:
    ControlSystem &cs;
    double c;
};

#endif // MOVESERVOTO_HPP_
