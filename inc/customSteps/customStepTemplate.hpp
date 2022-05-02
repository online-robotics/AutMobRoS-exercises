#ifndef CUSTOMSTEPTEMPLATE_HPP_
#define CUSTOMSTEPTEMPLATE_HPP_

#include <eeros/sequencer/Step.hpp>

class CustomStepName : public eeros::sequencer::Step
{
public:
    CustomStepName(std::string name, eeros::sequencer::Sequence *caller)
        : eeros::sequencer::Step(name, caller)
    {
        log.info() << "Step created: " << name;
    }

    int action()
    {
        // do something
        return 0;
    }

private:
    // Define variables, conditions, monitors, exception sequences, ...
};

#endif // CUSTOMSTEPTEMPLATE_HPP_
