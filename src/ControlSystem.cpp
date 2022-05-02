#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : myConstant(1.0), myGain(2.0),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    myConstant.setName("My constant");
    myGain.setName("My gain");

    // Name all signals
    myConstant.getOut().getSignal().setName("My constant value");
    myGain.getOut().getSignal().setName("My constant value multiplied with my gain");

    // Connect signals
    myGain.getIn().connect(myConstant.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(myConstant);
    timedomain.addBlock(myGain);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}