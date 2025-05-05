#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), signalChecker(-0.2, 0.2),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("q1");
    g.setName("g");
    signalChecker.setName("signalChecker");

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2");
    g.getOut().getSignal().setName("alpha");

    // Connect signals
    g.getIn().connect(q1.getOut());
    signalChecker.getIn().connect(g.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(q1);
    timedomain.addBlock(g);
    timedomain.addBlock(signalChecker);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}