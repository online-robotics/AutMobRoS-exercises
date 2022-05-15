#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), servo("servo1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("q1");
    g.setName("g");
    servo.setName("servo");

    // Name all signals
    q1.getOut().getSignal().setName("alpha/2");
    g.getOut().getSignal().setName("alpha");

    // Connect signals
    g.getIn().connect(q1.getOut());
    servo.getIn().connect(g.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(q1);
    timedomain.addBlock(g);
    timedomain.addBlock(servo);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}