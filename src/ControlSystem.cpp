#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), c(0.0), servo("servo1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("q1");
    g.setName("g");
    c.setName("c");
    servo.setName("servo");

    // Name all signals
    q1.getOut().getSignal().setName("beta/half [rad]");
    g.getOut().getSignal().setName("beta [rad]");
    c.getOut().getSignal().setName("Servo setpoint [rad]");

    // Connect signals
    g.getIn().connect(q1.getOut());
    servo.getIn().connect(c.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(q1);
    timedomain.addBlock(g);
    timedomain.addBlock(c);
    timedomain.addBlock(servo);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}