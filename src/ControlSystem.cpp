#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : c(0.0), servo("servo1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    c.setName("c");
    servo.setName("servo");

    // Name all signals
    c.getOut().getSignal().setName("Servo setpoint [rad]");

    // Connect signals
    servo.getIn().connect(c.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(c);
    timedomain.addBlock(servo);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}