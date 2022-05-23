#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E2("enc2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E2.setName("E2");

    // Name all signals
    E2.getOut().getSignal().setName("q2[rad]");

    // Connect signals

    // Add blocks to timedomain
    timedomain.addBlock(E2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}