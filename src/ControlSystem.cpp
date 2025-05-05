#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : q1("quat1"), g(2.0), motorVoltageSetpoint(0.0), motor("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    q1.setName("q1");
    g.setName("g");
    motorVoltageSetpoint.setName("motorVoltageSetpoint");
    motor.setName("motor");

    // Name all signals
    q1.getOut().getSignal().setName("beta/half [rad]");
    g.getOut().getSignal().setName("beta [rad]");
    motorVoltageSetpoint.getOut().getSignal().setName("Motor voltage setpoint [V]");

    // Connect signals
    g.getIn().connect(q1.getOut());
    motor.getIn().connect(motorVoltageSetpoint.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(q1);
    timedomain.addBlock(g);
    timedomain.addBlock(motorVoltageSetpoint);
    timedomain.addBlock(motor);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}