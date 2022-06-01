#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      fwKinOdom(0.15),
      RvRx(1.0),
      omegaR(0.0),
      invKin(0.15),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    E.setName("E");
    Ed.setName("Ed");
    fwKinOdom.setName("fwKinOdom");
    invKin.setName("invKin");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [m]");
    E2.getOut().getSignal().setName("q2 [m]");
    E.getOut().getSignal().setName("q [m]");
    E.getOut().getSignal().setName("qd [m/s]");

    // Connect signals
    E.getIn(0).connect(E1.getOut());
    E.getIn(1).connect(E2.getOut());
    Ed.getIn().connect(E.getOut());
    fwKinOdom.getIn().connect(Ed.getOut());
    invKin.getInRvRx().connect(RvRx.getOut());
    invKin.getInOmegaR().connect(omegaR.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(E);
    timedomain.addBlock(Ed);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(RvRx);
    timedomain.addBlock(omegaR);
    timedomain.addBlock(invKin);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}