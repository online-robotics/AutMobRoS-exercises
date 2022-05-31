#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      cont(1.0/dt, 0.7, 2.3, 3441.0 / 104.0 * 3441.0 / 104.0 * 6.8e-8),
      invMotMod(0.1, 21.2, 3441.0/104.0, 8.44e-3, 8.0),
      M1("motor1"),
      M2("motor2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    E_d.setName("E_d");
    E.setName("E");
    cont.setName("cont");
    invMotMod.setName("invMotMod");
    U.setName("U");
    M1.setName("M1");
    M2.setName("M2");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [rad]");
    E2.getOut().getSignal().setName("q2 [rad]");
    E_d.getOut().getSignal().setName("q_d [rad]");
    E.getOut().getSignal().setName("q [rad]");
    U.getOut(0).getSignal().setName("U1 [V]");
    U.getOut(1).getSignal().setName("U2 [V]");

    // Connect signals
    E_d.getIn(0).connect(E2.getOut());
    E_d.getIn(1).connect(E1.getOut());
    E.getIn(0).connect(E1.getOut());
    E.getIn(1).connect(E2.getOut());
    cont.getIn(0).connect(E_d.getOut());
    cont.getIn(1).connect(E.getOut());
    invMotMod.getIn(0).connect(cont.getOut(0));
    invMotMod.getIn(1).connect(cont.getOut(1));
    U.getIn().connect(invMotMod.getOut());
    M1.getIn().connect(U.getOut(0));
    M2.getIn().connect(U.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(E_d);
    timedomain.addBlock(E);
    timedomain.addBlock(cont);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(U);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}