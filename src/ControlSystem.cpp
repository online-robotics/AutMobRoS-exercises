#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E2("enc2"), cont(0.03/2.0/M_PI), QMax(0.1), iInv(104.0/3441.0), kMInv(1.0/8.44e-3), R(8.0), M1("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E2.setName("E2");
    cont.setName("cont");
    QMax.setName("QMax");
    iInv.setName("iInv");
    kMInv.setName("kMInv");
    R.setName("R");
    M1.setName("M1");

    // Name all signals
    E2.getOut().getSignal().setName("q2[rad]");
    cont.getOut().getSignal().setName("Q1[Nm]");
    QMax.getOut().getSignal().setName("Q1[Nm]");
    iInv.getOut().getSignal().setName("T1[Nm]");
    kMInv.getOut().getSignal().setName("I1[A]");
    R.getOut().getSignal().setName("U1[V]");

    // Connect signals
    cont.getIn().connect(E2.getOut());
    QMax.getIn().connect(cont.getOut());
    iInv.getIn().connect(QMax.getOut());
    kMInv.getIn().connect(iInv.getOut());
    R.getIn().connect(kMInv.getOut());
    M1.getIn().connect(R.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E2);
    timedomain.addBlock(cont);
    timedomain.addBlock(QMax);
    timedomain.addBlock(iInv);
    timedomain.addBlock(kMInv);
    timedomain.addBlock(R);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}