#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E2("enc2"), cont(21.2/2.0/M_PI), qdMax(21.2), i(3441/104), kM(8.44e-3), M1("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E2.setName("E2");
    cont.setName("cont");
    qdMax.setName("qdMax");
    i.setName("i");
    kM.setName("kM");
    M1.setName("M1");

    // Name all signals
    E2.getOut().getSignal().setName("q2[rad]");
    cont.getOut().getSignal().setName("qd1[rad/s]");
    qdMax.getOut().getSignal().setName("qd1[rad/s]");
    i.getOut().getSignal().setName("om1[rad/s]");
    kM.getOut().getSignal().setName("U[V]");

    // Connect signals
    cont.getIn().connect(E2.getOut());
    qdMax.getIn().connect(cont.getOut());
    i.getIn().connect(qdMax.getOut());
    kM.getIn().connect(i.getOut());
    M1.getIn().connect(kM.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E2);
    timedomain.addBlock(cont);
    timedomain.addBlock(qdMax);
    timedomain.addBlock(i);
    timedomain.addBlock(kM);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}