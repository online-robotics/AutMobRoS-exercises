#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      Kp(1.0 / dt / 4.6 / 0.7 * 1.0 / dt / 4.6 / 0.7),
      Kd(2.0 * 0.7 / dt / 4.6 / 0.7),
      M(3441.0 / 104.0 * 3441.0 / 104.0 * 6.8e-8),
      QMax(0.1),
      iInv(104.0 / 3441.0),
      kMInv(1.0 / 8.44e-3),
      R(8.0),
      qdMax(21.2),
      i(3441.0/104.0),
      kM(8.44e-3),
      M1("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    e.setName("e");
    Kp.setName("Kp");
    ed.setName("ed");
    Kd.setName("Kd");
    qdd_c.setName("qdd_c");
    M.setName("M");
    QMax.setName("QMax");
    iInv.setName("iInv");
    kMInv.setName("kMInv");
    R.setName("R");
    qd1.setName("qd1");
    qdMax.setName("qdMax");
    i.setName("i");
    kM.setName("kM");
    U1.setName("U1");
    M1.setName("M1");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [rad]");
    E2.getOut().getSignal().setName("q2 [rad]");
    e.getOut().getSignal().setName("e [rad]");
    Kp.getOut().getSignal().setName("qdd_cp [rad/s^2]");
    ed.getOut().getSignal().setName("ed [rad/s]");
    Kd.getOut().getSignal().setName("qdd_cd [rad/s^2]");
    qdd_c.getOut().getSignal().setName("qdd_c [rad/s^2]");
    M.getOut().getSignal().setName("Q1 [Nm]");
    QMax.getOut().getSignal().setName("Q1 [Nm]");
    iInv.getOut().getSignal().setName("T1 [Nm]");
    kMInv.getOut().getSignal().setName("I1 [A]");
    R.getOut().getSignal().setName("UR [V]");
    qd1.getOut().getSignal().setName("qd1 [rad/s]");
    qdMax.getOut().getSignal().setName("qd1 [rad/s]");
    i.getOut().getSignal().setName("om1 [rad/s]");
    kM.getOut().getSignal().setName("Uom [V]");
    U1.getOut().getSignal().setName("U1 [V]");

    // Connect signals
    e.getIn(0).connect(E2.getOut());
    e.getIn(1).connect(E1.getOut());
    e.negateInput(1);
    Kp.getIn().connect(e.getOut());
    ed.getIn().connect(e.getOut());
    Kd.getIn().connect(ed.getOut());
    qdd_c.getIn(0).connect(Kp.getOut());
    qdd_c.getIn(1).connect(Kd.getOut());
    M.getIn().connect(qdd_c.getOut());
    QMax.getIn().connect(M.getOut());
    iInv.getIn().connect(QMax.getOut());
    kMInv.getIn().connect(iInv.getOut());
    R.getIn().connect(kMInv.getOut());
    qd1.getIn().connect(E1.getOut());
    qdMax.getIn().connect(qd1.getOut());
    i.getIn().connect(qdMax.getOut());
    kM.getIn().connect(i.getOut());
    U1.getIn(0).connect(R.getOut());
    U1.getIn(1).connect(kM.getOut());
    M1.getIn().connect(U1.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(e);
    timedomain.addBlock(Kp);
    timedomain.addBlock(ed);
    timedomain.addBlock(Kd);
    timedomain.addBlock(qdd_c);
    timedomain.addBlock(M);
    timedomain.addBlock(QMax);
    timedomain.addBlock(iInv);
    timedomain.addBlock(kMInv);
    timedomain.addBlock(R);
    timedomain.addBlock(qd1);
    timedomain.addBlock(qdMax);
    timedomain.addBlock(i);
    timedomain.addBlock(kM);
    timedomain.addBlock(U1);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}