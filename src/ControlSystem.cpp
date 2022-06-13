#include "ControlSystem.hpp"

using namespace AMRSC;

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      KF1(AMRSC::KF::Ad, AMRSC::KF::Bd, AMRSC::KF::C, AMRSC::KF::Gd, AMRSC::KF::Q, AMRSC::KF::R),
      KF2(AMRSC::KF::Ad, AMRSC::KF::Bd, AMRSC::KF::C, AMRSC::KF::Gd, AMRSC::KF::Q, AMRSC::KF::R),
      fwKinOdom(ROB::B, ROB::L),
      tcpVecPosCont(TCPCont::fPos, TCPCont::D, TCPCont::VMAX),
      invKin(ROB::L, ROB::B),
      cont(1.0 / dt, CONT::D, CONT::s, CONT::M, CONT::ILIMIT),
      invMotMod(MOT::QMAX, MOT::qdMAX, MOT::i, MOT::KM, MOT::R),
      M1("motor1"),
      M2("motor2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    KF1.setName("KF1");
    KF2.setName("KF2");
    Ed.setName("Ed");
    fwKinOdom.setName("fwKinOdom");
    tcpVecPosCont.setName("tcpVecPosCont");
    invKin.setName("invKin");
    cont.setName("cont");
    invMotMod.setName("invMotMod");
    M.setName("M");
    M1.setName("M1");
    M2.setName("M2");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [m]");
    E2.getOut().getSignal().setName("q2 [m]");
    KF1.getX(0).getSignal().setName("q1o [m]");
    KF1.getX(1).getSignal().setName("qd1o [m/s]");
    KF1.getX(2).getSignal().setName("I1o [A]");
    KF1.getX(3).getSignal().setName("Offset qdd1o [m/s²]");
    KF2.getX(0).getSignal().setName("q2o [m]");
    KF2.getX(1).getSignal().setName("qd2o [m/s]");
    KF2.getX(2).getSignal().setName("I2o [A]");
    KF2.getX(3).getSignal().setName("Offset qdd2o [m/s²]");
    Ed.getOut().getSignal().setName("qd [m/s]");

    // Connect signals
    KF1.getY(0).connect(E1.getOut());
    KF2.getY(0).connect(E2.getOut());
    Ed.getIn(0).connect(KF1.getX(1));
    Ed.getIn(1).connect(KF2.getX(1));
    fwKinOdom.getIn().connect(Ed.getOut());
    tcpVecPosCont.getIn().connect(fwKinOdom.getOutGrT());
    invKin.getInGvTc().connect(tcpVecPosCont.getOut());
    invKin.getInPhi().connect(fwKinOdom.getOutPhi());
    cont.getIn(0).connect(invKin.getOut());
    cont.getIn(1).connect(Ed.getOut());
    invMotMod.getIn(0).connect(cont.getOut(0));
    invMotMod.getIn(1).connect(cont.getOut(1));
    M.getIn().connect(invMotMod.getOut());
    KF1.getU(0).connect(M.getOut(0));
    KF2.getU(0).connect(M.getOut(1));
    M1.getIn().connect(M.getOut(0));
    M2.getIn().connect(M.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(KF1.correct);
    timedomain.addBlock(KF2.correct);
    timedomain.addBlock(Ed);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(tcpVecPosCont);
    timedomain.addBlock(invKin);
    timedomain.addBlock(cont);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(M);
    timedomain.addBlock(KF1.predict);
    timedomain.addBlock(KF2.predict);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}