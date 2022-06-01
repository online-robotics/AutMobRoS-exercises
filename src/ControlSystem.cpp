#include "ControlSystem.hpp"

using namespace AMRSC;

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      fwKinOdom(ROB::B),
      pp(PP::K1, PP::K2, PP::K3, PP::POSTOL, PP::ROTTOL),
      invKin(ROB::B),
      cont(1.0 / dt, CONT::D, CONT::s, CONT::M, CONT::ILIMIT),
      invMotMod(MOT::QMAX, MOT::qdMAX, MOT::i, MOT::KM, MOT::R),
      M1("motor1"),
      M2("motor2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    E.setName("E");
    Ed.setName("Ed");
    fwKinOdom.setName("fwKinOdom");
    pp.setName("pp");
    invKin.setName("invKin");
    cont.setName("cont");
    invMotMod.setName("invMotMod");
    M.setName("M");
    M1.setName("M1");
    M2.setName("M2");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [m]");
    E2.getOut().getSignal().setName("q2 [m]");
    E.getOut().getSignal().setName("q [m]");
    Ed.getOut().getSignal().setName("qd [m/s]");

    // Connect signals
    E.getIn(0).connect(E1.getOut());
    E.getIn(1).connect(E2.getOut());
    Ed.getIn().connect(E.getOut());
    fwKinOdom.getIn().connect(Ed.getOut());
    pp.getInGrR().connect(fwKinOdom.getOutGrR());
    pp.getInphi().connect(fwKinOdom.getOutPhi());
    invKin.getInRvRx_d().connect(pp.getOutRvRx_d());
    invKin.getInOmegaR_d().connect(pp.getOutomegaR_d());
    cont.getIn(0).connect(invKin.getOut());
    cont.getIn(1).connect(Ed.getOut());
    invMotMod.getIn(0).connect(cont.getOut(0));
    invMotMod.getIn(1).connect(cont.getOut(1));
    M.getIn().connect(invMotMod.getOut());
    M1.getIn().connect(M.getOut(0));
    M2.getIn().connect(M.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(E);
    timedomain.addBlock(Ed);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(pp);
    timedomain.addBlock(invKin);
    timedomain.addBlock(cont);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(M);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}