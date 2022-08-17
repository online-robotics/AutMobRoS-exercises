#include "ControlSystem.hpp"

using namespace AMRSC;

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
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
    remote.setName("remote");
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
    remote.getOut(0).getSignal().setName("RvRx [m/s]");
    remote.getOut(1).getSignal().setName("omegaR [rad/s]");

    // Connect signals
    E.getIn(0).connect(E1.getOut());
    E.getIn(1).connect(E2.getOut());
    Ed.getIn().connect(E.getOut());
    invKin.getInRvRx_d().connect(remote.getOut(0));
    invKin.getInOmegaR_d().connect(remote.getOut(1));
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
    timedomain.addBlock(remote);
    timedomain.addBlock(invKin);
    timedomain.addBlock(cont);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(M);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}