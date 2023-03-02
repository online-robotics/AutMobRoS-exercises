#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/filter/KalmanFilter.hpp>
#include <eeros/control/Mux.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include "customBlocks/TCPVecPosCont.hpp"
#include "customBlocks/InvKin.hpp"
#include "customBlocks/Controller.hpp"
#include "customBlocks/InvMotMod.hpp"
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include "AutMobRoSConstants.hpp"

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    KalmanFilter<1, 1, 4, 1> KF1, KF2;
    Mux<2> Ed;
    FwKinOdom fwKinOdom;
    TCPVecPosCont tcpVecPosCont;
    InvKin invKin;
    Controller<eeros::math::Vector2> cont;
    InvMotMod<eeros::math::Vector2> invMotMod;
    DeMux<2> M;
    PeripheralOutput<> M1, M2;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP