#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include <eeros/control/Constant.hpp>
#include "customBlocks/InvKin.hpp"

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    Mux<2> E;
    D<eeros::math::Vector2> Ed;
    FwKinOdom fwKinOdom;
    Constant<> RvRx, omegaR;
    InvKin invKin;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP