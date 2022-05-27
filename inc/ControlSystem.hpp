#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include "customBlocks/Controller.hpp"
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    Mux<2> E;
    Mux<2> E_d;
    Controller<eeros::math::Vector2> cont;
    Saturation<eeros::math::Vector2> QMax;
    Gain<eeros::math::Vector2> iInv;
    Gain<eeros::math::Vector2> kMInv;
    Gain<eeros::math::Vector2> R;
    Saturation<eeros::math::Vector2> qdMax;
    Gain<eeros::math::Vector2> i;
    Gain<eeros::math::Vector2> kM;
    Sum<2, eeros::math::Vector2> U1;
    DeMux<2> U;
    PeripheralOutput<> M1;
    PeripheralOutput<> M2;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP