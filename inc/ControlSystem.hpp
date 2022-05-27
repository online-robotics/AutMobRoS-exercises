#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    Sum<> e;
    Gain<> Kp;
    D<> ed;
    Gain<> Kd;
    Sum<> qdd_c;
    Gain<> M;
    Saturation<> QMax;
    Gain<> iInv;
    Gain<> kMInv;
    Gain<> R;
    D<> qd1;
    Saturation<> qdMax;
    Gain<> i;
    Gain<> kM;
    Sum<> U1;
    PeripheralOutput<> M1;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP