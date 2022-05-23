#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E2;
    Gain<> cont;
    Saturation<> QMax;
    Gain<> iInv;
    Gain<> kMInv;
    Gain<> R;
    PeripheralOutput<> M1;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP