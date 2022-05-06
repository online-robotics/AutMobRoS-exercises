#ifndef AutMobRoSSAFETYPROPERTIES_HPP_
#define AutMobRoSSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class AutMobRoSSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    AutMobRoSSafetyProperties(ControlSystem &cs, double dt);

    // Define all possible events
    eeros::safety::SafetyEvent abort;
    eeros::safety::SafetyEvent shutdown;
    eeros::safety::SafetyEvent doSystemOn;
    eeros::safety::SafetyEvent systemStarted;
    eeros::safety::SafetyEvent emergency;
    eeros::safety::SafetyEvent resetEmergency;
    eeros::safety::SafetyEvent powerOn;
    eeros::safety::SafetyEvent powerOff;
    eeros::safety::SafetyEvent startMoving;
    eeros::safety::SafetyEvent stopMoving;
    eeros::safety::SafetyEvent motorsHalted;

    // Defina all possible levels
    eeros::safety::SafetyLevel slSystemOff;
    eeros::safety::SafetyLevel slShuttingDown;
    eeros::safety::SafetyLevel slBraking;
    eeros::safety::SafetyLevel slStartingUp;
    eeros::safety::SafetyLevel slEmergency;
    eeros::safety::SafetyLevel slEmergencyBraking;
    eeros::safety::SafetyLevel slSystemOn;
    eeros::safety::SafetyLevel slMotorPowerOn;
    eeros::safety::SafetyLevel slSystemMoving;

private:
    // Define all critical outputs
    eeros::hal::Output<bool>* greenLED;
    eeros::hal::Output<bool>* redLED;

    // Define all critical inputs
    eeros::hal::Input<bool>* buttonPause;
    eeros::hal::Input<bool>* buttonMode;

    ControlSystem &cs;
};

#endif // AutMobRoSSAFETYPROPERTIES_HPP_
