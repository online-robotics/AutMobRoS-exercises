#include "AutMobRoSSafetyProperties.hpp"

AutMobRoSSafetyProperties::AutMobRoSSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),

      slSystemOff("System is offline"),
      slShuttingDown("System shutting down"),
      slBraking("System braking"),
      slStartingUp("System starting up"),
      slEmergency("Emergency"),
      slEmergencyBraking("System halting"),
      slSystemOn("System is online"),
      slMotorPowerOn("Motors powered"),
      slSystemMoving("System moving"),

      abort("Abort"),
      shutdown("Shutdown"),
      doSystemOn("Do system on"),
      systemStarted("System started"),
      emergency("Emergency"),
      resetEmergency("Reset emergency"),
      powerOn("Power on"),
      powerOff("Power off"),
      startMoving("Start moving"),
      stopMoving("Stop moving"),
      motorsHalted("Motors halted")
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    greenLED = hal.getLogicOutput("onBoardLEDgreen");
    redLED = hal.getLogicOutput("onBoardLEDred");

    criticalOutputs = {greenLED, redLED};

    // Declare and add critical inputs
    buttonPause = eeros::hal::HAL::instance().getLogicInput("onBoardButtonPause");
    buttonMode = eeros::hal::HAL::instance().getLogicInput("onBoardButtonMode");

    criticalInputs = {buttonPause, buttonMode};

    // Add all safety levels to the safety system
    addLevel(slSystemOff);
    addLevel(slShuttingDown);
    addLevel(slBraking);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBraking);
    addLevel(slSystemOn);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(shutdown, slSystemOff, kPrivateEvent);
    slBraking.addEvent(motorsHalted, slShuttingDown, kPrivateEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPrivateEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPrivateEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPrivateEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergencyBraking, kPublicEvent);
    slSystemMoving.addEvent(abort, slBraking, kPublicEvent);

    // Add events to multiple safety levels
    addEventToAllLevelsBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelsBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);

    // Define input actions for all levels
    slSystemOff.setInputActions({ignore(buttonPause), ignore(buttonMode)});
    slShuttingDown.setInputActions({ignore(buttonPause), ignore(buttonMode)});
    slBraking.setInputActions({ignore(buttonPause), ignore(buttonMode)});
    slStartingUp.setInputActions({ignore(buttonPause), ignore(buttonMode)});
    slEmergency.setInputActions({ignore(buttonPause), check(buttonMode, true, resetEmergency)});
    slEmergencyBraking.setInputActions({ignore(buttonPause), ignore(buttonMode)});
    slSystemOn.setInputActions({check(buttonPause, false, powerOn), ignore(buttonMode)});
    slMotorPowerOn.setInputActions({ignore(buttonPause), check(buttonMode, false, emergency)});
    slSystemMoving.setInputActions({ignore(buttonPause), check(buttonMode, false, emergency)});

    // Define output actions for all levels
    slSystemOff.setOutputActions({set(greenLED, false), toggle(redLED)});
    slShuttingDown.setOutputActions({set(greenLED, false), toggle(redLED)});
    slBraking.setOutputActions({set(greenLED, false), toggle(redLED)});
    slStartingUp.setOutputActions({toggle(greenLED), set(redLED, false)});
    slEmergency.setOutputActions({set(greenLED, false), set(redLED, true)});
    slEmergencyBraking.setOutputActions({set(greenLED, false), set(redLED, true)});
    slSystemOn.setOutputActions({toggle(greenLED), toggle(redLED)});
    slMotorPowerOn.setOutputActions({set(greenLED, true), set(redLED, false)});
    slSystemMoving.setOutputActions({set(greenLED, true), set(redLED, false)});

    // Define and add level actions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext)
                               { eeros::Executor::stop(); });

    slShuttingDown.setLevelAction([&](SafetyContext *privateContext)
                                  {
        cs.fwKinOdom.disable();
        cs.cont.disable();
        cs.pp.disable();
        cs.timedomain.stop();
        privateContext->triggerEvent(shutdown); });

    slBraking.setLevelAction([&](SafetyContext *privateContext)
                             {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.disable();
        if (abs(cs.Ed.getOut().getSignal().getValue()(0)) < 1e-3 && abs(cs.Ed.getOut().getSignal().getValue()(1)) < 1e-3)
        {
        privateContext->triggerEvent(motorsHalted);
        } });

    slStartingUp.setLevelAction([&](SafetyContext *privateContext)
                                {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.disable();
        cs.timedomain.start();
        privateContext->triggerEvent(systemStarted); });

    slEmergency.setLevelAction([&](SafetyContext *privateContext)
                               {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.disable(); });

    slEmergencyBraking.setLevelAction([&](SafetyContext *privateContext)
                                      {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.disable();
        if (abs(cs.Ed.getOut().getSignal().getValue()(0)) < 1e-3 && abs(cs.Ed.getOut().getSignal().getValue()(1)) < 1e-3)
        {
            privateContext->triggerEvent(motorsHalted);
        } });

    slSystemOn.setLevelAction([&, dt](SafetyContext *privateContext)
                              {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.disable(); });

    slMotorPowerOn.setLevelAction([&, dt](SafetyContext *privateContext)
                                  {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.enable();
        if (abs(cs.Ed.getOut().getSignal().getValue()(0)) > 1e-1 || abs(cs.Ed.getOut().getSignal().getValue()(1)) > 1e-1)
        {
            privateContext->triggerEvent(startMoving);
        } });

    slSystemMoving.setLevelAction([&, dt](SafetyContext *privateContext)
                                  {
        cs.fwKinOdom.enable();
        cs.cont.enable();
        cs.pp.enable();
        if (abs(cs.Ed.getOut().getSignal().getValue()(0)) < 1e-3 && abs(cs.Ed.getOut().getSignal().getValue()(1)) < 1e-3)
        {
            privateContext->triggerEvent(stopMoving);
        } });

    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext)
                    { privateContext->triggerEvent(abort); });
}
