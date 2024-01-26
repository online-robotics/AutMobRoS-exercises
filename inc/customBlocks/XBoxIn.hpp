#ifndef CUSTOMBLOCKTEMPLATE_HPP_
#define CUSTOMBLOCKTEMPLATE_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/XBoxInput.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Gain.hpp>

using namespace eeros::control;

class XBoxIn : public Blockio<0,2,eeros::math::Matrix<8>, double>
{
public:
    XBoxIn(std::string dev = "/dev/input/js0", double RvRx = 0.25, double omegaR = 1.5)
        : xBoxInput(dev),
          RvRx(RvRx),
          omegaR(omegaR)
    {
        xBoxInput.getOut().getSignal().setName("xBox");
        xBoxValues.getOut(3).getSignal().setName("xBox Joystick UpDown");
        xBoxValues.getOut(4).getSignal().setName("xBox JoystickLeftRight");
        this->RvRx.getOut().getSignal().setName("Setpoint RvRx [m/s]");
        this->omegaR.getOut().getSignal().setName("Setpoint omegaR [rad/s]");

        xBoxValues.getIn().connect(xBoxInput.getOut());
        this->RvRx.getIn().connect(xBoxValues.getOut(3));
        this->omegaR.getIn().connect(xBoxValues.getOut(4));
    }

    virtual void run()
    {
        xBoxInput.run();
        xBoxValues.run();
        RvRx.run();
        omegaR.run();
        if (abs(RvRx.getOut().getSignal().getValue()) < 0.1 && abs(omegaR.getOut().getSignal().getValue()) < 0.5)
        {
            this->getOut(0).getSignal().setValue(0.0);
            this->getOut(1).getSignal().setValue(0.0);
        }
        else
        {
            this->getOut(0).getSignal().setValue(RvRx.getOut().getSignal().getValue());
            this->getOut(1).getSignal().setValue(omegaR.getOut().getSignal().getValue());
        }
        this->getOut(0).getSignal().setTimestamp(eeros::System::getTimeNs());
        this->getOut(1).getSignal().setTimestamp(eeros::System::getTimeNs());
    }

protected:
    XBoxInput xBoxInput;
    DeMux<8> xBoxValues;
    Gain<> RvRx, omegaR;
    
};

#endif //CUSTOMBLOCKTEMPLATE_HPP_
