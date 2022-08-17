#ifndef REMOTECONTROL_HPP_
#define REMOTECONTROL_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/KeyboardInput.hpp>

using namespace eeros::control;

class RemoteControl : public Blockio<0, 2>
{
public:
    RemoteControl() : keyboard{{'w', 'a', 'd', 's'}, {"w", "a", "d", "s"}}, k{0}, i{0}, j{0}, l{0}, debounce{100}, RvRx{0.0}, omegaR{0.0}, RvRxLast{0.0}, omegaRLast{0.0}, alpha{0.01}
    {
        // Connect subblocks, initialize variables, ...
    }

    // Implement getter functions for the inputs and outputs

    virtual void run()
    {
        keyboard.run();
        RvRx = 0.0;
        omegaR = 0.0;
        if (keyboard.getOut(0).getSignal().getValue() == true)
        {
            k = 1;
        }
        if (keyboard.getOut(1).getSignal().getValue() == true)
        {
            i = 1;
        }
        if (keyboard.getOut(2).getSignal().getValue() == true)
        {
            j = 1;
        }
        if (keyboard.getOut(3).getSignal().getValue() == true)
        {
            l = 1;
        }
        if (k > 0)
        {
            k++;
            RvRx = 0.3*alpha + RvRxLast*(1.0-alpha);
        }
        if (k > debounce)
        {
            k = 0;
        }
        if (i > 0)
        {
            i++;
            omegaR = -1.5*alpha + omegaRLast*(1.0-alpha);
        }
        if (i > debounce)
        {
            i = 0;
        }
        if (j > 0)
        {
            j++;
            omegaR = 1.5*alpha + omegaRLast*(1.0-alpha);
        }
        if (j > debounce)
        {
            j = 0;
        }
        if (i > 1 && j > 1)
        {
            j++;
            i++;
        }
        if (l > 0)
        {
            l++;
            RvRx = -0.3*alpha + RvRxLast*(1.0-alpha);
        }
        if (l > debounce)
        {
            l = 0;
        }
        if (k > 1 && l > 1)
        {
            k++;
            l++;
        }
        keyboard.reset(0);
        keyboard.reset(1);
        keyboard.reset(2);
        keyboard.reset(3);
        this->getOut(0).getSignal().setValue(RvRx);
        this->getOut(1).getSignal().setValue(omegaR);
        this->getOut(0).getSignal().setTimestamp(eeros::System::getTimeNs());
        this->getOut(1).getSignal().setTimestamp(eeros::System::getTimeNs());
        RvRxLast = RvRx;
        omegaRLast = omegaR;
    }

protected:
    KeyboardInput<4> keyboard;
    int k, i, j, l;
    int debounce;
    double RvRx, omegaR, RvRxLast, omegaRLast, alpha;
};

#endif // REMOTECONTROL_HPP_
