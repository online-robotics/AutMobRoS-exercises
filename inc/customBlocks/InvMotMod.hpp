#ifndef INVMOTMOD_HPP_
#define INVMOTMOD_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>

using namespace eeros::control;

/*!
 * Inverse motor modell class
 */
template <typename T = double>
class InvMotMod : public Block
{
public:
    /*!
     * Constructor
     *
     * @param QMax maximum torque
     * @param qdMax maximum velocity
     * @param i transmission ratio
     * @param kM motor konstant
     * @param R motor resistance
     */
    InvMotMod(double QMax, double qdMax, double i, double kM, double R)
        : QMax(QMax),
          iInv(1.0 / i),
          kMInv(1.0 / kM),
          R(R),
          qdMax(qdMax),
          i(i),
          kM(kM)
    {
        init();
    }

    /*!
     * Input getter function
     *
     * @param index input index
     */
    virtual Input<T> &getIn(uint8_t index)
    {
        if (index == 0)
        {
            return QMax.getIn();
        }
        else if (index == 1)
        {
            return qdMax.getIn();
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    /*!
     * Output getter function
     */
    virtual Output<T> &getOut()
    {
        return U.getOut();
    }

    /*!
     * run method
     */
    virtual void run()
    {
        QMax.run();
        iInv.run();
        kMInv.run();
        R.run();
        qdMax.run();
        i.run();
        kM.run();
        U.run();
    }

protected:
    Saturation<T> QMax, qdMax;
    Gain<T> iInv, kMInv, R, i, kM;
    Sum<2, T> U;

private:
    void init()
    {
        // Name all blocks
        QMax.setName("QMax");
        iInv.setName("iInv");
        kMInv.setName("kMInv");
        R.setName("R");
        qdMax.setName("qdMax");
        i.setName("i");
        kM.setName("kM");
        U.setName("U");

        // Name all signals
        QMax.getOut().getSignal().setName("Q [Nm]");
        iInv.getOut().getSignal().setName("T [Nm]");
        kMInv.getOut().getSignal().setName("I [A]");
        R.getOut().getSignal().setName("UR [V]");
        qdMax.getOut().getSignal().setName("qd [rad/s]");
        i.getOut().getSignal().setName("om [rad/s]");
        kM.getOut().getSignal().setName("Uom [V]");
        U.getOut().getSignal().setName("U [V]");

        // Connect signals
        iInv.getIn().connect(QMax.getOut());
        kMInv.getIn().connect(iInv.getOut());
        R.getIn().connect(kMInv.getOut());
        i.getIn().connect(qdMax.getOut());
        kM.getIn().connect(i.getOut());
        U.getIn(0).connect(R.getOut());
        U.getIn(1).connect(kM.getOut());
    }
};

#endif // INVMOTMOD_HPP_
