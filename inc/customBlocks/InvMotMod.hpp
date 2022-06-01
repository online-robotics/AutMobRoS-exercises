#ifndef INVMOTMOD_HPP_
#define INVMOTMOD_HPP_

/**
 * @file InvMotMod.hpp
 * @author Jonas Frei (jonas.frei@ost.ch)
 * @brief Inverse motor model block
 * @version 0.1
 * @date 2022-06-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <eeros/control/Block.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>

using namespace eeros::control;

/**
 * @brief Inverse motor modell class
 * 
 * @tparam T output type (default double)
 */
template <typename T = double>
class InvMotMod : public Block
{
public:
    /**
     * @brief Construct a new Inv Mot Mod object
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
        // Name all blocks
        this->QMax.setName("QMax");
        iInv.setName("iInv");
        kMInv.setName("kMInv");
        this->R.setName("R");
        this->qdMax.setName("qdMax");
        this->i.setName("i");
        this->kM.setName("kM");
        U.setName("U");

        // Name all signals
        this->QMax.getOut().getSignal().setName("Q [Nm]");
        iInv.getOut().getSignal().setName("T [Nm]");
        kMInv.getOut().getSignal().setName("I [A]");
        this->R.getOut().getSignal().setName("UR [V]");
        this->qdMax.getOut().getSignal().setName("qd [rad/s]");
        this->i.getOut().getSignal().setName("om [rad/s]");
        this->kM.getOut().getSignal().setName("Uom [V]");
        U.getOut().getSignal().setName("U [V]");

        // Connect signals
        iInv.getIn().connect(this->QMax.getOut());
        kMInv.getIn().connect(iInv.getOut());
        this->R.getIn().connect(kMInv.getOut());
        this->i.getIn().connect(this->qdMax.getOut());
        this->kM.getIn().connect(this->i.getOut());
        U.getIn(0).connect(this->R.getOut());
        U.getIn(1).connect(this->kM.getOut());
    }

    /**
     * @brief Input getter function
     * 
     * @param index input index
     * @return Input<T>& index 0: torque input, index 1: velocity input
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

    /**
     * @brief Output getter function
     * 
     * @return Output<T>& motor voltage
     */
    virtual Output<T> &getOut()
    {
        return U.getOut();
    }

    /**
     * @brief run method
     * 
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
};

#endif // INVMOTMOD_HPP_
