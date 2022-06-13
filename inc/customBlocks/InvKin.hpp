#ifndef INVKIN_HPP_
#define INVKIN_HPP_

/**
 * @file InvKin.hpp
 * @author Jonas Frei (jonas.frei@ost.ch)
 * @brief Inverse kinematics block
 * @version 0.1
 * @date 2022-06-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Gain.hpp>

using namespace eeros::control;

/**
 * @brief Inverse kinematics class for a differential drive robot
 *
 */
class InvKin : public Blockio<1, 0>
{
public:
    /**
     * @brief Construct a new Inv Kin object
     *
     * @param L distance between the TCP and the midpoint between the two wheels
     * @param B distance between the two wheels
     */
    InvKin(double L, double B)
        : RRG({1.0, 0.0, 0.0, 1.0}),
          RJT({1.0, 0.0, 0.0, 1.0 / L}),
          WJR({1.0, 1.0, -B / 2.0, B / 2.0})
    {
        // Name all blocks
        RRG.setName("InvKin->RRG");
        RJT.setName("InvKin->RJT");
        WJR.setName("InvKin->WJR");

        // Name all signals
        RRG.getOut().getSignal().setName("RvTc [m/s]");
        RJT.getOut().getSignal().setName("RvRxc [m/s], omegaRc [rad/s]");
        WJR.getOut().getSignal().setName("vW [m/s]");

        // Connect all blocks
        RJT.getIn().connect(RRG.getOut());
        WJR.getIn().connect(RJT.getOut());
    }

    /**
     * @brief Input getter function
     *
     * @return Input<>& Input for the global TCP velocity
     */
    Input<eeros::math::Vector2> &getInGvTc() { return RRG.getIn(); }

    /**
     * @brief Input getter function
     *
     * @return Input<>& Input for the global TCP orientation
     */
    Input<> &getInPhi() { return this->getIn(); }

    /**
     * @brief Ouput getter function
     *
     * @return Output<eeros::math::Vector2>& Output for the wheel velocities
     */
    Output<eeros::math::Vector2> &getOut() { return WJR.getOut(); }

    /**
     * @brief run method
     *
     */
    virtual void run()
    {
        double cphi = std::cos(this->getIn().getSignal().getValue());
        double sphi = std::sin(this->getIn().getSignal().getValue());
        RRG.setGain({cphi, -sphi, sphi, cphi});
        RRG.run();
        RJT.run();
        WJR.run();
    }

protected:
    Gain<eeros::math::Vector2, eeros::math::Matrix<2, 2>> RRG, RJT, WJR;
};

#endif // INVKIN_HPP_
