#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

/**
 * @file FwKinOdom.hpp
 * @author Jonas Frei (jonas.frei@ost.ch)
 * @brief Forward kinematic and odometry block
 * @version 0.1
 * @date 2022-06-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Output.hpp>

using namespace eeros::control;

/**
 * @brief Forward kinematics and odometry class for a differential drive robot
 *
 */
class FwKinOdom : public Block
{
public:
    /**
     * @brief Construct a new Fw Kin Odom object
     *
     * @param B distance between the two wheels
     * @param L distance between the TCP and the midpoint between the two wheels
     * @param GrRInit Initial global robot position
     * @param phiInit Initial global robot orientation
     */
    FwKinOdom(double B, double L, eeros::math::Vector2 GrRInit = 0.0, double phiInit = 0.0)
        : vw(this),
          RJW([&, B]()
              {
            eeros::math::Matrix<3,2> M = {0.5, 0.0, -1.0/B, 0.5, 0.0, 1.0/B};
            eeros::math::Vector3 out = M*RJW.getIn().getSignal().getValue();
            for (int i = 0; i < 3; i++)
            {
                RJW.getOut(i).getSignal().setValue(out(i));
                RJW.getOut(i).getSignal().setTimestamp(RJW.getIn().getSignal().getTimestamp());
            } }),
          TJR([&, L]()
              {
            TJR.getOut(0).getSignal().setValue(TJR.getIn(0).getSignal().getValue());
            TJR.getOut(0).getSignal().setTimestamp(TJR.getIn(0).getSignal().getTimestamp());
            TJR.getOut(1).getSignal().setValue(TJR.getIn(1).getSignal().getValue() + L*TJR.getIn(2).getSignal().getValue());
            TJR.getOut(1).getSignal().setTimestamp(TJR.getIn(1).getSignal().getTimestamp());
            TJR.getOut(2).getSignal().setValue(TJR.getIn(2).getSignal().getValue());
            TJR.getOut(2).getSignal().setTimestamp(TJR.getIn(2).getSignal().getTimestamp()); }),
          GRR([&]()
              {
            double phi = GRR.getIn(2).getSignal().getValue();
            double cphi = std::cos(phi);
            double sphi = std::sin(phi);
            eeros::math::Matrix<2,2> M = {cphi, sphi, -sphi, cphi};
            eeros::math::Vector2 v = {GRR.getIn(0).getSignal().getValue(), GRR.getIn(1).getSignal().getValue()};
            GRR.getOut().getSignal().setValue(M*v);
            GRR.getOut().getSignal().setTimestamp(GRR.getIn(0).getSignal().getTimestamp()); })
    {
        // Set initial integrator values
        GrR.setInitCondition(GrRInit);
        phi.setInitCondition(phiInit);

        // Name all blocks
        RJW.setName("FwKinOdom->RJW");
        TJR.setName("FwKinOdom-TJR");
        GRR.setName("FwKinOdom->GRR");
        GrR.setName("FwKinOdom->GrR");
        phi.setName("FwKinOdom->phi");

        // Name all signals
        RJW.getOut(0).getSignal().setName("RvRx [m/s]");
        RJW.getOut(1).getSignal().setName("RvRy [m/s]");
        RJW.getOut(2).getSignal().setName("omegaR [rad/s]");
        TJR.getOut(0).getSignal().setName("RvTx [m/s]");
        TJR.getOut(1).getSignal().setName("RvTy [m/s]");
        TJR.getOut(2).getSignal().setName("omegaT [rad/s]");
        GRR.getOut().getSignal().setName("GvT [m/s]");
        GrR.getOut().getSignal().setName("GrT [m]");
        phi.getOut().getSignal().setName("phi [rad]");

        // Connect all signals
        RJW.getIn().connect(vw);
        TJR.getIn(0).connect(RJW.getOut(0));
        TJR.getIn(1).connect(RJW.getOut(1));
        TJR.getIn(2).connect(RJW.getOut(2));
        GRR.getIn(0).connect(TJR.getOut(0));
        GRR.getIn(1).connect(TJR.getOut(1));
        GRR.getIn(2).connect(phi.getOut());
        phi.getIn().connect(TJR.getOut(2));
        GrR.getIn().connect(GRR.getOut());
    }

    /**
     * @brief Input getter function
     *
     * @return Input<eeros::math::Vector2>& wheel velocity input
     */
    Input<eeros::math::Vector2> &getIn() { return vw; }

    /**
     * @brief Output getter function
     *
     * @return Output<eeros::math::Vector2>& global TCP velocity
     */
    Output<eeros::math::Vector2> &getOutGvT() { return GRR.getOut(); }

    /**
     * @brief Output getter function
     *
     * @return Output<eeros::math::Vector2>& global TCP position
     */
    Output<eeros::math::Vector2> &getOutGrT() { return GrR.getOut(); }

    /**
     * @brief Output getter function
     *
     * @return Output<>& global TCP orientation
     */
    Output<> &getOutPhi() { return phi.getOut(); }

    /**
     * @brief Output getter function
     *
     * @return Output<>& global TCP angular velocity
     */
    Output<> &getOutOmegaT() { return TJR.getOut(2); }

    /**
     * @brief run method
     *
     */
    virtual void run()
    {
        RJW.run();
        TJR.run();
        phi.run();
        GRR.run();
        GrR.run();
    }

    /**
     * @brief Enable the integrators
     *
     */
    void enable(void)
    {
        GrR.enable();
        phi.enable();
    }

    /**
     * @brief Disable the integrators
     *
     */
    void disable(void)
    {
        GrR.disable();
        phi.disable();
    }

    /**
     * @brief Set position and orientation
     *
     * @param GrR global robot position
     * @param phi global robot orientation
     */
    void setPose(eeros::math::Vector2 GrR, double phi)
    {
        this->GrR.setInitCondition(GrR);
        this->phi.setInitCondition(phi);
    }

protected:
    InputSub<eeros::math::Vector2> vw;
    Blockio<1, 3, eeros::math::Vector2, double> RJW;
    Blockio<3, 3, double> TJR;
    Blockio<3, 1, double, eeros::math::Vector2> GRR;
    I<eeros::math::Vector2> GrR;
    I<> phi;
};

#endif // FWKINODOM_HPP_
