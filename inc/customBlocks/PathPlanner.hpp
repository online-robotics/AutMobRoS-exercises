#ifndef PATHPLANNER_HPP_
#define PATHPLANNER_HPP_

/**
 * @file pathPlanner.hpp
 * @author your name (you@domain.com)
 * @brief Path planner block
 * @version 0.1
 * @date 2022-06-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <cmath>
#include <mutex>

using namespace eeros::control;

/**
 * @brief Path planner class
 *
 */
class PathPlanner : public Block
{
public:
    /**
     * @brief Construct a new Path Planner object
     *
     * @param k1 konstant 1
     * @param k2 konstant 2
     * @param k3 konstant 3
     * @param tolPos position tolerance
     * @param tolRot rotation tolerance
     * @param xT x coordinate of target position
     * @param yT y coordinate of target position
     * @param phiT target orientation
     */
    PathPlanner(double k1, double k2, double k3,
                double tolPos = 5e-3, double tolRot = M_PI / 1000.0,
                double xT = 0.0, double yT = 0.0, double phiT = 0.0)
        : GrR(this),
          phi(this),
          xT(xT),
          yT(yT),
          phiT(phiT),
          k1(k1),
          k2(k2),
          k3(k3),
          tolPos(tolPos),
          tolRot(tolRot),
          targetReached(true),
          enabled(false),
          RvRx_d(this),
          omegaR_d(this)
    {
        // Name all signals
        this->RvRx_d.getSignal().setName("RvRx_d [m/s]");
        this->omegaR_d.getSignal().setName("omegaR_d [rad/s]");
    }

    /**
     * @brief Input getter function
     *
     * @return Input<eeros::math::Vector2>& GrR
     */
    Input<eeros::math::Vector2> &getInGrR() { return GrR; }

    /**
     * @brief Input getter function
     *
     * @return Input<>& phi
     */
    Input<> &getInphi() { return phi; }

    /**
     * @brief Output getter functions
     *
     * @return Output<>& RvRx_d
     */
    Output<> &getOutRvRx_d() { return RvRx_d; }
    /**
     * @brief Output getter functions
     *
     * @return Output<>& omegaR_d
     */
    Output<> &getOutomegaR_d() { return omegaR_d; }

    /**
     * @brief run method
     *
     */
    virtual void run()
    {
        // Check if the controller is enabled and the target position is not reached
        // If true: run the algorithm
        // Else: set the translational and angular velocities to 0
        if (enabled && !targetReached)
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Calculate the distance to the target
            rho = sqrt(square(xT - GrR.getSignal().getValue()[0]) +
                       square(yT - GrR.getSignal().getValue()[1]));
            // Check if the target is reached
            // If true: set the translational and angular velocities to 0
            // Else: calculate the translational and angular velocities
            if (rho <= tolPos)
            {
                targetReached = true;
                RvRx_d.getSignal().setValue(0.0);
                omegaR_d.getSignal().setValue(0.0);
            }
            else
            {
                // Calculate the orientation to the target
                gamma = constrainAngle(atan2(yT - GrR.getSignal().getValue()[1],
                                             xT - GrR.getSignal().getValue()[0]) -
                                       phi.getSignal().getValue());
                delta = constrainAngle(gamma + phi.getSignal().getValue() - phiT);
                // Check if orientation is already correct
                // If true: the term sin(gamma)/gamma is approximately 1
                // Else: use the formula as it was shown in the lecture
                if (fabs(gamma) <= tolRot)
                {
                    omegaR_d.getSignal().setValue(k2 * gamma +
                                                  k1 * cos(gamma) * (gamma + k3 * delta));
                }
                else
                {
                    omegaR_d.getSignal().setValue(k2 * gamma +
                                                  k1 * sin(gamma) * cos(gamma) * (gamma + k3 * delta) / gamma);
                }
                // Set the translational velocity
                RvRx_d.getSignal().setValue(k1 * rho * cos(gamma));
            }
        }
        else
        {
            RvRx_d.getSignal().setValue(0.0);
            omegaR_d.getSignal().setValue(0.0);
        }
        // Set timestamps of the output signals
        RvRx_d.getSignal().setTimestamp(GrR.getSignal().getTimestamp());
        omegaR_d.getSignal().setTimestamp(GrR.getSignal().getTimestamp());
    }

    /**
     * @brief enable the path planner
     *
     */
    void enable(void) { enabled = true; }

    /**
     * @brief disable the path planner
     *
     */
    void disable(void) { enabled = false; }

    /**
     * @brief Set new target position and orientation
     *
     * @param xT x coordinate of target position
     * @param yT y coordinate of target position
     * @param phiT target orientation
     */
    void setTarget(double xT, double yT, double phiT)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->xT = xT;
        this->yT = yT;
        this->phiT = phiT;
        this->targetReached = false;
    }

    /**
     * @brief Get the path planner status
     *
     * @return true target position reached
     * @return false path planner still running
     */
    bool getStatus(void) { return targetReached; }

protected:
    std::mutex mtx;
    Input<eeros::math::Vector2> GrR;
    Input<> phi;
    double rho, gamma, delta, xT, yT, phiT, k1, k2, k3, tolPos, tolRot;
    bool targetReached, enabled;
    Output<> RvRx_d, omegaR_d;

private:
    /**
     * @brief calculate the square
     *
     * @param x input value
     * @return double square of x
     */
    double square(double x) { return x * x; }

    /**
     * @brief constrain an angle to +-pi
     *
     * @param x angle
     * @return double angle between +-pi
     */
    double constrainAngle(double x)
    {
        x = fmod(x + M_PI, 2.0 * M_PI);
        if (x < 0.0)
            x += 2.0 * M_PI;
        return x - M_PI;
    }
};

#endif // PATHPLANNER_HPP_
