#ifndef TCPVECPOSCONT_HPP_
#define TCPVECPOSCONT_HPP_

/**
 * @file TCPVecPosCont.hpp
 * @author Jonas Frei (jonas.frei@ost.ch)
 * @brief TCP Vector Position Controller Block
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>

using namespace eeros::control;

/**
 * @brief TCP Vector Position Controller Class
 *
 */
class TCPVecPosCont : public Blockio<1, 1, eeros::math::Vector2>
{
public:
    /**
     * @brief Construct a new TCPVecPosCont object
     *
     * @param fClock control frequency
     * @param D damping ratio
     * @param vmax velocity limit
     * @param tol tolerance required to have reached the target, default = 1e-3 m
     */
    TCPVecPosCont(double fClock, double D, double vmax, double tol = 1e-3)
        : K(fClock / 3.2 / 2.0 / D),
          GvTcMax(vmax),
          tol(tol),
          enabled(false)
    {
        // Name all signals
        this->out.getSignal().setName("GvTc [m/s]");
    }

    /**
     * @brief Run method
     *
     */
    virtual void run()
    {
        if (enabled)
        {
            std::lock_guard<std::mutex> lock(mtx);
            eeros::math::Vector2 GeTd = GrTd - this->getIn().getSignal().getValue();
            double GeTdNorm = std::sqrt(GeTd(0) * GeTd(0) + GeTd(1) * GeTd(1));
            double GvTc = K * GeTdNorm;
            if (GvTc > GvTcMax)
            {
                GvTc = GvTcMax;
            }
            else if (GvTc < -GvTcMax)
            {
                GvTc = -GvTcMax;
            }
            this->getOut().getSignal().setValue(GvTc * GeTd / GeTdNorm);
            status = (GeTdNorm < tol);
        }
        else
        {
            this->getOut().getSignal().setValue(0.0);
        }
        this->getOut().getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
    }

    /**
     * @brief Enable the controller
     *
     */
    void enable(void)
    {
        this->enabled = true;
    }

    /**
     * @brief Disable the controller
     *
     */
    void disable(void)
    {
        this->enabled = false;
    }

    /**
     * @brief Set new target position
     *
     * @param GrTd Desired TCP Position
     */
    void setTarget(eeros::math::Vector2 GrTd)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->GrTd = GrTd;
        status = false;
    }

    /**
     * @brief Get the status of the controller
     *
     * @return _Bool 1: target position reached, 0: still running
     */
    bool getStatus(void)
    {
        return status;
    }

protected:
    std::mutex mtx;
    eeros::math::Vector2 GrTd;
    double K, GvTcMax, tol;
    bool enabled, status;
};

#endif // TCPVECPOSCONT_HPP_
