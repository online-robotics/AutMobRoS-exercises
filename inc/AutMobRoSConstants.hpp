#ifndef AMRSCONSTANTS_HPP_
#define AMRSCONSTANTS_HPP_

#include <eeros/math/Matrix.hpp>

namespace AMRSC
{
    namespace MOT
    {
        const double QMAX = 2.5;                    // [N]
        const double qdMAX = 0.848;                 // [m/s]
        const double i = 3441.0 / 104.0 / 0.04;     // [1/m]
        const double J = 6.8e-8;                    // [kgm^2]
        const double R = 8.0;                       // [hm]
        const double KM = 8.44e-3;                  // [Nm/A]
    }
    namespace CONT
    {
        const double D = 0.7;                       // [-]
        const double s = 2.2;                       // [-]
        const double M = MOT::i*MOT::i*MOT::J;      // [kg]
        const double ILIMIT = 0.1;                  // [m]
    }
    namespace PP
    {
        const double K1 = 0.5;
        const double K2 = 1.0;
        const double K3 = 1.0;
        const double POSTOL = 1e-3;                 // [m]
        const double ROTTOL = 1e-3;                 // [rad]
    }
    namespace TCPCont
    {
        const double D = 0.7;                       // [-]
        const double VMAX = 0.1;                    // [m/s]
        const double fPos = 10.0;                    // [Hz]
    }
    namespace ROB
    {
        const double B = 0.15;                      // [m]
        const double L = 0.17;                      // [m]
    }
    namespace KF
    {
        const eeros::math::Matrix<4, 4> Ad = {1.0, 0.0, 0.0, 0.0,
                                              9.901294350681072e-04, 0.980258870136214, -1.563880235077955, 0.0,
                                              1.187647358725703e-06, 0.002375294717451, -0.811830605100179, 0.0,
                                              -4.950647175340536e-07, -9.901294350681072e-04, 7.819401175389773e-04, 1.0};
        const eeros::math::Matrix<4, 1> Bd = {1.413865903244885e-06, 0.002827731806490, 0.224011184404549, 0.0};
        const eeros::math::Matrix<1, 4> C = {1.0, 0.0, 0.0, 0.0};
        const eeros::math::Matrix<4, 1> Gd = {-4.950647175340536e-07, -9.901294350681072e-04, 7.819401175389773e-04, 1.0};
        const double Q = 2.777777777777778e-08;
        const double R = 1.173912855809449e-09;
    }
}

#endif // AMRSCONSTANTS_HPP_