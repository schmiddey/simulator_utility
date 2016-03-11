/*
 * PIController.h
 *
 *  Created on: 11.03.2016
 *      Author: m1ch1
 */

#ifndef PICONTROLLER_H_
#define PICONTROLLER_H_

#include "Controller_base.h"

namespace controller
{

class PIController: public Controller_base
{
public:
   PIController(double max_vel_lin,
                double max_vel_ang,
                double lin_fac,
                double T,
                double T_n,
                double K_r)
   {
      _max_vel_lin  = max_vel_lin ;
      _max_vel_ang  = max_vel_ang ;
      _lin_fac      = lin_fac     ;
      _T            = T           ;
      _T_n          = T_n         ;
      _K_R          = K_r         ;

      _ang_err_old = 0;
      _ang_vel_old = 0;

   }
   virtual ~PIController()
   {}

   virtual controller::velocity control(double linear, double angular)
   {
      controller::velocity vel;

      int sng = angular > 0 ? 1 : -1;

      double ang = this->pi_control(angular);


      vel.angular = std::abs(ang) > _max_vel_ang ? _max_vel_ang * sng : ang;
      vel.linear = this->lin(_lin_fac, linear, _max_vel_lin);
      _ang_err_old = angular;
      _ang_vel_old = ang;

      return vel;
   }

private: //functions
   double lin(double scale, double value, double max_value)
   {
      max_value = std::abs(max_value);
      int sng = value > 0 ? 1 : -1;
      double tmp = 0;
      tmp = scale * value;
      return std::abs(tmp) > max_value ? (max_value * sng) : tmp;
   }

   double pi_control(double error)
   {
      return _ang_vel_old + _K_R * error - (_K_R* (1 - (_T/_T_n))) * _ang_err_old;
   }

private:
   double _max_vel_lin;
   double _max_vel_ang;
   double _lin_fac;

   double _T;
   double _T_n;
   double _K_R;

   double _ang_err_old;
   double _ang_vel_old;

};


} /* namespace controller */

#endif /* PICONTROLLER_H_ */
