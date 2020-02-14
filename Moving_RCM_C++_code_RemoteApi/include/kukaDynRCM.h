
/**
 * \file      kukaDynRCM.h
 *
 * \brief     C++ Header File template
 */

#ifndef _KUKA_DYN_RCM_
#define _KUKA_DYN_RCM_

/*******************************************************************************
 *       I N C L U D E - F I L E S
 ******************************************************************************/

#include <Eigen/Dense>
#include "common.h"


/*******************************************************************************
 *       N A M E S P A C E
 ******************************************************************************/
// static void rhs(dvec& x, dvec& dxdt, const double t );
// static void write_cout(dvec& x, const double t );

namespace DYN{
/*
 * \Struct holding the kinematic parameters
 */
struct Params{
  double gain, Xtr, Ytr, Ztr, lambda_desired, lambda, err_squaredNorm, z_axis_offset;
  int ee_length;
  int clientID;
  vecxd lbrJoint;
  vecxd target_pos;
  vecxd q, qd, qdd;
  MatXd k;
  vecxd p6, p7, rcm;
  Params() : qd(7), qdd(7), clientID(0), gain(0), Xtr(0), Ytr(0), Ztr(0), lambda_desired(0), lambda(0), err_squaredNorm(0), z_axis_offset(0), k(6,6), q(8), p6(3), p7(3), rcm(3), lbrJoint(7), target_pos(3){}
};

/**
 * \class kukaDyn
 *
 * \Calculates the direct kinematics and jacobian matrices of the kuka LWR for
 * \the 6th and 7th joints
 *
 */

class RCM{
  public:
    static vecxd kukaDynRCM(const vecxd& q, Params& kinParams);
  }; // class kukaDyn

} //namespace
#endif // _KUKA_DYN_RCM_
