
#include <iostream>
#include "forces.h"
#include "kukaDynRCM.h"
using namespace std;

void DYN::FORCE::external_forces_estimation(Params& kinParams,
                                            dynamicParams& dynParams,
                                            residualParams& residual,
                                            inertiaMat& iMat, vecxd q, vecxd qd, vecxd qdd){

  int N=dynParams.N;
  double fv=dynParams.fv, fc=dynParams.fc;
  vecxd tau_act(dynParams.N), fv_vec(7), masses(7), tau_ext(N), beta(N);
  fv_vec = dynParams.fv_vec;
  masses = dynParams.masses;
  vecxd g(3);
  vecxd q_tmp(7);
  vecxd qd_tmp(7);
  vecxd qdd_tmp(7);
  vecxd n(7);
  vecxd g_n(7);
  MatXd B_old(7,7);
  MatXd B(7,7);
  vecxd bi(7);

  q_tmp = q;
  qd_tmp = qd;
  qdd_tmp.setZero();
  g = residual.g;
  DYN::RESIDUAL::NewEul_Aux(kinParams, dynParams, residual, iMat, q_tmp, qd_tmp, qdd_tmp, g);
  n = residual.tau;
  residual.n = n;

  q_tmp = q;
  qd_tmp.setZero();
  qdd_tmp.setZero();
  g = residual.g;
  DYN::RESIDUAL::NewEul_Aux(kinParams, dynParams, residual, iMat, q_tmp, qd_tmp, qdd_tmp, g);
  g_n = residual.tau;
  residual.g_n = g_n;

  B.setZero();
  q_tmp = q;
  qd_tmp.setZero();
  g.setZero();
  for (int i=0; i<7; i++){
    qdd_tmp.setZero();
    qdd_tmp(i) = 1;
    DYN::RESIDUAL::NewEul_Aux(kinParams, dynParams, residual, iMat, q_tmp, qd_tmp, qdd_tmp, g);
    bi = residual.tau;
    B.block<7,1>(0,i) << bi;
  }
  residual.B = B;

  q_tmp = q;
  qd_tmp = qd;
  qdd_tmp.setZero();
  g.setZero();
  DYN::RESIDUAL::NewEul_Aux(kinParams, dynParams, residual, iMat, q_tmp, qd_tmp, qdd_tmp, g);
  residual.c = residual.tau;
}
