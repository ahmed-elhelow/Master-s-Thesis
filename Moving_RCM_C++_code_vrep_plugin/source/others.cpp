#include <iostream>
#include "common.h"
#include "forces.h"
#include "kukaDynRCM.h"

using namespace std;

DYN::inertiaMat inertiaMat;

void DYN::RESIDUAL::NewEul_Aux(Params& kinParams, dynamicParams& dynParams, residualParams& residual, inertiaMat& iMat, vecxd q, vecxd qd, vecxd qdd, vecxd g){
  NewEulForw(kinParams, dynParams, residual, iMat, q, qd, qdd, g);

  residual.tau = NewEulBack(kinParams, dynParams, residual, iMat, qd, qdd);
}

void DYN::RESIDUAL::NewEulForw(Params& kinParams, dynamicParams& dynParams, residualParams& residual, inertiaMat& iMat, vecxd q, vecxd qd, vecxd qdd, vecxd g){

  KukaKinematics(kinParams, dynParams, residual, iMat, q);

  vec3d prev_w(0,0,0);
  vec3d prev_wd(0,0,0);
  vec3d prev_pdd(0,0,0);

  prev_pdd = prev_pdd - g;

  vec3d curr_w(0,0,0);
  vec3d curr_wd(0,0,0);
  vec3d curr_pdd(0,0,0);

  vec3d curr_pcdd(0,0,0);
  vec3d curr_wmd(0,0,0);

   // Forward phase
   vec3d z0(0, 0, 1);
   for (int i=0; i<dynParams.N; i++){
     /*
      * \Vector from origin of frame i to the center of mass of link i
      * \(negative quantity siciliano's book)
      */
     vec3d rici(dynParams.r_i_ci.block<3,1>(0,i));

     /*
      * \Vector from the origin of frame j-1 (frame of link i) to the origin of
      * \frame j (j=i+1=i) from siciliano's book
      */
     vec3d rij(residual.j_r_ij.block<3,1>(0,i));
     /*
      * \R from frame i to frame i-1 (1->0, 2->1,...) we'll use its inverse
      */
     MatXd R(3,3);
     R = residual.Rot[i];
     /*
      * \Angular velocity of link i w.r.t frame i. (Equation 7.107 (Siciliano))
      */
     MatXd Rt(3,3);
     Rt = R.transpose();
     // angular velocity of link i expressed in frame i
     curr_w = Rt*(prev_w+qd(i)*z0);
     /*
      * \Angular acceleration w.r.t frame i. (Equation 7.108 (Siciliano))
      */
     // angular acceleration of link i expressed in frame i
     curr_wd = Rt*(prev_wd+qdd(i)*z0+(qd(i)*prev_w).cross(z0));
     /*
      * \Linear acceleration w.r.t frame i (Equation 7.109 (Siciliano))
      */
     curr_pdd = Rt*prev_pdd+curr_wd.cross(rij)+curr_w.cross(curr_w.cross(rij));
     /*
      * \linear acceleration of link i expressed in frame i (Equation 7.110(Siciliano))
      */
     curr_pcdd = curr_pdd+curr_wd.cross(rici)+curr_w.cross(curr_w.cross(rici));
     /*
      * \Angular acceleration of the rotor i w.r.t frame i-1 (Equation 7.111 (Siciliano))
      */
     /*
      * \List of velocities and accelerations to pass to the backward step
      */
     residual.Rot[i] = R;
     residual.W[i] = curr_w;
     residual.Wd[i] = curr_wd;
     residual.Pdd[i] = curr_pdd;
     residual.Pcdd[i] = curr_pcdd;
     residual.Wmd[i] = curr_wmd;
     residual.Rij[i] = rij;
     residual.Rici[i] = rici;
     prev_w = curr_w;
     prev_wd = curr_wd;
     prev_pdd = curr_pdd;
   }
}

vecxd DYN::RESIDUAL::NewEulBack(Params& kinParams, dynamicParams& dynParams, residualParams& residual, inertiaMat& iMat, vecxd qd_N, vecxd qdd_N){

  residual.I.push_back(iMat.I1);
  residual.I.push_back(iMat.I2);
  residual.I.push_back(iMat.I3);
  residual.I.push_back(iMat.I4);
  residual.I.push_back(iMat.I5);
  residual.I.push_back(iMat.I6);
  residual.I.push_back(iMat.I7);

  // Backwards
  vec3d z0(0,0,1);
  vec3d prev_f(0,0,0);
  vec3d prev_u(0,0,0);
  vec3d curr_f(0,0,0);
  vec3d curr_u(0,0,0);

  vecxd tau(dynParams.N);
  tau.setZero();

  vecxd qd(dynParams.N+1);
  qd << qd_N, 0;
  vecxd qdd(dynParams.N+1);
  qdd << qdd_N, 0;

  for (int i=dynParams.N-1; i>-1; i--){
    // Rotation matrix
    MatXd R_ahead(3,3);
    R_ahead = residual.Rot[i+1];
    // Rotation matrix transpose
    MatXd R(3,3);
    R = residual.Rot[i];
    // Linear acceleration
    vec3d pcdd(0,0,0);
    pcdd = residual.Pcdd[i];

    vec3d rij(0,0,0);
    rij = residual.Rij[i];

    vec3d rici(0,0,0);
    rici = residual.Rici[i];
    // Angular velocity
    vec3d w(0,0,0);
    w = residual.W[i];
    // Angular acceleration
    vec3d wd(0,0,0);
    wd = residual.Wd[i];
    // Rotor angular acceleration
    vec3d wmd(0,0,0);
    wmd = residual.Wmd[i];
    /*
     * \Force Equation (siciliano 7.112)
     */
    curr_f = R_ahead*prev_f + (dynParams.masses[i]*pcdd);

    vec3d prev_f_(0,0,0);
    prev_f_ = R_ahead*prev_f;
    curr_u = (R_ahead*prev_u)-curr_f.cross(rij+rici)+prev_f_.cross(rici)+(residual.I[i]*wd)+w.cross(residual.I[i]*w);

    /*
     * \Equation 7.114 for a revolute joint(Siciliano)
     */

    tau[i] = curr_u.transpose()*(R.transpose()*z0)+(dynParams.fv*qd[i]+dynParams.fc*sign(qd[i]));

    prev_f = curr_f;
    prev_u = curr_u;
  }

  return tau;
}

void DYN::RESIDUAL::KukaKinematics(Params& kinParams, dynamicParams& dynParams, residualParams& residual, inertiaMat& iMat, vecxd q){

  // q << 0, 0, 0, 0, 0, 0, 0;
  double lambda = kinParams.lambda;
  double q1=q(0), q2=q(1), q3=q(2), q4=q(3), q5=q(4), q6=q(5), q7=q(6);
  double s1=sin(q1), s2=sin(q2), s3=sin(q3), s4=sin(q4), s5=sin(q5), s6=sin(q6), s7=sin(q7);
  double c1=cos(q1), c2=cos(q2), c3=cos(q3), c4=cos(q4), c5=cos(q5), c6=cos(q6), c7=cos(q7);

  double d1, d3, d5, d7;
  d1 = 0.0;
  d3 = 0.4;
  d5 = 0.39;
  d7 = 0.078 + 0.23;

  MatXd a0(4,4);

  MatXd a1(4,4);
  a1 << c1, 0,  s1, 0,
        s1, 0, -c1, 0,
        0,  1,  0,  d1,
        0,  0,  0,  1;

  MatXd a2(4,4);
  a2 << c2, 0, -s2, 0,
        s2, 0,  c2, 0,
        0, -1,  0,  0,
        0,  0,  0,  1;

  MatXd a3(4,4);
  a3 << c3, 0, -s3, 0,
        s3, 0,  c3, 0,
        0, -1,  0,  d3,
        0,  0,  0,  1;

  MatXd a4(4,4);
  a4 << c4, 0,  s4, 0,
        s4, 0, -c4, 0,
        0,  1,  0,  0,
        0,  0,  0,  1;

  MatXd a5(4,4);
  a5 << c5, 0,  s5, 0,
        s5, 0, -c5, 0,
        0,  1,  0,  d5,
        0,  0,  0,  1;

  MatXd a6(4,4);
  a6 << c6, 0, -s6, 0,
        s6, 0,  c6, 0,
        0, -1,  0,  0,
        0,  0,  0,  1;

  MatXd a7(4,4);
  a7 << c7, -s7, 0,  0,
        s7,  c7, 0,  0,
        0,   0,  1,  d7,
        0,   0,  0,  1;

  MatXd kwf1(4,4);
  MatXd kwf2(4,4);
  MatXd kwf3(4,4);
  MatXd kwf4(4,4);
  MatXd kwf5(4,4);
  MatXd kwf6(4,4);
  MatXd kwf7(4,4);

  kwf1 = a1;
  kwf2 = kwf1*a2;
  kwf3 = kwf2*a3;
  kwf4 = kwf3*a4;
  kwf5 = kwf4*a5;
  kwf6 = kwf5*a6;
  kwf7 = kwf6*a7;

  // a0 << -1,  0, 0, -0.0000807,
  //        0, -1, 0,  0.0001082,
  //        0,  0, 1,  0.3105,
  //        0,  0, 0,  1;
  //
  // a7 << c7, -s7, 0,  0,
  //       s7,  c7, 0,  0,
  //       0,   0,  1,  0,
  //       0,   0,  0,  1;
  //
  // int i = 0;
  // vecxd rici(3), rici_hom(4), rici_abs(4);
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf1*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // i = 1;
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf2*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // i = 2;
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf3*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // i = 3;
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf4*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // i = 4;
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf5*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // i = 5;
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf6*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // i = 6;
  // rici << dynParams.r_i_ci.block<3,1>(0,i);
  // rici_hom << rici(0), rici(1), rici(2), 1;
  // rici_abs = a0*kwf7*rici_hom;
  // cout << "rici_abs " << i+1 << endl;
  // cout << rici_abs << endl;
  //
  // rici_abs 1
  // 0.0215581
  //  -0.03758
  //    0.3205
  // rici_abs 2
  // -0.000409175
  //   0.00422142
  //     0.392865
  // rici_abs 3
  // -0.000340033
  //  0.000208457
  //     0.596757
  // rici_abs 4
  // 0.00138418
  //  0.0001543
  //   0.859081
  // rici_abs 5
  // 0.000298448
  //  -0.0100173
  //     1.04515
  // rici_abs 6
  // -0.0021546
  // -0.0585103
  //     1.0557
  // rici_abs 7
  //  0.00037943
  // -0.00137072
  //     1.17206

  // // % my Generalized Steiner(T,I,m)
  // // wT0 = [-1 0 0 -0.0000807;0 -1 0 0.0001082;0 0 1 0.3105;0 0 0 1];
  // // T = wT0*0Ti;
  // // R = T(1:3,1:3);
  // // r = T(1:3,4);
  // // J = I + m*(r'*r*eye(3)-r*r');
  // // J = R*J*R';
  //
  // a0 << -1,  0, 0, -0.0000807,
  //        0, -1, 0,  0.0001082,
  //        0,  0, 1,  0.3105,
  //        0,  0, 0,  1;
  //
  // a7 << c7, -s7, 0,  0,
  //       s7,  c7, 0,  0,
  //       0,   0,  1,  0,
  //       0,   0,  0,  1;
  //
  // MatXd T1(4,4);
  // MatXd T2(4,4);
  // MatXd T3(4,4);
  // MatXd T4(4,4);
  // MatXd T5(4,4);
  // MatXd T6(4,4);
  // MatXd T7(4,4);
  //
  // T1 = a0*a1;
  // T2 = T1*a2;
  // T3 = T2*a3;
  // T4 = T3*a4;
  // T5 = T4*a5;
  // T6 = T5*a6;
  // T7 = T6*a7;
  //
  // // R = T(1:3,1:3);
  // MatXd rot1(3,3);
  // rot1 << T1.block<3,3>(0,0);
  // MatXd rot2(3,3);
  // rot2 << T2.block<3,3>(0,0);
  // MatXd rot3(3,3);
  // rot3 << T3.block<3,3>(0,0);
  // MatXd rot4(3,3);
  // rot4 << T4.block<3,3>(0,0);
  // MatXd rot5(3,3);
  // rot5 << T5.block<3,3>(0,0);
  // MatXd rot6(3,3);
  // rot6 << T6.block<3,3>(0,0);
  // MatXd rot7(3,3);
  // rot7 << T7.block<3,3>(0,0);
  //
  // // r = T(1:3,4);
  // vec3d pos1;
  // vec3d pos2;
  // vec3d pos3;
  // vec3d pos4;
  // vec3d pos5;
  // vec3d pos6;
  // vec3d pos7;
  // pos1 << dynParams.r_i_ci.block<3,1>(0,0);
  // pos2 << dynParams.r_i_ci.block<3,1>(0,1);
  // pos3 << dynParams.r_i_ci.block<3,1>(0,2);
  // pos4 << dynParams.r_i_ci.block<3,1>(0,3);
  // pos5 << dynParams.r_i_ci.block<3,1>(0,4);
  // pos6 << dynParams.r_i_ci.block<3,1>(0,5);
  // pos7 << dynParams.r_i_ci.block<3,1>(0,6);
  //
  // // J = I + m*(r'*r*eye(3)-r*r');
  // // J = R*J*R';
  // MatXd Imod1_(3,3);
  // MatXd Imod2_(3,3);
  // MatXd Imod3_(3,3);
  // MatXd Imod4_(3,3);
  // MatXd Imod5_(3,3);
  // MatXd Imod6_(3,3);
  // MatXd Imod7_(3,3);
  // MatXd Imod1(3,3);
  // MatXd Imod2(3,3);
  // MatXd Imod3(3,3);
  // MatXd Imod4(3,3);
  // MatXd Imod5(3,3);
  // MatXd Imod6(3,3);
  // MatXd Imod7(3,3);
  // Imod1_ = iMat.I1 + dynParams.masses[0]*(pos1.transpose()*pos1*MatXd::Identity(3,3)-pos1*pos1.transpose());
  // Imod2_ = iMat.I2 + dynParams.masses[1]*(pos2.transpose()*pos2*MatXd::Identity(3,3)-pos2*pos2.transpose());
  // Imod3_ = iMat.I3 + dynParams.masses[2]*(pos3.transpose()*pos3*MatXd::Identity(3,3)-pos3*pos3.transpose());
  // Imod4_ = iMat.I4 + dynParams.masses[3]*(pos4.transpose()*pos4*MatXd::Identity(3,3)-pos4*pos4.transpose());
  // Imod5_ = iMat.I5 + dynParams.masses[4]*(pos5.transpose()*pos5*MatXd::Identity(3,3)-pos5*pos5.transpose());
  // Imod6_ = iMat.I6 + dynParams.masses[5]*(pos6.transpose()*pos6*MatXd::Identity(3,3)-pos6*pos6.transpose());
  // Imod7_ = iMat.I7 + dynParams.masses[6]*(pos7.transpose()*pos7*MatXd::Identity(3,3)-pos7*pos7.transpose());
  // Imod1 = (rot1*Imod1_*rot1.transpose())/dynParams.masses[0];
  // Imod2 = (rot2*Imod2_*rot2.transpose())/dynParams.masses[1];
  // Imod3 = (rot3*Imod3_*rot3.transpose())/dynParams.masses[2];
  // Imod4 = (rot4*Imod4_*rot4.transpose())/dynParams.masses[3];
  // Imod5 = (rot5*Imod5_*rot5.transpose())/dynParams.masses[4];
  // Imod6 = (rot6*Imod6_*rot6.transpose())/dynParams.masses[5];
  // Imod7 = (rot7*Imod7_*rot7.transpose())/dynParams.masses[6];
  // // Imod1 = (Imod1_)/dynParams.masses[0];
  // // Imod2 = (Imod2_)/dynParams.masses[1];
  // // Imod3 = (Imod3_)/dynParams.masses[2];
  // // Imod4 = (Imod4_)/dynParams.masses[3];
  // // Imod5 = (Imod5_)/dynParams.masses[4];
  // // Imod6 = (Imod6_)/dynParams.masses[5];
  // // Imod7 = (Imod7_)/dynParams.masses[6];
  // // Imod1 = (rot1*iMat.I1*rot1.transpose())/dynParams.masses[0];
  // // Imod2 = (rot2*iMat.I2*rot2.transpose())/dynParams.masses[1];
  // // Imod3 = (rot3*iMat.I3*rot3.transpose())/dynParams.masses[2];
  // // Imod4 = (rot4*iMat.I4*rot4.transpose())/dynParams.masses[3];
  // // Imod5 = (rot5*iMat.I5*rot5.transpose())/dynParams.masses[4];
  // // Imod6 = (rot6*iMat.I6*rot6.transpose())/dynParams.masses[5];
  // // Imod7 = (rot7*iMat.I7*rot7.transpose())/dynParams.masses[6];
  // // Imod1 = (iMat.I1)/dynParams.masses[0];
  // // Imod2 = (iMat.I2)/dynParams.masses[1];
  // // Imod3 = (iMat.I3)/dynParams.masses[2];
  // // Imod4 = (iMat.I4)/dynParams.masses[3];
  // // Imod5 = (iMat.I5)/dynParams.masses[4];
  // // Imod6 = (iMat.I6)/dynParams.masses[5];
  // // Imod7 = (iMat.I7)/dynParams.masses[6];
  //
  // cout << "Imod1" << endl;
  // cout << Imod1 << endl;
  // cout << "Imod2" << endl;
  // cout << Imod2 << endl;
  // cout << "Imod3" << endl;
  // cout << Imod3 << endl;
  // cout << "Imod4" << endl;
  // cout << Imod4 << endl;
  // cout << "Imod5" << endl;
  // cout << Imod5 << endl;
  // cout << "Imod6" << endl;
  // cout << Imod6 << endl;
  // cout << "Imod7" << endl;
  // cout << Imod7 << endl;
  // cin.get();

  vec3d p1 = kwf1.col(3).head(3);
  vec3d p2 = kwf2.col(3).head(3);
  vec3d p3 = kwf3.col(3).head(3);
  vec3d p4 = kwf4.col(3).head(3);
  vec3d p5 = kwf5.col(3).head(3);
  vec3d p6 = kwf6.col(3).head(3);
  vec3d p7 = kwf7.col(3).head(3);

  MatXd r1(3,3);
  r1 << a1.block<3,3>(0,0);

  MatXd r2(3,3);
  r2 << a2.block<3,3>(0,0);

  MatXd r3(3,3);
  r3 << a3.block<3,3>(0,0);

  MatXd r4(3,3);
  r4 << a4.block<3,3>(0,0);

  MatXd r5(3,3);
  r5 << a5.block<3,3>(0,0);

  MatXd r6(3,3);
  r6 << a6.block<3,3>(0,0);

  MatXd r7(3,3);
  r7 << a7.block<3,3>(0,0);

  MatXd r8(3,3);
  r8 = MatXd::Identity(3,3);

  residual.Rot.push_back(r1);
  residual.Rot.push_back(r2);
  residual.Rot.push_back(r3);
  residual.Rot.push_back(r4);
  residual.Rot.push_back(r5);
  residual.Rot.push_back(r6);
  residual.Rot.push_back(r7);
  residual.Rot.push_back(r8);

  residual.A.push_back(a1);
  residual.A.push_back(a2);
  residual.A.push_back(a3);
  residual.A.push_back(a4);
  residual.A.push_back(a5);
  residual.A.push_back(a6);
  residual.A.push_back(a7);


  MatXd rwf1;
  rwf1 = r1;
  MatXd rwf2;
  rwf2 = rwf1*r2;
  MatXd rwf3;
  rwf3 = rwf2*r3;
  MatXd rwf4;
  rwf4 = rwf3*r4;
  MatXd rwf5;
  rwf5 = rwf4*r5;
  MatXd rwf6;
  rwf6 = rwf5*r6;
  MatXd rwf7;
  rwf7 = rwf6*r7;

  residual.rwf7 = rwf7;

  a0 << -1,  0, 0, -0.0000807,
         0, -1, 0,  0.0001082,
         0,  0, 1,  0.3105,
         0,  0, 0,  1;

  residual.rwf0 << a0.block<3,3>(0,0);

  vec3d stat(0,0,1);
  vec3d z_0(0,0,1);
  vec3d z_1;
  z_1 = rwf1*stat;
  vec3d z_2;
  z_2 = rwf2*stat;
  vec3d z_3;
  z_3 = rwf3*stat;
  vec3d z_4;
  z_4 = rwf4*stat;
  vec3d z_5;
  z_5 = rwf5*stat;

  // considering that the end effector is at joint 6
  vec3d p_6_e = p6;

  vec3d p_6_e_0 = p_6_e;
  vec3d p_6_e_1 = p_6_e - p1;
  vec3d p_6_e_2 = p_6_e - p2;
  vec3d p_6_e_3 = p_6_e - p3;
  vec3d p_6_e_4 = p_6_e - p4;
  vec3d p_6_e_5 = p_6_e - p5;

  vec3d J6_empty(0,0,0);

  MatXd linJ6(3,7);
  MatXd angJ6(3,7);
  linJ6.col(0) = z_0.cross(p_6_e_0);
  linJ6.col(1) = z_1.cross(p_6_e_1);
  linJ6.col(2) = z_2.cross(p_6_e_2);
  linJ6.col(3) = z_3.cross(p_6_e_3);
  linJ6.col(4) = z_4.cross(p_6_e_4);
  linJ6.col(5) = z_5.cross(p_6_e_5);
  linJ6.col(6) = J6_empty;
  angJ6.col(0) = z_0;
  angJ6.col(1) = z_1;
  angJ6.col(2) = z_2;
  angJ6.col(3) = z_3;
  angJ6.col(4) = z_4;
  angJ6.col(5) = z_5;
  angJ6.col(6) = J6_empty;

  MatXd J6(6,7);
  J6.block<3,7>(0,0) = linJ6;
  J6.block<3,7>(3,0) = angJ6;

  vec3d z_6;
  z_6 = rwf6*stat;

  vec3d p_7_e = p7;

  vec3d p_7_e_0 = p_7_e;
  vec3d p_7_e_1 = p_7_e - p1;
  vec3d p_7_e_2 = p_7_e - p2;
  vec3d p_7_e_3 = p_7_e - p3;
  vec3d p_7_e_4 = p_7_e - p4;
  vec3d p_7_e_5 = p_7_e - p5;
  vec3d p_7_e_6 = p_7_e - p6;

  MatXd linJ7(3,7);
  MatXd angJ7(3,7);
  linJ7.col(0) = z_0.cross(p_7_e_0);
  linJ7.col(1) = z_1.cross(p_7_e_1);
  linJ7.col(2) = z_2.cross(p_7_e_2);
  linJ7.col(3) = z_3.cross(p_7_e_3);
  linJ7.col(4) = z_4.cross(p_7_e_4);
  linJ7.col(5) = z_5.cross(p_7_e_5);
  linJ7.col(6) = z_6.cross(p_7_e_6);
  angJ7.col(0) = z_0;
  angJ7.col(1) = z_1;
  angJ7.col(2) = z_2;
  angJ7.col(3) = z_3;
  angJ7.col(4) = z_4;
  angJ7.col(5) = z_5;
  angJ7.col(6) = z_6;

  MatXd J7(6,7);
  J7.block<3,7>(0,0) = linJ7;
  J7.block<3,7>(3,0) = angJ7;

  // Jacobian RCM
  MatXd J_RCM(6,7); // 3x7 matrix
  J_RCM = J6+lambda*(J7-J6);

  residual.J_RCM = J_RCM;

  residual.Z.block<3,1>(0,0) = z_0;
  residual.Z.block<3,1>(0,1) = z_1;
  residual.Z.block<3,1>(0,2) = z_2;
  residual.Z.block<3,1>(0,3) = z_3;
  residual.Z.block<3,1>(0,4) = z_4;
  residual.Z.block<3,1>(0,5) = z_5;
  residual.Z.block<3,1>(0,6) = z_6;

  vec3d r_1_01(0,0,0);
  vec3d r_2_12(0,0,0);
  vec3d r_3_23(0,-d3,0);
  vec3d r_4_34(0,0,0);
  vec3d r_5_45(0,d5,0);
  vec3d r_6_56(0,0,0);
  vec3d r_7_67(0,0,0);

  residual.j_r_ij.block<3,1>(0,0) = r_1_01;
  residual.j_r_ij.block<3,1>(0,1) = r_2_12;
  residual.j_r_ij.block<3,1>(0,2) = r_3_23;
  residual.j_r_ij.block<3,1>(0,3) = r_4_34;
  residual.j_r_ij.block<3,1>(0,4) = r_5_45;
  residual.j_r_ij.block<3,1>(0,5) = r_6_56;
  residual.j_r_ij.block<3,1>(0,6) = r_7_67;
}
