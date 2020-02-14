// #define NON_MATLAB_PARSING
// #define MAX_EXT_API_CONNECTIONS 255
// #define DO_NOT_USE_SHARED_MEMORY


// #include "v_repExtKukaLwr.h"
// #include "scriptFunctionData.h"
// #include <iostream>
// #include "v_repLib.h"
// #include <unistd.h>
//
// // uncomment to recognize the sleep function on windows
// // #include "windows.h"
// #include <chrono>
// #include <vector>
//
// // #include <yaml-cpp/yaml.h>
// #include <sstream>
// #include <fstream>
// #include "kukaDynRCM.h"
// #include "forces.h"
// #include "common.h"
// #include <math.h>
// // #include <extApi.h>
// // #include <extApiPlatform.h>
// // #include <extApi.c>
// // #include <extApiPlatform.c>
//
// // #include "matplotlibcpp.h"
// #include <algorithm> // for std::iota
//
// #include <iostream>
// #include <boost/numeric/odeint.hpp>
//
// #include <vector>
// #include </usr/local/include/eigen3/Eigen/QR>
// #include </usr/local/include/eigen3/Eigen/Dense>
//
// #define PI 3.1415926535897932384626433832755
//
// // extern "C" {
// //   #include "extApi.h"
// // }
//
// using namespace std;
// using namespace boost::numeric::odeint;
//
// /* The type of container used to hold the state vector */
// typedef std::vector< double > state_type;

// // DYN::Params kinParams;
// // DYN::dynamicParams dynParams;
// // DYN::inertiaMat iMat;
// // DYN::residualParams residual;
//
// // Config file dir
// // static const std::string CONFIG_FILE = "../config/config.yaml";
// // Load the config file
// // YAML::Node params = YAML::LoadFile(CONFIG_FILE);
//
// // const int N_ = params["N"].as<int>();
// const int N_ = 7;
//
// // const double fv_ = params["fv"].as<double>();
// const double fv_ = 0;
//
// // const double fc_ = params["fc"].as<double>();
// const double fc_ = 0;
//
// // const double off_ = params["off"].as<double>();
// // Target position on the X axis
// // const double Xtr_ = params["Xtr"].as<double>();
// const double Xtr_ = 0.5;
// // Target position on the Y axis
// // const double Ytr_ = params["Ytr"].as<double>();
// const double Ytr_ = 0.5;
// // Target position on the Z axis
// // const double Ztr_ = params["Ztr"].as<double>();
// const double Ztr_ = 0.0;
// // Control gain
// // const double gain_ = params["gain"].as<double>();
// const double gain_ = 0.5;
// // residual gain
// // const double residual_gain_ = params["residual_gain"].as<double>();
// const double residual_gain_ = 1.5;
// // Robot's joints state vector
// // const dvec q_ = params["q"].as<dvec>();
// const dvec q_ = {0, 0, 0, 0, 0, 0, 0};
//
// // const dvec g_ = params["g"].as<dvec>();
// const dvec g_ = {0, 0, -9.81};
//
// // const dvec fv_vec_ = params["fv_vec"].as<dvec>();
// const dvec fv_vec_ = {0, 0, 0, 0, 0, 0, 0};
// // Masses vector
// // const dvec masses_ = params["masses"].as<dvec>();
// const dvec masses_ = {4.1948152162, 4.2996847737, 3.658530333, \
//       2.3846673548, 1.7035567183, 0.4000713156, 0.6501439811};
// // Robot links lengths in meters
// // const dvec lengths_ = params["lengths"].as<dvec>();
// // Robot Inertia matrix
// // const dvec inertiaVecx_ = params["inertiaVecx"].as<dvec>();
// const dvec inertiaVecx_ = {0.01,     0.000189,    0.01,      0.01,        0.01,        0.01,\
//                       0.04741,  0.05,        0.001601, -0.00000621,  0.0001166,  -0.000914,\
//                       0.046951, 0.0008344,   0.05,     -0.000577,    0.000409,   -0.000577,\
//                       0.012423, 0.007270,    0.009988, -0.000518,    0.00000002, -0.000548,\
//                       0.006322, 0.001202,    0.007080, -0.000216,    0.000006,   -0.005,\
//                       0.000527, 0,           0.003489,  0.000048,   -0.0000375,  -0.001030,\
//                       0,        0.000032,    0.000118, -0.00000005, -0.0000214,   0.000069};
// // r_i_ci
// // const dvec r_i_ci_ = params["r_i_ci"].as<dvec>();
// const dvec r_i_ci_ = {-0.0216387515,	 0.0003284751,	 0.0002593328,\
//     	-0.0014648843,	-0.0003791484,	 0.0020739022,	-0.0004601303,\
//       0.01,		      -0.0041132249,	 0.1137431845,	-0.0000461,	\
//       -0.0553526131,	 0.0586184696,	 0.0014789221, \
//       -0.0376881829,	 0.0823647642,	-0.000100257,	 0.148580959,	\
//       -0.0101255137,	-0.044799983,	   0.0715608282};

// vecxd DYN::RCM::kukaDynRCM(const vecxd& q, DYN::Params& kinParams){
//
//   double q1=q[0], q2=q[1], q3=q[2], q4=q[3], q5=q[4], q6=q[5], q7=q[6], lambda=q[7];
//   double s1=sin(q1), s2=sin(q2), s3=sin(q3), s4=sin(q4), s5=sin(q5), s6=sin(q6), s7=sin(q7);
//   double c1=cos(q1), c2=cos(q2), c3=cos(q3), c4=cos(q4), c5=cos(q5), c6=cos(q6), c7=cos(q7);
//
//   double ee_length=0.23;
//   double d1=0.0;
//   double d3=0.4;
//   double d5=0.39;
//   double d7=0.078 + ee_length;
//
//   // Transformations matrices
//   MatXd a1(4,4);
//   a1 << c1, 0,  s1, 0,
//         s1, 0, -c1, 0,
//         0,  1,  0,  d1,
//         0,  0,  0,  1;
//
//   MatXd a2(4,4);
//   a2 << c2,  0, -s2,  0,
//         s2,  0,  c2,  0,
//         0,  -1,   0,  0,
//         0,   0,   0,  1;
//
//   MatXd a3(4,4);
//   a3 << c3,  0, -s3,  0,
//         s3,  0,  c3,  0,
//         0,  -1,   0,  d3,
//         0,   0,   0,  1;
//
//   MatXd a4(4,4);
//   a4 << c4, 0,  s4,  0,
//         s4, 0, -c4,  0,
//         0,  1,   0,  0,
//         0,  0,   0,  1;
//
//   MatXd a5(4,4);
//   a5 << c5, 0,  s5, 0,
//         s5, 0, -c5, 0,
//         0,  1,   0, d5,
//         0,  0,   0, 1;
//
//   MatXd a6(4,4);
//   a6 << c6,  0, -s6,  0,
//         s6,  0,  c6,  0,
//         0,  -1,   0,  0,
//         0,   0,   0,  1;
//
//   MatXd a7(4,4);
//   a7 << c7, -s7, 0,  0,
//         s7,  c7, 0,  0,
//         0,    0, 1,  d7,
//         0,    0, 0,  1;
//
//   MatXd kwf1(4,4);
//   MatXd kwf2(4,4);
//   MatXd kwf3(4,4);
//   MatXd kwf4(4,4);
//   MatXd kwf5(4,4);
//   MatXd kwf6(4,4);
//   MatXd kwf7(4,4);
//
//   kwf1 = a1;
//   kwf2 = kwf1*a2;
//   kwf3 = kwf2*a3;
//   kwf4 = kwf3*a4;
//   kwf5 = kwf4*a5;
//   kwf6 = kwf5*a6;
//   kwf7 = kwf6*a7;
//
//   vec3d p1 = kwf1.col(3).head(3);
//   vec3d p2 = kwf2.col(3).head(3);
//   vec3d p3 = kwf3.col(3).head(3);
//   vec3d p4 = kwf4.col(3).head(3);
//   vec3d p5 = kwf5.col(3).head(3);
//   vec3d p6 = kwf6.col(3).head(3);
//   vec3d p7 = kwf7.col(3).head(3);
//
//   // Rotation matrices
//   MatXd r1(3,3);
//   r1 = a1.block<3,3>(0,0);
//
//   MatXd r2(3,3);
//   r2 = a2.block<3,3>(0,0);
//
//   MatXd r3(3,3);
//   r3 = a3.block<3,3>(0,0);
//
//   MatXd r4(3,3);
//   r4 = a4.block<3,3>(0,0);
//
//   MatXd r5(3,3);
//   r5 = a5.block<3,3>(0,0);
//
//   MatXd r6(3,3);
//   r6 = a6.block<3,3>(0,0);
//
//   MatXd r7(3,3);
//   r7 = a7.block<3,3>(0,0);
//
//   MatXd rwf1;
//   rwf1 = r1;
//   MatXd rwf2;
//   rwf2 = rwf1*r2;
//   MatXd rwf3;
//   rwf3 = rwf2*r3;
//   MatXd rwf4;
//   rwf4 = rwf3*r4;
//   MatXd rwf5;
//   rwf5 = rwf4*r5;
//   MatXd rwf6;
//   rwf6 = rwf5*r6;
//   MatXd rwf7;
//   rwf7 = rwf6*r7;
//
//   vec3d stat(0,0,1);
//   vec3d z_0(0,0,1);
//   vec3d z_1;
//   z_1 = rwf1*stat;
//   vec3d z_2;
//   z_2 = rwf2*stat;
//   vec3d z_3;
//   z_3 = rwf3*stat;
//   vec3d z_4;
//   z_4 = rwf4*stat;
//   vec3d z_5;
//   z_5 = rwf5*stat;
//
//   // considering that the end effector is at joint 6
//   vec3d p_6_e = p6;
//
//   vec3d p_6_e_0 = p_6_e;
//   vec3d p_6_e_1 = p_6_e - p1;
//   vec3d p_6_e_2 = p_6_e - p2;
//   vec3d p_6_e_3 = p_6_e - p3;
//   vec3d p_6_e_4 = p_6_e - p4;
//   vec3d p_6_e_5 = p_6_e - p5;
//
//   vec3d linJ6vec6(0,0,0);
//
//   MatXd linJ6(3,7);
//   linJ6.col(0) = z_0.cross(p_6_e_0);
//   linJ6.col(1) = z_1.cross(p_6_e_1);
//   linJ6.col(2) = z_2.cross(p_6_e_2);
//   linJ6.col(3) = z_3.cross(p_6_e_3);
//   linJ6.col(4) = z_4.cross(p_6_e_4);
//   linJ6.col(5) = z_5.cross(p_6_e_5);
//   linJ6.col(6) = linJ6vec6;
//
//   vec3d z_6;
//   z_6 = rwf6*stat;
//
//   vec3d p_7_e = p7;
//
//   vec3d p_7_e_0 = p_7_e;
//   vec3d p_7_e_1 = p_7_e - p1;
//   vec3d p_7_e_2 = p_7_e - p2;
//   vec3d p_7_e_3 = p_7_e - p3;
//   vec3d p_7_e_4 = p_7_e - p4;
//   vec3d p_7_e_5 = p_7_e - p5;
//   vec3d p_7_e_6 = p_7_e - p6;
//
//   MatXd linJ7(3,7);
//   linJ7.col(0) = z_0.cross(p_7_e_0);
//   linJ7.col(1) = z_1.cross(p_7_e_1);
//   linJ7.col(2) = z_2.cross(p_7_e_2);
//   linJ7.col(3) = z_3.cross(p_7_e_3);
//   linJ7.col(4) = z_4.cross(p_7_e_4);
//   linJ7.col(5) = z_5.cross(p_7_e_5);
//   linJ7.col(6) = z_6.cross(p_7_e_5);
//
//   if(lambda<0) lambda=0;
//
//   // Jacobian RCM
//   MatXd J_lambda(3,7); // 3x7 matrix
//   J_lambda = linJ6+lambda*(linJ7-linJ6);
//
//   vec3d g;
//   g=p7-p6; // 3D vector
//
//   MatXd J_rCM(3,8); // 3x8 matrix composed by the matrix J_lambda and vector g
//   J_rCM.block<3,7>(0,0)=J_lambda; // First 3x7 block
//   J_rCM.block<3,1>(0,7)=g; // 3x1 vector on the 8th column
//
//   // Task Jacobian
//   MatXd Jt(3,8); // 3x8 matrix
//   Jt.block<3,7>(0,0)=linJ6; // First 3x7 block
//
//   vec3d Jtvec7(0,0,0);
//   Jt.col(7) = Jtvec7;
//
//   // Extention Jacobian 6x8 it includes the J_RCM and Jt (the task Jacobian)
//   MatXd J_a(6,8);
//   // Eigen::Matrix<double,6,8> J_a;
//   J_a.block<3,8>(0,0)=J_rCM;
//   J_a.block<3,8>(3,0)=Jt;
//
//   double x_rcm=p6[0]+lambda*(p7[0]-p6[0]);
//   double y_rcm=p6[1]+lambda*(p7[1]-p6[1]);
//   double z_rcm=p6[2]+lambda*(p7[2]-p6[2]);
//
//   kinParams.p6 << p6[0], p6[1], p6[2];
//   kinParams.p7 << p7[0], p7[1], p7[2];
//   kinParams.rcm << x_rcm, y_rcm, z_rcm;
//
//   // kinParams.target_pos[2] = -0.3105 since joint1_z is shifted-up by -0.3105
//   // Ask about the target_pos
//   // kinParams.Ztr=-0.3105;
//   double z_axis_diff=sqrt(pow(0.078+ee_length,2)-pow((kinParams.target_pos[0]-kinParams.Xtr),2)-pow((kinParams.target_pos[1]-kinParams.Ytr),2));
//   double z_axis_offset = kinParams.z_axis_offset;
//   vec3d err_rcm(kinParams.target_pos[0]-x_rcm, kinParams.target_pos[1]-y_rcm, (kinParams.target_pos[2]+z_axis_offset-z_rcm));
//   vec3d err_t(kinParams.Xtr-p6[0], kinParams.Ytr-p6[1], (kinParams.Ztr+z_axis_diff*kinParams.lambda_desired+z_axis_offset-p6[2]));
//
//   kinParams.lambda = lambda;
//
//   MatXd err(6,1); // 6x1 matrix
//   err.block<3,1>(0,0)=err_rcm;
//   err.block<3,1>(3,0)=err_t;
//
//   kinParams.err_squaredNorm = err.squaredNorm();
//
//   // Position and orientation control with RCM constraint
//   MatXd identity = MatXd::Identity(8,8);
//   // Jacobian pseudoinverse
//   MatXd Jp(8,6);
//   Jp = J_a.completeOrthogonalDecomposition().pseudoInverse(); // 8x6 matrix
//   double err_lam=kinParams.lambda_desired-lambda;
//
//   vecxd w(8);
//   w[0]=-(1/8)*q1/5.9;
//   w[1]=-(1/8)*q2/4.2;
//   w[2]=-(1/8)*q3/5.9;
//   w[3]=-(1/8)*q4/4.2;
//   w[4]=-(1/8)*q5/5.9;
//   w[5]=-(1/8)*q6/4.2;
//   w[6]=-(1/8)*q7/5.9;
//   w[7]=-(1/8)*(lambda-0.5)/1;
//   w[7]+=kinParams.gain*err_lam;
//
//   vecxd u(8);
//   u.setZero();
//
//   // // 8x6 * 6x6 * 6x1 + (8x8-(6x8*8x6)) * 8x1 // dimensions to be checked
//   u=Jp*kinParams.k*err+(identity-Jp*J_a)*(w);
//
//   double u_max = 0.03, u_min = -0.03;
//   for(int i=0;i<8;i++)
//   {
//       if(u[i]>u_max) u[i]=u_max;
//       if(u[i]<u_min) u[i]=u_min;
//   }
//
//   vecxd q_dot(8);
//   q_dot.setZero();
//
//   q_dot << u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7];
//
//   return q_dot;
// }
//
// void lorenz( const state_type &x , state_type &dx ,  double t)
// {
//     int clientID = kinParams.clientID;
//     simFloat q_0=0, q_1=0, q_2=0, q_3=0, q_4=0, q_5=0, q_6=0;
//
//     vecxd q(7);
//     q.setZero();
//
//     while(q.squaredNorm()==0)
//     {
//         simGetJointPosition(kinParams.lbrJoint[0], &q_0);
//         simGetJointPosition(kinParams.lbrJoint[1], &q_1);
//         simGetJointPosition(kinParams.lbrJoint[2], &q_2);
//         simGetJointPosition(kinParams.lbrJoint[3], &q_3);
//         simGetJointPosition(kinParams.lbrJoint[4], &q_4);
//         simGetJointPosition(kinParams.lbrJoint[5], &q_5);
//         simGetJointPosition(kinParams.lbrJoint[6], &q_6);
//         q << q_0, q_1, q_2, q_3, q_4, q_5, q_6;
//     }
//
//     vecxd q_new(8);
//     q_new[0] = q_0; // x[0];
//     q_new[1] = q_1; // x[1];
//     q_new[2] = q_2; // x[2];
//     q_new[3] = q_3; // x[3];
//     q_new[4] = q_4; // x[4];
//     q_new[5] = q_5; // x[5];
//     q_new[6] = q_6; // x[6];
//     q_new[7] = x[7];
//
//     vecxd qrcm(8);
//     qrcm = DYN::RCM::kukaDynRCM(q_new, kinParams);
//
//     dx[0] =  qrcm[0];
//     dx[1] =  qrcm[1];
//     dx[2] =  qrcm[2];
//     dx[3] =  qrcm[3];
//     dx[4] =  qrcm[4];
//     dx[5] =  qrcm[5];
//     dx[6] =  qrcm[6];
//     dx[7] =  qrcm[7];
//
//     vecxd max_angle(7);
//     max_angle << 170*PI/180, 120*PI/180, 170*PI/180, 120*PI/180, 170*PI/180, 120*PI/180, 170*PI/180;
//
//     for (int i=0; i<kinParams.q.size()-1;i++){
//         if(x[i]>max_angle[i])
//             simSetJointTargetPosition(kinParams.lbrJoint[i], max_angle[i]);
//         else if(x[i]<-max_angle[i])
//             simSetJointTargetPosition(kinParams.lbrJoint[i], -max_angle[i]);
//         else
//             simSetJointTargetPosition(kinParams.lbrJoint[i], x[i]);
//     }
// }
//
// void write_lorenz( const state_type &x , const double t )
// {
// }

// int main(int argc, char** argv){
// int DYN::modified_main_class::modified_main(int jointHandles[7]){
//   int lbrJoint1=0, lbrJoint2=0, lbrJoint3=0, lbrJoint4=0, lbrJoint5=0, lbrJoint6=0, lbrJoint7=0;
//   vecxd lbrJoint(7);
//   vecxd jnt_max_vel(7);
//   int counter = 0;
//   bool WORK = true;
//   kinParams.q = vecxd::Map(q_.data(), q_.size());
//   kinParams.gain=gain_;
//   kinParams.Xtr=Xtr_;
//   kinParams.Ytr=Ytr_;
//   kinParams.Ztr=-0.3105;
//   kinParams.target_pos << kinParams.Xtr, kinParams.Ytr, kinParams.Ztr;
//   kinParams.lambda = 0.5;
//   kinParams.lambda_desired = 1;
//   MatXd K_rcm = kinParams.gain*MatXd::Identity(3,3);
//   MatXd K_t = kinParams.gain*MatXd::Identity(3,3);
//   kinParams.k.block<3,3>(0,0)=K_rcm;
//   kinParams.k.block<3,3>(3,3)=K_t;
//   kinParams.err_squaredNorm = 1;
//   kinParams.z_axis_offset = 0.23;
//   kinParams.Xtr=0.4;
//   kinParams.Ytr=0.4;
//
//   double delta_t = 0.25;
//   vecxd q(7);
//   q.setZero();
//
//   vecxd q_desired(7);
//   q_desired = kinParams.q;
//
//   vecxd q_old(7);
//   q_old.setZero();
//
//   vecxd qd(7);
//   qd.setZero();
//
//   vecxd qd_old(7);
//   qd_old.setZero();
//
//   vecxd qdd(7);
//   qdd.setZero();
//
//   // External forces part
//   dynParams.N=N_;
//   dynParams.fv=fv_;
//   dynParams.fc=fc_;
//   dynParams.fv_vec = vecxd::Map(fv_vec_.data(), fv_vec_.size());
//   dynParams.masses = vecxd::Map(masses_.data(), masses_.size());
//
//   int element = 0;
//   for (int i=0; i<7; i++){
//     dvec iVec;
//     for (int j=0; j<6; j++){
//       iVec.push_back(inertiaVecx_[element]);
//       element++;
//     }
//     dynParams.inertiaVectors.block<1,6>(i,0) << iVec[0], iVec[1], iVec[2], iVec[3], iVec[4], iVec[5];
//   }
//
//   vecEigens V(7);
//
//   for (int i=0; i<7; i++){
//     V[i] = dynParams.inertiaVectors.block<1,6>(i,0);
//   }
//
//   iMat.I1 = DYN::FORCE::calcInertiaMat(V[0]);
//   iMat.I2 = DYN::FORCE::calcInertiaMat(V[1]);
//   iMat.I3 = DYN::FORCE::calcInertiaMat(V[2]);
//   iMat.I4 = DYN::FORCE::calcInertiaMat(V[3]);
//   iMat.I5 = DYN::FORCE::calcInertiaMat(V[4]);
//   iMat.I6 = DYN::FORCE::calcInertiaMat(V[5]);
//   iMat.I7 = DYN::FORCE::calcInertiaMat(V[6]);
//
//   element = 0;
//   for (int i=0; i<3; i++){
//     dvec iVec;
//     for (int j=0; j<7; j++){
//       iVec.push_back(r_i_ci_[element]);
//       element++;
//     }
//     dynParams.r_i_ci.block<1,7>(i,0) << iVec[0], iVec[1], iVec[2], iVec[3], iVec[4], iVec[5], iVec[6];
//   }
//
//   residual.g = vecxd::Map(g_.data(), g_.size());
//   // Gear reduction ratio
//   residual.kri = 100;
//   // Rotor inertia
//   residual.Im = 0.01;
//   // Rotor mass
//   residual.mm = 0.5;
//
//
//   {
//      simFloat q_0=0, q_1=0, q_2=0, q_3=0, q_4=0, q_5=0, q_6=0;
//      simFloat torque_0=0, torque_1=0, torque_2=0, torque_3=0, torque_4=0, torque_5=0, torque_6=0;
//      simFloat torque_=0;
//
//      lbrJoint << jointHandles[0], jointHandles[1], jointHandles[2], jointHandles[3], jointHandles[4], jointHandles[5], jointHandles[6];
//      kinParams.lbrJoint << jointHandles[0], jointHandles[1], jointHandles[2], jointHandles[3], jointHandles[4], jointHandles[5], jointHandles[6];
//
//      vecxd upperVelocityLimit(7);
//      // sim.jointfloatparam_upper_limit : 2017
//      upperVelocityLimit << 110*PI/180, 110*PI/180, 128*PI/180, 128*PI/180, 204*PI/180, 184*PI/180, 184*PI/180;
//      for (int i=0; i<kinParams.q.size();i++){
//          simSetObjectFloatParameter(lbrJoint[i], 2017, upperVelocityLimit[i]);
//      }
//
//      char ch;
//      // restart
//      for (int i=0; i<kinParams.q.size();i++){
//          simSetJointTargetPosition(lbrJoint[i], q[i]);
//      }
//
//      simGetJointPosition(lbrJoint[0], &q_0);
//      simGetJointPosition(lbrJoint[1], &q_1);
//      simGetJointPosition(lbrJoint[2], &q_2);
//      simGetJointPosition(lbrJoint[3], &q_3);
//      simGetJointPosition(lbrJoint[4], &q_4);
//      simGetJointPosition(lbrJoint[5], &q_5);
//      simGetJointPosition(lbrJoint[6], &q_6);
//
//      state_type x(8);
//      x[0] = q_0;
//      x[1] = q_1;
//      x[2] = q_2;
//      x[3] = q_3;
//      x[4] = q_4;
//      x[5] = q_5;
//      x[6] = q_6;
//      x[7] = kinParams.lambda;
//
//      for(int i=0;i<4;i++)
//      {
//          if(kinParams.err_squaredNorm<1e-5)
//          {
//              if(i==1)
//                 kinParams.lambda_desired = 0.8;
//              if(i==2)
//                 kinParams.lambda_desired = 0.6;
//              if(i==3)
//                 kinParams.lambda_desired = 0.5;
//              kinParams.Xtr = 0.4;
//              kinParams.Ytr = 0.4;
//              kinParams.target_pos[0] = 0.5;
//              kinParams.target_pos[1] = 0.5;
//              kinParams.gain = 0.05;
//              // kinParams.z_axis_offset = 0.2028;
//              kinParams.z_axis_offset = 0.21;
//              // cout << "?" << endl;
//              // cin.get();
//              usleep(1000 * 1000); // milliseconds * 1000
//
//          }
//          while(true)
//          {
//              integrate_const( make_controlled( 1E-6 , 1E-6 , runge_kutta_dopri5< state_type >() ) ,
//                           lorenz , x , 0.0 , 0.1 , 0.1 , write_lorenz );
//              if(kinParams.err_squaredNorm<1e-5) break;
//          }
//      }
//
//      dynParams.residual_gain = residual_gain_;
//      MatXd K(7,7);
//      K = MatXd::Identity(7,7)*dynParams.residual_gain;
//
//      bool B_old_exist = false;
//      simFloat signalValue;
//      simFloat signalValue_fx;
//      simFloat signalValue_fy;
//      simFloat signalValue_fz;
//
//      vecxd torque(7);
//      torque.setZero();
//      vecxd f_ext(6);
//      f_ext.setZero();
//      vecxd beta(7), sumDyn(7), sumRes(7), residualOutput(7), reaction(7);
//      beta.setZero();
//      sumDyn.setZero();
//      sumRes.setZero();
//      residualOutput.setZero();
//      reaction.setZero();
//      MatXd J_RCM_transpose(7,6);
//      J_RCM_transpose.setZero();
//      MatXd J_RCM_tr_pinv(6,7);
//      J_RCM_tr_pinv.setZero();
//
//      int n = 0;
//      MatXd f_ext_estimated_vs_real_vs_time(55,8);
//      f_ext_estimated_vs_real_vs_time.setZero();
//      MatXd f_ext_estimated_vs_real_vs_time_short(55,4);
//      f_ext_estimated_vs_real_vs_time_short.setZero();
//
//      vecxd f_ext_est_0(55);
//      vecxd f_ext_est_1(55);
//      vecxd f_ext_real_0(55);
//      vecxd f_ext_real_1(55);
//      vecxd f_ext_real_x(55);
//      vecxd f_ext_time(55);
//
//      vecxd f_ext_xyz_base(3);
//      f_ext_xyz_base.setZero();
//      vecxd f_ext_xyz_ee(3);
//      f_ext_xyz_ee.setZero();
//      vecxd f_ext_xyz_world(3);
//      f_ext_xyz_world.setZero();
//
//      simInt breathOn = 1;
//      simSetIntegerSignal("breath", breathOn);
//      bool reactionOn{true};
//
//      kinParams.lambda_desired=0.5;
//      kinParams.lambda=0.64;
//
//      simGetJointPosition(lbrJoint[0], &q_0);
//      simGetJointPosition(lbrJoint[1], &q_1);
//      simGetJointPosition(lbrJoint[2], &q_2);
//      simGetJointPosition(lbrJoint[3], &q_3);
//      simGetJointPosition(lbrJoint[4], &q_4);
//      simGetJointPosition(lbrJoint[5], &q_5);
//      simGetJointPosition(lbrJoint[6], &q_6);
//      q_old << q_0, q_1, q_2, q_3, q_4, q_5, q_6;
//
//      auto t_now = std::chrono::high_resolution_clock::now();
//      auto t_prev = t_now;
//      double elapsed_time_s;
//
//      bool gotreaction{false};

//      while (true)
//      {
//          simxGetJointPosition(clientID, lbrJoint1, &q_0, simx_opmode_blocking);
//          simxGetJointPosition(clientID, lbrJoint2, &q_1, simx_opmode_blocking);
//          simxGetJointPosition(clientID, lbrJoint3, &q_2, simx_opmode_blocking);
//          simxGetJointPosition(clientID, lbrJoint4, &q_3, simx_opmode_blocking);
//          simxGetJointPosition(clientID, lbrJoint5, &q_4, simx_opmode_blocking);
//          simxGetJointPosition(clientID, lbrJoint6, &q_5, simx_opmode_blocking);
//          simxGetJointPosition(clientID, lbrJoint7, &q_6, simx_opmode_blocking);
//
//          q << q_0, q_1, q_2, q_3, q_4, q_5, q_6;
//
//          simxGetFloatSignal(clientID, "fx", &signalValue_fx, simx_opmode_blocking);
//          simxGetFloatSignal(clientID, "fy", &signalValue_fy, simx_opmode_blocking);
//          simxGetFloatSignal(clientID, "fz", &signalValue_fz, simx_opmode_blocking);
//          simxGetFloatSignal(clientID, "x", &signalValue, simx_opmode_blocking);
//
//          simxGetJointForce(clientID, lbrJoint1, &torque_0, simx_opmode_blocking);
//          simxGetJointForce(clientID, lbrJoint2, &torque_1, simx_opmode_blocking);
//          simxGetJointForce(clientID, lbrJoint3, &torque_2, simx_opmode_blocking);
//          simxGetJointForce(clientID, lbrJoint4, &torque_3, simx_opmode_blocking);
//          simxGetJointForce(clientID, lbrJoint5, &torque_4, simx_opmode_blocking);
//          simxGetJointForce(clientID, lbrJoint6, &torque_5, simx_opmode_blocking);
//          simxGetJointForce(clientID, lbrJoint7, &torque_6, simx_opmode_blocking);
//          torque << -torque_0, -torque_1, -torque_2, -torque_3, -torque_4, -torque_5, -torque_6;
//
//          t_now = std::chrono::high_resolution_clock::now();
//          // elapsed_time_s = std::chrono::duration<double, std::milli>(t_now-t_prev).count()/1000;
//          elapsed_time_s = 0.5;
//          qd = (q - q_old)/elapsed_time_s;
//          qdd = (qd - qd_old)/elapsed_time_s;
//          t_prev = t_now;
//
//          DYN::FORCE::external_forces_estimation(kinParams, dynParams, residual, iMat, q, qd, qdd);
//
//          // RESIDUAL METHOD
//          if(B_old_exist)
//          {
//            beta = residual.n - ((residual.B - residual.B_old)/elapsed_time_s)*qd;
//            sumDyn = sumDyn + (torque - beta)*elapsed_time_s;
//            sumRes = sumRes + residualOutput*elapsed_time_s;
//            residualOutput = K*(residual.B*qd - sumDyn -sumRes - 2*dynParams.fv_vec);
//
//            J_RCM_transpose = residual.J_RCM.transpose();
//            J_RCM_tr_pinv = J_RCM_transpose.completeOrthogonalDecomposition().pseudoInverse();
//            // pseudoInverse: A#=(A'*A)^-1 *A'
//            f_ext = J_RCM_tr_pinv*residualOutput;
//            f_ext_xyz_base << f_ext[0], f_ext[1], f_ext[2];
//            f_ext_xyz_ee = residual.rwf7.transpose()*f_ext_xyz_base;
//            f_ext_xyz_world = residual.rwf0*f_ext_xyz_base;
//            // f_ext_xyz_ee = f_ext_xyz_world;
//            cout << f_ext_xyz_ee.transpose() << endl;
//
//            if(reactionOn)
//            {
//                gotreaction = false;
//                if(f_ext_xyz_world[0]>0.1)
//                {
//                    kinParams.Xtr -= 0.01;
//                    kinParams.target_pos[0] -= 0.01;
//                    gotreaction = true;
//                }
//                if(f_ext_xyz_world[1]>0.1)
//                {
//                    kinParams.Ytr -= 0.01;
//                    kinParams.target_pos[1] -= 0.01;
//                    gotreaction = true;
//                }
//                if(f_ext_xyz_world[0]<-0.1)
//                {
//                    kinParams.Xtr += 0.01;
//                    kinParams.target_pos[0] += 0.01;
//                    gotreaction = true;
//                }
//                if(f_ext_xyz_world[1]<-0.1)
//                {
//                    kinParams.Ytr += 0.01;
//                    kinParams.target_pos[1] += 0.01;
//                    gotreaction = true;
//                }
//                if(gotreaction)
//                {
//                    while(true)
//                    {
//                        integrate_const( make_controlled( 1E-6 , 1E-6 , runge_kutta_dopri5< state_type >() ) ,
//                                     lorenz , x , 0.0 , 0.1 , 0.1 , write_lorenz );
//                        if(kinParams.err_squaredNorm<1e-5) break;
//                    }
//                }
//            }
//            f_ext_estimated_vs_real_vs_time(n,0) = f_ext_xyz_ee[0];
//            f_ext_estimated_vs_real_vs_time(n,1) = signalValue_fx;
//            f_ext_estimated_vs_real_vs_time(n,2) = f_ext_xyz_ee[1];
//            f_ext_estimated_vs_real_vs_time(n,3) = signalValue_fy;
//            f_ext_estimated_vs_real_vs_time(n,4) = f_ext_xyz_ee[2];
//            f_ext_estimated_vs_real_vs_time(n,5) = signalValue_fz;
//            f_ext_estimated_vs_real_vs_time(n,6) = signalValue;
//
//            if(n>0)
//               f_ext_estimated_vs_real_vs_time(n,7) = f_ext_estimated_vs_real_vs_time(n-1,7) + elapsed_time_s;
//
//            cout << n << endl;
//            n += 1;
//          }
//
//
//          residual.B_old = residual.B;
//          B_old_exist = true;
//
//          q_old = q;
//          qd_old = qd;
//
//          if(n==55)
//             break;
//      }
//
//      cout << "f_ext_estimated_vs_real_vs_time" << endl;
//      cout << f_ext_estimated_vs_real_vs_time << endl;
//      f_ext_est_0 = f_ext_estimated_vs_real_vs_time.block<55,1>(0,0);
//      f_ext_real_0 = f_ext_estimated_vs_real_vs_time.block<55,1>(0,1);
//      f_ext_est_1 = f_ext_estimated_vs_real_vs_time.block<55,1>(0,2);
//      f_ext_real_1 = f_ext_estimated_vs_real_vs_time.block<55,1>(0,3);
//      f_ext_real_x = f_ext_estimated_vs_real_vs_time.block<55,1>(0,6);
//      f_ext_time = f_ext_estimated_vs_real_vs_time.block<55,1>(0,7);
//      std::vector<double> f_ext_est_0_std(&f_ext_est_0[0], f_ext_est_0.data()+f_ext_est_0.cols()*f_ext_est_0.rows());
//      std::vector<double> f_ext_est_1_std(&f_ext_est_1[0], f_ext_est_1.data()+f_ext_est_1.cols()*f_ext_est_1.rows());
//      std::vector<double> f_ext_real_0_std(&f_ext_real_0[0], f_ext_real_0.data()+f_ext_real_0.cols()*f_ext_real_0.rows());
//      std::vector<double> f_ext_real_1_std(&f_ext_real_1[0], f_ext_real_1.data()+f_ext_real_1.cols()*f_ext_real_1.rows());
//      std::vector<double> f_ext_real_x_std(&f_ext_real_x[0], f_ext_real_x.data()+f_ext_real_x.cols()*f_ext_real_x.rows());
//      std::vector<double> f_ext_time_std(&f_ext_time[0], f_ext_time.data()+f_ext_time.cols()*f_ext_time.rows());
//      matplotlibcpp::named_plot("Estimated_x", f_ext_time_std, f_ext_est_0_std, "r");
//      matplotlibcpp::named_plot("Real_x", f_ext_time_std, f_ext_real_0_std, "b");
//      // matplotlibcpp::named_plot("Real_x", f_ext_time_std, f_ext_real_x_std, "y");
//      matplotlibcpp::legend();
//      matplotlibcpp::show();
//
//      matplotlibcpp::named_plot("Estimated_y", f_ext_time_std, f_ext_est_1_std, "g");
//      matplotlibcpp::named_plot("Real_y", f_ext_time_std, f_ext_real_1_std, "k");
//      // matplotlibcpp::named_plot("Real_x", f_ext_time_std, f_ext_real_x_std, "y");
//      matplotlibcpp::legend();
//      matplotlibcpp::show();
//  }
//
//   return 0;
// }
