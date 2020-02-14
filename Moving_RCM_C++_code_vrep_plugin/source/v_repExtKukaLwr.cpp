#include "v_repExtKukaLwr.h"
#include "scriptFunctionData.h"
#include <iostream>
#include "v_repLib.h"
#include <unistd.h>
#define WIN_AFX_MANAGE_STATE

#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include "kukaDynRCM.h"
#include "forces.h"
#include "common.h"
#include <math.h>
#include "matplotlibcpp.h"
#include <algorithm> // for std::iota
#include <iostream>
#include <boost/numeric/odeint.hpp>
#include </usr/local/include/eigen3/Eigen/QR>
#include </usr/local/include/eigen3/Eigen/Dense>
#define PI 3.1415926535897932384626433832755

using namespace std;
using namespace boost::numeric::odeint;
typedef std::vector< double > state_type;
DYN::Params kinParams;
DYN::dynamicParams dynParams;
DYN::inertiaMat iMat;
DYN::residualParams residual;

const int N_ = 7;
const double fv_ = 0;
const double fc_ = 0;
const double Xtr_ = 0.5;
const double Ytr_ = 0.5;
const double Ztr_ = 0.0;
const double gain_ = 0.5;
const double residual_gain_ = 10;
const dvec q_ = {0, 0, 0, 0, 0, 0, 0};
const dvec g_ = {0, 0, -9.81};
const dvec fv_vec_ = {0, 0, 0, 0, 0, 0, 0};
const dvec masses_ = {4.1948152162, 4.2996847737, 3.658530333, \
      2.3846673548, 1.7035567183, 0.4000713156, 0.6501439811};
const dvec inertiaVecx_ = {0.01,     0.000189,    0.01,      0.01,        0.01,        0.01,\
                      0.04741,  0.05,        0.001601, -0.00000621,  0.0001166,  -0.000914,\
                      0.046951, 0.0008344,   0.05,     -0.000577,    0.000409,   -0.000577,\
                      0.012423, 0.007270,    0.009988, -0.000518,    0.00000002, -0.000548,\
                      0.006322, 0.001202,    0.007080, -0.000216,    0.000006,   -0.005,\
                      0.000527, 0,           0.003489,  0.000048,   -0.0000375,  -0.001030,\
                      0,        0.000032,    0.000118, -0.00000005, -0.0000214,   0.000069};
const dvec r_i_ci_ = {-0.0216387515,	 0.0003284751,	 0.0002593328,\
    	-0.0014648843,	-0.0003791484,	 0.0020739022,	-0.0004601303,\
      0.01,		      -0.0041132249,	 0.1137431845,	-0.0000461,	\
      -0.0553526131,	 0.0586184696,	 0.0014789221, \
      -0.0376881829,	 0.0823647642,	-0.000100257,	 0.148580959,	\
      -0.0101255137,	-0.044799983,	   0.0715608282};


#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_NAME "KukaLwr"

LIBRARY vrepLib;

struct sKukaLwr
{
    int handle;
    int jointHandles[7];
    int sensorHandle;
    char* waitUntilZero;
};

std::vector<sKukaLwr> allKukaLwrs;
int nextKukaLwrHandle=0;

int getKukaLwrIndexFromHandle(int KukaLwrHandle)
{
    for (unsigned int i=0;i<allKukaLwrs.size();i++)
    {
        if (allKukaLwrs[i].handle==KukaLwrHandle)
            return(i);
    }
    return(-1);
}

vecxd q(7);
vecxd q_old(7);
vecxd qd(7);
vecxd qd_old(7);
vecxd qdd(7);
simFloat q_0=0, q_1=0, q_2=0, q_3=0, q_4=0, q_5=0, q_6=0;
simFloat torque_0=0, torque_1=0, torque_2=0, torque_3=0, torque_4=0, torque_5=0, torque_6=0;
state_type x(8);
bool finished{false};
int n = 0;
MatXd K(7,7);
bool B_old_exist{false};
vecxd f_ext(6), beta(7), sumDyn(7), sumRes(7), residualOutput(7), torque(7);
MatXd J_RCM_transpose(7,6), J_RCM_tr_pinv(6,7);;
vecxd f_ext_xyz_base(3), f_ext_xyz_ee(3), f_ext_xyz_world(3);
simFloat signalValue_fx=0, signalValue_fy=0, signalValue_fz=0;
bool gotreaction{false};
double u_max=0, u_min=0;

void paramsAssignment()
{
    kinParams.q = vecxd::Map(q_.data(), q_.size());
    kinParams.gain=gain_;
    kinParams.Xtr=Xtr_;
    kinParams.Ytr=Ytr_;
    kinParams.Ztr=-0.3105;
    kinParams.target_pos << kinParams.Xtr, kinParams.Ytr, kinParams.Ztr;
    kinParams.lambda = 0.5;
    kinParams.lambda_desired = 1;
    kinParams.k.block<3,3>(0,0)=kinParams.gain*MatXd::Identity(3,3); // K_rcm
    kinParams.k.block<3,3>(3,3)=kinParams.gain*MatXd::Identity(3,3); // K_t
    kinParams.err_squaredNorm = 1;
    kinParams.z_axis_offset = 0.23;
    kinParams.Xtr=0.4;
    kinParams.Ytr=0.4;

    // External forces part
    dynParams.N=N_;
    dynParams.fv=fv_;
    dynParams.fc=fc_;
    dynParams.fv_vec = vecxd::Map(fv_vec_.data(), fv_vec_.size());
    dynParams.masses = vecxd::Map(masses_.data(), masses_.size());
    int element = 0;
    for (int i=0; i<7; i++){
        dvec iVec;
        for (int j=0; j<6; j++){
            iVec.push_back(inertiaVecx_[element]);
            element++;
        }
        dynParams.inertiaVectors.block<1,6>(i,0) << iVec[0], iVec[1], iVec[2], iVec[3], iVec[4], iVec[5];
    }
    vecEigens V(7);
    for (int i=0; i<7; i++){
        V[i] = dynParams.inertiaVectors.block<1,6>(i,0);
    }
    iMat.I1 = DYN::FORCE::calcInertiaMat(V[0]);
    iMat.I2 = DYN::FORCE::calcInertiaMat(V[1]);
    iMat.I3 = DYN::FORCE::calcInertiaMat(V[2]);
    iMat.I4 = DYN::FORCE::calcInertiaMat(V[3]);
    iMat.I5 = DYN::FORCE::calcInertiaMat(V[4]);
    iMat.I6 = DYN::FORCE::calcInertiaMat(V[5]);
    iMat.I7 = DYN::FORCE::calcInertiaMat(V[6]);
    element = 0;
    for (int i=0; i<3; i++){
        dvec iVec;
        for (int j=0; j<7; j++){
            iVec.push_back(r_i_ci_[element]);
            element++;
        }
        dynParams.r_i_ci.block<1,7>(i,0) << iVec[0], iVec[1], iVec[2], iVec[3], iVec[4], iVec[5], iVec[6];
    }
    residual.g = vecxd::Map(g_.data(), g_.size());
    residual.kri = 100;
    residual.Im = 0.01;
    residual.mm = 0.5;

    q.setZero();
    q_old.setZero();
    qd.setZero();
    qd_old.setZero();
    qdd.setZero();

    for(int i=0; i<kinParams.q.size(); i++){
        x[i] = 0;
    }
    x[7] = kinParams.lambda;
    n = 0;

    dynParams.residual_gain = residual_gain_;
    K = MatXd::Identity(7,7)*dynParams.residual_gain;
    torque.setZero();
    f_ext.setZero();
    beta.setZero();
    sumDyn.setZero();
    sumRes.setZero();
    residualOutput.setZero();
    J_RCM_transpose.setZero();
    J_RCM_tr_pinv.setZero();
    f_ext_xyz_base.setZero();
    f_ext_xyz_ee.setZero();
    f_ext_xyz_world.setZero();
}



// --------------------------------------------------------------------------------------
// simExtKukaLwr_create
// --------------------------------------------------------------------------------------
#define LUA_CREATE_COMMAND "simKukaLwr.create"

const int inArgs_CREATE[]={
    2,
    sim_script_arg_int32|sim_script_arg_table,7,
    sim_script_arg_int32,0
};

void LUA_CREATE_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    int handle=-1;
    if (D.readDataFromStack(cb->stackID,inArgs_CREATE,inArgs_CREATE[0],LUA_CREATE_COMMAND))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        sKukaLwr KukaLwr;
        handle=nextKukaLwrHandle++;
        KukaLwr.handle=handle;
        KukaLwr.jointHandles[0]=inData->at(0).int32Data[0];
        KukaLwr.jointHandles[1]=inData->at(0).int32Data[1];
        KukaLwr.jointHandles[2]=inData->at(0).int32Data[2];
        KukaLwr.jointHandles[3]=inData->at(0).int32Data[3];
        KukaLwr.jointHandles[4]=inData->at(0).int32Data[4];
        KukaLwr.jointHandles[5]=inData->at(0).int32Data[5];
        KukaLwr.jointHandles[6]=inData->at(0).int32Data[6];
        KukaLwr.sensorHandle=inData->at(1).int32Data[0];
        KukaLwr.waitUntilZero=NULL;
        allKukaLwrs.push_back(KukaLwr);

        paramsAssignment();
        kinParams.lbrJoint << KukaLwr.jointHandles[0], KukaLwr.jointHandles[1], KukaLwr.jointHandles[2],\
              KukaLwr.jointHandles[3], KukaLwr.jointHandles[4], KukaLwr.jointHandles[5], KukaLwr.jointHandles[6];
    }
    D.pushOutData(CScriptFunctionDataItem(handle));
    D.writeDataToStack(cb->stackID);
}
// --------------------------------------------------------------------------------------

vecxd DYN::RCM::kukaDynRCM(const vecxd& q, DYN::Params& kinParams){

  double q1=q[0], q2=q[1], q3=q[2], q4=q[3], q5=q[4], q6=q[5], q7=q[6], lambda=q[7];
  double s1=sin(q1), s2=sin(q2), s3=sin(q3), s4=sin(q4), s5=sin(q5), s6=sin(q6), s7=sin(q7);
  double c1=cos(q1), c2=cos(q2), c3=cos(q3), c4=cos(q4), c5=cos(q5), c6=cos(q6), c7=cos(q7);

  double ee_length=0.23;
  double d1=0.0;
  double d3=0.4;
  double d5=0.39;
  double d7=0.078 + ee_length;

  // Transformations matrices
  MatXd a1(4,4);
  a1 << c1, 0,  s1, 0,
        s1, 0, -c1, 0,
        0,  1,  0,  d1,
        0,  0,  0,  1;

  MatXd a2(4,4);
  a2 << c2,  0, -s2,  0,
        s2,  0,  c2,  0,
        0,  -1,   0,  0,
        0,   0,   0,  1;

  MatXd a3(4,4);
  a3 << c3,  0, -s3,  0,
        s3,  0,  c3,  0,
        0,  -1,   0,  d3,
        0,   0,   0,  1;

  MatXd a4(4,4);
  a4 << c4, 0,  s4,  0,
        s4, 0, -c4,  0,
        0,  1,   0,  0,
        0,  0,   0,  1;

  MatXd a5(4,4);
  a5 << c5, 0,  s5, 0,
        s5, 0, -c5, 0,
        0,  1,   0, d5,
        0,  0,   0, 1;

  MatXd a6(4,4);
  a6 << c6,  0, -s6,  0,
        s6,  0,  c6,  0,
        0,  -1,   0,  0,
        0,   0,   0,  1;

  MatXd a7(4,4);
  a7 << c7, -s7, 0,  0,
        s7,  c7, 0,  0,
        0,    0, 1,  d7,
        0,    0, 0,  1;

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

  vec3d p1 = kwf1.col(3).head(3);
  vec3d p2 = kwf2.col(3).head(3);
  vec3d p3 = kwf3.col(3).head(3);
  vec3d p4 = kwf4.col(3).head(3);
  vec3d p5 = kwf5.col(3).head(3);
  vec3d p6 = kwf6.col(3).head(3);
  vec3d p7 = kwf7.col(3).head(3);

  // Rotation matrices
  MatXd r1(3,3);
  r1 = a1.block<3,3>(0,0);

  MatXd r2(3,3);
  r2 = a2.block<3,3>(0,0);

  MatXd r3(3,3);
  r3 = a3.block<3,3>(0,0);

  MatXd r4(3,3);
  r4 = a4.block<3,3>(0,0);

  MatXd r5(3,3);
  r5 = a5.block<3,3>(0,0);

  MatXd r6(3,3);
  r6 = a6.block<3,3>(0,0);

  MatXd r7(3,3);
  r7 = a7.block<3,3>(0,0);

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

  vec3d linJ6vec6(0,0,0);

  MatXd linJ6(3,7);
  linJ6.col(0) = z_0.cross(p_6_e_0);
  linJ6.col(1) = z_1.cross(p_6_e_1);
  linJ6.col(2) = z_2.cross(p_6_e_2);
  linJ6.col(3) = z_3.cross(p_6_e_3);
  linJ6.col(4) = z_4.cross(p_6_e_4);
  linJ6.col(5) = z_5.cross(p_6_e_5);
  linJ6.col(6) = linJ6vec6;

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
  linJ7.col(0) = z_0.cross(p_7_e_0);
  linJ7.col(1) = z_1.cross(p_7_e_1);
  linJ7.col(2) = z_2.cross(p_7_e_2);
  linJ7.col(3) = z_3.cross(p_7_e_3);
  linJ7.col(4) = z_4.cross(p_7_e_4);
  linJ7.col(5) = z_5.cross(p_7_e_5);
  linJ7.col(6) = z_6.cross(p_7_e_5);

  if(lambda<0) lambda=0;

  // Jacobian RCM
  MatXd J_lambda(3,7); // 3x7 matrix
  J_lambda = linJ6+lambda*(linJ7-linJ6);

  vec3d g;
  g=p7-p6; // 3D vector

  MatXd J_rCM(3,8); // 3x8 matrix composed by the matrix J_lambda and vector g
  J_rCM.block<3,7>(0,0)=J_lambda; // First 3x7 block
  J_rCM.block<3,1>(0,7)=g; // 3x1 vector on the 8th column

  // Task Jacobian
  MatXd Jt(3,8); // 3x8 matrix
  Jt.block<3,7>(0,0)=linJ6; // First 3x7 block

  vec3d Jtvec7(0,0,0);
  Jt.col(7) = Jtvec7;

  // Extention Jacobian 6x8 it includes the J_RCM and Jt (the task Jacobian)
  MatXd J_a(6,8);
  // Eigen::Matrix<double,6,8> J_a;
  J_a.block<3,8>(0,0)=J_rCM;
  J_a.block<3,8>(3,0)=Jt;

  double x_rcm=p6[0]+lambda*(p7[0]-p6[0]);
  double y_rcm=p6[1]+lambda*(p7[1]-p6[1]);
  double z_rcm=p6[2]+lambda*(p7[2]-p6[2]);

  kinParams.p6 << p6[0], p6[1], p6[2];
  kinParams.p7 << p7[0], p7[1], p7[2];
  kinParams.rcm << x_rcm, y_rcm, z_rcm;

  // kinParams.target_pos[2] = -0.3105 since joint1_z is shifted-up by -0.3105
  // Ask about the target_pos
  // kinParams.Ztr=-0.3105;
  double z_axis_diff=sqrt(pow(0.078+ee_length,2)-pow((kinParams.target_pos[0]-kinParams.Xtr),2)-pow((kinParams.target_pos[1]-kinParams.Ytr),2));
  double z_axis_offset = kinParams.z_axis_offset;
  vec3d err_rcm(kinParams.target_pos[0]-x_rcm, kinParams.target_pos[1]-y_rcm, (kinParams.target_pos[2]+z_axis_offset-z_rcm));
  vec3d err_t(kinParams.Xtr-p6[0], kinParams.Ytr-p6[1], (kinParams.Ztr+z_axis_diff*kinParams.lambda_desired+z_axis_offset-p6[2]));

  kinParams.lambda = lambda;

  MatXd err(6,1); // 6x1 matrix
  err.block<3,1>(0,0)=err_rcm;
  err.block<3,1>(3,0)=err_t;

  kinParams.err_squaredNorm = err.squaredNorm();

  // Position and orientation control with RCM constraint
  MatXd identity = MatXd::Identity(8,8);
  // Jacobian pseudoinverse
  MatXd Jp(8,6);
  Jp = J_a.completeOrthogonalDecomposition().pseudoInverse(); // 8x6 matrix
  double err_lam=kinParams.lambda_desired-lambda;

  vecxd w(8);
  w[0]=-(1/8)*q1/5.9;
  w[1]=-(1/8)*q2/4.2;
  w[2]=-(1/8)*q3/5.9;
  w[3]=-(1/8)*q4/4.2;
  w[4]=-(1/8)*q5/5.9;
  w[5]=-(1/8)*q6/4.2;
  w[6]=-(1/8)*q7/5.9;
  w[7]=-(1/8)*(lambda-0.5)/1;
  w[7]+=kinParams.gain*err_lam;

  vecxd u(8);
  u.setZero();

  // // 8x6 * 6x6 * 6x1 + (8x8-(6x8*8x6)) * 8x1 // dimensions to be checked
  u=Jp*kinParams.k*err+(identity-Jp*J_a)*(w);

  u_max = 0.5;
  u_min = -0.5;
  for(int i=0;i<8;i++)
  {
      if(u[i]>u_max) u[i]=u_max;
      if(u[i]<u_min) u[i]=u_min;
  }

  vecxd q_dot(8);
  q_dot.setZero();

  q_dot << u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7];

  return q_dot;
}

void Euler(state_type &x, double dt, DYN::Params& kinParams)
{
    simFloat q_0=0, q_1=0, q_2=0, q_3=0, q_4=0, q_5=0, q_6=0;
    vecxd q(7);
    q.setZero();
    while(q.squaredNorm()==0){
        simGetJointPosition(kinParams.lbrJoint[0], &q_0);
        simGetJointPosition(kinParams.lbrJoint[1], &q_1);
        simGetJointPosition(kinParams.lbrJoint[2], &q_2);
        simGetJointPosition(kinParams.lbrJoint[3], &q_3);
        simGetJointPosition(kinParams.lbrJoint[4], &q_4);
        simGetJointPosition(kinParams.lbrJoint[5], &q_5);
        simGetJointPosition(kinParams.lbrJoint[6], &q_6);
        q << q_0, q_1, q_2, q_3, q_4, q_5, q_6;
    }
    vecxd q_new(8);
    q_new << q_0, q_1, q_2, q_3, q_4, q_5, q_6, x[7];

    vecxd qrcm(8);
    qrcm = DYN::RCM::kukaDynRCM(q_new, kinParams);

    // dx[0] =  qrcm[0];
    // dx[1] =  qrcm[1];
    // dx[2] =  qrcm[2];
    // dx[3] =  qrcm[3];
    // dx[4] =  qrcm[4];
    // dx[5] =  qrcm[5];
    // dx[6] =  qrcm[6];
    // dx[7] =  qrcm[7];

    vecxd max_angle(7);
    max_angle << 170*PI/180, 120*PI/180, 170*PI/180, 120*PI/180, 170*PI/180, 120*PI/180, 170*PI/180;

    for (int i=0; i<kinParams.q.size()-1;i++){
        x[i] += qrcm[i]*dt;
        if(x[i]>max_angle[i])
            simSetJointTargetPosition(kinParams.lbrJoint[i], max_angle[i]);
        else if(x[i]<-max_angle[i])
            simSetJointTargetPosition(kinParams.lbrJoint[i], -max_angle[i]);
        else
            simSetJointTargetPosition(kinParams.lbrJoint[i], x[i]);
    }
    x[7] += qrcm[7]*dt;
}

void assign_q_old(){
    simGetJointPosition(kinParams.lbrJoint[0], &q_0);
    simGetJointPosition(kinParams.lbrJoint[1], &q_1);
    simGetJointPosition(kinParams.lbrJoint[2], &q_2);
    simGetJointPosition(kinParams.lbrJoint[3], &q_3);
    simGetJointPosition(kinParams.lbrJoint[4], &q_4);
    simGetJointPosition(kinParams.lbrJoint[5], &q_5);
    simGetJointPosition(kinParams.lbrJoint[6], &q_6);
    q_old << q_0, q_1, q_2, q_3, q_4, q_5, q_6;
}

void forceEstimationAndReaction(float dt)
{
    simGetJointPosition(kinParams.lbrJoint[0], &q_0);
    simGetJointPosition(kinParams.lbrJoint[1], &q_1);
    simGetJointPosition(kinParams.lbrJoint[2], &q_2);
    simGetJointPosition(kinParams.lbrJoint[3], &q_3);
    simGetJointPosition(kinParams.lbrJoint[4], &q_4);
    simGetJointPosition(kinParams.lbrJoint[5], &q_5);
    simGetJointPosition(kinParams.lbrJoint[6], &q_6);
    q << q_0, q_1, q_2, q_3, q_4, q_5, q_6;
    simGetJointForce(kinParams.lbrJoint[0], &torque_0);
    simGetJointForce(kinParams.lbrJoint[1], &torque_1);
    simGetJointForce(kinParams.lbrJoint[2], &torque_2);
    simGetJointForce(kinParams.lbrJoint[3], &torque_3);
    simGetJointForce(kinParams.lbrJoint[4], &torque_4);
    simGetJointForce(kinParams.lbrJoint[5], &torque_5);
    simGetJointForce(kinParams.lbrJoint[6], &torque_6);
    torque << -torque_0, -torque_1, -torque_2, -torque_3, -torque_4, -torque_5, -torque_6;
    qd = (q - q_old)/dt;
    qdd = (qd - qd_old)/dt;
    DYN::FORCE::external_forces_estimation(kinParams, dynParams, residual, iMat, q, qd, qdd);

    if(B_old_exist)
    {
        beta = residual.n - ((residual.B - residual.B_old)/dt)*qd;
        sumDyn = sumDyn + (torque - beta)*dt;
        sumRes = sumRes + residualOutput*dt;
        residualOutput = K*(residual.B*qd - sumDyn -sumRes - 2*dynParams.fv_vec);
        J_RCM_transpose = residual.J_RCM.transpose();
        J_RCM_tr_pinv = J_RCM_transpose.completeOrthogonalDecomposition().pseudoInverse();
        // pseudoInverse: A#=(A'*A)^-1 *A'
        f_ext = J_RCM_tr_pinv*residualOutput;
        f_ext_xyz_base << f_ext[0], f_ext[1], f_ext[2];
        f_ext_xyz_ee = residual.rwf7.transpose()*f_ext_xyz_base;
        f_ext_xyz_world = residual.rwf0*f_ext_xyz_base;
        simSetDoubleSignal("f_ext_x_ee",f_ext_xyz_ee[0]);
        simSetDoubleSignal("f_ext_y_ee",f_ext_xyz_ee[1]);
        simSetDoubleSignal("f_ext_z_ee",f_ext_xyz_ee[2]);

        // u_max = 0.5; u_min = -0.5;
        gotreaction = false;
        if(f_ext_xyz_world[0]>0.5)
        {
           kinParams.Xtr -= 0.001;
           kinParams.target_pos[0] -= 0.001;
           gotreaction = true;
        }
        if(f_ext_xyz_world[1]>0.5)
        {
           kinParams.Ytr -= 0.001;
           kinParams.target_pos[1] -= 0.001;
           gotreaction = true;
        }
        if(f_ext_xyz_world[0]<-0.5)
        {
           kinParams.Xtr += 0.001;
           kinParams.target_pos[0] += 0.001;
           gotreaction = true;
        }
        if(f_ext_xyz_world[1]<-0.5)
        {
           kinParams.Ytr += 0.001;
           kinParams.target_pos[1] += 0.001;
           gotreaction = true;
        }
        if(gotreaction)
        {
           Euler(x, dt, kinParams);
        }
    }
    B_old_exist = true;
    residual.B_old = residual.B;
    q_old = q;
    qd_old = qd;
}




VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.
    // Dynamically load and bind V-REP functions:
    char curDirAndFile[1024];
    getcwd(curDirAndFile, sizeof(curDirAndFile));

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);
    temp+="/libv_rep.so";

    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start 'KukaLwr' plugin.\n";
        return(0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in v_rep.dll. Cannot start 'KukaLwr' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Check the V-REP version:
    int vrepVer,vrepRev;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    simGetIntegerParameter(sim_intparam_program_revision,&vrepRev);
    if( (vrepVer<30400) || ((vrepVer==30400)&&(vrepRev<9)) )
    {
        std::cout << "Sorry, your V-REP copy is somewhat old, V-REP 3.4.0 rev9 or higher is required. Cannot start 'KukaLwr' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    simRegisterScriptVariable("simKukaLwr","require('simExtKukaLwr')",0);

    // Register the new functions:
    simRegisterScriptCallbackFunction(strConCat(LUA_CREATE_COMMAND,"@",PLUGIN_NAME),strConCat("number KukaLwrHandle=",LUA_CREATE_COMMAND,"(table_7 motorJointHandles,number sensorHandle)"),LUA_CREATE_CALLBACK);
    // simRegisterScriptCallbackFunction(strConCat(LUA_DESTROY_COMMAND,"@",PLUGIN_NAME),strConCat("boolean result=",LUA_DESTROY_COMMAND,"(number KukaLwrHandle)"),LUA_DESTROY_CALLBACK);
    // simRegisterScriptCallbackFunction(strConCat(LUA_START_COMMAND,"@",PLUGIN_NAME),strConCat("boolean result=",LUA_START_COMMAND,"(number KukaLwrHandle,number duration,boolean returnDirectly=false)"),LUA_START_CALLBACK);
    // simRegisterScriptCallbackFunction(strConCat(LUA_STOP_COMMAND,"@",PLUGIN_NAME),strConCat("boolean result=",LUA_STOP_COMMAND,"(number KukaLwrHandle)"),LUA_STOP_CALLBACK);

    // Following for backward compatibility:
    simRegisterScriptVariable("simExtKukaLwr_create",LUA_CREATE_COMMAND,-1);
    simRegisterScriptCallbackFunction(strConCat("simExtKukaLwr_create","@",PLUGIN_NAME),strConCat("Please use the ",LUA_CREATE_COMMAND," notation instead"),0);
    // simRegisterScriptVariable("simExtKukaLwr_destroy",LUA_DESTROY_COMMAND,-1);
    // simRegisterScriptCallbackFunction(strConCat("simExtKukaLwr_destroy","@",PLUGIN_NAME),strConCat("Please use the ",LUA_DESTROY_COMMAND," notation instead"),0);
    // simRegisterScriptVariable("simExtKukaLwr_start",LUA_START_COMMAND,-1);
    // simRegisterScriptCallbackFunction(strConCat("simExtKukaLwr_start","@",PLUGIN_NAME),strConCat("Please use the ",LUA_START_COMMAND," notation instead"),0);
    // simRegisterScriptVariable("simExtKukaLwr_stop",LUA_STOP_COMMAND,-1);
    // simRegisterScriptCallbackFunction(strConCat("simExtKukaLwr_stop","@",PLUGIN_NAME),strConCat("Please use the ",LUA_STOP_COMMAND," notation instead"),0);

    return(9); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
    // version 1 was for V-REP versions before V-REP 2.5.12
    // version 2 was for V-REP versions before V-REP 2.6.0
    // version 5 was for V-REP versions before V-REP 3.1.0
    // version 6 is for V-REP versions after V-REP 3.1.3
    // version 7 is for V-REP versions after V-REP 3.2.0 (completely rewritten)
    // version 8 is for V-REP versions after V-REP 3.3.0 (using stacks for data exchange with scripts)
    // version 9 is for V-REP versions after V-REP 3.4.0 (new API notation)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // This function should not generate any error messages:
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

    void* retVal=NULL;

    if (message==sim_message_eventcallback_modulehandle)
    {
        if ( (customData==NULL)||(std::string("KukaLwr").compare((char*)customData)==0) ) // is the command also meant for KukaLwr?
        {
            float dt=simGetSimulationTimeStep();
            for (unsigned int i=0;i<allKukaLwrs.size();i++)
            {
                if(n==1){
                    kinParams.lambda_desired = 0.8;
                    kinParams.z_axis_offset = 0.21;
                }
                else if(n==2){
                    kinParams.lambda_desired = 0.5;
                }
                else if(n==3){
                    simSetIntegerSignal("breath", 1);
                    assign_q_old();
                    n++;
                }
                else if(n==4){
                    forceEstimationAndReaction(dt);
                }
                if(kinParams.err_squaredNorm>1e-5)
                    Euler(x, dt, kinParams);
                else
                {
                    if(n==0 or n==1 or n==2){
                        kinParams.err_squaredNorm=1;
                        n++;
                    }
                }

            }
        }
    }

    if (message==sim_message_eventcallback_simulationended)
    { // simulation ended. Destroy all KukaLwr instances:
        allKukaLwrs.clear();
    }

    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}
