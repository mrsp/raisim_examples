// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
//#include <whole_body_ik/pin_wrapper.h>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-angular-momentum-equality.hpp>
#include <tsid/tasks/task-cop-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <lipm_control/ZMPDistributor.h>
#include <lipm_control/postureStabilizer.h>
#include <chrono>

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;
using namespace Eigen;
using namespace std::chrono;
const string atlas_model_path = "/home/master/talos_sot/src/example-robot-data/robots/atlas";


const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");


template <typename Derived>
void writeToCSVfile(std::string name, const Eigen::MatrixBase<Derived>& matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}




Eigen::MatrixXd parseCSV(string filename, int data_dim)
{
  ifstream in(filename);
  vector<vector<double>> fields;

  if (in)
  {
    string line;

    while (getline(in, line))
    {
      stringstream sep(line);
      string field;

      fields.push_back(vector<double>());

      while (getline(sep, field, ','))
      {
        fields.back().push_back(stod(field));
      }
    }
  }
  cout << "Size " << fields[0].size() << std::endl;
  cout << "Data Size " << data_dim << std::endl;
  Eigen::MatrixXd data(fields.size(), data_dim);
  int i = 0;
  int j = 0;

  for (auto row : fields)
  {
    for (auto field : row)
    {
      //cout << field << ' ';
      data(i, j) = field;
      j++;
    }
    j = 0;
    i++;
    //cout<<"\n";
    //cout<<"\n";
  }
  return data;
}



void swapQuatWXYZ(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[6];
  input_[4] = tmp[3];
  input_[5] = tmp[4];
  input_[6] = tmp[5];
}
void swapQuatXYZW(Eigen::VectorXd &input_)
{
  Eigen::VectorXd tmp(input_.size());
  tmp = input_;
  input_[3] = tmp[4];
  input_[4] = tmp[5];
  input_[5] = tmp[6];
  input_[6] = tmp[3];
}

double Gain = 0.5;
double QGain = 0.5;
struct humanoidTaskData
{
  VectorXd joint_states;
  std::vector<std::string> joint_names;
  Vector3d CoM_pos;
  Vector3d CoM_vel;
  Vector3d lfoot_pos;
  Vector3d rfoot_pos;
  Vector3d lfoot_linear_vel;
  Vector3d rfoot_linear_vel;
  Vector3d lfoot_angular_vel;
  Vector3d rfoot_angular_vel;
  Vector3d base_pos;
  Vector3d base_linear_vel;
  Vector3d base_angular_vel;
  Quaterniond base_orientation;
  Quaterniond lfoot_orientation;
  Quaterniond rfoot_orientation;
  std::string lfoot_frame;
  std::string rfoot_frame;
  std::string base_frame;
  double lfoot_weight;
  double rfoot_weight;
};

class LIPMControl
{
  Vector3d CoM_c, dCoM_c, ddCoM_c, dCoM_c_;
  Vector3d dCoM_d_, ZMP_d, ZMP_c;
  Vector3d DCM_ref, dDCM_ref, DCM;

  double omega, g, h, mass, dt;
  bool initialized;
  Matrix3d Kzmp, Kdcm, Kcom;

public:
  LIPMControl(double h_, double mass_, double dt_, double gain_x = 0.02, double gain_y = 0.02)
  {
    dt = dt_;
    mass = mass_;
    h = h_;
    g = 9.81;
    omega = sqrt(g / h);
    initialized = false;
    Kzmp.setZero();
    Kcom.setZero();
    Kdcm.setZero();
    // Kzmp(0, 0) = 0.02;
    // Kzmp(1, 1) = 0.02;
    // Kcom(0, 0) = omega+0.3;
    // Kcom(1, 1) = omega+0.3;
    // Kcom(2, 2) = omega+0.3;

    // Kdcm(0, 0) = 1.0 + 2.5;
    // Kdcm(1, 1) = 1.0 + 2.5;
    // Kdcm(2, 2) = 1.0 + 2.5;


    Kzmp(0, 0) = 5/omega;
    Kzmp(1, 1) = 5/omega;
    
    Kcom(0, 0) = 0.04;
    Kcom(1, 1) = 0.01;
    Kcom(2, 2) = 0.00;

    Kdcm(0, 0) = 1.0 + 150/omega;
    Kdcm(1, 1) = 1.0 + 150/omega;

    ddCoM_c.setZero();
    dCoM_c_.setZero();
    DCM.setZero();
    DCM_ref.setZero();
    dDCM_ref.setZero();
  }

  void Control(Vector3d ZMP, Vector3d CoM, Vector3d dCoM, Vector3d CoM_d, Vector3d dCoM_d, Vector3d ddCoM_d)
  {
    if (!initialized)
    {
      CoM_c = CoM_d;
      dCoM_c = dCoM_d;

      //dCoM_d_ = dCoM_d;

      initialized = true;
      return;
    }

    //Compute Desired ZMP based on the LIPM
    // Vector3d ddCoM_d = (dCoM_d - dCoM_d_) / dt;
    // dCoM_d_ = dCoM_d;


    ZMP_d = CoM_d - ddCoM_d / (omega * omega);
    ZMP_d(2) =  ZMP(2);



    DCM_ref = computeDCM(CoM_d,dCoM_d);
    dDCM_ref = computeDCM(dCoM_d,ddCoM_d);
    DCM = computeDCM(CoM,dCoM);


    ZMP_c = ZMP_d - Kdcm * (DCM_ref - DCM) + Kzmp  * (ZMP_d - ZMP);
    ZMP_c(2) =  ZMP(2);
    //Desired ZMP
    // ZMP_c = DCM_ref - 1/omega * dDCM_ref + Kdcm * (DCM-DCM_ref); 
    // ZMP_c(2) =  ZMP(2);
    cout<<"Desired ZMP "<<ZMP_d.transpose()<< "CMD ZMP "<<ZMP_c.transpose()<<" Measured ZMP "<<ZMP.transpose()<<endl;

    //Desired CoM
    //dCoM_c = dCoM_d - Kzmp * (ZMP_c - ZMP) + Kcom * (CoM_d - CoM);
    dCoM_c = dCoM_d - Kcom * (ZMP_c - ZMP);


    ddCoM_c = (dCoM_c - dCoM_c_)/dt;  
    dCoM_c_ = dCoM_c;

    CoM_c = CoM_c + dCoM_c * dt;
    cout<<"Desired CoM "<<CoM_c.transpose()<<" Measured CoM "<<CoM.transpose()<<endl;
    cout<<"Desired DCM "<<DCM_ref.transpose()<<" Measured DCM "<<DCM.transpose()<<endl;

  }

  Vector3d computeDCM(Vector3d CoM, Vector3d dCoM)
  {
    return CoM + 1/omega * dCoM;
  }

  Vector3d getDesiredCoMPosition()
  {
    return CoM_c;
  }
  Vector3d getDesiredCoMVelocity()
  {
    return dCoM_c;
  }
  Vector3d getDesiredCoMAcceleration()
  {
    return ddCoM_c;
  }
  Vector3d getDesiredZMP()
  {
    return ZMP_c;
  }
};

class AtlasInverseDynamics
{
  double t,dt;
  InverseDynamicsFormulationAccForce* tsid;
  TaskJointPosture* postureTask;
  Contact6d* contactLF;
  Contact6d* contactRF;
  TaskSE3Equality* rightFootTask;
  TaskSE3Equality* leftFootTask;
  TaskComEquality* comTask;
  TaskCopEquality* copTask;
  TaskJointBounds* jointBoundsTask;
  TaskAMEquality* amTask;
  pinocchio::Data data;
  RobotWrapper *robot;
  SolverHQPBase *solver;
  std::string lfoot_frame, rfoot_frame;
  bool RF_ContactTask = false;
  bool LF_ContactTask = false;
  bool RF_MotionTask = false;
  bool LF_MotionTask = false;
  bool MotionTasks = false;

public:
  const double lxp = 0.0729359;
  const double lxn = 0.0901216;
  const double lyp = 0.0669625;
  const double lyn = 0.0669625;
  const double lz =  0.0717;
  const double mu = 0.3;
  const double fMin = 400.0;
  const double fMax = 5000.0;
  const Vector3d contactNormal = Vector3d::UnitZ();
  const double w_posture =  1.0e-4;
  const double w_forceReg = 1.0e-6;
  const double w_leg = 1e3;
  const double w_com = 0.1;
  const double w_cop = 0.0005;
  const double w_am = 0.005;
  const double kp_am = 50.0;
  const double kp_com = 600.0;
  const double kp_leg = 50.0;
  const double kp_posture = 1.0;
  const double kp_contact = 0.005;
  VectorXd qd, dqd, ddqd, taud;


  AtlasInverseDynamics(std::vector<string> package_dirs, std::string urdfFileName, double dt_, std::string lfoot_frame_, std::string rfoot_frame_, VectorXd q, VectorXd dq)
  {

    t = 0.0;
    dt = dt_;
    lfoot_frame = lfoot_frame_;
    rfoot_frame = rfoot_frame_;

    robot = new RobotWrapper(urdfFileName, package_dirs, pinocchio::JointModelFreeFlyer());
    qd.setZero(robot->nq());
    dqd.setZero(robot->nv());
    ddqd.setZero(robot->nv());
    taud.setZero(robot->nv());
    qd = q;
    dqd = dq;
    tsid = new InverseDynamicsFormulationAccForce("tsid", *robot);
    tsid->computeProblemData(t, q, dq);

    data = tsid->data();


      Matrix3x contactPoints(3,4);
    contactPoints << -lxn, -lxn, +lxp, +lxp,
                     -lyn, +lyp, -lyn, +lyp,
                      lz,  lz,  lz,  lz;
    contactLF = new Contact6d("contact_lfoot", *robot, lfoot_frame,
                          contactPoints, contactNormal,
                          mu, fMin, fMax);

    contactRF = new Contact6d("contact_rfoot", *robot, rfoot_frame,
                          contactPoints, contactNormal,
                          mu, fMin, fMax);


    pinocchio::SE3 H_rf_ref = robot->position(data, robot->model().getJointId(rfoot_frame));
    pinocchio::SE3 H_lf_ref = robot->position(data, robot->model().getJointId(lfoot_frame));
    contactRF->Kp(kp_contact * Vector::Ones(6));
    contactRF->Kd(2.0 * contactRF->Kp().cwiseSqrt());
    contactLF->Kp(kp_contact * Vector::Ones(6));
    contactLF->Kd(2.0 * contactLF->Kp().cwiseSqrt());

    contactRF->setReference(H_rf_ref);
    tsid->addRigidContact(*contactRF, w_forceReg);
    RF_ContactTask = true;

    contactLF->setReference(H_lf_ref);
    tsid->addRigidContact(*contactLF, w_forceReg);
    LF_ContactTask = true;


    rightFootTask = new TaskSE3Equality("task-right-foot", *robot, rfoot_frame);
    rightFootTask->Kp(kp_leg * Vector::Ones(6));
    rightFootTask->Kd(2.0 * rightFootTask->Kp().cwiseSqrt());
    tsid->addMotionTask(*rightFootTask, w_leg, 1);


    leftFootTask = new TaskSE3Equality("task-left-foot", *robot, lfoot_frame);
    leftFootTask->Kp(kp_leg * Vector::Ones(6));
    leftFootTask->Kd(2.0 * leftFootTask->Kp().cwiseSqrt());
    tsid->addMotionTask(*leftFootTask, w_leg, 1);




    // Add the com task
    comTask = new TaskComEquality("task-com", *robot);
    comTask->Kp(kp_com * Vector::Ones(3));
    comTask->Kd(2.0 * comTask->Kp().cwiseSqrt());
    tsid->addMotionTask(*comTask, w_com, 1,0.0);


    amTask = new TaskAMEquality("task-am", *robot);
    amTask->Kp(kp_am * Vector::Ones(3));
    amTask->Kd(2.0 * amTask->Kp().cwiseSqrt());
    TrajectorySample sampleAM(3);
    amTask->setReference(sampleAM);
    tsid->addMotionTask(*amTask, w_am, 1,0.0);


    copTask = new TaskCopEquality("task-cop", *robot);
    //copTask->setReference(Vector3d(0,0,0));
    //tsid->addForceTask(*copTask, w_cop, 1,0.0);


    postureTask = new TaskJointPosture("task-posture", *robot);
    postureTask->Kp(kp_posture * Vector::Ones(robot->nv()-6));
    postureTask->Kd(2.0 * postureTask->Kp());
    tsid->addMotionTask(*postureTask, w_posture, 1,0.0);





    // Add the joint bounds task
    jointBoundsTask = new TaskJointBounds("task-joint-bounds", *robot, dt);
    Vector dq_max = 10*Vector::Ones(robot->na());
    Vector dq_min = -dq_max;
    jointBoundsTask->setVelocityBounds(dq_min, dq_max);
    tsid->addMotionTask(*jointBoundsTask, 1.0, 0);

    solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "solver-eiquadprog");
  }
  void computeInverseDynamics(VectorXd q, VectorXd dq, VectorXd q_ref, Vector3d CoM_pos_ref, Vector3d CoM_vel_ref, Vector3d CoM_acc_ref, Vector3d COP_ref, pinocchio::SE3 H_rf_ref,  pinocchio::SE3 H_lf_ref, pinocchio::Motion M_rf_ref,   pinocchio::Motion M_lf_ref,   pinocchio::Motion A_rf_ref,   pinocchio::Motion A_lf_ref, bool RSS, bool LSS) 
  {

    TrajectorySample sample(q_ref.size());
    sample.pos = q_ref;
    postureTask->setReference(sample);

    TrajectorySample sampleCoM;
    sampleCoM.pos = CoM_pos_ref;
    sampleCoM.vel = CoM_vel_ref;
    sampleCoM.acc = CoM_acc_ref;
    comTask->setReference(sampleCoM);


    // H_rf_ref.translation()(2) -=  0.0073177;
    // H_lf_ref.translation()(2) -=  0.0073177;

    
    //pinocchio::SE3 tmp;
    // cout<<"CoM ref "<<CoM_pos_ref.transpose()<<" "<<CoM_vel_ref.transpose()<<" "<<CoM_acc_ref<<endl;
    //tmp =  robot->position(tsid->data(), robot->model().getJointId(rfoot_frame));
    //cout<<"Right "<<H_rf_ref<<" "<<tmp<<endl;//<<" "<<M_rf_ref.linear()<<" "<<A_rf_ref.linear()<<endl;
    //tmp = robot->position(tsid->data(), robot->model().getJointId(lfoot_frame));
    //cout<<"Left "<<H_lf_ref<<" "<<tmp<<endl;//<<" "<<M_lf_ref.linear()<<" "<<A_lf_ref.linear()<<endl;
    //std::cout << "COP REF " <<COP_ref.transpose()<<std::endl;
    copTask->setReference(COP_ref);





    double eps = 5e-4;
    cout<<" Left "<<(A_lf_ref.linear()).squaredNorm()<<endl;
    cout<<" Right "<<(A_rf_ref.linear()).squaredNorm()<<endl;

    if ((M_rf_ref.linear()).squaredNorm() > eps && (M_lf_ref.linear()).squaredNorm() < eps)
    {
      if (RF_ContactTask)
      {
         cout<<"Removed RSS"<<endl;
        tsid->removeRigidContact(contactRF->name());
        RF_ContactTask = false;
      }
 
        //H_lf_ref = robot->position(tsid->data(), robot->model().getJointId(lfoot_frame));
        contactLF->setReference(H_lf_ref);
        TrajectorySample sl(12, 6);
        SE3ToVector(H_lf_ref, sl.pos);
        leftFootTask->setReference(sl);


        if (!LF_ContactTask)
        {
          cout<<"Added LSS"<<endl;
          tsid->addRigidContact(*contactLF, w_forceReg);
          LF_ContactTask = true;
        }

    //Size Pos 12 x y z RotMatrix, Size vel 6
    TrajectorySample sr(12, 6);
    SE3ToVector(H_rf_ref, sr.pos);
    sr.vel.head(3) = M_rf_ref.linear();
    sr.vel.tail(3) = M_rf_ref.angular();
    sr.acc.head(3) = A_rf_ref.linear();
    sr.acc.tail(3) = A_rf_ref.angular();
    rightFootTask->setReference(sr);

    }
   


    //H_rf_ref = robot->position(tsid->data(), robot->model().getJointId(rfoot_frame));


    if ((M_lf_ref.linear()).squaredNorm() > eps && (M_rf_ref.linear()).squaredNorm() < eps )
    {

      if (LF_ContactTask)
      {
         cout<<"Removed LSS"<<endl;
        tsid->removeRigidContact(contactLF->name());
        LF_ContactTask = false;
      }

        contactRF->setReference(H_rf_ref);
        TrajectorySample sr(12, 6);
        SE3ToVector(H_rf_ref, sr.pos);
        rightFootTask->setReference(sr);

        if (!RF_ContactTask)
        {
          cout<<"Added RSS"<<endl;
          tsid->addRigidContact(*contactRF, w_forceReg);
          RF_ContactTask = true;
        }
              TrajectorySample sl(12, 6);
      SE3ToVector(H_lf_ref, sl.pos);
      sl.vel.head(3) = M_lf_ref.linear();
      sl.vel.tail(3) = M_lf_ref.angular();
      sl.acc.head(3) = A_lf_ref.linear();
      sl.acc.tail(3) = A_lf_ref.angular();
      leftFootTask->setReference(sl);
    }
 


    const HQPData &HQPData = tsid->computeProblemData(t, q, dq);

    solver->resize(tsid->nVar(), tsid->nEq(), tsid->nIn());

    const HQPOutput &sol = solver->solve(HQPData);
    if (!sol.status == HQP_STATUS_OPTIMAL)
    {
      cout << "QP Not Solved " << endl;
    }


    ddqd = tsid->getAccelerations(sol);
    VectorXd tau = tsid->getActuatorForces(sol);

    // VectorXd f_RF = sol.x.segment<12>(robot->nv());
    // VectorXd f_LF = sol.x.segment<12>(robot->nv() + 12);
    // cout << "Right Force " << f_RF.transpose() << endl;
    // cout << "Left Force " << f_LF.transpose() << endl;
    // cout<<"Optimal Solution tau "<<tau.transpose()<<endl;

    //Desired Command Data
    taud.tail(robot->nv() - 6) = tau;

    dqd = dqd + dt * ddqd;
    // cout<<"Optimal Solution dq "<<dqd.transpose()<<endl;
    // cout<<"Optimal Solution tau "<<taud.transpose()<<endl;

    qd = pinocchio::integrate(robot->model(), qd, dt * dqd);
    t += dt;
  }
};

int
main(int argc, char *argv[])
{

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;
  double dt = 0.005;
  int micro_dt = 5000;
  world.setTimeStep(dt);
  world.setERP(0, 0);
  //world.setGravity(Eigen::Vector3d(0,0,0));

  /// create objects
  auto ground = world.addGround();

  auto atlas = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\atlas\\robot.urdf");

  vector<string> package_dirs;
  package_dirs.push_back(atlas_model_path);
  string urdfFileName = package_dirs[0] + "/robots/robot.urdf";
  RobotWrapper desired_robot(urdfFileName, package_dirs, pinocchio::JointModelFreeFlyer());

  Eigen::MatrixXd State = parseCSV("/home/master/talos_sot/src/crocoddyl/build/examples/AtlasStateX.csv", atlas->getGeneralizedCoordinateDim() + atlas->getDOF());
  Eigen::MatrixXd Control = parseCSV("/home/master/talos_sot/src/crocoddyl/build/examples/AtlasControlU.csv", atlas->getDOF() - 6);

  std::vector<std::string> JointNames = atlas->getMovableJointNames();
  std::cout << "Print " << JointNames.size() << " DoF " << atlas->getDOF() << std::endl;
  atlas->printOutFrameNamesInOrder();

  Eigen::VectorXd jointNominalConfig(atlas->getGeneralizedCoordinateDim()), jointVelocityTarget(atlas->getDOF()), jointTorqueTarget(atlas->getDOF());
  jointNominalConfig.setZero();
  jointTorqueTarget.setZero();

  jointNominalConfig << 0, 0, 0.89, 1, 0, 0, 0, 0, 0, 0, 0, -1.57, 0, 0, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  jointNominalConfig[7 + 1] = 0.035;
  jointNominalConfig[7 + 17] = 0;
  jointNominalConfig[7 + 18] = 0;
  jointNominalConfig[7 + 19] = -0.48760839223861694;
  jointNominalConfig[7 + 20] = 0.8850983381271362;
  jointNominalConfig[7 + 21] = -0.43169957399368286;

  jointNominalConfig[7 + 23] = 0;
  jointNominalConfig[7 + 24] = 0;
  jointNominalConfig[7 + 25] = -0.48760839223861694;
  jointNominalConfig[7 + 26] = 0.8850983381271362;
  jointNominalConfig[7 + 27] = -0.43169957399368286;
  //jointNominalConfig<<  0.0057802,  0.000153296,     0.873637, 1,0,0,0,   5.29486e-06,    0.0359425,  -0.00020097, -7.95555e-05,     -1.56859,  1.48486e-05,   0.00160975, -3.75723e-06,  8.09953e-05,  3.73558e-08,   7.9745e-05 ,     1.56866,  1.67398e-05,   -0.0013876,  4.51927e-06,   0.00012897,  9.52007e-08,   0.00107128,  7.45992e-05,    -0.479406,     0.932279,    -0.431951,  6.42818e-06,  0.000893931,  1.96781e-05,    -0.479232,     0.931801,     -0.43167,  0.000141119;
  for (int i = 7; i < jointNominalConfig.size(); i++)
  {
    std::cout << JointNames[i - 6] << " " << jointNominalConfig[i] << std::endl;
  }

  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(atlas->getDOF()), jointDgain(atlas->getDOF());
  //jointPgain<< 0,0,0,0,0,0, 5000, 5000, 5000, 2000, 1000, 200, 200, 50, 100, 50, 2000, 1000, 200, 200, 50, 100, 50,  50, 1900,2000, 2000, 2900, 2300, 50, 1900,2000, 2000, 2900, 2300;
  jointPgain.setZero();
  jointDgain.setZero();

  //jointPgain << 0, 0, 0, 0, 0, 0, 10000, 10000, 10000, 2000, 1000, 200, 200, 50, 100, 50, 2000, 1000, 200, 200, 50, 100, 50, 2000, 5900, 10000, 6000, 4900, 3300, 2000, 5900, 10000, 6000, 4900, 3300;
  jointPgain.tail(atlas->getDOF()-6) = 5500 * Vector::Ones(atlas->getDOF()-6);
  jointPgain[6 + 25] = 5500;
  jointPgain[6 + 26] = 5500;
  jointPgain[6 + 27] = 5500;
  jointPgain[6 + 19] = 5500;
  jointPgain[6 + 20] = 5500;
  jointPgain[6 + 21] = 5500;
  
  
  jointDgain.tail(atlas->getDOF()-6) = 650 * Vector::Ones(atlas->getDOF()-6);

  //jointDgain << 0, 0, 0, 0, 0, 0, 0.1, 2.0, 1.0, 3.0, 20.0, 3.0, 3.0, 0.1, 0.2, 0.1, 3.0, 20.0, 3.0, 3.0, 0.1, 0.2, 0.1, 0.1, 1.0, 10.0, 10.0, 2.0, 1.0, 0.1, 1.0, 10.0, 10.0, 2.0, 1.0;

  atlas->setGeneralizedCoordinate(jointNominalConfig);
  atlas->setGeneralizedVelocity(jointVelocityTarget);
  atlas->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  atlas->setGeneralizedForce(jointTorqueTarget);
  atlas->setPdGains(jointPgain, jointDgain);
  atlas->setName("atlas");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(atlas);

  Eigen::VectorXd q(atlas->getGeneralizedCoordinateDim()), dq(atlas->getDOF());
  std::vector<std::string> jnames = atlas->getMovableJointNames();
  Eigen::VectorXd qd(atlas->getGeneralizedCoordinateDim()), dqd(atlas->getDOF()), taud(atlas->getDOF());
  qd = jointNominalConfig;
  dqd = jointVelocityTarget;
  taud = jointTorqueTarget;
  auto itr = std::find(jnames.begin(), jnames.end(), "ROOT");
  if (itr != jnames.end())
    jnames.erase(itr);

  string lfoot_frame = "l_leg_akx";
  string rfoot_frame = "r_leg_akx";
  string base_frame = "pelvis";

  Vector3d command_CoM_pos, command_CoM_pos_;
  humanoidTaskData htd;

  htd.base_frame = base_frame;
  htd.lfoot_frame = lfoot_frame;
  htd.rfoot_frame = rfoot_frame;
  htd.joint_names = jnames;

  Vector3d CoM_pos_ref, CoM_vel_ref, CoM_acc_ref, lf_pos_ref, rf_pos_ref, lf_linear_vel_ref, rf_linear_vel_ref, lf_angular_vel_ref, rf_angular_vel_ref, base_pos_ref, base_linear_vel_ref, base_angular_vel_ref;
  Quaterniond lf_orientation_ref, rf_orientation_ref, base_orientation_ref;
  VectorXd joint_states_ref;


  //Add the contact constraints
  double t = 0.0;

  pinocchio::SE3 H_rf_ref;
  pinocchio::SE3 H_lf_ref;
  pinocchio::Motion M_rf_ref;
  pinocchio::Motion M_lf_ref;
  pinocchio::Motion A_rf_ref;
  pinocchio::Motion A_lf_ref;

  pinocchio::Data desired_data(desired_robot.model());
  swapQuatXYZW(jointNominalConfig);
  desired_robot.computeAllTerms(desired_data, jointNominalConfig, jointVelocityTarget);

  // Create an HQP solver
  //SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, "eiquadprog_fast");

  /** LIPM CONTROL **/
  double mass = atlas->getTotalMass();
  LIPMControl lc = LIPMControl(1.14398, mass, dt);
  cout << "Mass of Atlas " << mass << endl;
  RobotParameters atlasParams;
  ZMPDistributor atlasZMPDistributor(atlasParams);
  postureStabilizer atlasStabilizer(atlasParams);
  /* ------------- */

  int init_idx = 0;
  int idx = 0;
  int start_idx = 500;

  bool LSS;
  bool RSS;
  bool DS;
  bool tsid_initialized = false;
  AtlasInverseDynamics* AtlasID;

  VectorXd CoM_trajectory, CoM_ref_trajectory;

  while (1)
  { 
    auto start = high_resolution_clock::now();


    if (init_idx < 100)
    {
      init_idx++;
    }
    else
    {

      atlas->getState(q, dq);
      swapQuatXYZW(q);
      if(!tsid_initialized)
      {
        AtlasID =  new AtlasInverseDynamics(package_dirs,  urdfFileName,  dt,  lfoot_frame,  rfoot_frame,  q,  dq);
        tsid_initialized = true;
      }

      /* ----------------------------------*/
      /* ----------------------------------*/
      /* ----------------------------------*/
      /* Compute Contact Data */
      auto RfootIndex = atlas->getBodyIdx("r_foot");
      auto LfootIndex = atlas->getBodyIdx("l_foot");
      auto RfootFrameIndex = atlas->getFrameIdxByName("r_leg_akx");
      auto LfootFrameIndex = atlas->getFrameIdxByName("l_leg_akx");

      int jj = 0;
      Eigen::Vector3d RLegGRF, RLegGRT, LLegGRF, LLegGRT, footForce, footTorque, LLeg_COP, RLeg_COP;

      raisim::Vec<3> footPosition;
      raisim::Mat<3, 3> footOrientation;
      atlas->getFramePosition(RfootFrameIndex, footPosition);
      atlas->getFrameOrientation(RfootFrameIndex, footOrientation);
      Eigen::Vector3d RfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
      Eigen::Matrix3d RfootOrientation;
      RfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
      //std::cout<<"Right Foot Pos"<< RfootPosition.transpose()<<std::endl;
      Affine3d Tir;
      Tir.translation() = RfootPosition;
      Tir.linear() = RfootOrientation;
      atlas->getFramePosition(LfootFrameIndex, footPosition);
      atlas->getFrameOrientation(LfootFrameIndex, footOrientation);
      Eigen::Vector3d LfootPosition = Eigen::Vector3d(footPosition[0], footPosition[1], footPosition[2]);
      Eigen::Matrix3d LfootOrientation;
      LfootOrientation << footOrientation[0], footOrientation[1], footOrientation[2], footOrientation[3], footOrientation[4], footOrientation[5], footOrientation[6], footOrientation[7], footOrientation[8];
      Affine3d Til;
      Til.translation() = LfootPosition;
      Til.linear() = LfootOrientation;

      RLegGRT.setZero();
      LLegGRT.setZero();
      RLegGRF.setZero();
      LLegGRF.setZero();
      /// for all contacts on the robot, check ...
      RSS = false;
      LSS = false;
      DS = false;

      for (auto &contact : atlas->getContacts())
      {
        if (contact.skip())
          continue; /// if the contact is internal, one contact point is set to 'skip'
        if (RfootIndex == contact.getlocalBodyIndex())
        {
          //std::cout<<"Contact impulse in the contact frame: "<<contact.getImpulse()->e()/0.005<<std::endl;
          //std::cout<<"Total Mass"<<atlas->getTotalMass()<<std::endl;
          footForce = contact.getContactFrame().e().transpose() * contact.getImpulse()->e() / dt;
          RLegGRF += footForce;
          footTorque = (contact.getPosition().e() - RfootPosition).cross(footForce);
          RLegGRT += footTorque;
          //cout<<"Right Contact Point "<<(contact.getPosition().e() - RfootPosition).transpose()<<endl;
          if (contact.isObjectA())
            RSS = true;
          /// the impulse is acting from objectB to objectA. You can check if this object is objectA or B by
          //std::cout<<"Contact frame: \n"<<contact.getContactFrame().e().transpose()<<std::endl;
          /// contact frame is transposed.
          // std::cout<<"-------------"<<std::endl;
          // std::cout<<"Cotnact Position J "<<jj<<" "<<contact.getPosition()<<std::endl;
          // std::cout<<"Contact impulse in the world frame: "<<tmp<<std::endl;
          // std::cout<<"Contact Normal in the world frame: "<<contact.getNormal().e().transpose()<<std::endl;
          // std::cout<<"-------------"<<std::endl;
          jj++;
          //std::cout<<"Contact position in the world frame: "<<contact.getPosition().e().transpose()<<std::endl;
          //std::cout<<"It collides with: "<<world.getObject(contact.getPairObjectIndex())<<std::endl;
        }
        if (LfootIndex == contact.getlocalBodyIndex())
        {

          footForce = contact.getContactFrame().e().transpose() * contact.getImpulse()->e() / dt;
          LLegGRF += footForce;
          //cout<<"Left Contact Point "<<(contact.getPosition().e() - LfootPosition).transpose()<<endl;
          footTorque = (contact.getPosition().e() - LfootPosition).cross(footForce);
          LLegGRT += footTorque;
          if (contact.isObjectA())
            LSS = true;
        }
      }
      // cout<<"Left Force "<<LLegGRF(2)<<endl;
      // cout<<"Right Force "<<RLegGRF(2)<<endl;

      if (LSS && RSS)
        DS = true;

      // std::cout << "RSS " << RSS << std::endl;
      // std::cout << "LSS " << LSS << std::endl;
      // std::cout << "DS " << DS << std::endl;

      if (LLegGRF(2) > 10)
        LLeg_COP = Eigen::Vector3d(-LLegGRT(1) / LLegGRF(2), LLegGRT(0) / LLegGRF(2), 0) + LfootPosition;
      else
        LLeg_COP.setZero();

      if (RLegGRF(2) > 10)
        RLeg_COP = Eigen::Vector3d(-RLegGRT(1) / RLegGRF(2), RLegGRT(0) / RLegGRF(2), 0) + RfootPosition;
      else
        RLeg_COP.setZero();
      /* ----------------------------------*/
      /* ----------------------------------*/
      /* ----------------------------------*/
      /* ----------------------------------*/
      /* ----------------------------------*/

      if (idx > start_idx && idx < Control.rows() + start_idx)
      {
        //Read Desired Configuration Data
        jointNominalConfig = State.row(idx - start_idx).head(atlas->getGeneralizedCoordinateDim());
        jointVelocityTarget = State.row(idx - start_idx).tail(atlas->getDOF());
        jointTorqueTarget.tail(Control.cols()) = Control.row(idx - start_idx);

        //cout<<"Q REF"<<jointNominalConfig.transpose()<<endl;
        // cout<<"CoM "<<desired_CoM<<endl;
        // cout<<"H_rf_ref "<<lf_pos_ref.transpose()<<" actual "<<lf_pos.transpose()<<endl;
        // cout<<"H_lf_ref "<<rf_pos_ref.transpose()<<" actual "<<rf_pos.transpose()<<endl;
        // cout << "H_rf_ref " << lf_orient_ref.w() << lf_orient_ref.x() << lf_orient_ref.y() << lf_orient_ref.z() << " actual " << lf_orient.w() << lf_orient.x() << lf_orient.y() << lf_orient.z() << endl;
        // cout << "H_rf_ref " << rf_orient_ref.w() << rf_orient_ref.x() << rf_orient_ref.y() << rf_orient_ref.z() << " actual " << rf_orient.w() << rf_orient.x() << rf_orient.y() << rf_orient.z() << endl;
      }
      else if(idx > Control.rows() + start_idx)
      {
        break;
      }
      Vector3d ZMP = RLeg_COP + LLeg_COP;
      ZMP(2) /= 2.0;



      desired_robot.computeAllTerms(desired_data, jointNominalConfig, jointVelocityTarget);

      joint_states_ref = jointNominalConfig.tail(atlas->getGeneralizedCoordinateDim() - 7);

      base_pos_ref = Vector3d(jointNominalConfig[0], jointNominalConfig[1], jointNominalConfig[2]);
      base_linear_vel_ref = Vector3d(jointVelocityTarget[0], jointVelocityTarget[1], jointVelocityTarget[2]);

      base_orientation_ref = Quaterniond(jointNominalConfig[6], jointNominalConfig[3], jointNominalConfig[4], jointNominalConfig[5]);
      base_angular_vel_ref = Vector3d(jointVelocityTarget[3], jointVelocityTarget[4], jointVelocityTarget[5]);

      CoM_pos_ref = desired_robot.com(desired_data);
      CoM_vel_ref = desired_robot.com_vel(desired_data);
      CoM_acc_ref = desired_robot.com_acc(desired_data);

      raisim::Vec<3> linear_momentum = atlas->getLinearMomentum();
      raisim::Vec<3> center_of_mass = atlas->getCompositeCOM();
      Vector3d CoM_vel = Vector3d(linear_momentum(0), linear_momentum(1), linear_momentum(2)) / mass;
      Vector3d CoM_pos = Vector3d(center_of_mass(0), center_of_mass(1), center_of_mass(2));
      lc.Control(ZMP, CoM_pos, CoM_vel, CoM_pos_ref, CoM_vel_ref, CoM_acc_ref);
      Vector3d ZMP_d = lc.getDesiredZMP();

      atlasZMPDistributor.computeDistribution(ZMP_d, ZMP, LLegGRF, RLegGRF, LfootPosition, RfootPosition, Til, Tir, RSS, DS);
      atlasStabilizer.footTorqueStabilizer(atlasZMPDistributor.tauld, atlasZMPDistributor.taurd, LLegGRT, RLegGRT, RSS, LSS);

      Quaterniond lf_orientation_corr = AngleAxisd(atlasStabilizer.dL_Roll, Vector3d::UnitX()) * AngleAxisd(atlasStabilizer.dL_Pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
      Quaterniond rf_orientation_corr = AngleAxisd(atlasStabilizer.dR_Roll, Vector3d::UnitX()) * AngleAxisd(atlasStabilizer.dR_Pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());

      Quaterniond base_orientation = Quaterniond(q[6], q[3], q[4], q[5]);
      Vector3d euler = base_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
      double yaw = euler[0];
      double pitch = euler[1];
      double roll = euler[2];

      atlasStabilizer.baseOrientationStabilizer(roll, pitch, 0, 0);
      Quaterniond base_orientation_corr = AngleAxisd(atlasStabilizer.dbase_Roll, Vector3d::UnitX()) * AngleAxisd(atlasStabilizer.dbase_Pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());


    M_rf_ref = desired_robot.velocity(desired_data, desired_robot.model().getJointId(rfoot_frame));
    M_lf_ref = desired_robot.velocity(desired_data, desired_robot.model().getJointId(lfoot_frame));

    A_rf_ref = desired_robot.acceleration(desired_data, desired_robot.model().getJointId(rfoot_frame));
    A_lf_ref = desired_robot.acceleration(desired_data, desired_robot.model().getJointId(lfoot_frame));

    H_rf_ref = desired_robot.position(desired_data, desired_robot.model().getJointId(rfoot_frame));
    H_lf_ref = desired_robot.position(desired_data, desired_robot.model().getJointId(lfoot_frame));

    CoM_pos_ref = lc.getDesiredCoMPosition();
    CoM_vel_ref = lc.getDesiredCoMVelocity();

    CoM_acc_ref = lc.getDesiredCoMAcceleration();

    VectorXd vec_joined(CoM_trajectory.size() + CoM_pos.size());
    vec_joined << CoM_trajectory, CoM_pos;
    CoM_trajectory = vec_joined;

    vec_joined.resize(CoM_ref_trajectory.size() + CoM_pos_ref.size());
    vec_joined << CoM_ref_trajectory, CoM_pos_ref;
    CoM_ref_trajectory = vec_joined;

    
      AtlasID->computeInverseDynamics( q,  dq,  joint_states_ref,  CoM_pos_ref,  CoM_vel_ref,  CoM_acc_ref, ZMP_d,  H_rf_ref,   H_lf_ref,  M_rf_ref,   M_lf_ref,    A_rf_ref,    A_lf_ref, RSS, LSS);
      qd = AtlasID->qd;
      dqd = AtlasID->dqd;
      taud = AtlasID->taud;
      swapQuatWXYZ(qd);
    }
    //std::cout<<"QD "<<q.transpose()<<std::endl;

    atlas->setGeneralizedForce(taud);
    atlas->setPdTarget(qd, dqd);
    // atlas->setGeneralizedCoordinate(qd);
    // atlas->setGeneralizedVelocity(dqd);

    // std::cout<<"COP RIGHT "<<RLeg_COP.transpose()<<std::endl;
    // std::cout<<"COP LEFT "<<LLeg_COP.transpose()<<std::endl;

    server.integrateWorldThreadSafe();
    idx++;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout <<"Microseconds "<< duration.count() << endl;

    //std::this_thread::sleep_for(std::chrono::microseconds(micro_dt));

  }
    writeToCSVfile("Atlas_CoM_ref", CoM_ref_trajectory);
    writeToCSVfile("Atlas_CoM", CoM_trajectory);

  server.killServer();
}