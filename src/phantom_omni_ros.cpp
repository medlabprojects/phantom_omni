#include <phantom_omni/phantom_omni_ros.h>
#include <medlab_common/robotics_math.h>

#include <QObject>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <string>
#include <phantom_omni/OmniState.h>
#include <std_msgs/Float32.h>
#include <Mtransform.h>
#include <ros/ros.h>


using namespace Eigen;

PhantomOmniRos::PhantomOmniRos()
  : pose_(Eigen::Matrix4d::Identity())
  , pose_prev_(Eigen::Matrix4d::Identity())
  , motion_scale_factor_(0.10)
  , force_scale_factor_(5.0)
{
  qRegisterMetaType< RoboticsMath::Vector6d >();
}

PhantomOmniRos::~PhantomOmniRos()
{
  pub_set_force_scaling_.shutdown();
  sub_state_.shutdown();
}

void PhantomOmniRos::init(std::string node_name)
{
  node_name_ = node_name;

  // create publisher
  std::string topic_force_scaling = "/" + node_name_+ "/force_scaling";
  pub_set_force_scaling_ = nh_.advertise<std_msgs::Float32>(topic_force_scaling.c_str(),1);

  // create subscriber
  std::string topic_state = "/" + node_name_ + "/state";
  sub_state_ = nh_.subscribe(topic_state.c_str(), 1, &PhantomOmniRos::omniCallback, this);
}


void PhantomOmniRos::omniCallback(const phantom_omni::OmniState &msg)
{
  // copy transform from message and remap as a Matrix4f
  float omni_transform[16];
  std::copy(std::begin(msg.transform), std::end(msg.transform), std::begin(omni_transform));
  Eigen::Map< Eigen::Matrix<float,4,4, Eigen::ColMajor> > pose_temp(omni_transform);

  Eigen::Matrix4d pose_d = pose_temp.cast<double>(); // cast from float to double

  // Rotate Omni frame by 180 deg about Y to modify pose
  Eigen::Matrix4d omniReg1 = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd rotationY = Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
  Mtransform::SetRotation(omniReg1,rotationY);
  Eigen::Matrix4d omniReg1Inv = Mtransform::Inverse(omniReg1);
  Eigen::Matrix4d pose_temp_reg;
  pose_temp_reg = omniReg1Inv*pose_d*omniReg1;
  pose_ = pose_temp_reg.cast<double>();

  // update button states
  bool button_states_prev[2] = {button_states_[0], button_states_[1]};
  button_states_[0] = msg.button1;
  button_states_[1] = msg.button2;

  // check if either button was just pressed
  if(button_states_[0] != button_states_prev[0]){
    emit(omniButtonStateChanged(1, button_states_[0]));
  }
  if(button_states_[1] != button_states_prev[1]){
    emit(omniButtonStateChanged(2, button_states_[1]));
  }

  // TEST (should only be called by controller.step)
//  twistUpdate();
  // !TEST
}

void PhantomOmniRos::setForceScaling(double force_scaling)
{
  force_scale_factor_ = force_scaling;

  std_msgs::Float32 msg;
  msg.data = force_scale_factor_;
  pub_set_force_scaling_.publish(msg);
}

int PhantomOmniRos::getButtonState()
{
  // 0 => no button pressed
  // 1 => button 1 pressed
  // 2 => button 2 pressed
  // 3 => both 1 & 2 pressed
  return ((int)(button_states_[0])+(int)(button_states_[1]));
}

RoboticsMath::Vector6d PhantomOmniRos::twistUpdate()
{
  // This uses pose_ and posePrev_ to update desTwist

  // compute change
  Eigen::Matrix4d omniDeltaOmniPenCoords = Mtransform::Inverse(pose_prev_)*pose_;

  // convert [mm] to [m] and scale down by omniScaleFactor
  omniDeltaOmniPenCoords.block(0,3,3,1) = omniDeltaOmniPenCoords.block(0,3,3,1)/1.0E3;
  omniDeltaOmniPenCoords.block(0,3,3,1) = motion_scale_factor_*omniDeltaOmniPenCoords.block(0,3,3,1);

  // ????
  Eigen::Matrix4d RposePrev = RoboticsMath::assembleTransformation(pose_prev_.block(0,0,3,3), Eigen::Vector3d::Zero());
  Eigen::Matrix4d omniDeltaOmniBaseCoords = (Mtransform::Inverse(RposePrev.transpose())*omniDeltaOmniPenCoords*RposePrev.transpose());

//  Eigen::Matrix3d Rd = pose_.block<3,3>(0,0);
//  Eigen::Matrix3d Rc = pose_prev_.block<3,3>(0,0);
//  Eigen::Matrix3d Re = Rd*Rc.transpose();
//  Re = RoboticsMath::orthonormalize(Re);

  // desTwist should use OmniBaseCoords for linear velocity, and OmniPenCoords for angular velocity
  RoboticsMath::Vector6d desTwist = RoboticsMath::Vector6d::Zero();
  desTwist(0) = omniDeltaOmniBaseCoords(0,3); // vx
  desTwist(1) = omniDeltaOmniBaseCoords(1,3); // vy
  desTwist(2) = omniDeltaOmniBaseCoords(2,3); // vz
  desTwist(3) = omniDeltaOmniPenCoords(2,1);  // wx
  desTwist(4) = omniDeltaOmniPenCoords(0,2);  // wy
  desTwist(5) = omniDeltaOmniPenCoords(1,0);  // wz

  // scale down
//  desTwist = scaleOmniVelocity(desTwist);

  // update posePrev_
  pose_prev_ = pose_;

  emit newTwist(desTwist);
  return desTwist;
}

RoboticsMath::Vector6d PhantomOmniRos::scaleOmniVelocity(RoboticsMath::Vector6d desTwistDelta)
{
  RoboticsMath::Vector6d scaledDesTwistDelta = desTwistDelta;

  // TODO: get rosLoopRate from top level
  double rosLoopRate = 100.0; // [Hz]

  //double pStepMax = 2.0e-3; // 2 mm
  double vMax = 0.1; // [m/s]
  double pStepMax = vMax / rosLoopRate; // [mm]

  double pErr = desTwistDelta.topRows<3>().norm();
  double gain = 1.0;
  if (pErr > pStepMax)
  {
    gain = 	pStepMax / pErr;
  }
  scaledDesTwistDelta.topRows<3>() *= gain;

  //double angleStepMax = 0.05; //0.05 radians
  double wMax = 2.5; // [rad/s]
  double angleStepMax = wMax / rosLoopRate;
  double angleErr = desTwistDelta.bottomRows<3>().norm();
  gain = 1.0;
  if (angleErr > angleStepMax)
  {
    gain = angleStepMax / angleErr;
  }
  scaledDesTwistDelta.bottomRows<3>() *= gain;

  return scaledDesTwistDelta;
}
