#pragma once

#ifndef PHANTOMOMNIROS_H
#define PHANTOMOMNIROS_H

#include <QObject>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <phantom_omni/OmniState.h>
#include <medlab_common/robotics_math.h>
#include <ros/ros.h>

Q_DECLARE_METATYPE(RoboticsMath::Vector6d)

class PhantomOmniRos  : public QObject
{
  Q_OBJECT

public:
  PhantomOmniRos();
  ~PhantomOmniRos();

  void init(std::string node_name);
  double getMotionScaling(void) {return motion_scale_factor_;}

public slots:
  RoboticsMath::Vector6d twistUpdate(void); // creates a desired twist based on pose_ and posePrev_
  void omniCallback(const phantom_omni::OmniState &msg);
  bool resetTwist(void) {pose_prev_ = pose_;}
  void setMotionScaling(double motion_scaling) {motion_scale_factor_ = motion_scaling;}
  void setForceScaling(double force_scaling);
  int  getButtonState(void);

signals:
  void omniButtonStateChanged(int button, bool current_state);
  void newTwist(RoboticsMath::Vector6d twist);

private:
  std::string node_name_;
  ros::NodeHandle nh_;
  bool connected_ = false; // true when connected to ROS
  ros::Publisher pub_set_force_scaling_; // sets haptic damping force in [Newtons per m/s]
  ros::Subscriber sub_state_; // current Omni state
  Eigen::Matrix4d pose_;  // current Omni pose
  Eigen::Matrix4d pose_prev_; // pose_ when twistUpdate() was last called

  double motion_scale_factor_; // default = 0.3
  double force_scale_factor_;  // default = 2.0
  bool button_states_[2] = {false, false};

  RoboticsMath::Vector6d scaleOmniVelocity(RoboticsMath::Vector6d desTwistDelta);

};

#endif // PHANTOMOMNIROS_H
