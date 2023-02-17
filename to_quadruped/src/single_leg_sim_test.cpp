#include"pinocchio/parsers/urdf.hpp"
#include"pinocchio/algorithm/joint-configuration.hpp"
#include"pinocchio/algorithm/kinematics.hpp"
#include"pinocchio/algorithm/frames.hpp"
#include"pinocchio/algorithm/jacobian.hpp"
#include"pinocchio/algorithm/rnea.hpp"
#include"pinocchio/parsers/sample-models.hpp"
#include"pinocchio/algorithm/aba.hpp"
#include"pinocchio/algorithm/crba.hpp"
#include"pinocchio/algorithm/contact-dynamics.hpp"
#include"pinocchio/algorithm/centroidal.hpp"
#include"pinocchio/algorithm/energy.hpp"

#include<ros/ros.h>
#include<fstream>
#include<iostream>
#include<std_msgs/Float64.h>
#include<Eigen/Core>
#include<sensor_msgs/JointState.h>
#include<matplotlibcpp.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include<Eigen/Dense>
#include<matplotlibcpp.h>

namespace pin=pinocchio;
namespace plt=matplotlibcpp;

typedef Eigen::Matrix<double,3,1> JointAngles;
typedef Eigen::Matrix<double,6,1> State;
typedef Eigen::Matrix<double,3,1> Input;

double Kps[3]={50.0, 8.0, 5.0};
double Kds[3]={5.0, 5.0, 2.0};
double initial_angles[3]={0.0,0.0,0.0};
double torque[3]={-0.5,0.1,0.12};
JointAngles joint_angles;

State simulate_pin(const pin::Model& model, pin::Data& data, const State& state){
  auto q=state.head(3);
  auto v=state.tail(3);
  pin::forwardKinematics(model, data, q, v);
  pin::framesForwardKinematics(model, data, q);
  Input tau;
  for(int i=0;i<3;++i){
    tau[i]=torque[i];
  }
  State dstate;  dstate.setZero();
  dstate.head(3)=v;
  dstate.tail(3)=pin::aba(model,data,q,v,tau);
  
  return dstate;
}

void get_state(const sensor_msgs::JointState::ConstPtr& joint_state){
    for(int i=0;i<3;++i){
        joint_angles[i]=joint_state->position[i];
    }
}

int main(int argc, char *argv[])
{
    srand(time(0));
    ros::init(argc, argv, "single_leg_controller");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/single_leg/joint_states", 1, get_state);
    ros::Publisher servo_pub[3];
    servo_pub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/single_leg/FR_hip_controller/command", 1);
    servo_pub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/single_leg/FR_thigh_controller/command", 1);
    servo_pub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/single_leg/FR_calf_controller/command", 1);
    
    ros::Rate r(900);
    unitree_legged_msgs::MotorCmd motor_cmd;
    motor_cmd.mode=0x0A;
    
    std::vector<double> joint_angles_recorder[3];
    int counter=0;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state
    
    while(ros::ok()){
        if(counter>3000) break;
        for(int i=0;i<3;++i){
            motor_cmd.q=initial_angles[i];
            motor_cmd.Kp=Kps[i];
            motor_cmd.Kd=Kds[i];
            servo_pub[i].publish(motor_cmd);
        }
        ros::spinOnce();
        r.sleep();
        counter++;
    }
    
    counter=0;
    while(ros::ok()){
        if(counter>3000) break;
        for(int i=0;i<3;++i){
            motor_cmd.tau=torque[i];
            motor_cmd.Kp=motor_cmd.Kd=0;
            servo_pub[i].publish(motor_cmd);
        }
        ros::spinOnce();
        for(int i=0;i<3;++i){
            joint_angles_recorder[i].push_back(joint_angles[i]);
        }
        r.sleep();
        counter++;
    }

    State state;
    double time_step=0.0005;
    double total_time=3;

    std::vector<double> hip_angles_record1, hip_angles_record2;
    std::vector<double> thigh_angles_record1, thigh_angles_record2;
    std::vector<double> calf_angles_record1, calf_angles_record2;

    pin::Model model_pin;
    pin::urdf::buildModel("/home/zhangduo/catkin_ws/src/to_quadruped/urdf/single_leg.urdf",model_pin);
    pin::Data data(model_pin);

    state.setZero();
    for(int i=0;i<total_time / time_step;++i){
        auto k1=simulate_pin(model_pin, data, state);
        auto k2=simulate_pin(model_pin, data, state+0.5*k1*time_step);
        auto k3=simulate_pin(model_pin, data, state+0.5*k2*time_step);
        auto k4=simulate_pin(model_pin, data, state+k3*time_step);

        state += 1.0/6 * time_step * (k1+2*k2+2*k3+k4);
        if(i%2==0){
            hip_angles_record1.push_back(state[0]);
            thigh_angles_record1.push_back(state[1]);
            calf_angles_record1.push_back(state[2]);
        }
    }

    time_step=0.001;
    state.setZero();
    for(int i=0;i<total_time / time_step;++i){
        auto k1=simulate_pin(model_pin, data, state);
        auto k2=simulate_pin(model_pin, data, state+0.5*k1*time_step);
        auto k3=simulate_pin(model_pin, data, state+0.5*k2*time_step);
        auto k4=simulate_pin(model_pin, data, state+k3*time_step);

        state += 1.0/6 * time_step * (k1+2*k2+2*k3+k4);
        hip_angles_record2.push_back(state[0]);
        thigh_angles_record2.push_back(state[1]);
        calf_angles_record2.push_back(state[2]);
    }

    plt::figure();
    plt::named_plot("calf_joint",joint_angles_recorder[0]);
    plt::named_plot("calf_joint simulate step:0.0005",calf_angles_record1);
    plt::named_plot("calf_joint simulate step:0.001",calf_angles_record2);
    plt::grid(true);
    plt::legend();
    
    plt::figure();
    plt::named_plot("hip_joint",joint_angles_recorder[1]);
    plt::named_plot("hip_joint simulate step:0.0005",hip_angles_record1);
    plt::named_plot("hip_joint simulate step:0.001",hip_angles_record2);
    plt::grid(true);
    plt::legend();
    
    plt::figure();
    plt::named_plot("thigh_joint",joint_angles_recorder[2]);
    plt::named_plot("thigh_joint simulate step:0.0005",thigh_angles_record1);
    plt::named_plot("thigh_joint simulate step:0.001",thigh_angles_record2);
    plt::grid(true);
    plt::legend();
    
    plt::show();
    
    return 0;
}
