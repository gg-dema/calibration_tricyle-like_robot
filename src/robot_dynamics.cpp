#include "header/robot_model.hpp"
#include "header/robot_dynamics.hpp"
#include "eigen3/Eigen/Core"

using namespace cal_lib;

namespace RobotModel{
    namespace Dynamics{
    
    StateVector DynamicModel::euler_integration(const StateVector q, const StateVector q_dot, const float delta_time){
                return q + q_dot*delta_time;
            }
    
    Pose2d GrisettiModel::forward(const StateVector q, const Tick input){
        Pose2d pose;
        pose[0] = q[0] + input[0]*cos(q[2]);
        pose[1] = q[1] + input[0]*sin(q[2]);
        pose[2] = q[2] + input[1];
        return pose;
    }
    
    StateVector GrisettiModel::operator()(const StateVector q, std::vector<double> input){
        StateVector q_dot;
        q_dot[0] = input[0]*cos(q[2]);
        q_dot[1] = input[0]*sin(q[2]);
        q_dot[2] = input[0]*tan(q[2])/(params_->axis_lenght);
        q_dot[3] = input[1];
        return q_dot;
    }

    double GrisettiModel::sin_taylor_expansion(const double x){
        return x - x*x*x/6 + x*x*x*x*x/120;
    }
    double GrisettiModel::cos_taylor_expansion(const double x){
        return 1 - x*x/2 + x*x*x*x/24;
    }

    OrioloModel::OrioloModel(std::shared_ptr<model_parameters> params): DynamicModel(params){};

    StateVector OrioloModel::operator()(const StateVector q, std::vector<double> input){
                    
                double v = input[0];
                double omega = input[1];
                double delta_t = input[3];
                StateVector q_dot = forward_kin_model(q, v, omega);
                return euler_integration(q, q_dot, delta_t);
            }
            
    StateVector OrioloModel::forward_kin_model(const StateVector q, double v, double omega){
                cal_lib::StateVector velocity;
                velocity[0] = v*cos(q[2]);
                velocity[1] = v*sin(q[2]);
                velocity[2] = omega*tan(q[2])/(params_->axis_lenght);
                velocity[3] = v;
                return velocity;
            
            }
   
    } // end Dynamics namespace
}; // end RobotModel namespac

