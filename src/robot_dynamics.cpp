#include "header/robot_model.hpp"
#include "header/robot_dynamics.hpp"
#include "eigen3/Eigen/Core"
#include <iostream>

using namespace cal_lib;

namespace RobotModel{
    namespace Dynamics{
    
    void DynamicModel::euler_integration(StateVector& q, StateVector q_dot, const float delta_time){
                //std::cout << "euler: delta time" << delta_time << '\n';
                //StateVector out = q + q_dot*delta_time;
                //std::cout << out << '\n';
                q += q_dot*delta_time;
            }
    
    StateVector GrisettiModel::forward(const StateVector q, std::vector<double> input){
        /* return the displacement done after some imput*/
        StateVector out;
        double phi = input[1];
        double back_wheel_disp = input[0];

        // AKA delta_theta --> angular displacement of the reference frame
        out[2] = back_wheel_disp*(sin(phi)/params_->axis_lenght);
        //steering
        out[3] = 0;
        //x
        out[0] = input[0]*sin_taylor_expansion(out[2]);
        //y
        out[1] = input[0]*cos_taylor_expansion(out[2]);
        return out;
    }
    
    Pose2d GrisettiModel::operator()(StateVector& q, std::vector<double> velocity, double delta_t){

        StateVector delta_pose = forward(q, velocity);
        std::cout << "q: " << q << '\n';
        euler_integration(q, delta_pose, delta_t); //problem: this do extra operation : integrate also the steering angle 
        q[3] = velocity[1];
        //std::cout << "q: " << q << '\n';
        Pose2d pose = {q[0], q[1], q[2]};
        return pose;
    }

    double GrisettiModel::sin_taylor_expansion(const double x){
        return x - x*x*x/6 + x*x*x*x*x/120;
    }
    double GrisettiModel::cos_taylor_expansion(const double x){
        return 1 - x*x/2 + x*x*x*x/24;
    }

    OrioloModel::OrioloModel(std::shared_ptr<model_parameters> params): DynamicModel(params){};

    Pose2d OrioloModel::operator()(StateVector& q, std::vector<double> velocity, double delta_t){
                    
                //std::cout << "delta_t: " << delta_t << '\n';       
                StateVector q_dot = forward_kin_model(q, velocity);
                //std::cout << "q: " << q << '\n';
                //std::cout << "q_dot: " << q_dot << '\n';

                //update q state
                euler_integration(q, q_dot, delta_t);
                std::cout << "q: " << q << '\n';

                Pose2d pose = {q[0], q[1], q[2]};
                return pose;
            }
            
    StateVector OrioloModel::forward_kin_model(const StateVector q, std::vector<double> input_v){
                cal_lib::StateVector velocity;
                velocity[0] = input_v[0]*cos(q[2]);
                velocity[1] = input_v[0]*sin(q[2]);
                velocity[2] = input_v[0]*tan(q[2])/(params_->axis_lenght);
                velocity[3] = input_v[1];
                return velocity;
            
            }
   
    } // end Dynamics namespace
}; // end RobotModel namespac

