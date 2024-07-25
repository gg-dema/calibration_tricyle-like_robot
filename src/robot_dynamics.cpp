#include "header/robot_model.hpp"
#include "header/robot_dynamics.hpp"
#include "eigen3/Eigen/Core"
#include <iostream>

using namespace cal_lib;

namespace RobotModel{
    namespace Dynamics{
    
    void DynamicModel::euler_integration(StateVector& q, const StateVector q_dot, const float delta_time){
                q += q_dot*delta_time;
            }
     
    Pose2d GrisettiModel::operator()(StateVector& q, std::vector<long double> velocity, double delta_t){

        StateVector delta_pose = forward(q, velocity);
        euler_integration(q, delta_pose, delta_t); //problem: this do extra operation : integrate also the steering angle 
        q[3] = velocity[0];
        Pose2d pose = {q[0], q[1], q[2]};
        return pose;
    } 


    StateVector GrisettiModel::forward(const StateVector q, std::vector<long double> input){
        /* return the displacement done after some imput*/
        StateVector out;
        long double phi = input[0];
        long double delta_traction = input[1];

        // AKA delta_theta --> angular displacement of the reference frame
        out[2] = delta_traction*(sin(phi)/params_->axis_lenght);
        //steering
        out[3] = 0;  // this is set to 0 because later on i'll do an euler integration,
                     // i need a velocity for use this value. I prefer to set it to 0 as velocity and then
                     // set the real input in the operator() function 
        //x
        out[0] = delta_traction*sin_taylor_expansion(out[2]);
        //y
        out[1] = delta_traction*cos_taylor_expansion(out[2]);
        return out;
    }

    double GrisettiModel::sin_taylor_expansion(const double x){
        return x - pow(x, 3)/6 + pow(x, 5)/120;
    }
    double GrisettiModel::cos_taylor_expansion(const double x){
        return pow(x, 2)/2 +pow(x, 4)/24 + pow(x, 6)/720;
    }



    OrioloModel::OrioloModel(std::shared_ptr<model_parameters> params): DynamicModel(params){};
    
    Pose2d OrioloModel::operator()(StateVector& q, std::vector<long double> velocity, double delta_t){
                    
                //std::cout << "delta_t: " << delta_t << '\n';       
                StateVector q_dot = forward_kin_model(q, velocity);
                std::cout << "q: " << q << '\n';
                std::cout << "q_dot: " << q_dot << '\n';

                //update q state
                euler_integration(q, q_dot, delta_t);
                //std::cout << "q: " << q << '\n';

                Pose2d pose = {q[0], q[1], q[2]};
                return pose;
            }
            
    StateVector OrioloModel::forward_kin_model(const StateVector q, std::vector<long double> input_v){
                cal_lib::StateVector velocity;
                velocity[0] = input_v[0]*cos(q[2]);
                velocity[1] = input_v[0]*sin(q[2]);
                velocity[2] = input_v[0]*tan(q[2]) / (params_->axis_lenght);
                velocity[3] = input_v[1];
                return velocity;
            
            }
   
    } // end Dynamics namespace
}; // end RobotModel namespac

