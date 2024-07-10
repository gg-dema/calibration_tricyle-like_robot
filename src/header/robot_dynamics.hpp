#ifndef CALIBRATION_UTILITY_HPP
#define CALIBRATION_UTILITY_HPP


//#pragma once
#include <memory>
#include "eigen3/Eigen/Core"
#include "calibration_utility.hpp"

using namespace cal_lib;

namespace RobotModel{
namespace Dynamics{

    class DynamicModel{
    public:
        DynamicModel(std::shared_ptr<model_parameters> params) : params_(params){}
        virtual Pose2d operator()(StateVector& q, std::vector<double> velocity, double delta_t)=0;
        void euler_integration(StateVector& q, StateVector q_dot, float delta_time);
        
    protected:
        std::shared_ptr<model_parameters> params_;
    };
    
    class GrisettiModel : public DynamicModel {
    public:
        GrisettiModel(std::shared_ptr<model_parameters> params) : DynamicModel(params){}
        Pose2d operator()(StateVector& q, std::vector<double> velocity, double delta_t) override; 
        StateVector forward(const StateVector, std::vector<double> velocity);
        double sin_taylor_expansion(const double x);
        double cos_taylor_expansion(const double x);
    private:            
        static constexpr int taylor_order_ = 5;
        const Eigen::Matrix<double, 2, taylor_order_> taylor_coef_;

    };  

    class OrioloModel : public DynamicModel{
    
        public:
            OrioloModel(std::shared_ptr<model_parameters> params);
            Pose2d operator()(StateVector& q, std::vector<double> velocity, double delta_t) override; 
            cal_lib::StateVector forward_kin_model(const cal_lib::StateVector, std::vector<double> velocity);
    };

} // end Dynamics namespace
}// end RobotModel namespace

#endif