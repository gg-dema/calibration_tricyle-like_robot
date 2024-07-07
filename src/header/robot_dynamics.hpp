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
        virtual StateVector operator()(const StateVector q, std::vector<double> input)=0;
        StateVector euler_integration(const StateVector q, const StateVector q_dot, const float delta_time);
        
    protected:
        std::shared_ptr<model_parameters> params_;
    };
    
    class GrisettiModel : public DynamicModel {
    public:
        GrisettiModel(std::shared_ptr<model_parameters> params) : DynamicModel(params){}
        StateVector operator()(const StateVector q, std::vector<double> input) override; 
        Pose2d forward(const StateVector, const Tick);
        double sin_taylor_expansion(const double x);
        double cos_taylor_expansion(const double x);
    private:            
        static constexpr int taylor_order_ = 5;
        const Eigen::Matrix<double, 2, taylor_order_> taylor_coef_;

    };  

    class OrioloModel : public DynamicModel{
    
        public:
            OrioloModel(std::shared_ptr<model_parameters> params);
            StateVector operator()(const StateVector q, std::vector<double> input) override; 
            cal_lib::StateVector dead_reckording(const cal_lib::Pose2d, const cal_lib::Tick);
            cal_lib::StateVector forward_kin_model(const cal_lib::StateVector, double v, double omega);
    };

} // end Dynamics namespace
}// end RobotModel namespace

#endif