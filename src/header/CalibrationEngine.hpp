#include "utility.hpp"
#include "robotModels.hpp"
#include "DataModule.hpp"

#include <memory>
#include <eigen3/Eigen/Dense>




struct Statistics{
    int skipped_data = 0;
    std::vector<double> log_chi;
};


class CalibrationEngine{

    std::shared_ptr<Robot> robot_;
    
    // state dim : 2 params encoder, 2 params kinematics, 3 params (pose) sensors
    // tot state dim : 7
    Eigen::Matrix<double, 7, 7> _H;   
    Eigen::Matrix<double, 7, 1> _b;
    Eigen::Matrix<double, 7, 1> _perturbation;
    
    // the error is basically a 3d vector, let's use the pose2d
    pose2d _error;                      // euclidian parametrization of the error
    Eigen::Matrix<double, 3, 7> _jac;   // jacobian


    Statistics _statistics;

    public:
    CalibrationEngine(std::shared_ptr<Robot> robot) : robot_(robot){}


    // compute one step of the internal loop of the calibration:
    // given a single measurement (and a single input), update the jacobian for that values
    void OneRound(const tick64& encoder_measure, const pose2d& z_observation);



    void OneIteration(const DataObject& data);


    private: 
   
    void update_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    );
    void update_robot_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    );
    void update_sensor_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    );

    void perturb_parameters(Eigen::Matrix<double, 7, 1> perturbation);
    Eigen::Matrix<double, 7, 1> solver();
    
};