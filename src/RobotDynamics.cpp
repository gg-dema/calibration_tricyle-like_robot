#include "header/RobotModel.hpp"

namespace RobotModel{
    namespace Dynamics{

    cal_lib::state_vector oriolo_model(
        cal_lib::state_vector q,
        cal_lib::tick input,
        model_parameters params){
        cal_lib::state_vector q_dot;

        q_dot[0] = input[1] * cos(q[2]);
        q_dot[1] = input[1] * sin(q[2]);
        q_dot[2] = input[0];
        q_dot[3] = input[1] * tan(q[3]) / params.axis_lenght;
        return q_dot;
    }

    cal_lib::state_vector grisetti_model(
        // to change dyn model with the one
        // shown at lecture 
        cal_lib::state_vector q,
        cal_lib::tick input,
        model_parameters params){

        cal_lib::state_vector q_dot;
        q_dot[0] = input[1] * cos(q[2]);
        q_dot[1] = input[1] * sin(q[2]);
        q_dot[2] = input[0];
        q_dot[3] = input[1] * tan(q[3]) / params.axis_lenght;
        return -q_dot;
        }

    } // end Dynamics namespace
}// end RobotModel namespace