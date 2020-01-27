/*!
 * Example using a DDP controller.
 * \example testDDP.cpp
 */

#include <libddp/ddp.h>

#include <cppad/cg.hpp>
#include <iostream>

bool d(Eigen::MatrixXd a, int b, double c) { return true; }

int main(int argc, char** argv) {
    // Create dynamics_cost function
    // state and control dimensions
    size_t state_dim = 2;
    size_t control_dim = 2;

    // Input to model is state and control (single vector for autodiff
    // purposes)
    std::vector<CppAD::AD<CppAD::cg::CG<double>>> state_control(
        state_dim + control_dim);
    Independent(state_control);

    // Output of the model
    std::vector<CppAD::AD<CppAD::cg::CG<double>>> dyn_cost(state_dim + 1);

    // Model equations
    dyn_cost[0] =
        2 * state_control[0] * state_control[0] * state_control[2] +
        2 * state_control[1] * state_control[0];
    dyn_cost[1] = 3 * state_control[2] + 4 * state_control[3];
    dyn_cost[2] = 2 * state_control[0] * state_control[0] *
                  state_control[2] +
              2 * state_control[1] * state_control[0];

    CppAD::ADFun<CppAD::cg::CG<double>> dyn_cost_func(state_control, dyn_cost);

    // Create a DDP controller object
    ddp::ddpOptimizer testDDP(dyn_cost_func);

    // Test ddp functionality
    Eigen::MatrixXd a;
    Eigen::MatrixXd b;
    double c;

    testDDP.optimizeTrajectory(a, b, c, d);

    std::cout << "Test Completed!" << std::endl;
}
