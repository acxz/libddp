#pragma once

#include <Eigen/Dense>  // Eigen::MatrixXd
#include <cppad/cg.hpp>
#include <iostream>  // std::cout, std::endl
#include <tuple>     // std::tuple
#include <vector>    // std::vector

namespace ddp {

//! constructor
ddpOptimizer::ddpOptimizer(CppAD::ADFun<CppAD::cg::CG<double>> &dyn_cost_func) {
    std::cout << "Creating ddp optimizer" << std::endl;

    // Create the dynamic library
    // Generate source code
    CppAD::cg::ModelCSourceGen<double> dyn_cost_cgen(dyn_cost_func,
                                                     "dyn_cost_func");
    dyn_cost_cgen.setCreateJacobian(true);
    dyn_cost_cgen.setCreateHessian(true);
    CppAD::cg::ModelLibraryCSourceGen<double> dyn_cost_libcgen(dyn_cost_cgen);

    //// compile source code
    CppAD::cg::DynamicModelLibraryProcessor<double> dyn_cost_p(
        dyn_cost_libcgen);
    CppAD::cg::GccCompiler<double> compiler;

    // Add this line in and things break ISSUE needs to be created:
    // when creating two dynamic libs we get a segfault when computing jacobian
    // on the second one
    std::unique_ptr<CppAD::cg::DynamicLib<double>> dynamicLib_dyn_cost =
        dyn_cost_p.createDynamicLibrary(compiler);

    // Use the dynamic library
    // make this model a member of the class
    // TODO: also having a function go from vector form to EIgen::Matrix of the
    // relevant split dyn/cost jac/hess would be nice
    dyn_cost_model_ = dynamicLib_dyn_cost->model("dyn_cost_func");

    std::cout << "Created ddp optimizer" << std::endl;
}

//! destructor
ddpOptimizer::~ddpOptimizer() {}

//! optimize trajectory
// output: control trajectory
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> ddpOptimizer::optimizeTrajectory(
    Eigen::MatrixXd initial_state, Eigen::MatrixXd initialTrajectory, double dt,
    bool (*control_update_converged)(Eigen::MatrixXd du, int iteration_num,
                                     double time_elapsed)) {
    std::cout << "Outline" << std::endl;
    std::cout << "= Prelims" << std::endl;
    std::cout << "= Optimize control trajectory" << std::endl;
    std::cout << "== Calculate initial state trajectory" << std::endl;

    std::cout << "== Calculate initial trajectory cost" << std::endl;
    std::cout << "== while control traj has not converged" << std::endl;
    std::cout << "=== handle time elapsed properly, how to exit out"
              << std::endl;
    std::cout << "=== maybe add prediction for the optimization loop but what "
                 "about inner optimiations?"
              << std::endl;
    std::cout << "=== i think having a thread monitoring the time (inside this "
                 "method) wuold prob be the best thing"
              << std::endl;
    std::cout << "=== Calculate dynamics and cost function derivatives"
              << std::endl;
    std::vector<double> xu{5, 2, 3, 4};
    std::vector<double> dyn_cost_hessian_multiplier{1, 1, 1};

    std::vector<double> dyn_cost_jacobian = dyn_cost_model->Jacobian(xu);
    std::vector<double> dyn_cost_hessian =
        dyn_cost_model->Hessian(xu, dyn_cost_hessian_multiplier);

    // print out the result
    std::cout << "Dynamics Cost Jacobian" << std::endl;
    std::cout << dyn_cost_jacobian[0] << std::endl;

    std::cout << "Dynamics Cost Hessian" << std::endl;
    std::cout << dyn_cost_hessian[1] << std::endl;
    std::cout
        << "=== actually do we even need to do this, just call the derivatives "
           "with the values we want in the backward pass cuz technically we "
           "have forward pass, backward pass, forward pass rn"
        << std::endl;
    std::cout << "=== backpass: Calculate value function jacob/hess"
              << std::endl;
    std::cout << "=== forpass: Calculate new u from gains and dx" << std::endl;
    std::cout << "=== forpass: Calculate dx via dynamics or plant?"
              << std::endl;
    std::cout << "=== Calculate state trajectory from current u" << std::endl;
    std::cout << "=== Calculate trajectory cost from current (x,u)"
              << std::endl;

    Eigen::MatrixXd feedforward_gains;
    Eigen::MatrixXd feedback_gains;
    Eigen::MatrixXd new_control;
    return std::make_tuple(feedforward_gains, feedback_gains);
}

}  // namespace ddp
