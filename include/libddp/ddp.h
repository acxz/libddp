#pragma once

#include <Eigen/Dense>  // Eigen::MatrixXd
#include <cppad/cg.hpp>
#include <tuple>  // std::tuple

namespace ddp {

class ddpOptimizer {
   public:
    //! constructor
    ddpOptimizer(CppAD::ADFun<CppAD::cg::CG<double>> &dyn_cost_func);

    //! destructor
    ~ddpOptimizer();

    // TODO: would like to spit out just the gains from this primarily
    // Remember the different between the plant (world) and the dyanmics (model)

    //! compute feedforward and feedback gains
    //! optimize trajectory
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> optimizeTrajectory(
        Eigen::MatrixXd initial_state, Eigen::MatrixXd initialTrajectory,
        double dt,
        bool (*control_update_converged)(Eigen::MatrixXd du, int iteration_num,
                                         double time_elapsed));

   private:
    // Model for computing jacobians/hessians of dynamics/cost function
    std::unique_ptr<CppAD::cg::GenericModel<double>> dyn_cost_model_;
};

}  // namespace ddp

#include <libddp/ddp-impl.h>
