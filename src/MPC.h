#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/utility/vector.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Variables
extern size_t x_start  ;
extern size_t y_start  ;
extern size_t psi_start;
extern size_t v_start  ;
extern size_t cte_start;
extern size_t epsi_start;

// Actuators
extern size_t delta_start;
extern size_t a_start ;

extern double ref_v;

class MPC {
 public:
  MPC();

  static const size_t N = 20 ;
  static constexpr double  dt = 0.05 ;

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  CppAD::vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs,
                            double acc_last_value, double delta_last_value);
};

#endif /* MPC_H */
