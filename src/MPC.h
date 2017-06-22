#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Utils.h"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  double steering_;
  double a_;
  vector<double> predicted_x_vals;
  vector<double> predicted_y_vals;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
