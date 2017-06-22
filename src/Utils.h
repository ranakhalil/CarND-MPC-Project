#ifndef Utils_H
#define Utils_H

#include <iostream>
#include <math.h>
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cassert>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class Utils {
	public:
		Utils();
		virtual ~Utils();
		// Fit x and y vals
		static Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
		
		// Evaluate a polynomial.
		static double polyeval(Eigen::VectorXd coeffs, double x);
		
		// return the next state
		static Eigen::VectorXd globalKinematic(Eigen::VectorXd state, Eigen::VectorXd actuators, double dt, double Lf);
		
		// transform from global space to local car space
		static void to_local_points(vector<double> ptsx, vector<double> ptsy, double x, double y, double psi,
			vector<double>& transformed_pts_x, vector<double>& transformed_pts_y);
};

#endif /* Utils_H */