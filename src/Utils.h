#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class Utils {
	public:
		Utils();
		virtual ~Utils();
		// Fit x and y vals
		Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
		// Evaluate a polynomial.
		double polyeval(Eigen::VectorXd coeffs, double x);
};