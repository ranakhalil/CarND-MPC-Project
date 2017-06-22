#include "Utils.h"


//
// Utils class definition implementation.
//
Utils::Utils() {}
Utils::~Utils() {}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Utils::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	//cout << "Vals : " << (xvals.size() == yvals.size()) << endl;
	//cout << "Order Checks : " << (order >= 1 && order <= xvals.size() - 1) << endl;

	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	//cout << "Result : " << result;
	return result;
};

// Evaluate a polynomial.
double Utils::polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
};

// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd Utils::globalKinematic(Eigen::VectorXd state,
	Eigen::VectorXd actuators, double dt, double Lf) {

	// Create a new vector for the next state.
	Eigen::VectorXd next_state(state.size());

	auto x = state(0);
	auto y = state(1);
	auto psi = state(2);
	auto v = state(3);

	auto delta = actuators(0);
	auto a = actuators(1);

	// Recall the equations for the model:
	// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	// v_[t+1] = v[t] + a[t] * dt
	next_state(0) = x + v * cos(psi) * dt;
	next_state(1) = y + v * sin(psi) * dt;
	next_state(2) = psi + v / Lf * delta * dt;
	next_state(3) = v + a * dt;
	return next_state;
}

void Utils::to_local_points(vector<double> ptsx, vector<double> ptsy, double x, double y, double psi, 
	vector<double>& transformed_pts_x, vector<double>& transformed_pts_y)
{
	assert(ptsx.size() == ptsy.size());

	for (int p = 0; p < ptsx.size(); p++)
	{
		const double x_diff = ptsx[p] - x;
		const double y_diff = ptsy[p] - y;

		transformed_pts_x.push_back(x_diff * cos(-psi) - y_diff * sin(-psi));
		transformed_pts_y.push_back(y_diff * cos(-psi) + x_diff * cos(-psi));
	}
}
