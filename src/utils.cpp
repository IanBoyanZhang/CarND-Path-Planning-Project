//
// Created by ian zhang on 7/17/17.
//
#include "utils.h"

/**
 * A function that returns a value between 0 and 1 for x in the
   range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
   Useful for cost functions.
 * @param x
 * @return
 */
double utils::logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

/**
 Calculates the derivative of a polynomial and returns
 the corresponding coefficients.
 * @param coefficients
 * @return
 */
vector<double> utils::differentiate(const vector<double>& coefficients) {
  vector<double> new_cos;
  for (auto i = 1; i < coefficients.size(); i+=1) {
    new_cos.push_back(coefficients[i] * (i + 1));
  }
  return new_cos;
}

/**
 * How can we use template here?
 * @param s_state
 * @return
 */
vector<double> utils::s_t_to_vec(s_t s_state) {
  vector<double> s_state_vec;
  s_state_vec.push_back(s_state.s);
  s_state_vec.push_back(s_state.s_dot);
  s_state_vec.push_back(s_state.s_ddot);
  return s_state_vec;
}

vector<double> utils::d_t_to_vec(d_t d_state) {
  vector<double> d_state_vec;
  d_state_vec.push_back(d_state.d);
  d_state_vec.push_back(d_state.d_dot);
  d_state_vec.push_back(d_state.d_ddot);
  return d_state_vec;
}

vector<double> utils::combine_states(s_t s_state, d_t d_state){
  vector<double> s_vec = s_t_to_vec(s_state);
  vector<double> d_vec = d_t_to_vec(d_state);
  s_vec.insert(s_vec.begin(), s_vec.end(), d_vec.end());
  return s_vec;
}

function<double(double)> utils::to_equation(const vector<double>& coefficients) {
  function<double(double)> f = [&coefficients](double t)-> double {
    double total = 0.0;
    for (auto i = 0; i < coefficients.size(); i+=1) {
      total += coefficients[i] * pow(t, i);
    }
    return total;
  };
  return f;
}

/**
 * Calculates the closest distance to any vehicle during a trajectory.
 * @param traj
 * @param vehicles
 * @return
 */
double utils::nearest_appraoch(vector<double> traj, vector<Vehicle> vehicles) {
  // Not implemented
  return 0;
}

vector<function<double(double)> >
utils::get_f_and_N_derivatitves(const vector<double>& coeffs) {
  vector<function<double(double)> > functions;
  functions.push_back(to_equation(coeffs));
  int N = 3;
  vector<double> deri_coeffs = coeffs;
  for (auto i = 0; i < N; i+=1) {
    deri_coeffs = differentiate(deri_coeffs);
    functions.push_back(to_equation(deri_coeffs));
  }
  return functions;
}

double utils::evaluate_function(const vector<double> &coeffs, double x) {
  double sum = 0;
  int N = coeffs.size();
  for (auto i = 0; i < N; i+=1) {
    sum += coeffs[i] * pow(x, i);
  }
  return sum;
}
