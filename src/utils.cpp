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
 * Evaluate a polynomial
 * @param t
 * @param coefficients
 * @return
 */
// TODO: using lambda expression
// TODO: serializer
double utils::eval_equation(double t, s_t coefficients) {
  // How about using
  return coefficients.s + coefficients.s_dot * pow(t, 1) + coefficients.s_ddot * pow(t, 2);
}
/**
 Calculates the derivative of a polynomial and returns
 the corresponding coefficients.
 * @param coefficients
 * @return
 */
vector<double> utils::differentiate(s_t coefficients) {
  vector<double> new_cos;
  new_cos.push_back(coefficients.s_dot * 2);
  new_cos.push_back(coefficients.s_ddot * 3);
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

