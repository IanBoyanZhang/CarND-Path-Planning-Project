//
// Created by ian zhang on 7/25/17.
//

#include "Vehicle.h"

Vehicle::Vehicle(state_t &start) {
  _start_state = start;
}

Vehicle::Vehicle(vector<double> start) {
  _start_state_vec = start;
}

state_t Vehicle::state_in(double t) {
  s_t s = _state_to_s_state();
  d_t d = _state_to_d_state();

  state_t state;
  state.s = s.s + (s.s_dot * t) + s.s_ddot * pow(t, 2) / 2.0;
  state.s_dot = s.s_dot + s.s_ddot * t;
  state.s_ddot = s.s_ddot;

  state.d = d.d + (d.d_dot * t) + d.d_ddot * pow(t, 2) / 2.0;
  state.d_dot = d.d_dot + d.d_ddot * t;
  state.d_ddot = d.d_ddot;
  return state;
}

vector<double> Vehicle::state_in_vec(double t) {
  vector<double> s;
  vector<double> d;

  vector<double> state;

  int N = 3;
  for (auto i = 0; i < N; i+=1) {
    s.push_back(_start_state_vec[i]);
  }

  for (auto i = N; i < N + N; i+=1) {
    d.push_back(_start_state_vec[i]);
  }

  state.push_back(s[0] + (s[1] * t) + s[2] * pow(t, 2) / 2.0);
  state.push_back(s[1] + s[2] * t);
  state.push_back(s[2]);
  state.push_back(d[0] + (d[1] * t) + d[2] * pow(t, 2) / 2.0);
  state.push_back(d[1] + d[2] * t);
  state.push_back(d[2]);
  return state;
}

s_t Vehicle::_state_to_s_state() {
  s_t s;
  s.s = _start_state.s;
  s.s_dot = _start_state.s_dot;
  s.s_ddot = _start_state.s_ddot;
  return s;
}

d_t Vehicle::_state_to_d_state() {
  d_t d;
  d.d = _start_state.d;
  d.d_dot = _start_state.d_dot;
  d.d_ddot = _start_state.d_ddot;
  return d;
}
