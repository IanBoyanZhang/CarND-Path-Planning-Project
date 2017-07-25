//
// Created by ian zhang on 7/25/17.
//

#include "Vehicle.h"

Vehicle::Vehicle(state_t &start) {
  _start_state = start;
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
