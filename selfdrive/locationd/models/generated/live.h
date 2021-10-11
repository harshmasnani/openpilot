#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1084645820625030813);
void live_err_fun(double *nom_x, double *delta_x, double *out_8947158462103013375);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4061268117809520269);
void live_H_mod_fun(double *state, double *out_8591565397288708318);
void live_f_fun(double *state, double dt, double *out_7580672591969843864);
void live_F_fun(double *state, double dt, double *out_7068416710927820604);
void live_h_3(double *state, double *unused, double *out_6088864723485081738);
void live_H_3(double *state, double *unused, double *out_1986419105982664102);
void live_h_4(double *state, double *unused, double *out_6876148528223897640);
void live_H_4(double *state, double *unused, double *out_6344833586531428533);
void live_h_9(double *state, double *unused, double *out_8840992975488946774);
void live_H_9(double *state, double *unused, double *out_2733375654672410288);
void live_h_10(double *state, double *unused, double *out_1551314788006877217);
void live_H_10(double *state, double *unused, double *out_3081100594872628387);
void live_h_12(double *state, double *unused, double *out_9015443757225940224);
void live_H_12(double *state, double *unused, double *out_8451389098270638102);
void live_h_31(double *state, double *unused, double *out_4829708797847652565);
void live_H_31(double *state, double *unused, double *out_5776299323127218281);
void live_h_32(double *state, double *unused, double *out_5183630887013422113);
void live_H_32(double *state, double *unused, double *out_6194206008400054994);
void live_h_13(double *state, double *unused, double *out_7653774508843975170);
void live_H_13(double *state, double *unused, double *out_4254741394706743992);
void live_h_14(double *state, double *unused, double *out_8840992975488946774);
void live_H_14(double *state, double *unused, double *out_2733375654672410288);
void live_h_19(double *state, double *unused, double *out_6353213147385666857);
void live_H_19(double *state, double *unused, double *out_1699304196430020893);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}