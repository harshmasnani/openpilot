#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9111537122853556368);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7950463516499194187);
void gnss_H_mod_fun(double *state, double *out_4505254737572714755);
void gnss_f_fun(double *state, double dt, double *out_2139132427401851486);
void gnss_F_fun(double *state, double dt, double *out_4176412462162612297);
void gnss_h_6(double *state, double *sat_pos, double *out_20050153005147789);
void gnss_H_6(double *state, double *sat_pos, double *out_676377329000984951);
void gnss_h_20(double *state, double *sat_pos, double *out_7807621166270542711);
void gnss_H_20(double *state, double *sat_pos, double *out_1818511196591280591);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1070783429472919825);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3562828785441470947);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1070783429472919825);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3562828785441470947);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}