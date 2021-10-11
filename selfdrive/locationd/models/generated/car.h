#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_770089559317486534);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2260117302986445697);
void car_H_mod_fun(double *state, double *out_5006523026536552636);
void car_f_fun(double *state, double dt, double *out_6296187746280925078);
void car_F_fun(double *state, double dt, double *out_6197713281736708713);
void car_h_25(double *state, double *unused, double *out_5044925537597907416);
void car_H_25(double *state, double *unused, double *out_6285680037066311835);
void car_h_24(double *state, double *unused, double *out_5414685952180020913);
void car_H_24(double *state, double *unused, double *out_7802395113141637028);
void car_h_30(double *state, double *unused, double *out_1407476009343885066);
void car_H_30(double *state, double *unused, double *out_3328218873882871445);
void car_h_26(double *state, double *unused, double *out_2165326154504703876);
void car_H_26(double *state, double *unused, double *out_4944243903370664283);
void car_h_27(double *state, double *unused, double *out_43252269937061771);
void car_H_27(double *state, double *unused, double *out_6784913923355198034);
void car_h_29(double *state, double *unused, double *out_7806517526757734212);
void car_H_29(double *state, double *unused, double *out_7152017643011703398);
void car_h_28(double *state, double *unused, double *out_8504321228287529848);
void car_H_28(double *state, double *unused, double *out_6261466653729714663);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}