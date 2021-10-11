#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.8                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9111537122853556368) {
   out_9111537122853556368[0] = delta_x[0] + nom_x[0];
   out_9111537122853556368[1] = delta_x[1] + nom_x[1];
   out_9111537122853556368[2] = delta_x[2] + nom_x[2];
   out_9111537122853556368[3] = delta_x[3] + nom_x[3];
   out_9111537122853556368[4] = delta_x[4] + nom_x[4];
   out_9111537122853556368[5] = delta_x[5] + nom_x[5];
   out_9111537122853556368[6] = delta_x[6] + nom_x[6];
   out_9111537122853556368[7] = delta_x[7] + nom_x[7];
   out_9111537122853556368[8] = delta_x[8] + nom_x[8];
   out_9111537122853556368[9] = delta_x[9] + nom_x[9];
   out_9111537122853556368[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7950463516499194187) {
   out_7950463516499194187[0] = -nom_x[0] + true_x[0];
   out_7950463516499194187[1] = -nom_x[1] + true_x[1];
   out_7950463516499194187[2] = -nom_x[2] + true_x[2];
   out_7950463516499194187[3] = -nom_x[3] + true_x[3];
   out_7950463516499194187[4] = -nom_x[4] + true_x[4];
   out_7950463516499194187[5] = -nom_x[5] + true_x[5];
   out_7950463516499194187[6] = -nom_x[6] + true_x[6];
   out_7950463516499194187[7] = -nom_x[7] + true_x[7];
   out_7950463516499194187[8] = -nom_x[8] + true_x[8];
   out_7950463516499194187[9] = -nom_x[9] + true_x[9];
   out_7950463516499194187[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4505254737572714755) {
   out_4505254737572714755[0] = 1.0;
   out_4505254737572714755[1] = 0.0;
   out_4505254737572714755[2] = 0.0;
   out_4505254737572714755[3] = 0.0;
   out_4505254737572714755[4] = 0.0;
   out_4505254737572714755[5] = 0.0;
   out_4505254737572714755[6] = 0.0;
   out_4505254737572714755[7] = 0.0;
   out_4505254737572714755[8] = 0.0;
   out_4505254737572714755[9] = 0.0;
   out_4505254737572714755[10] = 0.0;
   out_4505254737572714755[11] = 0.0;
   out_4505254737572714755[12] = 1.0;
   out_4505254737572714755[13] = 0.0;
   out_4505254737572714755[14] = 0.0;
   out_4505254737572714755[15] = 0.0;
   out_4505254737572714755[16] = 0.0;
   out_4505254737572714755[17] = 0.0;
   out_4505254737572714755[18] = 0.0;
   out_4505254737572714755[19] = 0.0;
   out_4505254737572714755[20] = 0.0;
   out_4505254737572714755[21] = 0.0;
   out_4505254737572714755[22] = 0.0;
   out_4505254737572714755[23] = 0.0;
   out_4505254737572714755[24] = 1.0;
   out_4505254737572714755[25] = 0.0;
   out_4505254737572714755[26] = 0.0;
   out_4505254737572714755[27] = 0.0;
   out_4505254737572714755[28] = 0.0;
   out_4505254737572714755[29] = 0.0;
   out_4505254737572714755[30] = 0.0;
   out_4505254737572714755[31] = 0.0;
   out_4505254737572714755[32] = 0.0;
   out_4505254737572714755[33] = 0.0;
   out_4505254737572714755[34] = 0.0;
   out_4505254737572714755[35] = 0.0;
   out_4505254737572714755[36] = 1.0;
   out_4505254737572714755[37] = 0.0;
   out_4505254737572714755[38] = 0.0;
   out_4505254737572714755[39] = 0.0;
   out_4505254737572714755[40] = 0.0;
   out_4505254737572714755[41] = 0.0;
   out_4505254737572714755[42] = 0.0;
   out_4505254737572714755[43] = 0.0;
   out_4505254737572714755[44] = 0.0;
   out_4505254737572714755[45] = 0.0;
   out_4505254737572714755[46] = 0.0;
   out_4505254737572714755[47] = 0.0;
   out_4505254737572714755[48] = 1.0;
   out_4505254737572714755[49] = 0.0;
   out_4505254737572714755[50] = 0.0;
   out_4505254737572714755[51] = 0.0;
   out_4505254737572714755[52] = 0.0;
   out_4505254737572714755[53] = 0.0;
   out_4505254737572714755[54] = 0.0;
   out_4505254737572714755[55] = 0.0;
   out_4505254737572714755[56] = 0.0;
   out_4505254737572714755[57] = 0.0;
   out_4505254737572714755[58] = 0.0;
   out_4505254737572714755[59] = 0.0;
   out_4505254737572714755[60] = 1.0;
   out_4505254737572714755[61] = 0.0;
   out_4505254737572714755[62] = 0.0;
   out_4505254737572714755[63] = 0.0;
   out_4505254737572714755[64] = 0.0;
   out_4505254737572714755[65] = 0.0;
   out_4505254737572714755[66] = 0.0;
   out_4505254737572714755[67] = 0.0;
   out_4505254737572714755[68] = 0.0;
   out_4505254737572714755[69] = 0.0;
   out_4505254737572714755[70] = 0.0;
   out_4505254737572714755[71] = 0.0;
   out_4505254737572714755[72] = 1.0;
   out_4505254737572714755[73] = 0.0;
   out_4505254737572714755[74] = 0.0;
   out_4505254737572714755[75] = 0.0;
   out_4505254737572714755[76] = 0.0;
   out_4505254737572714755[77] = 0.0;
   out_4505254737572714755[78] = 0.0;
   out_4505254737572714755[79] = 0.0;
   out_4505254737572714755[80] = 0.0;
   out_4505254737572714755[81] = 0.0;
   out_4505254737572714755[82] = 0.0;
   out_4505254737572714755[83] = 0.0;
   out_4505254737572714755[84] = 1.0;
   out_4505254737572714755[85] = 0.0;
   out_4505254737572714755[86] = 0.0;
   out_4505254737572714755[87] = 0.0;
   out_4505254737572714755[88] = 0.0;
   out_4505254737572714755[89] = 0.0;
   out_4505254737572714755[90] = 0.0;
   out_4505254737572714755[91] = 0.0;
   out_4505254737572714755[92] = 0.0;
   out_4505254737572714755[93] = 0.0;
   out_4505254737572714755[94] = 0.0;
   out_4505254737572714755[95] = 0.0;
   out_4505254737572714755[96] = 1.0;
   out_4505254737572714755[97] = 0.0;
   out_4505254737572714755[98] = 0.0;
   out_4505254737572714755[99] = 0.0;
   out_4505254737572714755[100] = 0.0;
   out_4505254737572714755[101] = 0.0;
   out_4505254737572714755[102] = 0.0;
   out_4505254737572714755[103] = 0.0;
   out_4505254737572714755[104] = 0.0;
   out_4505254737572714755[105] = 0.0;
   out_4505254737572714755[106] = 0.0;
   out_4505254737572714755[107] = 0.0;
   out_4505254737572714755[108] = 1.0;
   out_4505254737572714755[109] = 0.0;
   out_4505254737572714755[110] = 0.0;
   out_4505254737572714755[111] = 0.0;
   out_4505254737572714755[112] = 0.0;
   out_4505254737572714755[113] = 0.0;
   out_4505254737572714755[114] = 0.0;
   out_4505254737572714755[115] = 0.0;
   out_4505254737572714755[116] = 0.0;
   out_4505254737572714755[117] = 0.0;
   out_4505254737572714755[118] = 0.0;
   out_4505254737572714755[119] = 0.0;
   out_4505254737572714755[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2139132427401851486) {
   out_2139132427401851486[0] = dt*state[3] + state[0];
   out_2139132427401851486[1] = dt*state[4] + state[1];
   out_2139132427401851486[2] = dt*state[5] + state[2];
   out_2139132427401851486[3] = state[3];
   out_2139132427401851486[4] = state[4];
   out_2139132427401851486[5] = state[5];
   out_2139132427401851486[6] = dt*state[7] + state[6];
   out_2139132427401851486[7] = dt*state[8] + state[7];
   out_2139132427401851486[8] = state[8];
   out_2139132427401851486[9] = state[9];
   out_2139132427401851486[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4176412462162612297) {
   out_4176412462162612297[0] = 1;
   out_4176412462162612297[1] = 0;
   out_4176412462162612297[2] = 0;
   out_4176412462162612297[3] = dt;
   out_4176412462162612297[4] = 0;
   out_4176412462162612297[5] = 0;
   out_4176412462162612297[6] = 0;
   out_4176412462162612297[7] = 0;
   out_4176412462162612297[8] = 0;
   out_4176412462162612297[9] = 0;
   out_4176412462162612297[10] = 0;
   out_4176412462162612297[11] = 0;
   out_4176412462162612297[12] = 1;
   out_4176412462162612297[13] = 0;
   out_4176412462162612297[14] = 0;
   out_4176412462162612297[15] = dt;
   out_4176412462162612297[16] = 0;
   out_4176412462162612297[17] = 0;
   out_4176412462162612297[18] = 0;
   out_4176412462162612297[19] = 0;
   out_4176412462162612297[20] = 0;
   out_4176412462162612297[21] = 0;
   out_4176412462162612297[22] = 0;
   out_4176412462162612297[23] = 0;
   out_4176412462162612297[24] = 1;
   out_4176412462162612297[25] = 0;
   out_4176412462162612297[26] = 0;
   out_4176412462162612297[27] = dt;
   out_4176412462162612297[28] = 0;
   out_4176412462162612297[29] = 0;
   out_4176412462162612297[30] = 0;
   out_4176412462162612297[31] = 0;
   out_4176412462162612297[32] = 0;
   out_4176412462162612297[33] = 0;
   out_4176412462162612297[34] = 0;
   out_4176412462162612297[35] = 0;
   out_4176412462162612297[36] = 1;
   out_4176412462162612297[37] = 0;
   out_4176412462162612297[38] = 0;
   out_4176412462162612297[39] = 0;
   out_4176412462162612297[40] = 0;
   out_4176412462162612297[41] = 0;
   out_4176412462162612297[42] = 0;
   out_4176412462162612297[43] = 0;
   out_4176412462162612297[44] = 0;
   out_4176412462162612297[45] = 0;
   out_4176412462162612297[46] = 0;
   out_4176412462162612297[47] = 0;
   out_4176412462162612297[48] = 1;
   out_4176412462162612297[49] = 0;
   out_4176412462162612297[50] = 0;
   out_4176412462162612297[51] = 0;
   out_4176412462162612297[52] = 0;
   out_4176412462162612297[53] = 0;
   out_4176412462162612297[54] = 0;
   out_4176412462162612297[55] = 0;
   out_4176412462162612297[56] = 0;
   out_4176412462162612297[57] = 0;
   out_4176412462162612297[58] = 0;
   out_4176412462162612297[59] = 0;
   out_4176412462162612297[60] = 1;
   out_4176412462162612297[61] = 0;
   out_4176412462162612297[62] = 0;
   out_4176412462162612297[63] = 0;
   out_4176412462162612297[64] = 0;
   out_4176412462162612297[65] = 0;
   out_4176412462162612297[66] = 0;
   out_4176412462162612297[67] = 0;
   out_4176412462162612297[68] = 0;
   out_4176412462162612297[69] = 0;
   out_4176412462162612297[70] = 0;
   out_4176412462162612297[71] = 0;
   out_4176412462162612297[72] = 1;
   out_4176412462162612297[73] = dt;
   out_4176412462162612297[74] = 0;
   out_4176412462162612297[75] = 0;
   out_4176412462162612297[76] = 0;
   out_4176412462162612297[77] = 0;
   out_4176412462162612297[78] = 0;
   out_4176412462162612297[79] = 0;
   out_4176412462162612297[80] = 0;
   out_4176412462162612297[81] = 0;
   out_4176412462162612297[82] = 0;
   out_4176412462162612297[83] = 0;
   out_4176412462162612297[84] = 1;
   out_4176412462162612297[85] = dt;
   out_4176412462162612297[86] = 0;
   out_4176412462162612297[87] = 0;
   out_4176412462162612297[88] = 0;
   out_4176412462162612297[89] = 0;
   out_4176412462162612297[90] = 0;
   out_4176412462162612297[91] = 0;
   out_4176412462162612297[92] = 0;
   out_4176412462162612297[93] = 0;
   out_4176412462162612297[94] = 0;
   out_4176412462162612297[95] = 0;
   out_4176412462162612297[96] = 1;
   out_4176412462162612297[97] = 0;
   out_4176412462162612297[98] = 0;
   out_4176412462162612297[99] = 0;
   out_4176412462162612297[100] = 0;
   out_4176412462162612297[101] = 0;
   out_4176412462162612297[102] = 0;
   out_4176412462162612297[103] = 0;
   out_4176412462162612297[104] = 0;
   out_4176412462162612297[105] = 0;
   out_4176412462162612297[106] = 0;
   out_4176412462162612297[107] = 0;
   out_4176412462162612297[108] = 1;
   out_4176412462162612297[109] = 0;
   out_4176412462162612297[110] = 0;
   out_4176412462162612297[111] = 0;
   out_4176412462162612297[112] = 0;
   out_4176412462162612297[113] = 0;
   out_4176412462162612297[114] = 0;
   out_4176412462162612297[115] = 0;
   out_4176412462162612297[116] = 0;
   out_4176412462162612297[117] = 0;
   out_4176412462162612297[118] = 0;
   out_4176412462162612297[119] = 0;
   out_4176412462162612297[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_20050153005147789) {
   out_20050153005147789[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_676377329000984951) {
   out_676377329000984951[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_676377329000984951[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_676377329000984951[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_676377329000984951[3] = 0;
   out_676377329000984951[4] = 0;
   out_676377329000984951[5] = 0;
   out_676377329000984951[6] = 1;
   out_676377329000984951[7] = 0;
   out_676377329000984951[8] = 0;
   out_676377329000984951[9] = 0;
   out_676377329000984951[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7807621166270542711) {
   out_7807621166270542711[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1818511196591280591) {
   out_1818511196591280591[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1818511196591280591[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1818511196591280591[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1818511196591280591[3] = 0;
   out_1818511196591280591[4] = 0;
   out_1818511196591280591[5] = 0;
   out_1818511196591280591[6] = 1;
   out_1818511196591280591[7] = 0;
   out_1818511196591280591[8] = 0;
   out_1818511196591280591[9] = 1;
   out_1818511196591280591[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1070783429472919825) {
   out_1070783429472919825[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3562828785441470947) {
   out_3562828785441470947[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[6] = 0;
   out_3562828785441470947[7] = 1;
   out_3562828785441470947[8] = 0;
   out_3562828785441470947[9] = 0;
   out_3562828785441470947[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1070783429472919825) {
   out_1070783429472919825[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3562828785441470947) {
   out_3562828785441470947[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3562828785441470947[6] = 0;
   out_3562828785441470947[7] = 1;
   out_3562828785441470947[8] = 0;
   out_3562828785441470947[9] = 0;
   out_3562828785441470947[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9111537122853556368) {
  err_fun(nom_x, delta_x, out_9111537122853556368);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7950463516499194187) {
  inv_err_fun(nom_x, true_x, out_7950463516499194187);
}
void gnss_H_mod_fun(double *state, double *out_4505254737572714755) {
  H_mod_fun(state, out_4505254737572714755);
}
void gnss_f_fun(double *state, double dt, double *out_2139132427401851486) {
  f_fun(state,  dt, out_2139132427401851486);
}
void gnss_F_fun(double *state, double dt, double *out_4176412462162612297) {
  F_fun(state,  dt, out_4176412462162612297);
}
void gnss_h_6(double *state, double *sat_pos, double *out_20050153005147789) {
  h_6(state, sat_pos, out_20050153005147789);
}
void gnss_H_6(double *state, double *sat_pos, double *out_676377329000984951) {
  H_6(state, sat_pos, out_676377329000984951);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7807621166270542711) {
  h_20(state, sat_pos, out_7807621166270542711);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1818511196591280591) {
  H_20(state, sat_pos, out_1818511196591280591);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1070783429472919825) {
  h_7(state, sat_pos_vel, out_1070783429472919825);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3562828785441470947) {
  H_7(state, sat_pos_vel, out_3562828785441470947);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1070783429472919825) {
  h_21(state, sat_pos_vel, out_1070783429472919825);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3562828785441470947) {
  H_21(state, sat_pos_vel, out_3562828785441470947);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
