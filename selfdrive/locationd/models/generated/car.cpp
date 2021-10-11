#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 5.991464547107981;

/******************************************************************************
 *                       Code generated with sympy 1.8                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_770089559317486534) {
   out_770089559317486534[0] = delta_x[0] + nom_x[0];
   out_770089559317486534[1] = delta_x[1] + nom_x[1];
   out_770089559317486534[2] = delta_x[2] + nom_x[2];
   out_770089559317486534[3] = delta_x[3] + nom_x[3];
   out_770089559317486534[4] = delta_x[4] + nom_x[4];
   out_770089559317486534[5] = delta_x[5] + nom_x[5];
   out_770089559317486534[6] = delta_x[6] + nom_x[6];
   out_770089559317486534[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2260117302986445697) {
   out_2260117302986445697[0] = -nom_x[0] + true_x[0];
   out_2260117302986445697[1] = -nom_x[1] + true_x[1];
   out_2260117302986445697[2] = -nom_x[2] + true_x[2];
   out_2260117302986445697[3] = -nom_x[3] + true_x[3];
   out_2260117302986445697[4] = -nom_x[4] + true_x[4];
   out_2260117302986445697[5] = -nom_x[5] + true_x[5];
   out_2260117302986445697[6] = -nom_x[6] + true_x[6];
   out_2260117302986445697[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_5006523026536552636) {
   out_5006523026536552636[0] = 1.0;
   out_5006523026536552636[1] = 0.0;
   out_5006523026536552636[2] = 0.0;
   out_5006523026536552636[3] = 0.0;
   out_5006523026536552636[4] = 0.0;
   out_5006523026536552636[5] = 0.0;
   out_5006523026536552636[6] = 0.0;
   out_5006523026536552636[7] = 0.0;
   out_5006523026536552636[8] = 0.0;
   out_5006523026536552636[9] = 1.0;
   out_5006523026536552636[10] = 0.0;
   out_5006523026536552636[11] = 0.0;
   out_5006523026536552636[12] = 0.0;
   out_5006523026536552636[13] = 0.0;
   out_5006523026536552636[14] = 0.0;
   out_5006523026536552636[15] = 0.0;
   out_5006523026536552636[16] = 0.0;
   out_5006523026536552636[17] = 0.0;
   out_5006523026536552636[18] = 1.0;
   out_5006523026536552636[19] = 0.0;
   out_5006523026536552636[20] = 0.0;
   out_5006523026536552636[21] = 0.0;
   out_5006523026536552636[22] = 0.0;
   out_5006523026536552636[23] = 0.0;
   out_5006523026536552636[24] = 0.0;
   out_5006523026536552636[25] = 0.0;
   out_5006523026536552636[26] = 0.0;
   out_5006523026536552636[27] = 1.0;
   out_5006523026536552636[28] = 0.0;
   out_5006523026536552636[29] = 0.0;
   out_5006523026536552636[30] = 0.0;
   out_5006523026536552636[31] = 0.0;
   out_5006523026536552636[32] = 0.0;
   out_5006523026536552636[33] = 0.0;
   out_5006523026536552636[34] = 0.0;
   out_5006523026536552636[35] = 0.0;
   out_5006523026536552636[36] = 1.0;
   out_5006523026536552636[37] = 0.0;
   out_5006523026536552636[38] = 0.0;
   out_5006523026536552636[39] = 0.0;
   out_5006523026536552636[40] = 0.0;
   out_5006523026536552636[41] = 0.0;
   out_5006523026536552636[42] = 0.0;
   out_5006523026536552636[43] = 0.0;
   out_5006523026536552636[44] = 0.0;
   out_5006523026536552636[45] = 1.0;
   out_5006523026536552636[46] = 0.0;
   out_5006523026536552636[47] = 0.0;
   out_5006523026536552636[48] = 0.0;
   out_5006523026536552636[49] = 0.0;
   out_5006523026536552636[50] = 0.0;
   out_5006523026536552636[51] = 0.0;
   out_5006523026536552636[52] = 0.0;
   out_5006523026536552636[53] = 0.0;
   out_5006523026536552636[54] = 1.0;
   out_5006523026536552636[55] = 0.0;
   out_5006523026536552636[56] = 0.0;
   out_5006523026536552636[57] = 0.0;
   out_5006523026536552636[58] = 0.0;
   out_5006523026536552636[59] = 0.0;
   out_5006523026536552636[60] = 0.0;
   out_5006523026536552636[61] = 0.0;
   out_5006523026536552636[62] = 0.0;
   out_5006523026536552636[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_6296187746280925078) {
   out_6296187746280925078[0] = state[0];
   out_6296187746280925078[1] = state[1];
   out_6296187746280925078[2] = state[2];
   out_6296187746280925078[3] = state[3];
   out_6296187746280925078[4] = state[4];
   out_6296187746280925078[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6296187746280925078[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6296187746280925078[7] = state[7];
}
void F_fun(double *state, double dt, double *out_6197713281736708713) {
   out_6197713281736708713[0] = 1;
   out_6197713281736708713[1] = 0;
   out_6197713281736708713[2] = 0;
   out_6197713281736708713[3] = 0;
   out_6197713281736708713[4] = 0;
   out_6197713281736708713[5] = 0;
   out_6197713281736708713[6] = 0;
   out_6197713281736708713[7] = 0;
   out_6197713281736708713[8] = 0;
   out_6197713281736708713[9] = 1;
   out_6197713281736708713[10] = 0;
   out_6197713281736708713[11] = 0;
   out_6197713281736708713[12] = 0;
   out_6197713281736708713[13] = 0;
   out_6197713281736708713[14] = 0;
   out_6197713281736708713[15] = 0;
   out_6197713281736708713[16] = 0;
   out_6197713281736708713[17] = 0;
   out_6197713281736708713[18] = 1;
   out_6197713281736708713[19] = 0;
   out_6197713281736708713[20] = 0;
   out_6197713281736708713[21] = 0;
   out_6197713281736708713[22] = 0;
   out_6197713281736708713[23] = 0;
   out_6197713281736708713[24] = 0;
   out_6197713281736708713[25] = 0;
   out_6197713281736708713[26] = 0;
   out_6197713281736708713[27] = 1;
   out_6197713281736708713[28] = 0;
   out_6197713281736708713[29] = 0;
   out_6197713281736708713[30] = 0;
   out_6197713281736708713[31] = 0;
   out_6197713281736708713[32] = 0;
   out_6197713281736708713[33] = 0;
   out_6197713281736708713[34] = 0;
   out_6197713281736708713[35] = 0;
   out_6197713281736708713[36] = 1;
   out_6197713281736708713[37] = 0;
   out_6197713281736708713[38] = 0;
   out_6197713281736708713[39] = 0;
   out_6197713281736708713[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6197713281736708713[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6197713281736708713[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6197713281736708713[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6197713281736708713[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6197713281736708713[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6197713281736708713[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6197713281736708713[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6197713281736708713[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6197713281736708713[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6197713281736708713[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6197713281736708713[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6197713281736708713[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6197713281736708713[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6197713281736708713[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6197713281736708713[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6197713281736708713[56] = 0;
   out_6197713281736708713[57] = 0;
   out_6197713281736708713[58] = 0;
   out_6197713281736708713[59] = 0;
   out_6197713281736708713[60] = 0;
   out_6197713281736708713[61] = 0;
   out_6197713281736708713[62] = 0;
   out_6197713281736708713[63] = 1;
}
void h_25(double *state, double *unused, double *out_5044925537597907416) {
   out_5044925537597907416[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6285680037066311835) {
   out_6285680037066311835[0] = 0;
   out_6285680037066311835[1] = 0;
   out_6285680037066311835[2] = 0;
   out_6285680037066311835[3] = 0;
   out_6285680037066311835[4] = 0;
   out_6285680037066311835[5] = 0;
   out_6285680037066311835[6] = 1;
   out_6285680037066311835[7] = 0;
}
void h_24(double *state, double *unused, double *out_5414685952180020913) {
   out_5414685952180020913[0] = state[4];
   out_5414685952180020913[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7802395113141637028) {
   out_7802395113141637028[0] = 0;
   out_7802395113141637028[1] = 0;
   out_7802395113141637028[2] = 0;
   out_7802395113141637028[3] = 0;
   out_7802395113141637028[4] = 1;
   out_7802395113141637028[5] = 0;
   out_7802395113141637028[6] = 0;
   out_7802395113141637028[7] = 0;
   out_7802395113141637028[8] = 0;
   out_7802395113141637028[9] = 0;
   out_7802395113141637028[10] = 0;
   out_7802395113141637028[11] = 0;
   out_7802395113141637028[12] = 0;
   out_7802395113141637028[13] = 1;
   out_7802395113141637028[14] = 0;
   out_7802395113141637028[15] = 0;
}
void h_30(double *state, double *unused, double *out_1407476009343885066) {
   out_1407476009343885066[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3328218873882871445) {
   out_3328218873882871445[0] = 0;
   out_3328218873882871445[1] = 0;
   out_3328218873882871445[2] = 0;
   out_3328218873882871445[3] = 0;
   out_3328218873882871445[4] = 1;
   out_3328218873882871445[5] = 0;
   out_3328218873882871445[6] = 0;
   out_3328218873882871445[7] = 0;
}
void h_26(double *state, double *unused, double *out_2165326154504703876) {
   out_2165326154504703876[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4944243903370664283) {
   out_4944243903370664283[0] = 0;
   out_4944243903370664283[1] = 0;
   out_4944243903370664283[2] = 0;
   out_4944243903370664283[3] = 0;
   out_4944243903370664283[4] = 0;
   out_4944243903370664283[5] = 0;
   out_4944243903370664283[6] = 0;
   out_4944243903370664283[7] = 1;
}
void h_27(double *state, double *unused, double *out_43252269937061771) {
   out_43252269937061771[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6784913923355198034) {
   out_6784913923355198034[0] = 0;
   out_6784913923355198034[1] = 0;
   out_6784913923355198034[2] = 0;
   out_6784913923355198034[3] = 1;
   out_6784913923355198034[4] = 0;
   out_6784913923355198034[5] = 0;
   out_6784913923355198034[6] = 0;
   out_6784913923355198034[7] = 0;
}
void h_29(double *state, double *unused, double *out_7806517526757734212) {
   out_7806517526757734212[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7152017643011703398) {
   out_7152017643011703398[0] = 0;
   out_7152017643011703398[1] = 1;
   out_7152017643011703398[2] = 0;
   out_7152017643011703398[3] = 0;
   out_7152017643011703398[4] = 0;
   out_7152017643011703398[5] = 0;
   out_7152017643011703398[6] = 0;
   out_7152017643011703398[7] = 0;
}
void h_28(double *state, double *unused, double *out_8504321228287529848) {
   out_8504321228287529848[0] = state[5];
   out_8504321228287529848[1] = state[6];
}
void H_28(double *state, double *unused, double *out_6261466653729714663) {
   out_6261466653729714663[0] = 0;
   out_6261466653729714663[1] = 0;
   out_6261466653729714663[2] = 0;
   out_6261466653729714663[3] = 0;
   out_6261466653729714663[4] = 0;
   out_6261466653729714663[5] = 1;
   out_6261466653729714663[6] = 0;
   out_6261466653729714663[7] = 0;
   out_6261466653729714663[8] = 0;
   out_6261466653729714663[9] = 0;
   out_6261466653729714663[10] = 0;
   out_6261466653729714663[11] = 0;
   out_6261466653729714663[12] = 0;
   out_6261466653729714663[13] = 0;
   out_6261466653729714663[14] = 1;
   out_6261466653729714663[15] = 0;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_770089559317486534) {
  err_fun(nom_x, delta_x, out_770089559317486534);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2260117302986445697) {
  inv_err_fun(nom_x, true_x, out_2260117302986445697);
}
void car_H_mod_fun(double *state, double *out_5006523026536552636) {
  H_mod_fun(state, out_5006523026536552636);
}
void car_f_fun(double *state, double dt, double *out_6296187746280925078) {
  f_fun(state,  dt, out_6296187746280925078);
}
void car_F_fun(double *state, double dt, double *out_6197713281736708713) {
  F_fun(state,  dt, out_6197713281736708713);
}
void car_h_25(double *state, double *unused, double *out_5044925537597907416) {
  h_25(state, unused, out_5044925537597907416);
}
void car_H_25(double *state, double *unused, double *out_6285680037066311835) {
  H_25(state, unused, out_6285680037066311835);
}
void car_h_24(double *state, double *unused, double *out_5414685952180020913) {
  h_24(state, unused, out_5414685952180020913);
}
void car_H_24(double *state, double *unused, double *out_7802395113141637028) {
  H_24(state, unused, out_7802395113141637028);
}
void car_h_30(double *state, double *unused, double *out_1407476009343885066) {
  h_30(state, unused, out_1407476009343885066);
}
void car_H_30(double *state, double *unused, double *out_3328218873882871445) {
  H_30(state, unused, out_3328218873882871445);
}
void car_h_26(double *state, double *unused, double *out_2165326154504703876) {
  h_26(state, unused, out_2165326154504703876);
}
void car_H_26(double *state, double *unused, double *out_4944243903370664283) {
  H_26(state, unused, out_4944243903370664283);
}
void car_h_27(double *state, double *unused, double *out_43252269937061771) {
  h_27(state, unused, out_43252269937061771);
}
void car_H_27(double *state, double *unused, double *out_6784913923355198034) {
  H_27(state, unused, out_6784913923355198034);
}
void car_h_29(double *state, double *unused, double *out_7806517526757734212) {
  h_29(state, unused, out_7806517526757734212);
}
void car_H_29(double *state, double *unused, double *out_7152017643011703398) {
  H_29(state, unused, out_7152017643011703398);
}
void car_h_28(double *state, double *unused, double *out_8504321228287529848) {
  h_28(state, unused, out_8504321228287529848);
}
void car_H_28(double *state, double *unused, double *out_6261466653729714663) {
  H_28(state, unused, out_6261466653729714663);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
