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
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6523780156245555698) {
   out_6523780156245555698[0] = delta_x[0] + nom_x[0];
   out_6523780156245555698[1] = delta_x[1] + nom_x[1];
   out_6523780156245555698[2] = delta_x[2] + nom_x[2];
   out_6523780156245555698[3] = delta_x[3] + nom_x[3];
   out_6523780156245555698[4] = delta_x[4] + nom_x[4];
   out_6523780156245555698[5] = delta_x[5] + nom_x[5];
   out_6523780156245555698[6] = delta_x[6] + nom_x[6];
   out_6523780156245555698[7] = delta_x[7] + nom_x[7];
   out_6523780156245555698[8] = delta_x[8] + nom_x[8];
   out_6523780156245555698[9] = delta_x[9] + nom_x[9];
   out_6523780156245555698[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3674528916290538446) {
   out_3674528916290538446[0] = -nom_x[0] + true_x[0];
   out_3674528916290538446[1] = -nom_x[1] + true_x[1];
   out_3674528916290538446[2] = -nom_x[2] + true_x[2];
   out_3674528916290538446[3] = -nom_x[3] + true_x[3];
   out_3674528916290538446[4] = -nom_x[4] + true_x[4];
   out_3674528916290538446[5] = -nom_x[5] + true_x[5];
   out_3674528916290538446[6] = -nom_x[6] + true_x[6];
   out_3674528916290538446[7] = -nom_x[7] + true_x[7];
   out_3674528916290538446[8] = -nom_x[8] + true_x[8];
   out_3674528916290538446[9] = -nom_x[9] + true_x[9];
   out_3674528916290538446[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_7143516308511921074) {
   out_7143516308511921074[0] = 1.0;
   out_7143516308511921074[1] = 0;
   out_7143516308511921074[2] = 0;
   out_7143516308511921074[3] = 0;
   out_7143516308511921074[4] = 0;
   out_7143516308511921074[5] = 0;
   out_7143516308511921074[6] = 0;
   out_7143516308511921074[7] = 0;
   out_7143516308511921074[8] = 0;
   out_7143516308511921074[9] = 0;
   out_7143516308511921074[10] = 0;
   out_7143516308511921074[11] = 0;
   out_7143516308511921074[12] = 1.0;
   out_7143516308511921074[13] = 0;
   out_7143516308511921074[14] = 0;
   out_7143516308511921074[15] = 0;
   out_7143516308511921074[16] = 0;
   out_7143516308511921074[17] = 0;
   out_7143516308511921074[18] = 0;
   out_7143516308511921074[19] = 0;
   out_7143516308511921074[20] = 0;
   out_7143516308511921074[21] = 0;
   out_7143516308511921074[22] = 0;
   out_7143516308511921074[23] = 0;
   out_7143516308511921074[24] = 1.0;
   out_7143516308511921074[25] = 0;
   out_7143516308511921074[26] = 0;
   out_7143516308511921074[27] = 0;
   out_7143516308511921074[28] = 0;
   out_7143516308511921074[29] = 0;
   out_7143516308511921074[30] = 0;
   out_7143516308511921074[31] = 0;
   out_7143516308511921074[32] = 0;
   out_7143516308511921074[33] = 0;
   out_7143516308511921074[34] = 0;
   out_7143516308511921074[35] = 0;
   out_7143516308511921074[36] = 1.0;
   out_7143516308511921074[37] = 0;
   out_7143516308511921074[38] = 0;
   out_7143516308511921074[39] = 0;
   out_7143516308511921074[40] = 0;
   out_7143516308511921074[41] = 0;
   out_7143516308511921074[42] = 0;
   out_7143516308511921074[43] = 0;
   out_7143516308511921074[44] = 0;
   out_7143516308511921074[45] = 0;
   out_7143516308511921074[46] = 0;
   out_7143516308511921074[47] = 0;
   out_7143516308511921074[48] = 1.0;
   out_7143516308511921074[49] = 0;
   out_7143516308511921074[50] = 0;
   out_7143516308511921074[51] = 0;
   out_7143516308511921074[52] = 0;
   out_7143516308511921074[53] = 0;
   out_7143516308511921074[54] = 0;
   out_7143516308511921074[55] = 0;
   out_7143516308511921074[56] = 0;
   out_7143516308511921074[57] = 0;
   out_7143516308511921074[58] = 0;
   out_7143516308511921074[59] = 0;
   out_7143516308511921074[60] = 1.0;
   out_7143516308511921074[61] = 0;
   out_7143516308511921074[62] = 0;
   out_7143516308511921074[63] = 0;
   out_7143516308511921074[64] = 0;
   out_7143516308511921074[65] = 0;
   out_7143516308511921074[66] = 0;
   out_7143516308511921074[67] = 0;
   out_7143516308511921074[68] = 0;
   out_7143516308511921074[69] = 0;
   out_7143516308511921074[70] = 0;
   out_7143516308511921074[71] = 0;
   out_7143516308511921074[72] = 1.0;
   out_7143516308511921074[73] = 0;
   out_7143516308511921074[74] = 0;
   out_7143516308511921074[75] = 0;
   out_7143516308511921074[76] = 0;
   out_7143516308511921074[77] = 0;
   out_7143516308511921074[78] = 0;
   out_7143516308511921074[79] = 0;
   out_7143516308511921074[80] = 0;
   out_7143516308511921074[81] = 0;
   out_7143516308511921074[82] = 0;
   out_7143516308511921074[83] = 0;
   out_7143516308511921074[84] = 1.0;
   out_7143516308511921074[85] = 0;
   out_7143516308511921074[86] = 0;
   out_7143516308511921074[87] = 0;
   out_7143516308511921074[88] = 0;
   out_7143516308511921074[89] = 0;
   out_7143516308511921074[90] = 0;
   out_7143516308511921074[91] = 0;
   out_7143516308511921074[92] = 0;
   out_7143516308511921074[93] = 0;
   out_7143516308511921074[94] = 0;
   out_7143516308511921074[95] = 0;
   out_7143516308511921074[96] = 1.0;
   out_7143516308511921074[97] = 0;
   out_7143516308511921074[98] = 0;
   out_7143516308511921074[99] = 0;
   out_7143516308511921074[100] = 0;
   out_7143516308511921074[101] = 0;
   out_7143516308511921074[102] = 0;
   out_7143516308511921074[103] = 0;
   out_7143516308511921074[104] = 0;
   out_7143516308511921074[105] = 0;
   out_7143516308511921074[106] = 0;
   out_7143516308511921074[107] = 0;
   out_7143516308511921074[108] = 1.0;
   out_7143516308511921074[109] = 0;
   out_7143516308511921074[110] = 0;
   out_7143516308511921074[111] = 0;
   out_7143516308511921074[112] = 0;
   out_7143516308511921074[113] = 0;
   out_7143516308511921074[114] = 0;
   out_7143516308511921074[115] = 0;
   out_7143516308511921074[116] = 0;
   out_7143516308511921074[117] = 0;
   out_7143516308511921074[118] = 0;
   out_7143516308511921074[119] = 0;
   out_7143516308511921074[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1509902691536634229) {
   out_1509902691536634229[0] = dt*state[3] + state[0];
   out_1509902691536634229[1] = dt*state[4] + state[1];
   out_1509902691536634229[2] = dt*state[5] + state[2];
   out_1509902691536634229[3] = state[3];
   out_1509902691536634229[4] = state[4];
   out_1509902691536634229[5] = state[5];
   out_1509902691536634229[6] = dt*state[7] + state[6];
   out_1509902691536634229[7] = dt*state[8] + state[7];
   out_1509902691536634229[8] = state[8];
   out_1509902691536634229[9] = state[9];
   out_1509902691536634229[10] = state[10];
}
void F_fun(double *state, double dt, double *out_1922216833939025678) {
   out_1922216833939025678[0] = 1;
   out_1922216833939025678[1] = 0;
   out_1922216833939025678[2] = 0;
   out_1922216833939025678[3] = dt;
   out_1922216833939025678[4] = 0;
   out_1922216833939025678[5] = 0;
   out_1922216833939025678[6] = 0;
   out_1922216833939025678[7] = 0;
   out_1922216833939025678[8] = 0;
   out_1922216833939025678[9] = 0;
   out_1922216833939025678[10] = 0;
   out_1922216833939025678[11] = 0;
   out_1922216833939025678[12] = 1;
   out_1922216833939025678[13] = 0;
   out_1922216833939025678[14] = 0;
   out_1922216833939025678[15] = dt;
   out_1922216833939025678[16] = 0;
   out_1922216833939025678[17] = 0;
   out_1922216833939025678[18] = 0;
   out_1922216833939025678[19] = 0;
   out_1922216833939025678[20] = 0;
   out_1922216833939025678[21] = 0;
   out_1922216833939025678[22] = 0;
   out_1922216833939025678[23] = 0;
   out_1922216833939025678[24] = 1;
   out_1922216833939025678[25] = 0;
   out_1922216833939025678[26] = 0;
   out_1922216833939025678[27] = dt;
   out_1922216833939025678[28] = 0;
   out_1922216833939025678[29] = 0;
   out_1922216833939025678[30] = 0;
   out_1922216833939025678[31] = 0;
   out_1922216833939025678[32] = 0;
   out_1922216833939025678[33] = 0;
   out_1922216833939025678[34] = 0;
   out_1922216833939025678[35] = 0;
   out_1922216833939025678[36] = 1;
   out_1922216833939025678[37] = 0;
   out_1922216833939025678[38] = 0;
   out_1922216833939025678[39] = 0;
   out_1922216833939025678[40] = 0;
   out_1922216833939025678[41] = 0;
   out_1922216833939025678[42] = 0;
   out_1922216833939025678[43] = 0;
   out_1922216833939025678[44] = 0;
   out_1922216833939025678[45] = 0;
   out_1922216833939025678[46] = 0;
   out_1922216833939025678[47] = 0;
   out_1922216833939025678[48] = 1;
   out_1922216833939025678[49] = 0;
   out_1922216833939025678[50] = 0;
   out_1922216833939025678[51] = 0;
   out_1922216833939025678[52] = 0;
   out_1922216833939025678[53] = 0;
   out_1922216833939025678[54] = 0;
   out_1922216833939025678[55] = 0;
   out_1922216833939025678[56] = 0;
   out_1922216833939025678[57] = 0;
   out_1922216833939025678[58] = 0;
   out_1922216833939025678[59] = 0;
   out_1922216833939025678[60] = 1;
   out_1922216833939025678[61] = 0;
   out_1922216833939025678[62] = 0;
   out_1922216833939025678[63] = 0;
   out_1922216833939025678[64] = 0;
   out_1922216833939025678[65] = 0;
   out_1922216833939025678[66] = 0;
   out_1922216833939025678[67] = 0;
   out_1922216833939025678[68] = 0;
   out_1922216833939025678[69] = 0;
   out_1922216833939025678[70] = 0;
   out_1922216833939025678[71] = 0;
   out_1922216833939025678[72] = 1;
   out_1922216833939025678[73] = dt;
   out_1922216833939025678[74] = 0;
   out_1922216833939025678[75] = 0;
   out_1922216833939025678[76] = 0;
   out_1922216833939025678[77] = 0;
   out_1922216833939025678[78] = 0;
   out_1922216833939025678[79] = 0;
   out_1922216833939025678[80] = 0;
   out_1922216833939025678[81] = 0;
   out_1922216833939025678[82] = 0;
   out_1922216833939025678[83] = 0;
   out_1922216833939025678[84] = 1;
   out_1922216833939025678[85] = dt;
   out_1922216833939025678[86] = 0;
   out_1922216833939025678[87] = 0;
   out_1922216833939025678[88] = 0;
   out_1922216833939025678[89] = 0;
   out_1922216833939025678[90] = 0;
   out_1922216833939025678[91] = 0;
   out_1922216833939025678[92] = 0;
   out_1922216833939025678[93] = 0;
   out_1922216833939025678[94] = 0;
   out_1922216833939025678[95] = 0;
   out_1922216833939025678[96] = 1;
   out_1922216833939025678[97] = 0;
   out_1922216833939025678[98] = 0;
   out_1922216833939025678[99] = 0;
   out_1922216833939025678[100] = 0;
   out_1922216833939025678[101] = 0;
   out_1922216833939025678[102] = 0;
   out_1922216833939025678[103] = 0;
   out_1922216833939025678[104] = 0;
   out_1922216833939025678[105] = 0;
   out_1922216833939025678[106] = 0;
   out_1922216833939025678[107] = 0;
   out_1922216833939025678[108] = 1;
   out_1922216833939025678[109] = 0;
   out_1922216833939025678[110] = 0;
   out_1922216833939025678[111] = 0;
   out_1922216833939025678[112] = 0;
   out_1922216833939025678[113] = 0;
   out_1922216833939025678[114] = 0;
   out_1922216833939025678[115] = 0;
   out_1922216833939025678[116] = 0;
   out_1922216833939025678[117] = 0;
   out_1922216833939025678[118] = 0;
   out_1922216833939025678[119] = 0;
   out_1922216833939025678[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6126328667901575111) {
   out_6126328667901575111[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_481273702180241460) {
   out_481273702180241460[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_481273702180241460[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_481273702180241460[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_481273702180241460[3] = 0;
   out_481273702180241460[4] = 0;
   out_481273702180241460[5] = 0;
   out_481273702180241460[6] = 1;
   out_481273702180241460[7] = 0;
   out_481273702180241460[8] = 0;
   out_481273702180241460[9] = 0;
   out_481273702180241460[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6761703325721666598) {
   out_6761703325721666598[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5722761928697664157) {
   out_5722761928697664157[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5722761928697664157[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5722761928697664157[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5722761928697664157[3] = 0;
   out_5722761928697664157[4] = 0;
   out_5722761928697664157[5] = 0;
   out_5722761928697664157[6] = 1;
   out_5722761928697664157[7] = 0;
   out_5722761928697664157[8] = 0;
   out_5722761928697664157[9] = 1;
   out_5722761928697664157[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_9058711149749889561) {
   out_9058711149749889561[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8339105362783740381) {
   out_8339105362783740381[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[6] = 0;
   out_8339105362783740381[7] = 1;
   out_8339105362783740381[8] = 0;
   out_8339105362783740381[9] = 0;
   out_8339105362783740381[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_9058711149749889561) {
   out_9058711149749889561[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8339105362783740381) {
   out_8339105362783740381[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8339105362783740381[6] = 0;
   out_8339105362783740381[7] = 1;
   out_8339105362783740381[8] = 0;
   out_8339105362783740381[9] = 0;
   out_8339105362783740381[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6523780156245555698) {
  err_fun(nom_x, delta_x, out_6523780156245555698);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3674528916290538446) {
  inv_err_fun(nom_x, true_x, out_3674528916290538446);
}
void gnss_H_mod_fun(double *state, double *out_7143516308511921074) {
  H_mod_fun(state, out_7143516308511921074);
}
void gnss_f_fun(double *state, double dt, double *out_1509902691536634229) {
  f_fun(state,  dt, out_1509902691536634229);
}
void gnss_F_fun(double *state, double dt, double *out_1922216833939025678) {
  F_fun(state,  dt, out_1922216833939025678);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6126328667901575111) {
  h_6(state, sat_pos, out_6126328667901575111);
}
void gnss_H_6(double *state, double *sat_pos, double *out_481273702180241460) {
  H_6(state, sat_pos, out_481273702180241460);
}
void gnss_h_20(double *state, double *sat_pos, double *out_6761703325721666598) {
  h_20(state, sat_pos, out_6761703325721666598);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5722761928697664157) {
  H_20(state, sat_pos, out_5722761928697664157);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9058711149749889561) {
  h_7(state, sat_pos_vel, out_9058711149749889561);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8339105362783740381) {
  H_7(state, sat_pos_vel, out_8339105362783740381);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9058711149749889561) {
  h_21(state, sat_pos_vel, out_9058711149749889561);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8339105362783740381) {
  H_21(state, sat_pos_vel, out_8339105362783740381);
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
