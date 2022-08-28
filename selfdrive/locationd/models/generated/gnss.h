#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6523780156245555698);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3674528916290538446);
void gnss_H_mod_fun(double *state, double *out_7143516308511921074);
void gnss_f_fun(double *state, double dt, double *out_1509902691536634229);
void gnss_F_fun(double *state, double dt, double *out_1922216833939025678);
void gnss_h_6(double *state, double *sat_pos, double *out_6126328667901575111);
void gnss_H_6(double *state, double *sat_pos, double *out_481273702180241460);
void gnss_h_20(double *state, double *sat_pos, double *out_6761703325721666598);
void gnss_H_20(double *state, double *sat_pos, double *out_5722761928697664157);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9058711149749889561);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8339105362783740381);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9058711149749889561);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8339105362783740381);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}