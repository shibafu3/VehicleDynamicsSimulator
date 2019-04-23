#define _USE_MATH_DEFINES

#include <math.h>

class BicycleModel {
    const double DEG2RAD = M_PI / 180.0;
    const double RAD2DEG = 180.0 / M_PI;
    const double KG2G = 9.80665;
    const double G2KG = 1.0 / 9.80665;
public :
    double V = 100;
    double delta = 0.08726646259;
    double m = 1500;
    double lf = 1.0453;
    double lr = 1.7546;
    double I = 3024;
    double Bf = 0.2;
    double Cf = 1.5;
    double Df = 376.0;
    double Br = 0.31;
    double Cr = 1.28;
    double Dr = 252.0;

    double Wf = 0.0;
    double Wr = 0.0;
    double beta = 0.0;
    double beta_dot = 0.0;
    double betaf = 0.0;
    double betar = 0.0;
    double yaw = 0.0;
    double yaw_rate = 0.0;
    double yaw_rate_rate = 0.0;
    double moment = 0.0;
    double Ff = 0.0;
    double Fr = 0.0;
    double Ff_dot = 0.0;
    double Fr_dot = 0.0;
    double kf = 10000 * 9.8;
    double kr = 10000 * 9.8;
    double Cpr = 0.0;
    double Cpf = 0.0;
    double Gy = 0.0;
    double Vx = 0.0;
    double Vy = 0.0;
    double x = 0.0;
    double y = 0.0;

    double step = 0.001;

    BicycleModel(double handle_angle_rad,
                 double velocity_mps,
                 double mass_kg,
                 double length_front_m,
                 double length_rear_m,
                 double inertia_yaw_moment,
                 double Bf_in,
                 double Cf_in,
                 double Df_in,
                 double Br_in,
                 double Cr_in,
                 double Dr_in,
                 double step_in) {
        SetControllData(handle_angle_rad, velocity_mps);
        SetVehicleData(mass_kg, length_front_m, length_rear_m, inertia_yaw_moment);
        SetTireData(Bf_in, Cf_in, Df_in, Br_in, Cr_in, Dr_in);
        SetStep(step_in);
    }
    int SetSteer(double handle_angle_rad) {
        delta = handle_angle_rad;
        return 0;
    }
    int SetVelocity(double velocity_mps) {
        V = velocity_mps;
        return 0;
    }
    int SetControllData(double handle_angle_rad, double velocity_mps) {
        SetSteer(handle_angle_rad);
        SetVelocity(velocity_mps);
        return 0;
    }
    int SetVehicleData(double mass_kg, double length_front_m, double length_rear_m, double inertia_yaw_moment) {
        m = mass_kg;
        lf = length_front_m;
        lr = length_rear_m;
        I = inertia_yaw_moment;
        return 0;
    }
    int SetTireData(double Bf_in, double Cf_in, double Df_in, double Br_in, double Cr_in, double Dr_in) {
        Bf = Bf_in;
        Cf = Cf_in;
        Df = Df_in;
        Br = Br_in;
        Cr = Cr_in;
        Dr = Dr_in;
        return 0;
    }
    int SetStep(double step_in) {
        step = step_in;
        return 0;
    }


    int CalcWeight() {
        Wf = (m*lf) / (lf+lr);
        Wr = (m*lr) / (lf+lr);
        return 0;
    }
    int CalcBeta() {
        beta_dot = (Ff + Fr) / (m*V) - yaw_rate;
        beta += beta_dot * step;
        return 0;
    }
    int CalcYaw() {
        yaw_rate_rate = (lf*Ff - lr*Fr) / I;
        yaw_rate += yaw_rate_rate * step;
        yaw += yaw_rate * step;
        moment = I * yaw_rate_rate;
        return 0;
    }
    int CalcBetafr() {
        betaf = delta - beta - (lf*yaw_rate / V);
        betar = -beta + (lr*yaw_rate / V);
        return 0;
    }
    int CalcCp() {
        Cpf = (Bf*Cf*Df*cos(Cf*atan(Bf*betaf*180.0 / M_PI)) / (atan(Bf*betaf*180.0 / M_PI)*atan(Bf*betaf*180.0 / M_PI) + 1)) * 9.8 * 2;
        Cpr = (Br*Cr*Dr*cos(Cr*atan(Br*betar*180.0 / M_PI)) / (atan(Br*betar*180.0 / M_PI)*atan(Br*betar*180.0 / M_PI) + 1)) * 9.8 * 2;
        return 0;
    }
    int CalcF() {
        //Ff = Df * sin(Cf*atan(Bf*betaf * 180.0 / M_PI)) * 9.8 * 2.0;
        //Fr = Dr * sin(Cr*atan(Br*betar * 180.0 / M_PI)) * 9.8 * 2.0;
        Ff_dot = V * kf*(Cpf*betaf * RAD2DEG - Ff) / Cpf;
        Fr_dot = V * kr*(Cpr*betar * RAD2DEG - Fr) / Cpr;
        Ff += Ff_dot * step;
        Fr += Fr_dot * step;
        return 0;
    }
    int CalcGy() {
        Gy = V * (beta_dot + yaw_rate) / 9.8;
        return 0;
    }
    int CalcV() {
        Vx = V * cos(beta_dot);
        Vy = V * sin(beta_dot);
        return 0;
    }
    int CalcXY() {
        x += (Vx * cos(yaw) + Vy * cos(yaw + M_PI_2)) * step;
        y += (Vx * sin(yaw) + Vy * sin(yaw + M_PI_2)) * step;
        return 0;
    }

    int Step() {
        CalcWeight();
        CalcBeta();
        CalcYaw();
        CalcBetafr();
        CalcCp();
        CalcF();
        CalcGy();
        CalcV();
        CalcXY();
        return 0;
    }
};