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
    double bf = 0.2;
    double cf = 1.5;
    double df = 376.0;
    double br = 0.31;
    double cr = 1.28;
    double dr = 252.0;

    double Wf = 0.0;
    double Wr = 0.0;
    double B = 0.0;
    double dB = 0.0;
    double Bf = 0.0;
    double Br = 0.0;
    double yaw = 0.0;
    double dyaw = 0.0;
    double ddyaw = 0.0;
    double moment = 0.0;
    double Ff = 0.0;
    double Fr = 0.0;
    double dFf = 0.0;
    double dFr = 0.0;
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
        bf = Bf_in;
        cf = Cf_in;
        df = Df_in;
        br = Br_in;
        cr = Cr_in;
        dr = Dr_in;
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
        dB = (Ff + Fr) / (m*V) - dyaw;
        B += dB * step;
        return 0;
    }
    int CalcYaw() {
        ddyaw = (lf*Ff - lr*Fr) / I;
        dyaw += ddyaw * step;
        yaw += dyaw * step;
        moment = I * ddyaw;
        return 0;
    }
    int CalcBetafr() {
        Bf = delta - B - (lf*dyaw / V);
        Br = -B + (lr*dyaw / V);
        return 0;
    }
    int CalcCp() {
        Cpf = (bf*cf*df*cos(cf*atan(bf*Bf*180.0 / M_PI)) / (atan(bf*Bf*180.0 / M_PI)*atan(bf*Bf*180.0 / M_PI) + 1)) * 9.8 * 2;
        Cpr = (br*cr*dr*cos(cr*atan(br*Br*180.0 / M_PI)) / (atan(br*Br*180.0 / M_PI)*atan(br*Br*180.0 / M_PI) + 1)) * 9.8 * 2;
        return 0;
    }
    int CalcF() {
        //Ff = df * sin(cf*atan(bf*Bf * 180.0 / M_PI)) * 9.8 * 2.0;
        //Fr = dr * sin(cr*atan(br*Br * 180.0 / M_PI)) * 9.8 * 2.0;
        dFf = V * kf*(Cpf*Bf * RAD2DEG - Ff) / Cpf;
        dFr = V * kr*(Cpr*Br * RAD2DEG - Fr) / Cpr;
        Ff += dFf * step;
        Fr += dFr * step;
        return 0;
    }
    int CalcGy() {
        Gy = V * (dB + dyaw) / 9.8;
        return 0;
    }
    int CalcV() {
        Vx = V * cos(dB);
        Vy = V * sin(dB);
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