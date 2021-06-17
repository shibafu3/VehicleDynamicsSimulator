#pragma once

#include <cmath>

// http://akiracing.com/2018/01/23/arduino_rc_filter/
class FirstOrderLag {
public :
    double t;
    double data;
    double dt;
    double a;
    FirstOrderLag() {}
    FirstOrderLag(double time_constant, double initial_data, double sampling_time) {
        t = time_constant;
        data = initial_data;
        dt = sampling_time;
        a = t / (t + dt);
    }
    double Input(double input) {
        return data = (1.0 - a)*input + a*data;
    }
};

class Differentiator {
public :
    double prev;
    double dt;
    Differentiator() {}
    Differentiator(double initial_data, double delta_time) {
        prev = initial_data;
        dt = delta_time;
    }
    double Diff(double data) {
        double diff = (data - prev) / dt;
        prev = data;
        return diff;
    }
};

class BicycleModel {

    //                      Z(UP)
    //                      ^
    //             X  ^     |
    //      (FORWARD)  \    |
    //                  \   |
    //                   \  |
    //                    \ |
    //  Y(LEFT)            \|
    //   <------------------

protected :
    const double DEG2RAD = M_PI / 180.0;
    const double RAD2DEG = 180.0 / M_PI;
    const double g = 9.80665;
    const double KGF2N = g;
    const double N2KGF = 1.0 / g;

    double a11() {
        return -2*(Kf + Kr) / (m*V);
    }
    double a12() {
        return -1 - 2/(m*V*V) * (lf*Kf - lr*Kr);
    }
    double a21() {
        return -2 * (lf*Kf - lr*Kr) / I;
    }
    double a22() {
        return -2 * (lf*lf*Kf + lr*lr*Kr) / (I*V);
    }
    double b1() {
        return 2*Kf / (m*V);
    }
    double b2() {
        return 2*lf*Kf/I;
    }

    double CalcWeightf() {
        return (m*lf) / (lf+lr);;
    }
    double CalcWeightr() {
        return (m*lr) / (lf+lr);;
    }
    double CalcdBeta() {
        return a11()*B + a12()*dyaw + b1()*delta;
    }
    virtual double CalcddYaw() {
        return a21()*B + a22()*dyaw + b2()*delta;
    }
    double CalcBeta() {
        return B + dB * dt;
    }
    double CalcdYaw() {
        return dyaw + ddyaw * dt;
    }
    double CalcYaw() {
        return yaw + dyaw * dt;
    }
    double CalcBetaf() {
        return B + lf*dyaw/V - delta;
    }
    double CalcBetar() {
        return B - lr*dyaw/V;
    }
    virtual double CalcKf() = 0;
    virtual double CalcKr() = 0;
    double CalcGy() {
        return V * (dB + dyaw) / g;
    }
    double CalcVx() {
        return V * cos(B);
    }
    double CalcVy() {
        return V * sin(B);
    }
    double CalcX() {
        return x + (Vx * cos(yaw) + Vy * cos(yaw + M_PI_2)) * dt;
    }
    double CalcY() {
        return y + (Vx * sin(yaw) + Vy * sin(yaw + M_PI_2)) * dt;
    }

public :
    // Control  inputs
    double delta = 0.04;     // Input_Tire_Angle [rad]
    double V = 140.0/3.6;    // Velocity [m/s]

    // Vehicle specifications
    double m = 1500;         // Vehicle_Mass [kg]
    double lf = 1.1;         // [m]
    double lr = 1.6;         // [m]
    double I = 2500;         // Inertia [kg*m^2]
    double Wf = 0.0;         // Front_tire_mass [kg]
    double Wr = 0.0;         // Front_tire_mass [kg]
    double Kf = 55000.0;     // Cornaring_Power_front [N/rad]
    double Kr = 60000.0;     // Cornaring_Power_rear [N/rad]

    //Physical phenomenon
    double B = 0.0;          // Vehicle_Slip_angle [rad]
    double dB = 0.0;         // Vehicle_Slip_angle_rate [rad/s]
    double Bf = 0.0;         // Front_Tire_Slip_angle [rad/s]
    double Br = 0.0;         // Rear_Tire_Slip_angle [rad/s]
    double yaw = 0.0;        // Yaw_Angle [rad]
    double dyaw = 0.0;       // Yaw_Rate [rad/s]
    double ddyaw = 0.0;      // Yaw_Rate_Rate [rad/s^2]
    double moment = 0.0;     // Moment [N*m]
    double Yf = 0.0;         // Cornaring_Force_front [N]
    double Yr = 0.0;         // Cornaring_Force_rear [N]
    double dYf = 0.0;        // Cornaring_Force_Rate_front [N]
    double dYr = 0.0;        // Cornaring_Force_Rate_rear [N]
    double Gy = 0.0;         // Side_G [G]
    double Vx = 0.0;         // Velocity_X [m/s]
    double Vy = 0.0;         // Velocity_Y [m/s]
    double x = 0.0;          // Position_X [m]
    double y = 0.0;          // Position_Y [m]

    double dt = 0.01;        // Sampling time [s]

    BicycleModel(double handle_angle_rad,
                 double velocity_mps,
                 double mass_kg,
                 double length_front_m,
                 double length_rear_m,
                 double inertia_yaw_moment,
                 double delta_time) {
        SetControllData(handle_angle_rad, velocity_mps);
        SetVehicleData(mass_kg, length_front_m, length_rear_m, inertia_yaw_moment);
        SetStep(delta_time);
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
    int SetStep(double delta_time) {
        dt = delta_time;
        return 0;
    }

    int Step() {
        Kf = CalcKf();
        Kr = CalcKr();
        dB = CalcdBeta();
        ddyaw = CalcddYaw();
        B = CalcBeta();
        dyaw = CalcdYaw();
        yaw = CalcYaw();
        Bf = CalcBetaf();
        Br = CalcBetar();
        Gy = CalcGy();
        Vx = CalcVx();
        Vy = CalcVy();
        x = CalcX();
        y = CalcY();
        return 0;
    }
    int Step(int step_count) {
        for (int i = 0; i < step_count; ++i) {
            Step();
        }
        return 0;
    }
};

class BicycleModelWithCorneringPower : public BicycleModel {
public :
    BicycleModelWithCorneringPower(double handle_angle_rad,
                                   double velocity_mps,
                                   double mass_kg,
                                   double length_front_m,
                                   double length_rear_m,
                                   double inertia_yaw_moment,
                                   double cornering_power_front,
                                   double cornering_power_rear,
                                   double delta_time) :
                                   BicycleModel(handle_angle_rad,
                                                velocity_mps,
                                                mass_kg,
                                                length_front_m,
                                                length_rear_m,
                                                inertia_yaw_moment,
                                                delta_time) {
        Kf = cornering_power_front;
        Kr = cornering_power_rear;
    }
    double CalcKf() override {
        return Kf;
    }
    double CalcKr() override {
        return Kr;
    }
};

class BicycleModelWithMagicFomula : public BicycleModel {
protected :
    double bf = 0.2;         // Magic_Fomula_front_B []
    double cf = 1.5;         // Magic_Fomula_front_C []
    double df = 376.0;       // Magic_Fomula_front_D []
    double br = 0.31;        // Magic_Fomula_rear_B []
    double cr = 1.28;        // Magic_Fomula_rear_C []
    double dr = 252.0;       // Magic_Fomula_rear_D []
    int SetTireData(double Bf_in, double Cf_in, double Df_in, double Br_in, double Cr_in, double Dr_in) {
        bf = Bf_in;
        cf = Cf_in;
        df = Df_in;
        br = Br_in;
        cr = Cr_in;
        dr = Dr_in;
        return 0;
    }
public :
    BicycleModelWithMagicFomula(double handle_angle_rad,
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
                                   double delta_time) :
                                   BicycleModel(handle_angle_rad,
                                                velocity_mps,
                                                mass_kg,
                                                length_front_m,
                                                length_rear_m,
                                                inertia_yaw_moment,
                                                delta_time) {
        SetTireData(Bf_in, Cf_in, Df_in, Br_in, Cr_in, Dr_in);
    }
    double CalcKf() override {
        return (bf*cf*df*cos(cf*atan(bf*Bf*180.0 / M_PI)) / (atan(bf*Bf*180.0 / M_PI)*atan(bf*Bf*180.0 / M_PI) + 1.0)) * KGF2N * 2.0;
    }
    double CalcKr() override {
        return (br*cr*dr*cos(cr*atan(br*Br*180.0 / M_PI)) / (atan(br*Br*180.0 / M_PI)*atan(br*Br*180.0 / M_PI) + 1.0)) * KGF2N * 2.0;
    }

};
