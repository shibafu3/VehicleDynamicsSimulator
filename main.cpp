#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include "BicycleModel.hpp"

using namespace std;

void plot(vector<double> time, vector<double> delta, vector<double> dyaw, vector<double> B) {
    FILE *gp;               // gnuplotへのパイプ
    gp = popen("gnuplot","w");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set xlabel \"time [s]\"\n");
    fprintf(gp, "set yrange [-0.06:0.06]\n");
    fprintf(gp, "set ylabel \"delta, beta [rad]\"\n");
    fprintf(gp, "set y2range [-0.6:0.6]\n");
    fprintf(gp, "set y2label \"yaw rate [rad/s]\"\n");
    fprintf(gp, "set y2tics\n");

    fprintf(gp, "plot '-' with lines title \"delta\", '-' with lines axis x1y2 title \"yaw rate\", '-' with lines title \"Beta\"\n");
    for (int i = 0; i < delta.size(); ++i) {
        fprintf(gp,"%f\t%f\n", time[i], delta[i]);
    }
    fprintf(gp,"e\n");
    for (int i = 0; i < dyaw.size(); ++i) {
        fprintf(gp,"%f\t%f\n", time[i], dyaw[i]);
    }
    fprintf(gp,"e\n");
    for (int i = 0; i < B.size(); ++i) {
        fprintf(gp,"%f\t%f\n", time[i], B[i]);
    }
    fprintf(gp,"e\n");
    fflush(gp);

    getchar();
    pclose(gp);
}

int main() {
    double dt = 0.001;
    BicycleModel car1(0.0,       // Input_Tire_Angle [rad]
                      140 / 3.6, // Velocity [m/s]
                      1500,      // Vehicle_Mass [kg]
                      1.1,       // [m]
                      1.6,       // [m]
                      2500.0,    // Inertia [kg*m^2]
                      0.2,       // Magic_Fomula_front_B []
                      1.5,       // Magic_Fomula_front_C []
                      270,       // Magic_Fomula_front_D []
                      0.31,      // Magic_Fomula_rear_B []
                      1.28,      // Magic_Fomula_rear_C []
                      252.0,     // Magic_Fomula_rear_D []
                      dt
                 );

    ofstream ofs("./plot_data.csv");
    vector<double> x;
    vector<double> y;
    vector<double> Bf;
    vector<double> time;
    vector<double> dyaw;
    vector<double> B;
    vector<double> delta;

    double steer = 0.0;
    for (int i = 0; i < 4 / dt; ++i) {
        if (i < 500) {
            steer = 0.0;
        }
        else {
            steer = 0.04;
        }
        car1.SetSteer(steer);
        car1.Step();

        ofs    << i*dt
        << "," << steer
        << "," << car1.dyaw
        << "," << car1.B
        << "," << car1.x
        << "," << car1.y
        << endl;
        time.push_back(i*dt);
        delta.push_back(steer);
        B.push_back(car1.B);
        dyaw.push_back(car1.dyaw);
        x.push_back(car1.x);
        y.push_back(car1.y);
    }

    plot(time, delta, dyaw, B);
    return 0;
}