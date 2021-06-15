#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include "BicycleModel.hpp"

using namespace std;

int main() {
    BicycleModel car1(0.0,
        30 / 3.6,
        1500,
        1.1,
        1.6,
        2500.0,
        0.2,
        1.5,
        270,
        0.31,
        1.28,
        252.0,
        0.001);


    ofstream ofs("./aaa.csv");

    vector<double> x;
    vector<double> y;
    vector<double> Bf;
    vector<double> step;

    double steer = 3.0 * M_PI / 180.0;
    double steerd = steer * 0.002;
    car1.Step();
    car1.SetSteer(steer);
    for (int i = 0; i < 10 / 0.001; ++i) {
        if (i < 1000) {
            car1.SetSteer(steer);
        }
        else {
            steer = steer - steerd;
            car1.SetSteer(max(0.0, steer));
            car1.SetSteer(0.0);
        }

        car1.Step();
        //cout << "beta " << car1.beta * 180 / M_PI << endl;
        //cout << "yawrate " << car1.yaw_rate * 180 / M_PI << endl;
        //cout << "x " << car1.x * 180 / M_PI << endl;
        //cout << "y " << car1.y * 180 / M_PI << endl;
        ofs << car1.x << "," << car1.y << "," << steer << "," << car1.Bf << "," << car1.Br << "," << car1.dYf << "," << car1.Yf << endl;
        x.push_back(car1.x);
        y.push_back(car1.y);
        step.push_back(i);
        Bf.push_back(car1.Bf);
    }

    return 0;
}