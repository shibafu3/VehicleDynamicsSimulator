#include <iostream>
#include <fstream>
#include "BicycleModel.hpp"

using namespace std;

int main() {
    BicycleModel car1(0.0,
        1 / 3.6,
        1500,
        1.0453,
        1.7546,
        3024.0,
        0.2,
        1.5,
        376.0,
        0.31,
        1.28,
        252.0, 
        0.001);


    ofstream ofs("C:/Users/0133752/Desktop/aaa.csv");

    car1.Step();
    car1.SetSteer(5.0 * M_PI /180.0);
    for (int i = 0; i < 10 / 0.001; ++i) {
        car1.Step();
        //cout << "beta " << car1.beta * 180 / M_PI << endl;
        //cout << "yawrate " << car1.yaw_rate * 180 / M_PI << endl;
        //cout << "x " << car1.x * 180 / M_PI << endl;
        //cout << "y " << car1.y * 180 / M_PI << endl;
        ofs << car1.x << "," << car1.y << endl;
    }

    return 0;
}