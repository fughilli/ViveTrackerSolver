#include "solver.hpp"
#include "VectorLib/Vector.h"
#include "VectorLib/Matrix.h"

#include <cstdio>
#include <iostream>

#define DEVICE_SENSOR_COUNT 10

typedef Device<DEVICE_SENSOR_COUNT> Device4;
typedef Lighthouse<DEVICE_SENSOR_COUNT> Lighthouse4;

typedef Solver<DEVICE_SENSOR_COUNT, Device, Lighthouse> Solver4;

using namespace std;

int main(void)
{
    //Vector3d sensor_positions[] = {Vector3d(4.8,0,0.3), Vector3d(0,0,4.0), Vector3d(-2.1,4.1,0.3), Vector3d(-2.1,-4.1,0.3)};
    //Vector3d sensor_positions[] = {Vector3d(4.65,0,-0.925), Vector3d(-0.15,0,2.775), Vector3d(-2.25,4.1,-0.925), Vector3d(-2.25,-4.1,-0.925)};
    Vector3d sensor_positions[] = {Vector3d(0.571543,-0.270028,0.885034),Vector3d(0.481701,-0.080215,-0.607670),Vector3d(0.602618,-0.375345,-0.388451),Vector3d(0.088804,-0.635852,-0.507959),Vector3d(0.527620,0.097859,-0.740301),Vector3d(-0.038893,-0.811077,-0.857740),Vector3d(0.129494,-0.639096,-0.546194),Vector3d(-0.189927,-0.791854,-0.392226),Vector3d(-0.004552,-0.149026,-0.021949),Vector3d(0.749208,0.946610,-0.134386)};
    Device4 target_device(sensor_positions);

    Device4 current_device(sensor_positions);

    current_device.set_transform(Matrix4x4(Vector3d(0.1,0.1,0.1)));

    current_device.update_world_positions();

    //for(int i = 0; i < 4; i++)
    //    cout << current_device.sensors_wp[i] << " ";
    //cout << endl;
    //for(int i = 0; i < 4; i++)
    //    cout << current_device.sensors[i] << " ";
    //cout << endl;

    Matrix4x4 base_transform = Matrix4x4(Vector3d(10,5,1));

    //cout << base_transform << ", " << base_transform * Vector3d::zero << "," << base_transform.translation() << endl;

    for(int i = 0; i < 1000; i++)
    {
        target_device.set_transform(base_transform * Matrix4x4(Vector3d::k * 5 * sin(i/20.0)) * Matrix4x4(Vector3d::one.rotationAroundAxis(i * 0.05)));
        Lighthouse4 lighthouse;
        lighthouse.rays_from_target(target_device);

//        cout << "Target permutation " << i << ":" << endl;

        cout << current_device.world_origin << ";";
        for(int i = 0; i < DEVICE_SENSOR_COUNT; i++)
        {
            cout << current_device.sensors_wp[i];
            if(i != (DEVICE_SENSOR_COUNT - 1))
                cout << ":";
        }
        cout << ";" << target_device.world_origin << ";";
        for(int i = 0; i < DEVICE_SENSOR_COUNT; i++)
        {
            cout << target_device.sensors_wp[i];
            if(i != (DEVICE_SENSOR_COUNT - 1))
                cout << ":";
        }
        cout << endl;

        current_device.set_transform(Solver4::solve(current_device, lighthouse, 10));
        current_device.update_world_positions();

//        cout << "\tTarget transform: " << target_device.transform << endl;
//        cout << "\tResult transform: " << current_device.transform << endl;

//        printf("\tError after solve: %f\n", Solver4::get_error(current_device, lighthouse));
    }
}
