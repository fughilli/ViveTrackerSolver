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
    Vector3d sensor_positions[] = {Vector3d(4.8,0,0.3),
                                   Vector3d(0,0,4.0),
                                   Vector3d(-2.1,4.1,0.3),
                                   Vector3d(-2.1,-4.1,0.3)};

    Device4 target_device(sensor_positions);
    Device4 current_device(sensor_positions);

    current_device.set_transform(Matrix4x4(Vector3d(0.1,0.1,0.1)));
    current_device.update_world_positions();

    Matrix4x4 base_transform = Matrix4x4(Vector3d(10,5,1));

    for(int i = 0; i < 1000; i++)
    {
        target_device.set_transform(
            base_transform * Matrix4x4(Vector3d::k * 5 * sin(i/20.0)) *
            Matrix4x4(Vector3d::one.rotationAroundAxis(i * 0.05)));

        Lighthouse4 lighthouse;
        lighthouse.rays_from_target(target_device);

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

        current_device.set_transform(Solver4::solve(current_device, lighthouse,
                                                    10).orthogonalize());
        current_device.update_world_positions();
    }
}
