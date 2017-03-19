#ifndef SOLVER_H
#define SOLVER_H

#include "VectorLib/Vector.h"
#include "VectorLib/Matrix.h"

#include <stdio.h>
#include <iostream>

#include <stdint.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

template<uint32_t numsensors> struct Device
{
    Vector3d sensors[numsensors];
    Vector3d sensors_wp[numsensors];
    Vector3d world_origin;

    Matrix4x4 transform;

    bool sensors_wp_valid;

    Device(Vector3d _sensors[numsensors])
    {
        memcpy(sensors, _sensors, sizeof(sensors));
        sensors_wp_valid = false;
    }

    /**
     * Write the transformed sensor positions into sensors_wp_out.
     */
    void update_world_positions()
    {
        if(sensors_wp_valid)
            return;

        for(uint32_t i = 0; i < numsensors; i++)
        {
            sensors_wp[i] = transform * sensors[i];
        }

        world_origin = transform.translation();

        sensors_wp_valid = true;
    }

    void set_transform(Matrix4x4 _transform)
    {
        sensors_wp_valid = false;

        transform = _transform;
    }
};

template<uint32_t numrays> struct Lighthouse
{
    Vector3d rays[numrays];
    bool rays_valid[numrays];

    void rays_from_target(Device<numrays>& device)
    {
        device.update_world_positions();

        for(uint32_t i = 0; i < numrays; i++)
        {
            rays[i] = device.sensors_wp[i];
        }
    }

    void rays_from_measurements(fp_type h_meas[numrays],
                                fp_type v_meas[numrays])
    {
        for(uint32_t i = 0; i < numrays; i++)
        {
            fp_type h_ang = PI * (h_meas[i] - 0.5);
            fp_type v_ang = PI * (v_meas[i] - 0.5);
            fp_type c_v_ang = cos(v_ang), s_v_ang = sin(v_ang),
                    c_h_ang = cos(h_ang), s_h_ang = sin(h_ang);
            rays[i] = Vector3d(c_v_ang * c_h_ang, c_v_ang * s_h_ang, s_v_ang);
        }
    }
};

template<uint32_t N, template<uint32_t N> class D,
         template<uint32_t N> class L> struct Solver
{
    static Vector3d ray_nearest(const Vector3d& o, const Vector3d& d,
                                const Vector3d& p)
    {
        return (p - o).project(d) + o;
    }

    static Vector3d ray_nearest_delta(const Vector3d& o, const Vector3d& d,
                                      const Vector3d& p)
    {
        return ray_nearest(o, d, p) - p;
    }

    static void get_nearest(D<N>& device, const L<N>& lighthouse, Vector3d out_nearest[N])
    {
        device.update_world_positions();

        for(uint32_t i = 0; i < N; i++)
        {
            out_nearest[i] = ray_nearest(Vector3d::zero, lighthouse.rays[i], device.sensors_wp[i]);
        }
    }

    static void get_nearest_deltas(D<N>& device, const L<N>& lighthouse, Vector3d out_nearest_deltas[N])
    {
        device.update_world_positions();

        for(uint32_t i = 0; i < N; i++)
        {
            out_nearest_deltas[i] = ray_nearest_delta(Vector3d::zero, lighthouse.rays[i], device.sensors_wp[i]);
        }
    }

    static float get_error(D<N>& device, const L<N>& lighthouse)
    {
        float ret = 0;

        device.update_world_positions();

        for(uint32_t i = 0; i < N; i++)
        {
            ret += ray_nearest_delta(Vector3d::zero, lighthouse.rays[i],
                                     device.sensors_wp[i]).magnitude();
        }

        return ret;
    }

    /**
     * @brief Return the minimum and maximum coordinates of the axis-aligned
     *        bounding box enclosing points.
     *
     * @param[out] out_min The minimum corner of the AABB
     * @param[out] out_max The maximum corner of the AABB
     */
    static void get_AABB(const Vector3d points[N], Vector3d& out_min,
                         Vector3d& out_max)
    {
        out_min = Vector3d::max;
        out_max = Vector3d::min;

        for(uint32_t i = 0; i < N; i++)
        {
            out_min = Vector3d::min2(points[i], out_min);
            out_max = Vector3d::max2(points[i], out_max);
        }
    }

    /**
     * @brief Return the size of the axis-aligned bounding box enclosing points.
     * @param points Array of points.
     *
     * return Size of AABB around points
     */
    static float get_AABB_size(const Vector3d points[N])
    {
        Vector3d v_min, v_max;

        get_AABB(points, v_min, v_max);

        return (v_min - v_max).magnitude();
    }

    static Vector3d average_vec(const Vector3d points[N])
    {
        Vector3d ret = Vector3d::zero;

        for(uint32_t i = 0; i < N; i++)
        {
            ret += points[i];
        }

        return ret / N;
    }

    static Quaternion average_quat(const Quaternion rots[N])
    {
        Quaternion ret = Quaternion::zero;

        for(uint32_t i = 0; i < N; i++)
        {
            ret += rots[i];
        }

        return ret / N;
    }

    static Matrix4x4 solve_translation_step(D<N>& device,
                                            const L<N>& lighthouse)
    {
        // Ensure that device properties are up-to-date
        device.update_world_positions();

        Vector3d target_points[N];
        Vector3d target_deltas[N];
        Vector3d correction_vector;

        /* TODO: optimize; consider defining device as having sensorpos
         * average = Vector3d::zero?
         */
        get_nearest(device, lighthouse, target_points);
        get_nearest_deltas(device, lighthouse, target_deltas);

        Vector3d target_delta = average_vec(target_deltas);

        float target_size = get_AABB_size(target_points);
        float current_size = get_AABB_size(device.sensors_wp);

        if(target_size > 0)
        {
            /* Correction vector accounts for change in AABB size (moves device
             * towards or away from lighthouse to correct for apparent scale
             * changes)
             */
            correction_vector = (target_delta + device.world_origin) *
                                (current_size / target_size - 1);

            return Matrix4x4(target_delta + correction_vector);
        }
        else
        {
            return Matrix4x4(target_delta);
        }
    }

    static Matrix4x4 solve_rotation_step(D<N>& device,
                                         const L<N>& lighthouse)
    {
        device.update_world_positions();

        Vector3d target_points[N];
        Vector3d target_deltas[N];

        get_nearest(device, lighthouse, target_points);
        get_nearest_deltas(device, lighthouse, target_deltas);

        Quaternion target_rots[N];

        for(uint32_t i = 0; i < N; i++)
        {
            Vector3d device_rsp = device.sensors_wp[i] - device.world_origin;
            target_rots[i] = Quaternion::rotationBetween(device_rsp,
                                                         device_rsp +
                                                         target_deltas[i]);
        }

        return Matrix4x4(average_quat(target_rots));
    }

    /**
     * Solve for the new device transform.
     */
    static Matrix4x4 solve(D<N>& device, const L<N>& lighthouse,
                           uint32_t iterations)
    {
        D<N> intermediate = device;
        for(uint32_t i = 0; i < iterations; i++)
        {
            Matrix4x4 i_trans = solve_translation_step(intermediate,
                                                       lighthouse);
            intermediate.set_transform(i_trans * intermediate.transform);
            Matrix4x4 i_rot = solve_rotation_step(intermediate,
                                                  lighthouse);
            intermediate.set_transform(i_rot * intermediate.transform);
        }

        return intermediate.transform;
    }
};

#endif // SOLVER_H
