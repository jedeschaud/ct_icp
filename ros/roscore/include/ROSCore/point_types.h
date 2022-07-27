#ifndef ROSCORE_POINT_TYPES_H
#define ROSCORE_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace slam {

    // A XYZT point compatible with pcl
    struct XYZTPoint {

        inline XYZTPoint(const XYZTPoint &pt) : x(pt.x), y(pt.y), z(pt.z), timestamp(pt.timestamp) {
            data[3] = 1.0f;
        }

        inline XYZTPoint &operator=(const slam::XYZTPoint &pt) {
            x = pt.x;
            y = pt.y;
            z = pt.z;
            timestamp = pt.timestamp;
            return *this;
        }

        inline XYZTPoint() : x(0.f), y(0.f), z(0.f), timestamp(0.) {
            data[3] = 1.0f;
        }

        PCL_ADD_POINT4D

        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    } EIGEN_ALIGN16;

    // LidarPoint: Point Structure for Kitware's SLAM
    // see https://gitlab.kitware.com/keu-computervision/slam/-/blob/master/slam_lib/include/LidarSlam/LidarPoint.h
    struct LidarPoint {

        inline LidarPoint(const LidarPoint &p) : x(p.x), y(p.y), z(p.z), time(p.time), intensity(p.intensity),
                                                 laser_id(p.laser_id), device_id(p.device_id), label(p.label) {
            data[3] = 1.0f;
        }

        inline LidarPoint &operator=(const slam::LidarPoint &p) {
            x = p.x;
            y = p.y;
            z = p.z;
            time = p.time;
            intensity = p.intensity;
            laser_id = p.laser_id;
            device_id = p.device_id;
            label = p.label;
            return *this;
        }

        inline LidarPoint() : x(0.0f), y(0.0f), z(0.0f),
                              time(0.0), intensity(0.0f), laser_id(0), device_id(0), label(0) {
            data[3] = 1.0f;
        }

        PCL_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
        double time;
        float intensity;
        std::uint16_t laser_id;
        std::uint8_t device_id;
        std::uint8_t label;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;


} // namespace slam


POINT_CLOUD_REGISTER_POINT_STRUCT (slam::XYZTPoint,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT (slam::LidarPoint,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (double, time, time)
                                           (float, intensity, intensity)
                                           (std::uint16_t, laser_id, laser_id)
                                           (std::uint8_t, device_id, device_id)
                                           (std::uint8_t, label, label))

#endif //ROSCORE_POINT_TYPES_H
