#ifndef CT_ICP_VIZ3D_UTILS_H
#define CT_ICP_VIZ3D_UTILS_H

#ifdef CT_ICP_WITH_VIZ

#include <viz3d/types.h>
#include <ct_icp/types.h>

namespace ct_icp {

    /*!
     * Converts an array of ct_icp::Point3D points to raw points
     * @param world_points  Whether to use world points or raw points
     */
    viz::ArrayV3f ct_icp_to_viz3d_pc(const std::vector<slam::WPoint3D> &points, bool world_points = true);


    enum COLOR_SCHEME {
        JET,
        MAGMA,
        VIRIDIS
    };

    /*! Converts an array of scalar to a vector of RGB colors
     * @param normalize Whether to normalize (between 0 and 1) the scalars or not
     */
    viz::ArrayV3f get_viz3d_color(const std::vector<double> &scalars, bool normalize = true, COLOR_SCHEME cmap = JET);

    enum COLOR_FIELD {
        X, Y, Z, T
    };

    /*!
     * Returns an array of color for a point cloud
     */
    viz::ArrayV3f get_field_color(const std::vector<slam::WPoint3D> &points,
                                  COLOR_SCHEME cmap = JET, COLOR_FIELD cfield = T);

    /*!
     * Returns an array of poses from a trajectory
     */
    viz::ArrayM4f ct_icp_to_viz3d_poses(const std::vector<TrajectoryFrame>& trajectory);
}

#endif

#endif //CT_ICP_VIZ3D_UTILS_H
