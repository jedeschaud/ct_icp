#ifndef CT_ICP_NEIGHBORHOOD_STRATEGY_H
#define CT_ICP_NEIGHBORHOOD_STRATEGY_H

#include <SlamCore/experimental/neighborhood.h>

#include "ct_icp/map.h"

namespace ct_icp {

    /*!
     * @brief A Neighborhood strategy constructs neighborhoods from a given map
     */
    class ANeighborhoodStrategy {
    public:

        /*!
         * @brief Constructs a neighborhood of a query in a map in place
         */
        virtual bool ComputeNeighborhoodInPlace(const ISlamMap &map,
                                                const slam::WPoint3D &query,
                                                slam::Neighborhood &,
                                                Eigen::Vector3d *sensor_location = nullptr) const = 0;

        /*!
         * @brief Constructs a neighborhood of a query in a map
         */
        std::optional<slam::Neighborhood> ComputeNeighborhood(const ISlamMap &map,
                                                              const slam::WPoint3D &query,
                                                              Eigen::Vector3d *sensor_location = nullptr) const {
            slam::Neighborhood neighborhood;
            if (ComputeNeighborhoodInPlace(map, query, neighborhood, sensor_location))
                return {std::move(neighborhood)};
            return {};
        };
    };

    struct INeighborStrategyOptions {

        virtual ~INeighborStrategyOptions() = 0;

        virtual std::string GetType() const { return "INVALID_STRATEGY"; }

        virtual std::shared_ptr<ct_icp::ANeighborhoodStrategy> MakeStrategyFromOptions() const {
            throw std::runtime_error("Not implemented Error");
        }

        int max_num_neighbors = 20;

        int min_num_neighbors = 8;

        virtual void FromYAML(const YAML::Node &node) {
            FIND_OPTION(node, (*this), max_num_neighbors, int);
            FIND_OPTION(node, (*this), min_num_neighbors, int);
        }
    };

    /*!
     * @brief Default Nearest strategy (simply select a neighborhood based on nearest neighbors)
     */
    struct DefaultNearestNeighborStrategy : ANeighborhoodStrategy {

        struct Options : INeighborStrategyOptions {

            static std::string Type() { return "NEAREST_NEIGHBOR_STRATEGY"; }

            std::string GetType() const override { return Type(); }

            std::shared_ptr<ct_icp::ANeighborhoodStrategy> MakeStrategyFromOptions() const override {
                return std::make_shared<ct_icp::DefaultNearestNeighborStrategy>(*this);
            }
        } options;

        DefaultNearestNeighborStrategy() {};

        DefaultNearestNeighborStrategy(const Options &options_) : options(options_) {}

        bool ComputeNeighborhoodInPlace(const ISlamMap &map,
                                        const slam::WPoint3D &query,
                                        slam::Neighborhood &neighborhood,
                                        Eigen::Vector3d *location) const override {
            map.ComputeNeighborhoodInPlace(query.world_point, options.max_num_neighbors, neighborhood);
            return neighborhood.points.size() >= options.min_num_neighbors;
        }

    };


    /*!
     * @brief A Neighborhood strategy constructs neighborhood with radius which is adapted with the distance
     *
     * @note The radius is computed with the distance to the sensor, using the formula:
     *       $alpha= (min(radius, distance_{max}) / (distance_{max}))^{exponent}$
     *       $radius=(1.0 - alpha) * radius_{min} + alpha * radius_{max}$
     */
    class DistanceBasedStrategy : public ANeighborhoodStrategy {
    public:

        struct Options : public INeighborStrategyOptions {

            static std::string Type() { return "DISTANCE_BASED_STRATEGY"; }

            std::string GetType() const override { return DistanceBasedStrategy::Options::Type(); }

            std::shared_ptr<ct_icp::ANeighborhoodStrategy> MakeStrategyFromOptions() const override {
                return std::make_shared<DistanceBasedStrategy>(*this);
            }

            void FromYAML(const YAML::Node &node) override {
                INeighborStrategyOptions::FromYAML(node);
                FIND_OPTION(node, (*this), distance_max, double);
                FIND_OPTION(node, (*this), radius_min, double);
                FIND_OPTION(node, (*this), radius_max, double);
                FIND_OPTION(node, (*this), exponent, double);
            }

            double distance_max = 60.; //< (m) Distance maximum for which the radius is maximum

            double radius_min = 0.1; //< (m) Minimum radius for points at distance 0m from the sensor

            double radius_max = 2.0; //< (m) Maximum radius for points at distance greater than `distance_max` from the sensor

            double exponent = 1.0;  //< determines the search radius based on the distance to the sensor

        } options;


        explicit DistanceBasedStrategy(const Options &options_) : options(options_) {}

        inline double ComputeRadius(double distance_to_sensor) const {
            double alpha = std::pow(std::min(std::abs(distance_to_sensor),
                                             options.radius_max) / options.radius_max,
                                    options.exponent);
            return alpha * options.radius_max + (1 - alpha) * options.radius_min;
        }

        bool ComputeNeighborhoodInPlace(const ISlamMap &map,
                                        const slam::WPoint3D &query,
                                        slam::Neighborhood &neighborhood,
                                        Eigen::Vector3d *sensor_location) const override {
            const double radius = ComputeRadius(query.raw_point.point.norm());
            map.RadiusSearchInPlace(query.world_point, neighborhood, radius,
                                    options.max_num_neighbors, true, sensor_location);
            return true;
        }

    };

    // TODO: Graduated Distance: Max radius which diminishes with iterations / motion

} // namespace ct_icp

#endif //CT_ICP_NEIGHBORHOOD_STRATEGY_H
