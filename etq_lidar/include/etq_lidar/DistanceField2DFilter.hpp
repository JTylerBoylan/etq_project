#pragma once

#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

    class DistanceField2DFilter : public filters::FilterBase<GridMap> {

        public:

            DistanceField2DFilter();

            ~DistanceField2DFilter() override;

            bool configure() override;

            bool update(const GridMap& mapIn, GridMap& mapOut) override;

        private:

            std::string inputLayer_;

            std::string outputLayer_;

            Matrix& distance_transform_2D(const Matrix& grid);

            Matrix& distance_transform_1D(const Matrix& grid, int row);

    };

} /* namespace */
