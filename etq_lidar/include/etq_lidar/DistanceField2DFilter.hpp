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

            void distance_transform_2D(Matrix& sedt);

            void distance_transform_1D(Matrix& sedt, int row);

    };

} /* namespace */
