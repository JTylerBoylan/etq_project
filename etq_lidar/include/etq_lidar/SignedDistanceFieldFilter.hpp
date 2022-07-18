#pragma once

#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>

namespace grid_map {

    class SignedDistanceFieldFilter : public filters::FilterBase<GridMap> {

        public:

            SignedDistanceFieldFilter();

            ~SignedDistanceFieldFilter() override;

            bool configure() override;

            bool update(const GridMap& mapIn, GridMap& mapOut) override;

        private:

            std::string inputLayer_;

            std::string outputLayer_;

            void compute_edt(Matrix& sedt);

            void horizontal_pass(Matrix& sedt, int row);

    };

} /* namespace */