#include <etq_lidar/DistanceField2DFilter.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

    DistanceField2DFilter::DistanceField2DFilter() {}

    DistanceField2DFilter::~DistanceField2DFilter() = default;

    bool DistanceField2DFilter::configure() {

        if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
            ROS_ERROR("SDF Filter did not find parameter 'input_layer'");
            return false;
        }

        if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
            ROS_ERROR("SDF Filter did not find parameter 'output_layer'");
            return false;
        }

        return true;
    }

    bool DistanceField2DFilter::update(const GridMap& mapIn, GridMap& mapOut) {

        mapOut = mapIn;
        mapOut.add(outputLayer_);

        mapOut[outputLayer_] = mapIn[inputLayer_];

        distance_transform_2D(mapOut[outputLayer_]);

        return true;
    }

    void DistanceField2DFilter::distance_transform_2D(Matrix& grid) {
        
        for (int r = 0; r < grid.rows(); r++)
            distance_transform_1D(grid, r);
        
        grid.transposeInPlace();
        
        for (int r = 0; r < grid.cols(); r++)
            distance_transform_1D(grid, r);

        grid.transposeInPlace();

        grid = grid.cwiseSqrt();

    }

    void DistanceField2DFilter::distance_transform_1D(Matrix& grid, int row) {

        int row_size = grid.row(row).size();
        
        int v[row_size+1], z[row_size+1];

        v[0] = 0;
        z[0] = -1000000000;
        z[1] = +1000000000;

        for (int q = 1, k = 0, s = 0; q < row_size; q++) {
            
            do {
                const int r = v[k];
                s = (grid(row,q) - grid(row,r) + q*q - r*r) / (q-r) / 2;
            } while (s <= z[k--]);
            
            k += 2;
            v[k] = q;
            z[k] = s;
            z[k+1] = +1000000000;

        }
        
        for (int q = 0, k = 0; q < row_size; q++) {
            
            while (z[k+1] < q) k++;

            const int r = v[k];

            grid(row, q) = grid(row,r) + (q - r) * (q - r);
            
        }

        
    }

}

PLUGINLIB_EXPORT_CLASS(grid_map::DistanceField2DFilter, filters::FilterBase<grid_map::GridMap>)
