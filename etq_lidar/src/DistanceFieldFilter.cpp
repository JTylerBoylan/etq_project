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

        mapOut[outputLayer_] = distance_transform_2D(mapIn[inputLayer_]);

        return true;
    }

    Matrix& DistanceField2DFilter::distance_transform_2D(const Matrix& grid) {

        Matrix Df = grid;
        
        for (int r = 0; r < grid.rows(); r++)
            Df = distance_transform_1D(Df, r);
        
        Df.transposeInPlace();
        
        for (int r = 0; r < grid.cols(); r++)
            Df = distance_transform_1D(Df, r);

        Df.transposeInPlace();

        Df = Df.cwiseSqrt();

        return Df;
    }

    Matrix& DistanceField2DFilter::distance_transform_1D(const Matrix& ref, int row) {

        Matrix Df = ref;
        
        int row_size = ref.row(row).size();
        
        float v[row_size], z[row_size];
        int k = 1;

        v[0] = 0;
        z[0] = -INFINITY;
        z[1] = INFINITY;


        for (int q = 1; q < row_size; q++) {
            
            float s;
            do {
                k--;
                s = ((ref(row,q) + q*q) - (ref(row, v[k]) + v[k]*v[k])) / (2*q - 2*v[k]);
            } while (s <= z[k]);
            
            k++;
            v[k] = q;
            z[k] = s;
            z[k+1] = INFINITY;
        }
        
        k = 0;
        for (int q = 0; q < row_size; q++) {
            
            while (z[k+1] < q) k++;
            
            Df(row, q) = (q - v[k])*(q - v[k]) + ref(row, v[k]);
            
        }
        
        return Df;
    }

}

PLUGINLIB_EXPORT_CLASS(grid_map::DistanceField2DFilter, filters::FilterBase<grid_map::GridMap>)
