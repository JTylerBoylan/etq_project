#include <etq_lidar/SignedDistanceFieldFilter.hpp>
#include <pluginlib/class_list_macros.h>

using namespace filters;

namespace grid_map {

    SignedDistanceFieldFilter::SignedDistanceFieldFilter() {}

    SignedDistanceFieldFilter::~SignedDistanceFieldFilter() = default;

    bool SignedDistanceFieldFilter::configure() {

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

    bool SignedDistanceFieldFilter::update(const GridMap& mapIn, GridMap& mapOut) {

        mapOut = mapIn;
        mapOut.add(outputLayer_);

        Matrix field = mapIn[inputLayer_];

        compute_edt(field);

        mapOut[outputLayer_] = field;

        return true;
    }

    void SignedDistanceFieldFilter::compute_edt(Matrix& sedt) {

        for (int r = 0; r < sedt.rows(); r++)
            horizontal_pass(sedt, r);

        sedt.transpose();

        for (int r = 0; r < sedt.rows(); r++)
            horizontal_pass(sedt, r);

        sedt.transpose();

        sedt.cwiseSqrt();

    }

    void SignedDistanceFieldFilter::horizontal_pass(Matrix& sedt, int row) {

        int row_size = sedt.row(row).size();

        /* find_hull_parabolas */

        Eigen::MatrixXf hull_vertices(2, row_size), hull_intersections(2, row_size);
        int k = 0;

        hull_vertices(0,0) = 0;
        hull_intersections(0,0) = -INFINITY;
        hull_intersections(0,1) = INFINITY;

        for (int i = 1; i < row_size; i++) {

            Eigen::Vector2f q(i, sedt(row,i)), p;
            float s, n, d;

            do {
                p = hull_vertices.col(k);
                n = (q.y() + q.x()*q.x()) - (p.y() + p.x()*p.x());
                d = 2*q.x() - 2*p.x();
                s = d == 0 ? INFINITY : n / d;
            } while(s <= hull_intersections(k--, 0));

            k += 2;

            hull_vertices.col(k) = q;

            hull_intersections(0,k) = s;
            hull_intersections(0,k+1) = INFINITY;

        }

        /* march_parabolas */

        k = 0;
        for (int q = 0; q < row_size; q++) {
            while (hull_intersections(0,k+1) < q)
                k++;
            float dx = q - hull_vertices(0,k);
            sedt(row,q) = dx * dx + hull_vertices(1,k);
        }

    }

}

PLUGINLIB_EXPORT_CLASS(grid_map::SignedDistanceFieldFilter, filters::FilterBase<grid_map::GridMap>)