//
// Created by yaozhuo on 2024/6/1.
//

#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // yz: determine priority of conflicts
    bool operator<(const Conflict &conflict1, const Conflict &conflict2) // return true if conflict2 has higher priority
    {
//        if (conflict1.priority == conflict2.priority) {
//            if (conflict1.type == conflict2.type) {
//                if (conflict1.secondary_priority == conflict2.secondary_priority) {
//                    return rand() % 2;
//                }
//                return conflict1.secondary_priority > conflict2.secondary_priority;
//            }
//            return conflict1.type > conflict2.type;
//        }
//        return conflict1.priority > conflict2.priority;
         return conflict1.t1 > conflict2.t2;
    }

    bool isSamePath(const std::vector<size_t>& path1, const std::vector<size_t>& path2) {
        if(path1.size() != path2.size()) { return false; }
        for(int t = 0; t < path1.size(); t++) {
            if(path1[t] != path2[t]) {
                return false;
            }
        }
        return true;
    }

    RotateMatrix initializeRotateMatrix(const int& dim, const int& orient) {
        RotateMatrix retv = RotateMatrix(dim, std::vector<int>(dim, 0));
        // 1, get unit matrix
        for(int i=0; i<dim; i++) {
            retv[dim][dim] = 1;
        }
        // 2，set value
        return retv;
    }

    // for calculate rotate matrix about 2D and 3D maps
    RotateMatrixs initializeRotateMatrixs(const int& dim) {
        assert(dim == 2);
        RotateMatrixs retv;
        for(int orient=0; orient<2*dim; orient++) {
            double theta = orientToPi_2D(orient);
            std::cout << " theta = " << theta << std::endl;
            retv.push_back(initRotateMatrix_2D(theta));
        }
        return retv;
    }

    std::vector<RotateMatrixs> initializeAllRotateMatrixs() {
        std::vector<RotateMatrixs> retv;
        retv.push_back({}); // occupied 0 dimension
        retv.push_back({}); // occupied 1 dimension
//        for(int dim=2; dim<=3; dim++) {
//            retv.push_back(initializeRotateMatrixs(dim));
//        }
        retv.push_back(initializeRotateMatrixs(2));
        return retv;
    }

    std::vector<RotateMatrixs> ROTATE_MATRIXS = initializeAllRotateMatrixs();

    RotateMatrix initRotateMatrix_2D(const double& angle) {
        RotateMatrix mt = RotateMatrix(2, std::vector<int>(2, 0));
        mt[0][0] =  cos(angle);
        mt[0][1] =  sin(angle);
        mt[1][0] = -sin(angle);
        mt[1][1] =  cos(angle);
        return mt;
    }

    double orientToPi_2D(const int& orient) {
        double theta = 0;
        switch (orient) {
            case 0:
                theta = 0;
                break;
            case 1:
                theta = M_PI;
                break;
            case 2:
                theta = M_PI/2;
                break;
            case 3:
                theta = 3*M_PI/2;
                break;
            default:
                std::cerr << "wrong 2D orient = " << orient << std::endl;
                exit(0);
                break;
        }
        return theta;
    }

    Pointi<2> pointRotate_2D(const Pointi<2>& pt, const int& orient) {
        return rotatePoint<int, 2>(pt, ROTATE_MATRIXS[2][orient]);
    }

    Pointi<2> pointTransfer_2D(const Pointi<2>& pt, const Pose<2>& pose) {
        return pointRotate_2D(pt + pose.pt_, pose.orient_);
    }


}