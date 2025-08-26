//
// Created by yunfan on 11/8/24.
//

#ifndef SUPER_EIGEN_ALIAS_HPP
#define SUPER_EIGEN_ALIAS_HPP

#include <Eigen/Eigen>


#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))
#define SIGN(x) ((x > 0) - (x < 0))

namespace super_utils{

/*
 * @\brief Rename the float type used in lib

    Default is set to be double, but user can change it to float.
*/
    using decimal_t = double;

///Pre-allocated std::vector for Eigen using vec_E
    template<typename T>
    using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector
    template<int N>
    using Vecf = Eigen::Matrix<decimal_t, N, 1>;
///Eigen 1D int vector
    template<int N>
    using Veci = Eigen::Matrix<int, N, 1>;
///MxN Eigen matrix
    template<int M, int N>
    using Matf = Eigen::Matrix<decimal_t, M, N>;
///MxN Eigen matrix with M unknown
    template<int N>
    using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;

///MxN Eigen matrix with N unknown
    template<int M>
    using MatMDf = Eigen::Matrix<decimal_t, M, Eigen::Dynamic>;

///Vector of Eigen 1D float vector
    template<int N>
    using vec_Vecf = vec_E<Vecf<N>>;
///Vector of Eigen 1D int vector
    template<int N>
    using vec_Veci = vec_E<Veci<N>>;

///Eigen 1D float vector of size 2
    using Vec2f = Vecf<2>;
///Eigen 1D int vector of size 2
    using Vec2i = Veci<2>;
///Eigen 1D float vector of size 3
    using Vec3f = Vecf<3>;
///Eigen 1D int vector of size 3
    using Vec3i = Veci<3>;
///Eigen 1D float vector of size 4
    using Vec4f = Vecf<4>;
///Column vector in float of size 6
    using Vec6f = Vecf<6>;

///Vector of type Vec2f.
    using vec_Vec2f = vec_E<Vec2f>;
///Vector of type Vec2i.
    using vec_Vec2i = vec_E<Vec2i>;
///Vector of type Vec3f.
    using vec_Vec3f = vec_E<Vec3f>;
///Vector of type Vec3i.
    using vec_Vec3i = vec_E<Vec3i>;

///2x2 Matrix in float
    using Mat2f = Matf<2, 2>;
///3x3 Matrix in float
    using Mat3f = Matf<3, 3>;
///4x4 Matrix in float
    using Mat4f = Matf<4, 4>;
///6x6 Matrix in float
    using Mat6f = Matf<6, 6>;

///Dynamic Nx1 Eigen float vector
    using VecDf = Vecf<Eigen::Dynamic>;
///Dynamic Nx1 Eigen int vector
    using VecDi = Veci<Eigen::Dynamic>;
///Nx2 Eigen float matrix
    using MatD2f = MatDNf<2>;
///Nx3 Eigen float matrix
    using MatD3f = MatDNf<3>;
///Nx4 Eigen float matrix
    using MatD4f = MatDNf<4>;
///4xM Eigen float matrix
    using Mat4Df = MatMDf<4>;
    using MatPlanes = MatD4f;
///3xM Eigen float matrix
    using Mat3Df = MatMDf<3>;
    using MatPoints = Mat3Df;

    using PolyhedronV = Mat3Df;
    using PolyhedronH = MatD4f;
    using PolyhedraV = vec_E<PolyhedronV>;
    using PolyhedraH = vec_E<PolyhedronH>;

///Dynamic MxN Eigen float matrix
    using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;

///Allias of Eigen::Affine2d
    using Aff2f = Eigen::Transform<decimal_t, 2, Eigen::Affine>;
///Allias of Eigen::Affine3d
    using Aff3f = Eigen::Transform<decimal_t, 3, Eigen::Affine>;


#ifndef EIGEN_QUAT
#define EIGEN_QUAT
///Allias of Eigen::Quaterniond
    using Quatf = Eigen::Quaternion<decimal_t>;
#endif

#ifndef EIGEN_EPSILON
#define EIGEN_EPSILON
///Compensate for numerical error
    constexpr decimal_t
            epsilon_ = 1e-10; // numerical calculation error
#endif
// Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> way_pts_E(way_pts[0].data(), 3, way_pts.size());

    using vec_Vec3f = vec_E<Vec3f>;


    using StatePVA = Eigen::Matrix<double, 3, 3>;
    using StatePVAJ = Eigen::Matrix<double, 3, 4>;
    using TimePosPair = std::pair<double, Vec3f>;
    using Line = std::pair<Vec3f, Vec3f>;
    using Pose = std::pair<Vec3f, Quatf>;


}


#endif //SUPER_EIGEN_ALIAS_HPP
