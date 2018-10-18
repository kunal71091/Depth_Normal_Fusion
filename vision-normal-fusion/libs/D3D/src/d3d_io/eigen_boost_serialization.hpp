#ifndef EIGEN_BOOST_SERIALIZATION_HPP
#define EIGEN_BOOST_SERIALIZATION_HPP

#include <Eigen/Core>
#include <boost/serialization/array.hpp>

namespace boost
{
namespace serialization {
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(
        Archive & ar,
        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
        const unsigned int file_version
        )
{
    size_t rows = t.rows(), cols = t.cols();
    ar & rows;
    ar & cols;
    if( rows != static_cast<unsigned long>(t.rows()) || cols != static_cast<unsigned long>(t.cols()) )
        t.resize( rows, cols );

    for(size_t i=0; i< static_cast<unsigned long>(t.size()); i++)
        ar & t.data()[i];
}
}
}
#endif // EIGEN_BOOST_SERIALIZATION_HPP
