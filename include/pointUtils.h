#ifndef POINTSUTILS_H_
#define POINTSUTILS_H_

#include <type_traits>
#include <concepts>

namespace utils {
    // Concept: checks if PointT has x, y, z fields convertible to float
    template <typename PointT>
    concept HasXYZ = requires(PointT pt) {
        { pt.x } -> std::convertible_to<float>;
        { pt.y } -> std::convertible_to<float>;
        { pt.z } -> std::convertible_to<float>;
    };
}

#endif // POINTSUTILS_H_