#ifndef _DG_EDGE_HPP_
#define _DG_EDGE_HPP_

#include "common.hpp"
#include <algorithm>
#include <concepts>
#include <map>
#include <type_traits>

namespace dg {
template <typename EdgeT>
struct Edge {
    EdgeT data;                                   /**< edge data */
    static inline constexpr bool HAS_DATA = true; /**< Indicating there is data member. */

    Edge() = default;

    Edge(const EdgeT& data_);

    operator EdgeT&();

    operator EdgeT() const;
};

template <>
struct Edge<void> {
    const void* data                      = nullptr; /**< no data actually */
    static inline constexpr bool HAS_DATA = false;   /**< Indicating there is no data member. */

    Edge() = default;
};

template <typename EdgeT>
Edge<EdgeT>::Edge(const EdgeT& data_) : data(data_) {}

template <typename EdgeT>
Edge<EdgeT>::operator EdgeT&() {
    return data;
}

template <typename EdgeT>
Edge<EdgeT>::operator EdgeT() const {
    return data;
}

namespace _protected {
template <IDVT IDT, EdgeVT EdgeT>
static inline std::map<IDT, Edge<EdgeT>> mapAsEdgeMap(const std::map<IDT, EdgeT>& map) {
    std::map<IDT, Edge<EdgeT>> transformed;
    std::transform(map.begin(), map.end(), std::inserter(transformed, transformed.end()),
                   [](const auto& elem) { return std::make_pair(elem.first, static_cast<Edge<EdgeT>>(elem.second)); });
    return transformed;
}

} // namespace _protected

} // namespace dg

#endif
