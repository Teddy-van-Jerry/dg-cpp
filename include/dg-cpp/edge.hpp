#ifndef _DG_EDGE_HPP_
#define _DG_EDGE_HPP_

#include <concepts>
#include <type_traits>

namespace dg {
template <typename EdgeT>
struct Edge {
    EdgeT data;                                   /**< edge data */
    static inline constexpr bool HAS_DATA = true; /**< Indicating there is data member. */

    Edge(const EdgeT& data_);

    operator EdgeT&();

    operator EdgeT() const;
};

template <>
struct Edge<void> {
    const void* const data                = nullptr; /**< no data actually */
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

} // namespace dg

#endif
