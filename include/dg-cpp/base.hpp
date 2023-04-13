#ifndef _DG_BASE_HPP_
#define _DG_BASE_HPP_

#include "dg-cpp/common.hpp"
#include "dg-cpp/edge.hpp"
#include "dg-cpp/except.hpp"
#include <map>
#include <string>
#include <vector>

/**
 * @brief Directed Graph namespace.
 *
 * @details You can use the following code to use classes without specifying the dg namespace.
 * @code {.cpp}
 * using namespace dg; // use namespace for directed graph
 * @endcode
 */
namespace dg {
template <typename EdgeT = void, typename IDT = std::string>
class DGraphBase {
  protected:
    using _ET = Edge<EdgeT>;        /**< edge type */
    using _CN = std::map<IDT, _ET>; /**< connection */

  public:
    DGraphBase() = default;

    bool hasNode(IDT id_) const;

    bool hasEdge(IDT from_id, IDT to_id) const;

    size_t numNodes() const;

    size_t numEdges() const;

    EdgeT edge(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT>;

    EdgeT& edge(IDT from_id, IDT to_id)
        requires EdgeVT<EdgeT>;

  private:
    std::map<IDT, _CN> g; /**< the main graph */
};

template <typename EdgeT, typename IDT>
inline bool DGraphBase<EdgeT, IDT>::hasNode(IDT id_) const {
    return g.contains(id_);
}

template <typename EdgeT, typename IDT>
inline bool DGraphBase<EdgeT, IDT>::hasEdge(IDT from_id, IDT to_id) const {
    auto&& from_n = g.find(from_id);
    return from_n != g.end() && from_n->second.contains(to_id);
}

template <typename EdgeT, typename IDT>
inline size_t DGraphBase<EdgeT, IDT>::numNodes() const {
    return g.size();
}

template <typename EdgeT, typename IDT>
inline size_t DGraphBase<EdgeT, IDT>::numEdges() const {
    size_t s = 0;
    for (auto&& cn : g) s += cn.first.size();
    return s;
}

template <typename EdgeT, typename IDT>
EdgeT DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id) const
    requires EdgeVT<EdgeT> {
    auto&& from_n = g.find(from_id);
    if (from_n == g.end()) [[unlikely]]
        throw node_not_exists("from_id does not exist in method 'edge'");
    auto&& to_n = from_n->second.find(to_id);
    if (to_n == from_n->second.end()) [[unlikely]]
        throw node_not_exists("to_id does not exist in method 'edge'");
    return to_n->second;
}

template <typename EdgeT, typename IDT>
EdgeT& DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id)
    requires EdgeVT<EdgeT> {
    auto&& from_n = g.find(from_id);
    if (from_n == g.end()) [[unlikely]]
        throw node_not_exists("from_id does not exist in method 'edge'");
    auto&& to_n = from_n->second.find(to_id);
    if (to_n == from_n->second.end()) [[unlikely]]
        throw node_not_exists("to_id does not exist in method 'edge'");
    return to_n->second;
}

} // namespace dg

#endif
