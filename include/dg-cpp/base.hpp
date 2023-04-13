#ifndef _DG_BASE_HPP_
#define _DG_BASE_HPP_

#include "common.hpp"
#include "edge.hpp"
#include "except.hpp"
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
template <typename EdgeT = void, IDVT IDT = std::string>
class DGraphBase {
  protected:
    using _ET = Edge<EdgeT>;        /**< edge type */
    using _CN = std::map<IDT, _ET>; /**< connection */
    template <typename T = EdgeT>
    using EdgeT_R_ = typename std::conditional<std::is_void_v<T>, int, T>::type&;
    template <typename T = EdgeT>
    using EdgeT_CR_ = const typename std::conditional<std::is_void_v<T>, int, T>::type&;
    using EdgeT_R   = EdgeT_R_<>;
    using EdgeT_CR  = EdgeT_CR_<>;

  public:
    DGraphBase() = default;

    bool hasNode(IDT id) const;

    bool hasEdge(IDT from_id, IDT to_id) const;

    size_t numNodes() const;

    size_t numEdges() const;

    EdgeT edge(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT>;

    EdgeT_R edge(IDT from_id, IDT to_id)
        requires EdgeVT<EdgeT>;

    std::vector<IDT> nodesID() const;

    void insertNode(IDT id);

  protected:
    void insertNode(IDT id, const _CN& cn, bool node_exists = false);

  public:
    void insertNode(IDT id, const std::map<IDT, EdgeT>& map, bool node_exists = false)
        requires EdgeVT<EdgeT>;

    void removeNode(IDT id, bool keep_edge = false);

    void removeNodeIfExists(IDT id, bool keep_edge = false);

    void insertEdge(IDT from_id, IDT to_id, bool to_exists = false)
        requires std::is_void_v<EdgeT>;

    void insertEdge(IDT from_id, IDT to_id, EdgeT_CR data, bool to_exists = false)
        requires EdgeVT<EdgeT>;

    bool operator==(const DGraphBase<EdgeT, IDT>& dg) const;

    bool operator!=(const DGraphBase<EdgeT, IDT>& dg) const;

    EdgeT operator()(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT>;

    EdgeT_R operator()(IDT from_id, IDT to_id)
        requires EdgeVT<EdgeT>;

  private:
    size_t removeNodeConnectedEdges(IDT id);

  private:
    std::map<IDT, _CN> g; /**< the main graph */
};

template <typename EdgeT, IDVT IDT>
inline bool DGraphBase<EdgeT, IDT>::hasNode(IDT id) const {
    return g.contains(id);
}

template <typename EdgeT, IDVT IDT>
inline bool DGraphBase<EdgeT, IDT>::hasEdge(IDT from_id, IDT to_id) const {
    auto&& from_n = g.find(from_id);
    return from_n != g.end() && from_n->second.contains(to_id);
}

template <typename EdgeT, IDVT IDT>
inline size_t DGraphBase<EdgeT, IDT>::numNodes() const {
    return g.size();
}

template <typename EdgeT, IDVT IDT>
inline size_t DGraphBase<EdgeT, IDT>::numEdges() const {
    size_t s = 0;
    for (auto&& cn : g) s += cn.second.size();
    return s;
}

template <typename EdgeT, IDVT IDT>
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

template <typename EdgeT, IDVT IDT>
typename DGraphBase<EdgeT, IDT>::EdgeT_R DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id)
    requires EdgeVT<EdgeT> {
    auto&& from_n = g.find(from_id);
    if (from_n == g.end()) [[unlikely]]
        throw node_not_exists("from_id does not exist in method 'edge'");
    auto&& to_n = from_n->second.find(to_id);
    if (to_n == from_n->second.end()) [[unlikely]]
        throw node_not_exists("to_id does not exist in method 'edge'");
    return to_n->second;
}

template <typename EdgeT, IDVT IDT>
std::vector<IDT> DGraphBase<EdgeT, IDT>::nodesID() const {
    std::vector<IDT> keys;
    std::transform(g.begin(), g.end(), std::back_inserter(keys), [](const auto& pair) { return pair.first; });
    return keys;
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertNode(IDT id) {
    g[id] = _CN();
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertNode(IDT id, const _CN& cn, bool node_exists) {
    g[id] = cn;
    if (!node_exists) {
        // update graph nodes
        for (auto&& n : cn)
            if (auto&& key = n.first; !g.contains(key)) g[key] = _CN();
    }
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertNode(IDT id, const std::map<IDT, EdgeT>& map, bool node_exists)
    requires EdgeVT<EdgeT> {
    insertNode(id, _protected::mapAsEdgeMap(map), node_exists);
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::removeNode(IDT id, bool keep_edge) {
    if (!g.erase(id)) throw node_not_exists("remove a non-existent node");
    if (!keep_edge) removeNodeConnectedEdges(id);
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::removeNodeIfExists(IDT id, bool keep_edge) {
    g.erase(id);
    if (!keep_edge) removeNodeConnectedEdges(id);
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertEdge(IDT from_id, IDT to_id, bool to_exists)
    requires std::is_void_v<EdgeT> {
    g[from_id][to_id] = Edge<void>();
    if (!to_exists && !g.contains(to_id)) g[to_id] = _CN();
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertEdge(IDT from_id, IDT to_id, typename DGraphBase<EdgeT, IDT>::EdgeT_CR data,
                                        bool to_exists)
    requires EdgeVT<EdgeT> {
    g[from_id][to_id] = data;
    if (!to_exists && !g.contains(to_id)) g[to_id] = _CN();
}

template <typename EdgeT, IDVT IDT>
inline bool DGraphBase<EdgeT, IDT>::operator==(const DGraphBase<EdgeT, IDT>& dg) const {
    return this->g == dg.g;
}

template <typename EdgeT, IDVT IDT>
inline bool DGraphBase<EdgeT, IDT>::operator!=(const DGraphBase<EdgeT, IDT>& dg) const {
    return this->g != dg.g;
}

template <typename EdgeT, IDVT IDT>
inline EdgeT DGraphBase<EdgeT, IDT>::operator()(IDT from_id, IDT to_id) const
    requires EdgeVT<EdgeT> {
    return edge(from_id, to_id);
}

template <typename EdgeT, IDVT IDT>
inline typename DGraphBase<EdgeT, IDT>::EdgeT_R DGraphBase<EdgeT, IDT>::operator()(IDT from_id, IDT to_id)
    requires EdgeVT<EdgeT> {
    return edge(from_id, to_id);
}

template <typename EdgeT, IDVT IDT>
size_t DGraphBase<EdgeT, IDT>::removeNodeConnectedEdges(IDT id) {
    size_t s = 0;
    for (auto&& cn : g) s += cn.second.erase(id);
    return s;
}

} // namespace dg

#endif
