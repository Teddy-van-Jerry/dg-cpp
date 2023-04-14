#ifndef _DG_BASE_HPP_
#define _DG_BASE_HPP_

#include "common.hpp"
#include "edge.hpp"
#include "except.hpp"
#include <functional>
#include <map>
#include <stack>
#include <string>
#include <unordered_set>
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
    using EdgeT__ = typename std::conditional<std::is_void_v<T>, int, T>::type;
    template <typename T = EdgeT>
    using EdgeT_R_ = typename std::conditional<std::is_void_v<T>, int, T>::type&;
    template <typename T = EdgeT>
    using EdgeT_CR_ = const typename std::conditional<std::is_void_v<T>, int, T>::type&;
    using EdgeT_    = EdgeT__<>;
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

    // primarily for lambda expression
    template <typename Ret, NonFunc Fn, typename... Args>
    Ret edge(IDT from_id, IDT to_id, Fn func, Args... args) const
        requires EdgeVT<EdgeT>;

    template <typename Ret, typename... Args>
    Ret edge(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func, Args... args) const
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

    bool isConnected(IDT from_id, IDT to_id) const;

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
template <typename Ret, NonFunc Fn, typename... Args>
inline Ret DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id, Fn func, Args... args) const
    requires EdgeVT<EdgeT> {
    static_assert(std::is_invocable_r_v<Ret, Fn, DGraphBase<EdgeT, IDT>::EdgeT_, Args...>,
                  "3rd argument (non std::function) must be invocable in method 'edge'.");
    return func(edge(from_id, to_id), args...);
}

template <typename EdgeT, IDVT IDT>
template <typename Ret, typename... Args>
inline Ret DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
                                        Args... args) const
    requires EdgeVT<EdgeT> {
    return func(edge(from_id, to_id), args...);
}

// template <typename EdgeT, IDVT IDT>
// // template <typename Ret, typename... Args>
// // inline Ret DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
// //                                         Args... args) const
// template <typename Ret, typename Fn, typename... Args>
// Ret DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id, Fn func, Args... args) const
//         requires EdgeVT<EdgeT> && std::is_invocable_v<Ret, Fn, DGraphBase<EdgeT, IDT>::EdgeT_, Args...> {
//     return func(edge(from_id, to_id), args...);
// }

// template <typename EdgeT, IDVT IDT>
// template <typename Ret, typename... Args>
// inline Ret& DGraphBase<EdgeT, IDT>::edge(IDT from_id, IDT to_id, std::function<Ret&(Args...)> func, Args... args)
//     requires EdgeVT<EdgeT> {
//     return func(edge(from_id, to_id), args...);
// }

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
inline bool DGraphBase<EdgeT, IDT>::isConnected(IDT from_id, IDT to_id) const {
    // check if nodes exist
    if (!hasNode(from_id) || !hasNode(to_id)) return false;
    // use DFS to search for path from 'from_id' to 'to_id'
    std::unordered_set<IDT> visited; // set of visited nodes
    std::stack<IDT> s;               // stack for DFS
    visited.insert(from_id);
    s.push(from_id);
    while (!s.empty()) {
        IDT node = s.top();
        s.pop();
        auto& edges = g.at(node);
        for (auto& [neighbor, edge] : edges) {
            if (neighbor == to_id) return true; // found path
            if (!visited.count(neighbor)) {
                visited.insert(neighbor);
                s.push(neighbor);
            }
        }
    }
    return false; // no path found
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
