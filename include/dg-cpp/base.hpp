#ifndef _DG_BASE_HPP_
#define _DG_BASE_HPP_

#include "common.hpp"
#include "edge.hpp"
#include "except.hpp"
#include <functional>
#include <map>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
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

    bool hasNode(IDT id) const noexcept;

    bool hasEdge(IDT from_id, IDT to_id) const noexcept;

    size_t numNodes() const noexcept;

    size_t numEdges() const noexcept;

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

    // obtain the maximum weight and its corresponding path from from_id to to_id
    std::pair<EdgeT_, std::vector<IDT>> maxWeightPath(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT> && std::is_arithmetic_v<EdgeT>;

    // implementation that accepts lambda
    template <typename Ret, NonFunc Fn, typename... Args>
    std::pair<Ret, std::vector<IDT>> maxWeightPath(IDT from_id, IDT to_id, Fn func, Args... args) const
        requires EdgeVT<EdgeT>;

    // implementation that accepts std::function
    template <typename Ret, typename... Args>
    std::pair<Ret, std::vector<IDT>> maxWeightPath(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
                                                   Args... args) const
        requires EdgeVT<EdgeT>;

    // obtain the minimum weight and its corresponding path from from_id to to_id
    std::pair<EdgeT_, std::vector<IDT>> minWeightPath(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT> && std::is_arithmetic_v<EdgeT>;

    // implementation that accepts lambda
    template <typename Ret, NonFunc Fn, typename... Args>
    std::pair<Ret, std::vector<IDT>> minWeightPath(IDT from_id, IDT to_id, Fn func, Args... args) const
        requires EdgeVT<EdgeT>;

    // implementation that accepts std::function
    template <typename Ret, typename... Args>
    std::pair<Ret, std::vector<IDT>> minWeightPath(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
                                                   Args... args) const
        requires EdgeVT<EdgeT>;

  private:
    template <typename Ret, typename... Args>
    std::pair<Ret, std::vector<IDT>> minOrMaxWeightPath(bool min, IDT from_id, IDT to_id,
                                                        std::function<Ret(EdgeT_, Args...)> func, Args... args) const
        requires EdgeVT<EdgeT>;

  public:
    void insertSubGraph(const DGraphBase<EdgeT, IDT>& dg);

    void merge(const DGraphBase<EdgeT, IDT>& dg);

    void clear();

    void clearEdges();

    bool operator==(const DGraphBase<EdgeT, IDT>& dg) const;

    bool operator!=(const DGraphBase<EdgeT, IDT>& dg) const;

    DGraphBase& operator+=(const DGraphBase<EdgeT, IDT>& dg);

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
inline bool DGraphBase<EdgeT, IDT>::hasNode(IDT id) const noexcept {
    return g.contains(id);
}

template <typename EdgeT, IDVT IDT>
inline bool DGraphBase<EdgeT, IDT>::hasEdge(IDT from_id, IDT to_id) const noexcept {
    auto&& from_n = g.find(from_id);
    return from_n != g.end() && from_n->second.contains(to_id);
}

template <typename EdgeT, IDVT IDT>
inline size_t DGraphBase<EdgeT, IDT>::numNodes() const noexcept {
    return g.size();
}

template <typename EdgeT, IDVT IDT>
inline size_t DGraphBase<EdgeT, IDT>::numEdges() const noexcept {
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
inline std::pair<typename DGraphBase<EdgeT, IDT>::EdgeT_, std::vector<IDT>>
DGraphBase<EdgeT, IDT>::maxWeightPath(IDT from_id, IDT to_id) const
    requires EdgeVT<EdgeT> && std::is_arithmetic_v<EdgeT> {
    return maxWeightPath<EdgeT>(from_id, to_id, [](EdgeT e) { return e; }); // call the lambda one
}

template <typename EdgeT, IDVT IDT>
template <typename Ret, NonFunc Fn, typename... Args>
inline std::pair<Ret, std::vector<IDT>> DGraphBase<EdgeT, IDT>::maxWeightPath(IDT from_id, IDT to_id, Fn func,
                                                                              Args... args) const
    requires EdgeVT<EdgeT> {
    // check if Fn is valid
    static_assert(std::is_invocable_r_v<Ret, Fn, EdgeT_, Args...>,
                  "3rd argument (non std::function) must be invocable in method 'maxWeightPath'.");
    // call the std::function one
    return maxWeightPath<Ret>(from_id, to_id, std::function<Ret(EdgeT_, Args...)>(func), args...);
}

template <typename EdgeT, IDVT IDT>
template <typename Ret, typename... Args>
inline std::pair<Ret, std::vector<IDT>> DGraphBase<EdgeT, IDT>::maxWeightPath(IDT from_id, IDT to_id,
                                                                              std::function<Ret(EdgeT_, Args...)> func,
                                                                              Args... args) const
    requires EdgeVT<EdgeT> {
    return minOrMaxWeightPath<Ret>(false, from_id, to_id, func, args...);
}

template <typename EdgeT, IDVT IDT>
inline std::pair<typename DGraphBase<EdgeT, IDT>::EdgeT_, std::vector<IDT>>
DGraphBase<EdgeT, IDT>::minWeightPath(IDT from_id, IDT to_id) const
    requires EdgeVT<EdgeT> && std::is_arithmetic_v<EdgeT> {
    return minWeightPath<EdgeT>(from_id, to_id, [](EdgeT e) { return e; }); // call the lambda one
}

template <typename EdgeT, IDVT IDT>
template <typename Ret, NonFunc Fn, typename... Args>
inline std::pair<Ret, std::vector<IDT>> DGraphBase<EdgeT, IDT>::minWeightPath(IDT from_id, IDT to_id, Fn func,
                                                                              Args... args) const
    requires EdgeVT<EdgeT> {
    // check if Fn is valid
    static_assert(std::is_invocable_r_v<Ret, Fn, EdgeT_, Args...>,
                  "3rd argument (non std::function) must be invocable in method 'maxWeightPath'.");
    // call the std::function one
    return minWeightPath<Ret>(from_id, to_id, std::function<Ret(EdgeT_, Args...)>(func), args...);
}

template <typename EdgeT, IDVT IDT>
template <typename Ret, typename... Args>
inline std::pair<Ret, std::vector<IDT>> DGraphBase<EdgeT, IDT>::minWeightPath(IDT from_id, IDT to_id,
                                                                              std::function<Ret(EdgeT_, Args...)> func,
                                                                              Args... args) const
    requires EdgeVT<EdgeT> {
    return minOrMaxWeightPath<Ret>(true, from_id, to_id, func, args...);
}

template <typename EdgeT, IDVT IDT>
template <typename Ret, typename... Args>
inline std::pair<Ret, std::vector<IDT>>
DGraphBase<EdgeT, IDT>::minOrMaxWeightPath(bool min, IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
                                           Args... args) const
    requires EdgeVT<EdgeT> {
    // Check if both nodes exist in the graph
    if (!hasNode(from_id) || !hasNode(to_id)) throw node_not_exists("node not found in graph");

    // Check if the from_id is the to_id
    if (from_id == to_id) return { func(g.at(from_id).at(to_id), args...), { from_id } };

    // Calculate the maximum weight path from the source node to all other nodes using Dijkstra's algorithm
    std::map<IDT, Ret> dist;
    std::map<IDT, IDT> prev;
    std::set<std::pair<Ret, IDT>> pq;

    IDT default_ID;
    if constexpr (std::is_same_v<IDT, std::string>) default_ID = "__default_ID";
    else default_ID = IDT(-1);

    // There can not be negative weights in Dijkstra's algorithm
    Ret bound = min ? std::numeric_limits<Ret>::infinity() : Ret(0);

    // Initialize distances and heap
    for (auto&& [id, _] : g) {
        dist[id] = bound;
        prev[id] = default_ID;
        pq.insert({ dist[id], id });
    }
    dist[from_id] = 0;
    pq.erase({ bound, from_id });
    pq.insert({ dist[from_id], from_id });

    while (!pq.empty()) {
        auto curr_id = pq.begin()->second;
        pq.erase(pq.begin());

        if (curr_id == to_id) break;

        for (auto&& [nei_id, weight] : g.at(curr_id)) {
            auto alt = dist[curr_id] + func(weight);
            if (min ? (alt < dist[nei_id]) : (alt > dist[nei_id])) {
                pq.erase({ dist[nei_id], nei_id });
                dist[nei_id] = alt;
                prev[nei_id] = curr_id;
                pq.insert({ dist[nei_id], nei_id });
            }
        }
    }

    // Reconstruct the path by backtracking from the target to the source using the prev map
    std::vector<IDT> path;
    IDT curr_id = to_id;
    while (curr_id != default_ID && prev.find(curr_id) != prev.end()) {
        path.push_back(curr_id);
        curr_id = prev[curr_id];
    }
    std::reverse(path.begin(), path.end());

    // Check if a path was found
    if (path.size() < 1) throw runtime_error("no path found between nodes in method");

    return { dist[to_id], path };
}

template <typename EdgeT, IDVT IDT>
inline void DGraphBase<EdgeT, IDT>::insertSubGraph(const DGraphBase<EdgeT, IDT>& dg) {
    merge(dg);
}

template <typename EdgeT, IDVT IDT>
inline void DGraphBase<EdgeT, IDT>::merge(const DGraphBase<EdgeT, IDT>& dg) {
    for (auto&& [id, cn] : dg.g) {
        if (auto&& i = g.find(id); i == g.end()) g[id] = cn; // not previously existent in *this
        else i->second.insert(cn.begin(), cn.end()); // the node previously exists, so the connected edges are merged
    }
}

template <typename EdgeT, IDVT IDT>
inline void DGraphBase<EdgeT, IDT>::clear() {
    g.clear();
}

template <typename EdgeT, IDVT IDT>
inline void DGraphBase<EdgeT, IDT>::clearEdges() {
    for (auto&& cn : g) cn.second.clear();
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
inline DGraphBase<EdgeT, IDT>& DGraphBase<EdgeT, IDT>::operator+=(const DGraphBase<EdgeT, IDT>& dg) {
    merge(dg);
    return *this;
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
