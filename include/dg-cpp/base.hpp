/**
 * @file base.hpp
 * @author Wuqiong Zhao (me@wqzhao.org)
 * @brief Directed Graph Base Class
 * @version 0.1.0
 * @date 2023-04-16
 *
 * @copyright Copyright (c) 2023 Wuqiong Zhao (Teddy van Jerry)
 *
 */

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
/**
 * @brief Directed graph base class.
 *
 * @details If the edge only represents connection, the EdgeT can be void.
 * @tparam EdgeT Type of edge.
 * @tparam IDT Type of ID.
 */
template <typename EdgeT = void, IDVT IDT = std::string>
class DGraphBase {
  protected:
    using _ET = Edge<EdgeT>;        /**< edge type */
    using _CN = std::map<IDT, _ET>; /**< connection */

    /**
     * @brief Edge type that avoids void.
     *
     * @details Since some operations do not allow void, so we change the void type to a non-void type,
     *          while retaining other types as the same.
     * @tparam T Same as EdgeT. Do not specify this type yourself.
     */
    template <typename T = EdgeT>
    using EdgeT__ = typename std::conditional<std::is_void_v<T>, int, T>::type;

    /**
     * @brief Reference to edge type that avoids void.
     *
     * @details Since some operations do not allow void&, so we change the void type to non-void type,
     *          while retaining other types as the same reference.
     * @tparam T Same as EdgeT. Do not specify this type yourself.
     */
    template <typename T = EdgeT>
    using EdgeT_R_ = typename std::conditional<std::is_void_v<T>, int, T>::type&;

    /**
     * @brief Const reference to edge type that avoids void.
     *
     * @details Since some operations do not allow const void&, so we change the void type to non-void type,
     *          while retaining other types as the same const reference.
     * @tparam T Same as EdgeT. Do not specify this type yourself.
     */
    template <typename T = EdgeT>
    using EdgeT_CR_ = const typename std::conditional<std::is_void_v<T>, int, T>::type&;

    using EdgeT_   = EdgeT__<>;   /**< safe edge type */
    using EdgeT_R  = EdgeT_R_<>;  /**< safe edge reference type */
    using EdgeT_CR = EdgeT_CR_<>; /**< safe edge const reference type */

  public:
    /**
     * @brief Construct a new DGraphBase object.
     *
     * This is the default constructor.
     */
    DGraphBase() = default;

    /**
     * @brief Check if the graph has a node with id.
     *
     * @param id The node id (which should be unique).
     * @retval true There exists the node in the graph.
     * @retval false There does not exist the node in the graph.
     */
    bool hasNode(IDT id) const noexcept;

    /**
     * @brief Check if the graph has an edge linking from_id and to_id.
     *
     * @details If either from_id or to_id node does not exist, we conclude there is no such edge.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @retval true There exists the edge in the graph.
     * @retval false There does not exist the edge in the graph.
     */
    bool hasEdge(IDT from_id, IDT to_id) const noexcept;

    /**
     * @brief The total number of nodes in the graph.
     *
     * @return (size_t) The number of nodes.
     */
    size_t numNodes() const noexcept;

    /**
     * @brief The total number of edges in the graph.
     *
     * @return (size_t) The number of edges.
     */
    size_t numEdges() const noexcept;

    /**
     * @brief The total number of nodes in the graph.
     *
     * @details This is the alias for numNodes().
     * @return (size_t) The number of edges.
     */
    size_t size() const noexcept;

    /**
     * @brief Get the edge data between two nodes.
     *
     * @note The EdgeT should not be void.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @return (EdgeT) The edge data.
     */
    EdgeT edge(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Get the edge data reference between two nodes.
     *
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @return (EdgeT_R) The edge data reference.
     */
    EdgeT_R edge(IDT from_id, IDT to_id)
        requires EdgeVT<EdgeT>;

    /**
     * @brief Get the edge data after processing with a Lambda expression.
     *
     * @details You can also use the `std::function` version instead of the Lambda expression.\n
     *          You need to manually set the return type as the function template argument, for example
     * @code {.cpp}
     * // dgb1 is of type DGraphBase<int, std::string>.
     * int result = dgb1.edge<int>(std::string("n1"), std::string("n2"), [](int weight) {
     *     return weight + 1;
     * });
     * @endcode
     *
     * @note The EdgeT should not be void.
     * @tparam Ret The return type of the Lambda expression as well as this function.
     * @tparam Fn The Lambda expression type.
     * @tparam Args The argument types of the Lambda expression.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The Lambda expression.
     * @param args The arguments of the Lambda expression.
     * @return (Ret) The result of the Lambda expression.
     */
    template <typename Ret, NonFunc Fn, typename... Args>
    Ret edge(IDT from_id, IDT to_id, Fn func, Args... args) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Get the edge data after processing with a `std::function`.
     *
     * @details You have to specifically define the `std::function`, for example
     * @code {.cpp}
     * // dgb1 is of type DGraphBase<int, std::string>.
     * int result = dgb1.edge(std::string("n1"), std::string("n2"),
     *                        std::function<int(int, int)>([](int weight, int n) { return weight + n; }), 2);
     * @endcode
     * You can also use the Lambda version of the `edge` method instead of the `std::function` version.
     *
     * @note The EdgeT should not be void.
     * @tparam Ret The return type of the `std::function`.
     * @tparam Args The argument types of the `std::function`.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The `std::function`.
     * @param args The arguments of the `std::function`.
     * @return (Ret) The result of the `std::function`.
     */
    template <typename Ret, typename... Args>
    Ret edge(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func, Args... args) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Check if the graph is strictly valid.
     *
     * @details A graph is strictly valid if:
     *            1. All nodes set by the edges are in the graph.
     * @retval true
     * @retval false
     */
    bool strictCheck() const;

    /**
     * @brief Get all nodes IS in the graph.
     *
     * @return (std::vector<IDT>) A vector of nodes ID.
     */
    std::vector<IDT> nodesID() const;

    /**
     * @brief Get all edge connections in the graph.
     *
     * @return (std::vector<std::pair<IDT, IDT>>) A vector of edge connections, in `std::pair`.
     */
    std::vector<std::pair<IDT, IDT>> edges() const;

    /**
     * @brief Insert a node.
     *
     * @details If the node id exists before, its edge connections will be cleared.
     *          If the node id does not exist, it will be created.
     * @param id The node id.
     */
    void insertNode(IDT id);

  protected:
    /**
     * @brief Insert a node.
     *
     * @param id The node id.
     * @param cn The edge connections.
     * @param nodes_exist Whether the node exists before insertion.
     */
    void insertNode(IDT id, const _CN& cn, bool nodes_exist = false);

  public:
    /**
     * @brief Insert a node with edge data.
     *
     * @details If the node id exists before, its edge connections will be cleared.
     *          If the node id does not exist, it will be created.\n
     *          If `nodes_exist` is set to true, the node graph will not be updated.
     *          Otherwise (the default behavior) the node graph will be updated by checking all connected edges.
     * @note `nodes_exist` does not mean whether the node with id exists.
     *       It specifies whether the connected nodes exist.
     * @note EdgeT should not be void.
     * @param id The node id.
     * @param map The edge data map.
     * @param nodes_exist Whether all nodes in edge connections exist before insertion (default as false).
     *                    You should be sure when setting this to true. Otherwise there can be unexpected behaviors.
     */
    void insertNode(IDT id, const std::map<IDT, EdgeT>& map, bool nodes_exist = false)
        requires EdgeVT<EdgeT>;

    /**
     * @brief Remove the node from the graph.
     *
     * @exception dg::node_not_exists Removes a non-existent node.
     *            Use removeNodeIfExists() to avoid the exception.
     * @param id The node id.
     * @param keep_edge Whether keeps the connected edges after removing the node (default as false).
     */
    void removeNode(IDT id, bool keep_edge = false);

    /**
     * @brief Remove the node from the graph if it exists.
     *
     * @details Nothing will be done if the node does not exist.
     * @param id The node id.
     * @param keep_edge Whether keeps the connected edges after removing the node (default as false).
     */
    void removeNodeIfExists(IDT id, bool keep_edge = false);

    void insertEdgeToExists(IDT from_id, IDT to_id)
        requires std::is_void_v<EdgeT>;

    void insertEdgeToExists(IDT from_id, IDT to_id, EdgeT_CR data)
        requires EdgeVT<EdgeT>;

    /**
     * @brief Insert an edge to the graph without data.
     *
     * @param from_id The from node id. If it does not exist, it will be created automatically.
     * @param to_id The to node id. If it does not exist, it will be created automatically.
     * @param to_exists Whether the to node exists before insertion.
     *                  If this is set to true, the from node graph will not be updated.
     *                  Please be sure before setting this to true.
     */
    void insertEdge(IDT from_id, IDT to_id, bool to_exists = false)
        requires std::is_void_v<EdgeT>;

    /**
     * @brief Insert an edge to the graph with data.
     *
     * @param from_id The from node id. If it does not exist, it will be created automatically.
     * @param to_id The to node id. If it does not exist, it will be created automatically.
     * @param data The edge data.
     * @param to_exists Whether the to node exists before insertion.
     *                  If this is set to true, the from node graph will not be updated.
     *                  Please be sure before setting this to true.
     */
    void insertEdge(IDT from_id, IDT to_id, EdgeT_CR data, bool to_exists = false)
        requires EdgeVT<EdgeT>;

    /**
     * @brief Check whether to node are connected.
     *
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @retval true There exists a path.
     * @retval false There does not exist a path.
     */
    bool isConnected(IDT from_id, IDT to_id) const;

    /**
     * @brief Obtain the maximum weight and its corresponding path between two nodes.
     *
     * @details This finds the maximum weight path with the Djkstra algorithm.
     *          So all weights should be **non-negative**.\n
     *          Example use:
     * @code {.cpp}
     * DGraphBase<double> dgb3;
     * dgb3.insertEdge("A", "B", 1.2);
     * dgb3.insertEdge("A", "C", 0.5);
     * dgb3.insertEdge("C", "B", 1.5);
     * auto [w, n] = dgb3.maxWeightPath("A", "B"); // w is weight, n is path
     * @endcode
     *
     * @note EdgeT should be arithmetic, i.e. edge data represents the weight.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @return (std::pair<EdgeT, std::vector<IDT>>) The maximum weight and its corresponding path.
     */
    std::pair<EdgeT_, std::vector<IDT>> maxWeightPath(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT> && std::is_arithmetic_v<EdgeT>;

    /**
     * @brief Obtain the maximum weight and its corresponding path between two nodes.
     *
     * @details This finds the maximum weight path with the Djkstra algorithm.
     *          So all weights should be **non-negative**.\n
     *          Example use:
     * @code {.cpp}
     * struct DataT {
     *     double data = 0;
     *     int foo     = 0;
     * };
     * DGraphBase<DataT> dgb2;
     * dgb2.insertEdge("A", "B", DataT{ 1.2, 1 });
     * dgb2.insertEdge("A", "C", DataT{ 0.5, 1 });
     * dgb2.insertEdge("C", "B", DataT{ 1.5, 1 });
     * auto [w, n] = dgb2.maxWeightPath<double>("A", "B", [](DataT x) { return x.data; });
     * @endcode
     *
     * @tparam Ret The weight type, i.e. Lambda function return type.
     * @tparam Fn The Lambda expression type.
     * @tparam Args The Lambda expression argument types.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The Lambda expression.
     * @param args The Lambda expression arguments.
     * @return (std::pair<Ret, std::vector<IDT>>) The maximum weight and its corresponding path.
     */
    template <typename Ret, NonFunc Fn, typename... Args>
    std::pair<Ret, std::vector<IDT>> maxWeightPath(IDT from_id, IDT to_id, Fn func, Args... args) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Obtain the maximum weight and its corresponding path between two nodes.
     *
     * @details This finds the maximum weight path with the Djkstra algorithm.
     *          So all weights should be **non-negative**.
     * @tparam Ret The weight type, i.e. `std::function` return type.
     * @tparam Args `std::function` argument types.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The `std::function`.
     * @param args The `std::function` arguments.
     * @return (std::pair<Ret, std::vector<IDT>>) The maximum weight and its corresponding path.
     */
    template <typename Ret, typename... Args>
    std::pair<Ret, std::vector<IDT>> maxWeightPath(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
                                                   Args... args) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Obtain the minimum weight and its corresponding path between two nodes.
     *
     * @details This finds the minimum weight path with the Djkstra algorithm.
     *          So all weights should be **non-negative**.\n
     *          Example see maxWeightPath().
     * @note EdgeT should be arithmetic, i.e. edge data represents the weight.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @return (std::pair<EdgeT, std::vector<IDT>>) The minimum weight and its corresponding path.
     */
    std::pair<EdgeT_, std::vector<IDT>> minWeightPath(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT> && std::is_arithmetic_v<EdgeT>;

    /**
     * @brief Obtain the minimum weight and its corresponding path between two nodes.
     *
     * @details This finds the minimum weight path with the Djkstra algorithm.
     *          So all weights should be **non-negative**.\n
     *          Example see maxWeightPath().
     * @tparam Ret The weight type, i.e. Lambda function return type.
     * @tparam Fn The Lambda expression type.
     * @tparam Args The Lambda expression argument types.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The Lambda expression.
     * @param args The Lambda expression arguments.
     * @return (std::pair<Ret, std::vector<IDT>>) The maximum weight and its corresponding path.
     */
    template <typename Ret, NonFunc Fn, typename... Args>
    std::pair<Ret, std::vector<IDT>> minWeightPath(IDT from_id, IDT to_id, Fn func, Args... args) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Obtain the minimum weight and its corresponding path between two nodes.
     *
     * @details This finds the minimum weight path with the Djkstra algorithm.
     *          So all weights should be **non-negative**.
     * @tparam Ret The weight type, i.e. `std::function` return type.
     * @tparam Args `std::function` argument types.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The `std::function`.
     * @param args The `std::function` arguments.
     * @return (std::pair<Ret, std::vector<IDT>>) The minimum weight and its corresponding path.
     */
    template <typename Ret, typename... Args>
    std::pair<Ret, std::vector<IDT>> minWeightPath(IDT from_id, IDT to_id, std::function<Ret(EdgeT_, Args...)> func,
                                                   Args... args) const
        requires EdgeVT<EdgeT>;

  private:
    /**
     * @brief Obtain the minimum/maximum weight and its corresponding path between two nodes.
     *
     * @tparam Ret The weight type, i.e. Lambda function return type.
     * @tparam Args `std::function` argument types.
     * @param min Whether to find the minimum or maximum weight.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @param func The `std::function`.
     * @param args The `std::function` arguments.
     * @return (std::pair<Ret, std::vector<IDT>>) The minimum/maximum weight and its corresponding path.
     */
    template <typename Ret, typename... Args>
    std::pair<Ret, std::vector<IDT>> minOrMaxWeightPath(bool min, IDT from_id, IDT to_id,
                                                        std::function<Ret(EdgeT_, Args...)> func, Args... args) const
        requires EdgeVT<EdgeT>;

  public:
    /**
     * @brief Insert subgraph into the main graph.
     *
     * @details This is an alias for merge();
     * @param dg The subgraph.
     */
    void insertSubGraph(const DGraphBase<EdgeT, IDT>& dg);

    /**
     * @brief Insert a subgraph at a node.
     *
     * @details This is equivalent to remove a node and insert a subgraph.
     * @param dg The subgraph.
     * @param id The node id.
     */
    void insertSubGraph(const DGraphBase<EdgeT, IDT>& dg, IDT id);

    /**
     * @brief Merge a graph into the main graph.
     *
     * @param dg The graph to be merged.
     */
    void merge(const DGraphBase<EdgeT, IDT>& dg);

    /**
     * @brief Clear the whole graph.
     *
     * @details Remove all the nodes and the connected edges.
     */
    void clear();

    /**
     * @brief Clear all edges of the graph.
     *
     */
    void clearEdges();

    /**
     * @brief Check if two graphs are identical.
     *
     * @details All nodes and edges should be the same.
     * @param dg Another graph.
     * @retval true The two graphs are identical.
     * @retval false The two graphs are different.
     */
    bool operator==(const DGraphBase<EdgeT, IDT>& dg) const;

    /**
     * @brief Check if two graphs are different.
     *
     * @param dg Another graph.
     * @retval true The two graphs are different.
     * @retval false The two graphs are identical.
     */
    bool operator!=(const DGraphBase<EdgeT, IDT>& dg) const;

    /**
     * @brief Merge two graphs.
     *
     * @details This is an alias for merge().
     * @param dg Another graph.
     * @return (DGraphBase&) The merged graph (as a reference).
     */
    DGraphBase& operator+=(const DGraphBase<EdgeT, IDT>& dg);

    /**
     * @brief Get the edge data between two nodes.
     *
     * @note EdgeT should not be void.
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @return (EdgeT) The edge data.
     */
    EdgeT operator()(IDT from_id, IDT to_id) const
        requires EdgeVT<EdgeT>;

    /**
     * @brief Get the edge data reference between two nodes.
     *
     * @param from_id The from node id.
     * @param to_id The to node id.
     * @return (EdgeT_R) The edge data reference.
     */
    EdgeT_R operator()(IDT from_id, IDT to_id)
        requires EdgeVT<EdgeT>;

  private:
    /**
     * @brief Remove edges connected to a node.
     *
     * @param id The node id.
     * @return (size_t) The number of removed edges.
     */
    size_t removeNodeConnectedEdges(IDT id);

  protected:
    /**
     * @brief Return the const reference to the graph map.
     *
     * @return (const std::map<IDT, _CN>&) Const reference to the graph map.
     */
    const std::map<IDT, _CN>& graph() const;

    /**
     * @brief Return the reference to the graph map.
     *
     * @return (std::map<IDT, _CN>&) reference to the graph map.
     */
    std::map<IDT, _CN>& graph();

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
inline size_t DGraphBase<EdgeT, IDT>::size() const noexcept {
    return numNodes();
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
inline bool DGraphBase<EdgeT, IDT>::strictCheck() const {
    auto nodes = nodesID();
    std::vector<IDT> cn_nodes;
    for (auto&& cn : g)
        for (auto&& n : cn.second) cn_nodes.push_back(n.first);
    std::sort(cn_nodes.begin(), cn_nodes.end());
    cn_nodes.erase(std::unique(cn_nodes.begin(), cn_nodes.end()), cn_nodes.end());
    return std::includes(nodes.begin(), nodes.end(), cn_nodes.begin(), cn_nodes.end());
}

template <typename EdgeT, IDVT IDT>
std::vector<IDT> DGraphBase<EdgeT, IDT>::nodesID() const {
    std::vector<IDT> keys;
    std::transform(g.begin(), g.end(), std::back_inserter(keys), [](const auto& pair) { return pair.first; });
    return keys;
}

template <typename EdgeT, IDVT IDT>
inline std::vector<std::pair<IDT, IDT>> DGraphBase<EdgeT, IDT>::edges() const {
    std::vector<std::pair<IDT, IDT>> e;
    for (auto&& [from_id, cn] : g) {
        for (auto&& edge : cn) e.push_back({ from_id, edge.second });
    }
    return e;
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertNode(IDT id) {
    g[id] = _CN();
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertNode(IDT id, const _CN& cn, bool nodes_exist) {
    g[id] = cn;
    if (!nodes_exist) {
        // update graph nodes
        for (auto&& n : cn)
            if (auto&& key = n.first; !g.contains(key)) g[key] = _CN();
    }
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertNode(IDT id, const std::map<IDT, EdgeT>& map, bool nodes_exist)
    requires EdgeVT<EdgeT> {
    insertNode(id, _protected::mapAsEdgeMap(map), nodes_exist);
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
inline void DGraphBase<EdgeT, IDT>::insertEdgeToExists(IDT from_id, IDT to_id)
    requires std::is_void_v<EdgeT> {
    g[from_id][to_id] = Edge<void>();
}

template <typename EdgeT, IDVT IDT>
inline void DGraphBase<EdgeT, IDT>::insertEdgeToExists(IDT from_id, IDT to_id, EdgeT_CR data)
    requires EdgeVT<EdgeT> {
    g[from_id][to_id] = data;
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertEdge(IDT from_id, IDT to_id, bool to_exists)
    requires std::is_void_v<EdgeT> {
    insertEdgeToExists(from_id, to_id);
    if (!to_exists && !g.contains(to_id)) g[to_id] = _CN();
}

template <typename EdgeT, IDVT IDT>
void DGraphBase<EdgeT, IDT>::insertEdge(IDT from_id, IDT to_id, typename DGraphBase<EdgeT, IDT>::EdgeT_CR data,
                                        bool to_exists)
    requires EdgeVT<EdgeT> {
    insertEdgeToExists(from_id, to_id, data);
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
inline void DGraphBase<EdgeT, IDT>::insertSubGraph(const DGraphBase<EdgeT, IDT>& dg, IDT id) {
    removeNode(id);
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

template <typename EdgeT, IDVT IDT>
inline const std::map<IDT, typename DGraphBase<EdgeT, IDT>::_CN>& DGraphBase<EdgeT, IDT>::graph() const {
    return g;
}

template <typename EdgeT, IDVT IDT>
inline std::map<IDT, typename DGraphBase<EdgeT, IDT>::_CN>& DGraphBase<EdgeT, IDT>::graph() {
    return g;
}

} // namespace dg

#endif
