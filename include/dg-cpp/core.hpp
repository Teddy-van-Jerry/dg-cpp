/**
 * @file core.hpp
 * @author Wuqiong Zhao (me@wqzhao.org)
 * @brief Directed Graph Class
 * @version 0.1.0
 * @date 2023-04-16
 *
 * @copyright Copyright (c) 2023 Wuqiong Zhao (Teddy van Jerry)
 *
 */

#ifndef _DG_CORE_HPP_
#define _DG_CORE_HPP_

#include "base.hpp"

namespace dg {
/**
 * @brief Directed graph class.
 *
 * @tparam NodeT Node type.
 * @tparam EdgeT Edge type.
 * @tparam IDT ID type (should not be void and should be comparable).
 */
template <typename NodeT, typename EdgeT = void, typename IDT = std::string>
class DGraph : public DGraphBase<EdgeT, IDT> {
  protected:
    using _CN      = typename DGraphBase<EdgeT, IDT>::_CN;
    using EdgeT_CR = typename DGraphBase<EdgeT, IDT>::EdgeT_CR;

  private:
    using DGraphBase<EdgeT, IDT>::insertNode;

  public:
    NodeT node(IDT id) const;

    NodeT& node(IDT id);

    bool strictCheck() const;

    void insertNode(IDT id, const NodeT& data);

    void insertNode(IDT id, const NodeT& data, const _CN& cn);

    void removeNode(IDT id, bool keep_edge = false);

    void insertEdge(IDT from_id, IDT to_id, [[maybe_unused]] bool to_exists = true)
        requires std::is_void_v<EdgeT>;

    void insertEdge(IDT from_id, IDT to_id, EdgeT_CR data, [[maybe_unused]] bool to_exists = true)
        requires EdgeVT<EdgeT>;

    NodeT operator[](IDT id) const;

    NodeT& operator[](IDT id);

    bool operator==(const DGraph& dg) const;

    bool operator!=(const DGraph& dg) const;

  private:
    std::map<IDT, NodeT> node_data; /**< node data */
};

template <typename EdgeT, typename IDT>
class DGraph<void, EdgeT, IDT> : public DGraphBase<EdgeT, IDT> {};

template <typename NodeT, typename EdgeT, typename IDT>
inline NodeT DGraph<NodeT, EdgeT, IDT>::node(IDT id) const {
    return node_data[id];
}

template <typename NodeT, typename EdgeT, typename IDT>
inline NodeT& DGraph<NodeT, EdgeT, IDT>::node(IDT id) {
    return node_data[id];
}

template <typename NodeT, typename EdgeT, typename IDT>
inline bool DGraph<NodeT, EdgeT, IDT>::strictCheck() const {
    return this->size() == node_data.size() && this->DGraphBase<EdgeT, IDT>::strictCheck() &&
           std::equal(this->graph().begin(), this->graph().end(), node_data.begin(),
                      [](auto a, auto b) { return a.first == b.first; });
}

template <typename NodeT, typename EdgeT, typename IDT>
inline void DGraph<NodeT, EdgeT, IDT>::insertNode(IDT id, const NodeT& data) {
    this->DGraphBase<EdgeT, IDT>::insertNode(id);
    node_data[id] = data;
}

template <typename NodeT, typename EdgeT, typename IDT>
inline void DGraph<NodeT, EdgeT, IDT>::insertNode(IDT id, const NodeT& data, const _CN& cn) {
    this->DGraphBase<EdgeT, IDT>::insertNode(id, cn);
    node_data[id] = data;
}

template <typename NodeT, typename EdgeT, typename IDT>
inline void DGraph<NodeT, EdgeT, IDT>::removeNode(IDT id, bool keep_edge) {
    this->DGraphBase<EdgeT, IDT>::removeNode(id);
    node_data.erase(id);
}

template <typename NodeT, typename EdgeT, typename IDT>
inline void DGraph<NodeT, EdgeT, IDT>::insertEdge(IDT from_id, IDT to_id, bool to_exists)
    requires std::is_void_v<EdgeT> {
    this->insertEdgeToExists(from_id, to_id);
}

template <typename NodeT, typename EdgeT, typename IDT>
inline void DGraph<NodeT, EdgeT, IDT>::insertEdge(IDT from_id, IDT to_id, EdgeT_CR data, bool to_exists)
    requires EdgeVT<EdgeT> {
    this->insertEdgeToExists(from_id, to_id, data);
}

template <typename NodeT, typename EdgeT, typename IDT>
inline NodeT DGraph<NodeT, EdgeT, IDT>::operator[](IDT id) const {
    return node(id);
}

template <typename NodeT, typename EdgeT, typename IDT>
inline NodeT& DGraph<NodeT, EdgeT, IDT>::operator[](IDT id) {
    return node(id);
}

template <typename NodeT, typename EdgeT, typename IDT>
inline bool DGraph<NodeT, EdgeT, IDT>::operator==(const DGraph& dg) const {
    return this->DGraphBase<EdgeT, IDT>::operator==(dg) && dg.node_data == node_data;
}

template <typename NodeT, typename EdgeT, typename IDT>
inline bool DGraph<NodeT, EdgeT, IDT>::operator!=(const DGraph& dg) const {
    return this->DGraphBase<EdgeT, IDT>::operator!=(dg) || dg.node_data != node_data;
}

} // namespace dg

#endif
