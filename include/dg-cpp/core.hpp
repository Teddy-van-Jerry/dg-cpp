#ifndef _DG_CORE_HPP_
#define _DG_CORE_HPP_

#include "base.hpp"

namespace dg {
/**
 * @brief Directed graph class.
 *
 * @tparam NodeT Node type.
 * @tparam EdgeT Edge type.
 * @tparam IDT ID type (should not be none).
 */
template <typename NodeT, typename EdgeT = void, typename IDT = std::string>
class DGraph : public DGraphBase<EdgeT, IDT> {
  private:
    std::map<IDT, NodeT> node_data; /**< node data */
};

template <typename EdgeT, typename IDT>
class DGraph<void, EdgeT, IDT> : public DGraphBase<EdgeT, IDT> {};

} // namespace dg

#endif
