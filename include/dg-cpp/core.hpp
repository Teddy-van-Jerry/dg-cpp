#ifndef _DG_CORE_HPP_
#define _DG_CORE_HPP_

#include "dg-cpp/base.hpp"

namespace dg {
/**
 * @brief Directed graph class.
 *
 * @tparam NodeT Node type.
 * @tparam EdgeT Edge type.
 * @tparam IDT ID type (should not be none).
 */
template <typename NodeT = void, typename EdgeT = void, typename IDT = std::string>
class DGraph : public DGraphBase<EdgeT, IDT> {
  private:
    std::map<IDT, NodeT> node_data; /**< node data */
};
} // namespace dg

#endif
