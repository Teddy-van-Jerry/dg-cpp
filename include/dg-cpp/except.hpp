#ifndef _DG_EXCEPT_HPP_
#define _DG_EXCEPT_HPP_

#include <exception>
#include <stdexcept>

namespace dg {
class out_of_range : public std::out_of_range {
    using std::out_of_range::out_of_range;
};
class node_not_exists : public out_of_range {
    using out_of_range::out_of_range;
};
} // namespace dg

#endif
