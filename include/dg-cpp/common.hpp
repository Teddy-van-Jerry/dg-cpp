/**
 * @file common.hpp
 * @author Wuqiong Zhao (me@wqzhao.org)
 * @brief Common Utilities for DG-CPP
 * @version 0.1.0
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023 Wuqiong Zhao (Teddy van Jerry)
 *
 */

#ifndef _DG_COMMON_HPP_
#define _DG_COMMON_HPP_

#include <concepts>

namespace dg {
template <typename T>
concept IDVT = !std::is_void_v<T>
#ifdef __cpp_lib_concepts
               && std::equality_comparable<T>
#endif
    ;
template <typename T>
concept EdgeVT = !std::is_void_v<T>;
template <typename T>
concept NonFunc = !std::is_function_v<T>;
} // namespace dg

#endif
