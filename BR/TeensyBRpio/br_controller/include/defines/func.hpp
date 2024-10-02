#ifndef _DEFINE_FUNC_HPP_
#define _DEFINE_FUNC_HPP_

template <class... Ts>
struct overload : Ts... {
    using Ts::operator()...;
};

#endif