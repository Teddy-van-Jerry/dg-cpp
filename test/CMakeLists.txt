project(dg-cpp-test)

add_executable(dg_cpp_test test.cpp)

include(CTest)
enable_testing()

add_test(NAME test COMMAND dg_cpp_test)
