project(dg-cpp-test)

add_executable(dg-cpp-test test.cpp)

include(CTest)
enable_testing()

add_test(NAME test COMMAND dg-cpp-test)
