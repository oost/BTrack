
# add_definitions(-DBOOST_TEST_NO_MAIN)
# add_definitions(-DBOOST_TEST_ALTERNATIVE_INIT_API)

add_executable(test_btrack Test_BTrack.cpp)

find_package(Boost COMPONENTS unit_test_framework REQUIRED)
# include_directories (${Boost_INCLUDE_DIRS})
target_link_libraries(test_btrack PRIVATE
                      Boost::boost
                      Boost::unit_test_framework
                      btrack
                      )


