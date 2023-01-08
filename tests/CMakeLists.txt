include(CTest)
enable_testing()

find_package(Catch2 CONFIG REQUIRED)
include(Catch) 


add_executable(test_fft_operator test_fft_operator.cpp)

target_link_libraries(test_fft_operator PUBLIC btrack Catch2::Catch2 Catch2::Catch2WithMain)
get_target_property(btrack_compile_definition btrack COMPILE_DEFINITIONS)

# Not sure why we need that...
target_compile_definitions(test_fft_operator PUBLIC ${btrack_compile_definition})
get_target_property(test_fft_operator_compile_definitions test_fft_operator COMPILE_DEFINITIONS)

message("btrack_compile_definition ${btrack_compile_definition}")
message("test_fft_operator_compile_definitions ${test_fft_operator_compile_definitions}")

catch_discover_tests(test_fft_operator)



add_executable(test_btrack test_btrack.cpp)
target_link_libraries(test_btrack PUBLIC btrack Catch2::Catch2 Catch2::Catch2WithMain)
target_compile_definitions(test_btrack PUBLIC ${btrack_compile_definition})
target_include_directories(test_btrack PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../src)
catch_discover_tests(test_btrack)


add_executable(test_btrack_original test_btrack_original.cpp)
target_link_libraries(test_btrack_original PUBLIC btrack_orig Catch2::Catch2 Catch2::Catch2WithMain)
target_compile_definitions(test_btrack_original PUBLIC ${btrack_compile_definition})
catch_discover_tests(test_btrack_original)
