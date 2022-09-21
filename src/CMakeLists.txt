find_package(SampleRate CONFIG REQUIRED)
add_subdirectory(btrack/fft)

add_library(btrack btrack/BTrack.cpp btrack/OnsetDetectionFunction.cpp)
target_link_libraries(btrack PRIVATE btrack_fft)

target_include_directories(btrack PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)

# install(TARGETS btrack
#       EXPORT btrackTargets
#       LIBRARY DESTINATION lib
#       ARCHIVE DESTINATION lib
#       RUNTIME DESTINATION bin
#       INCLUDES DESTINATION include
# )