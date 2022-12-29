
add_library(btrack_orig STATIC BTrack.cpp OnsetDetectionFunction.cpp)
target_link_libraries(btrack_orig PUBLIC ${FFT_LIBRARY} SampleRate::samplerate)
target_compile_definitions(btrack_orig PUBLIC ${BTRACK_COMPILE_DEFINITIONS})
target_include_directories(btrack_orig PUBLIC ${CMAKE_CURRENT_LIST_DIR})

	# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/original>
	# $<INSTALL_INTERFACE:${PKGINCLUDEDIR}>)

