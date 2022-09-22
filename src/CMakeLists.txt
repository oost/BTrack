find_package(SampleRate CONFIG REQUIRED)

if(ENABLE_FFTW)
  message("Building with FFTW")
	# add_compile_definitions(USE_FFTW)
	set(BTRACK_COMPILE_DEFINITIONS USE_FFTW)
  find_package(FFTW3 CONFIG REQUIRED)
  set(FFT_LIBRARY FFTW3::fftw3)

  # add_library(btrack_fft btrack_fft_fftw.cpp)
else() 
  message("Building with KISS FFT")
	# add_compile_definitions(USE_KISS_FFT)
	set(BTRACK_COMPILE_DEFINITIONS USE_KISS_FFT)
  set(KISSFFT_STATIC ON)
  find_package(unofficial-kissfft CONFIG REQUIRED)
  set(FFT_LIBRARY unofficial::kissfft::kissfft)
  # add_library(btrack_fft btrack_fft_kissfft.cpp)
endif()


set(PKGINCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/btrack")
message(STATUS "PKGINCLUDEDIR is ${PKGINCLUDEDIR}")

add_library(btrack BTrack.cpp OnsetDetectionFunction.cpp)
target_link_libraries(btrack PUBLIC ${FFT_LIBRARY} SampleRate::samplerate)
target_compile_definitions(btrack PUBLIC ${BTRACK_COMPILE_DEFINITIONS})

# target_link_libraries(btrack PRIVATE btrack_fft)

target_include_directories(btrack PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:${PKGINCLUDEDIR}>)

install(TARGETS btrack
      EXPORT btrackTargets
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
)


install(TARGETS btrack EXPORT btrack
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

install(FILES btrack_config.h
              BTrack.hh
              CircularBuffer.h
              OnsetDetectionFunction.h
        DESTINATION "${PKGINCLUDEDIR}")