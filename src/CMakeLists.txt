find_package(SampleRate CONFIG REQUIRED)

add_library(btrack beat_tracker.cpp onset_detection_function.cpp )


if(ENABLE_FFTW)
  message("Building with FFTW")
	set(BTRACK_COMPILE_DEFINITIONS USE_FFTW)
  find_package(FFTW3 CONFIG REQUIRED)
  set(FFT_LIBRARY FFTW3::fftw3)

  set(BTRACK_INCLUDE_DIRECTORIES "")
  set(FFT_OPERATOR_SOURCES transformers/fft/fft_operator_fftw.cpp)
  # add_library(btrack_fft btrack_fft_fftw.cpp)
else() 

  message("Building with KISS FFT")
  set(KISSFFT_DATATYPE double)
  set(KISSFFT_STATIC ON)
  
  list(APPEND BTRACK_COMPILE_DEFINITIONS USE_KISS_FFT kiss_fft_scalar=${KISSFFT_DATATYPE})
  message("${BTRACK_COMPILE_DEFINITIONS}")
  
  # find_package(unofficial-kissfft CONFIG REQUIRED)
  # set(FFT_LIBRARY unofficial::kissfft::kissfft)
  set(BTRACK_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/../libs/kiss_fft130)
  target_include_directories(btrack PRIVATE ${BTRACK_INCLUDE_DIRECTORIES})
  set(FFT_OPERATOR_SOURCES transformers/fft/fft_operator_kissfft.cpp ${CMAKE_CURRENT_LIST_DIR}/../libs/kiss_fft130/kiss_fft.c)

  # add_library(btrack_fft btrack_fft_kissfft.cpp)
endif()


message("FFT_OPERATOR_SOURCES = ${FFT_OPERATOR_SOURCES}")
message("BTRACK_COMPILE_DEFINITIONS = ${BTRACK_COMPILE_DEFINITIONS}")

target_sources(btrack PRIVATE ${FFT_OPERATOR_SOURCES} transformers.cpp)
target_link_libraries(btrack PUBLIC 
  ${FFT_LIBRARY} 
  SampleRate::samplerate   
  fmt::fmt
  spdlog::spdlog 
  range-v3 range-v3-meta range-v3::meta range-v3-concepts
)
target_compile_definitions(btrack PUBLIC ${BTRACK_COMPILE_DEFINITIONS})
target_include_directories(btrack PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)


add_subdirectory(original)
