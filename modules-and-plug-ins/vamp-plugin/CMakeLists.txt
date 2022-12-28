find_package(PkgConfig REQUIRED)
pkg_check_modules(VAMPSDK REQUIRED vamp-sdk)

add_library(btrackvamp SHARED BTrackVamp.cpp plugins.cpp)

target_link_libraries(btrackvamp PUBLIC btrack  ${VAMPSDK_LINK_LIBRARIES})
target_include_directories(btrackvamp PUBLIC ${VAMPSDK_INCLUDE_DIRS})
target_compile_options(btrackvamp PUBLIC ${VAMPSDK_CFLAGS_OTHER})

# set(FLAG "-exported_symbols_list ${EXP_PATH}")
# set_target_properties(${TARGET} PROPERTIES LINK_FLAGS ${FLAG})
