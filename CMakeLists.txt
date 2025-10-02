# Collect all .c files in src/
file(GLOB ZEPHYR_ABS_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

zephyr_library_named(zephyr_abs)
zephyr_library_sources(${ZEPHYR_ABS_SRC})
zephyr_include_directories(include)
