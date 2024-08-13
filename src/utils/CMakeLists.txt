set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/defs.h
)

add_library(utils INTERFACE ${HEADERS})
target_include_directories(utils INTERFACE .)
