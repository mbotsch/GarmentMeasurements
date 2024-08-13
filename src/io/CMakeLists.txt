set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/io_measurement.h
    ${CMAKE_CURRENT_SOURCE_DIR}/io_selection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/io_toolbox.h
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/io_measurement.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io_selection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/io_toolbox.cpp
)

add_library(io STATIC ${SOURCES} ${HEADERS})

target_link_libraries(io PUBLIC pmp utils nlohmann_json::nlohmann_json)
