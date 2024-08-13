add_executable(measurements main.cpp)


set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/measure_body.h
    ${CMAKE_CURRENT_SOURCE_DIR}/measurement.h
    ${CMAKE_CURRENT_SOURCE_DIR}/measurement_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/measurement_utils.h
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/measure_body.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/measurement.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/measurement_utils.cpp
)

target_sources(measurements PUBLIC ${SOURCES} ${HEADERS})

target_link_libraries(measurements PUBLIC pmp io utils CGAL::CGAL argparse)

set_target_properties(measurements PROPERTIES COMPILE_WARNINGS_AS_ERRORS ON)
