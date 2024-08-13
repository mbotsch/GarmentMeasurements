set(SOURCES Viewer.cpp
            main.cpp
)

set(HEADERS Viewer.h)

add_executable(generate_shapes ${SOURCES} ${HEADERS})
target_link_libraries(generate_shapes PUBLIC templatefitting pmp_vis nlohmann_json::nlohmann_json)

