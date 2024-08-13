set(SOURCES
    BIM_file.cpp
    Character.cpp
    CharacterRenderer.cpp
    FBX_file.cpp
    IO_bim.cpp
    IO_character.cpp
    Joint.cpp
    Mean_value_coordinates.cpp
    PCA.cpp
    Skeleton.cpp
    SkinnedMesh.cpp
)


set(HEADERS
    BIM_file.h
    Character.h
    CharacterRenderer.h
    FBX_file.h
    IO_character.h
    Joint.h
    Mean_value_coordinates.h
    PCA.h
    Skeleton.h
    SkinnedMesh.h
    Skinning.h
)

add_library(character STATIC ${SOURCES} ${HEADERS})
target_link_libraries(character ${FBX_LIBRARIES} pmp glew CGAL::CGAL)
