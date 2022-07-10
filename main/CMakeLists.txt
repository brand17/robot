set(COMPONENT_SRCS "example.cpp" "main.cpp" "main.cpp example.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

# Create and import the library targets
add_subdirectory(C:/Users/FA.PROJECTOR-MSK/YandexDisk/Software/cpp/eigen C:/Users/FA.PROJECTOR-MSK/YandexDisk/Software/cpp/eigen/bin/esp-idf)

# Publicly link `foo` to `main` component
target_link_libraries(${COMPONENT_LIB} PUBLIC eigen)