add_executable(osseus-sandbox
        src/main.cpp
)

target_link_libraries(
        osseus-sandbox
        PRIVATE
            Osseus
)