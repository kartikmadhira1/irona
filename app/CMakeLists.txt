add_executable(irona-app main.cpp UserInterface.cpp WarehouseManager.cpp)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(irona-app 
${OpenCV_LIBS}
)
