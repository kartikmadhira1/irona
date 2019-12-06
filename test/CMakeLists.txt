set(GTEST_SHUFFLE 1)
cmake_minimum_required(VERSION 2.8)
add_executable(
    cpp-test
    test.cpp
    ControllerTest.cpp
    DetectionTest.cpp
    #NavigationTest.cpp
    UserInterfaceTest.cpp
    WarehouseManagerTest.cpp
)
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include ../vendor/googletest/googlemock/include 
                                           ${CMAKE_SOURCE_DIR}/include
                                           ${OpenCV_INCLUDE_DIRS}
                                           ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cpp-test PUBLIC gtest gmock)
target_link_libraries(cpp-test PUBLIC ${OpenCV_LIBS})