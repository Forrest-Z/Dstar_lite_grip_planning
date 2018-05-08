set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    NewDstarLiteTest.cpp
    ../app/NewDstarLite.cpp
    ../app/BuildingMap.cpp
    ../app/Map.cpp
)
include_directories(
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/include
    ${gmock_SOURCE_DIR}/include
)


target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ../vendor/googletest/googlemock/include
                                           ${CMAKE_SOURCE_DIR}/include)
#target_link_libraries(cpp-test PUBLIC gtest gmock ${OpenCV_LIBS})
target_link_libraries(cpp-test PUBLIC  gmock_main ${OpenCV_LIBS})
