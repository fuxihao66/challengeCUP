cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)
project(KinectGrabberQT LANGUAGES CXX)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )


find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
find_package( PCL 1.8 REQUIRED )
find_package( KinectSDK2 REQUIRED )

set(project_ui 
    mainwindow.ui)

set(project_headers 
    mainwindow.h
    kinect2_grabber.h
    Client.h)

set(project_sources 
    main.cpp
    mainwindow.cpp
    Client.cpp)

qt5_wrap_ui(project_headers_wrapped ${project_ui})
qt5_wrap_cpp(project_sources_moc ${project_headers})

add_executable(${PROJECT_NAME} ${project_headers} ${project_sources} 
    ${project_sources_moc} ${project_headers_wrapped})


if( PCL_FOUND AND KinectSDK2_FOUND)
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( ${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} )
  target_link_libraries( ${PROJECT_NAME} PUBLIC ${KinectSDK2_LIBRARIES} )
endif()

target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets)




