# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Stefano Dafarra <stefano.dafarra@iit.it>

project(ZMPPreviewWalkingController)

set(ZMP_HEADERS)

set(ZMP_SOURCES src/main.cpp)

# linux needs to explicitly ask for pthreads
if (UNIX AND NOT APPLE)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
endif()

add_executable(${PROJECT_NAME} ${ZMP_HEADERS} ${ZMP_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${YARP_INCLUDE_DIRS}
                                                          ${EIGEN3_INCLUDE_DIR}
                                                          ${ctrlLib_INCLUDE_DIRS})

list(APPEND LINKLIST ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} ctrlLib WalkingControllersComponents)

if (UNIX AND NOT APPLE)
    list(APPEND LINKLIST Threads::Threads)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${LINKLIST})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

include(AddUninstallTarget)

