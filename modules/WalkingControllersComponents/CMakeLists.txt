# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Stefano Dafarra <stefano.dafarra@iit.it>

project(WalkingControllersComponents CXX)

set(libraryname WalkingControllersComponents)

find_package(UnicyclePlanner 0.1.100 REQUIRED)

set(COMPONENTS_HDR include/RobotComponent.h
                   include/JointsSources.h
                   include/JointsSinks.h
                   include/Utils.h
                   include/Utils.tpp
                   include/PIDHandlerComponent.h
                   include/NonLinearIKComponent.h
                   include/YarpSigVectorPortSource.h
                   include/YarpSigVectorPortSink.h
                   include/UnicycleGeneratorComponent.h
                   include/UnicycleReferenceSource.h
                   include/UnicycleJoypadSource.h
                   include/UnicyclePortSource.h)

set(COMPONENTS_SRC src/RobotComponent.cpp
                   src/JointsSources.cpp
                   src/JointsSinks.cpp
                   src/Utils.cpp
                   src/PIDHandlerComponent.cpp
                   src/NonLinearIKComponent.cpp
                   src/YarpSigVectorPortSource.cpp
                   src/YarpSigVectorPortSink.cpp
                   src/UnicycleGeneratorComponent.cpp
                   src/UnicycleReferenceSource.cpp
                   src/UnicycleJoypadSource.cpp
                   src/UnicyclePortSource.cpp)

add_library(${libraryname} ${COMPONENTS_HDR} ${COMPONENTS_SRC})
target_include_directories(${libraryname} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${libraryname} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR} ${IPOPT_INCLUDE_DIRS})
target_link_libraries(${libraryname} ${iDynTree_LIBRARIES} ${YARP_LIBRARIES} UnicyclePlanner)

add_subdirectory(app)

