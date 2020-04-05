cmake_minimum_required(VERSION 2.8.3)

project(vertigo_ros)
include(ExternalProject)

add_compile_options(-std=c++14)

set(DEFAULT_BUILD "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   message(STATUS "Setting build type to '${DEFAULT_BUILD}' as none was specified.")
   set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD} CASHE STRING "Choose the type of build." FORCE)
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug"
		                                        "Release")
endif()

# Find catkin
find_package(catkin REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(GTSAMCMakeTools REQUIRED)
set(GTSAM_BUILD_WITH_MARCH_NATIV OFF CACHE BOOL "Enable/Disable building all instructions supposrted by native architecture (binary may not be portable!)" FORCE)
include(GtsamBuildTypes)
find_package(GTSAM REQUIRED)

# Add include directories
include_directories(
	include
	${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

# Set src directory
set(vertigo_src "${CMAKE_CURRENT_SOURCE_DIR}/vertigo")

# Add vertigo folder
ExternalProject_Add(vertigo #Preserve the name of the original library
	SOURCE_DIR "${vertigo_src}"
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
	BUILD_COMMAND CXXFLAGS=-i${CATKIN_DEVEL_PREFIX}/include make
	INSTALL_COMMAND make install
	)

# Catkin dependencies
catkin_package(
	INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/vertigo/include # Use the original include folder
	LIBRARIES vertigo
	)

# install
install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vertigo/include # Use the original include folder
	DESTINATION ${CMAKE_INSTALL_PREFIX}
	)

install(
	FILES ${CATKIN_DEVEL_PREFIX}/lib/libVertigo${CMAKE_SHARED_LIBRARY_SUFFIX} # Use the name of the library binary
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	)
add_subdirectory(vertigo)
