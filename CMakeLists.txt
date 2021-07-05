# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5)
project(rosaria)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(prefix "/usr/local")
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
set(Aria_INCLUDE_DIR /usr/local/Aria/include)
set(Aria_LIBRARY_RELEASE /usr/local/Aria/lib)
set(Aria_LIB /usr/local/Aria/src)

#message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
#add_definitions(-DADEPT_PKG)
#include_directories(${prefix}/Aria/include)
#link_directories(${prefix}/Aria/lib)




find_package(Aria QUIET)
if(Aria_FOUND)
  message("CMake package for Aria was found, using that.")
  # Aria package for CMake was found
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  # No Aria package for CMake was found, search ourselves
	# The installation package provided by Adept doesn't follow normal Linux
	# installation locations, but the repackaged Debian package and AriaCoda do.
  # If user set ARIA environment variable, look there, otherwise look in
  # /usr/local/.
  if(DEFINED ENV{ARIA})
    message("ARIA environment variable defined, checking there...")
    set(prefix $ENV{ARIA})
  else()
    set(prefix "/usr/local")
  endif()
  message("Looking for Aria in ${prefix}/Aria and ${prefix}...")

	if(EXISTS "${prefix}/Aria/include/Aria.h")
    message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
	  add_definitions(-DADEPT_PKG)
	  include_directories(${prefix}/Aria/include)
	  link_directories(${prefix}/Aria/lib)
  else()
    if(EXISTS "${prefix}/include/Aria.h")
      add_definitions(-DADEPT_PKG)
      include_directories(${prefix}/include)
      link_directories(${prefix}/lib)
    else()
      if(EXISTS "${prefix}/include/Aria/Aria.h")
        message("Found ${prefix}/include/Aria/Aria.h, assuming AriaCoda.")
        add_definitions(-DARIACODA)
        include_directories(${prefix}/include)
        link_directories(${prefix}/lib)
      else()
        message("Aria.h not found in ${prefix}. Continuing with default header and library paths.")
      endif()
    endif()
	endif()
endif()


find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

file(GLOB_RECURSE _HDRS "ArTimeToROSTime.h")


find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
            "msg/BumperState.msg"
           DEPENDENCIES std_msgs
        )




include_directories(include ${Boost_INCLUDE_DIR} ${Aria_INCLUDE_DIR} ${Aria_LIB})
add_executable(RosAria RosAria.cpp LaserPublisher.cpp)
link_directories(${Aria_LIBRARY_RELEASE})


ament_target_dependencies(
        RosAria
        "Boost"
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "std_srvs"
        "geometry_msgs"
        "tf2"
        "tf2_ros"
        "nav_msgs"
                   
)

install(TARGETS RosAria
 DESTINATION lib/${PROJECT_NAME})
 
rosidl_target_interfaces(RosAria
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# cause problems
#target_link_libraries(RosAria Aria_lib)


target_link_libraries(RosAria ${Boost_LIBRARIES} Aria pthread dl rt)
#set_target_properties(RosAria PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(RosAria P#ROPERTIES LINK_FLAGS "-Wl")


ament_package()
