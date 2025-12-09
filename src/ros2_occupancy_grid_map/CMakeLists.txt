cmake_minimum_required(VERSION 3.14)
project(occupancy_grid_map)

option(USE_CUPOCH "Enable GPU acceleration via cupoch" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


find_package(ament_cmake_auto REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

if(USE_CUPOCH)
  if(NOT DEFINED CUPOCH_ROOT AND DEFINED ENV{CUPOCH_ROOT})
    set(CUPOCH_ROOT $ENV{CUPOCH_ROOT})
  endif()
  if(NOT CUPOCH_ROOT)
    set(CUPOCH_ROOT "${PROJECT_SOURCE_DIR}/../cupoch")
  endif()

  set(CUPOCH_INCLUDE_HINTS
      ${CUPOCH_ROOT}/src
      ${CUPOCH_ROOT}/install/include
      /usr/local/include
      /usr/include)
  find_path(CUPOCH_INCLUDE_DIR cupoch/cupoch.h PATHS ${CUPOCH_INCLUDE_HINTS})

  set(CUPOCH_LIB_HINTS
      ${CUPOCH_ROOT}/build/lib
      ${CUPOCH_ROOT}/install/lib
      /usr/local/lib
      /usr/lib)

  set(CUPOCH_FOUND TRUE)
  set(CUPOCH_LIB_NAMES
      cupoch_geometry
      cupoch_utility
      cupoch_knn
      cupoch_camera
      cupoch_registration
      cupoch_integration
      cupoch_planning
      cupoch_collision
      cupoch_io
      cupoch_imageproc
      cupoch_kinematics
      cupoch_kinfu
      cupoch_odometry
      cupoch_visualization)

  foreach(lib_name ${CUPOCH_LIB_NAMES})
    find_library(CUPOCH_${lib_name}_LIB NAMES ${lib_name} PATHS ${CUPOCH_LIB_HINTS})
    if(CUPOCH_${lib_name}_LIB)
      list(APPEND CUPOCH_LIBRARIES ${CUPOCH_${lib_name}_LIB})
    else()
      set(CUPOCH_FOUND FALSE)
    endif()
  endforeach()

  if(CUPOCH_FOUND AND CUPOCH_INCLUDE_DIR)
    find_package(CUDAToolkit REQUIRED)
    list(APPEND CUPOCH_LIBRARIES CUDA::cudart)
  else()
    message(WARNING "cupoch headers or libraries not found. Disabling GPU mapping acceleration.")
    set(CUPOCH_FOUND FALSE)
  endif()
endif()

ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${GRID_MAP_INCLUDE_DIR}
  $<$<BOOL:${CUPOCH_FOUND}>:${CUPOCH_INCLUDE_DIR}>
)


ament_auto_add_executable(${PROJECT_NAME}_exe src/occupancygrid.cpp)

target_link_libraries(${PROJECT_NAME}_exe ${rclcpp_LIBRARIES} ${PCL_LIBRARIES})

if(CUPOCH_FOUND)
  target_link_libraries(${PROJECT_NAME}_exe ${CUPOCH_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME}_exe PUBLIC USE_CUPOCH)
endif()


install(TARGETS
${PROJECT_NAME}_exe
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
