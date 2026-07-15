#
# SPDX-FileCopyrightText: Copyright (c) 2019 - 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

OPTIX_add_sample_executable( optixTriangle target_name
  optixTriangle.cu
  optixTriangle.cpp
  OptixRangeTracer.cpp
  OptixRangeTracer.h
  OPTIONS -rdc true
  )

target_link_libraries( ${target_name}
  ${CUDA_LIBRARIES}
  )

option(OPTIX_BUILD_PYTORCH_BINDING "Build the optix_range_tracer Python module" OFF)

if(OPTIX_BUILD_PYTORCH_BINDING)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  find_package(Torch REQUIRED)
  find_library(TORCH_PYTHON_LIBRARY torch_python
    PATHS "${TORCH_INSTALL_PREFIX}/lib"
    NO_DEFAULT_PATH
    REQUIRED)

  add_library(optix_range_tracer MODULE
    torch_bindings.cpp
    OptixRangeTracer.cpp
    OptixRangeTracer.h
    )
  add_dependencies(optix_range_tracer optixTriangle)
  target_include_directories(optix_range_tracer PRIVATE
    ${Python3_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/support"
    )
  target_link_libraries(optix_range_tracer PRIVATE
    ${TORCH_LIBRARIES}
    ${TORCH_PYTHON_LIBRARY}
    Python3::Python
    sutil_7_sdk
    ${CUDA_LIBRARIES}
    )
  separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
  target_compile_options(optix_range_tracer PRIVATE ${TORCH_CXX_FLAGS_LIST})
  target_compile_features(optix_range_tracer PRIVATE cxx_std_17)
  target_compile_definitions(optix_range_tracer PRIVATE
    OPTIX_SAMPLE_NAME_DEFINE=optixTriangle
    OPTIX_SAMPLE_DIR_DEFINE=optixTriangle
    )
  set_target_properties(optix_range_tracer PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    BUILD_RPATH "${TORCH_INSTALL_PREFIX}/lib;${CMAKE_BINARY_DIR}/lib"
    )
  if(UNIX)
    target_link_options(optix_range_tracer PRIVATE "-Wl,--disable-new-dtags")
  endif()
endif()
