if (CUDAToolkit_FOUND OR OpenCL_FOUND)
  message(STATUS "bite")
  add_subdirectory(GPURuntime)
endif (CUDAToolkit_FOUND OR OpenCL_FOUND)
message(STATUS "bite2")

set(LLVM_COMMON_DEPENDS ${LLVM_COMMON_DEPENDS} PARENT_SCOPE)
