cmake_minimum_required(VERSION 3.14)
project(finalproject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# We require the GNU compiler for this assignment.
if (NOT "${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    set(Prg-Intel "PrgEnv-intel")
    set(Prg-Clang "PrgEnv-cray")
    message(WARNING
            "Must use GNU Compiler for submission. Make sure you ran:\n"
            "module swap ${Prg-${CMAKE_C_COMPILER_ID}} PrgEnv-gnu")
endif ()

add_executable(serial main.cpp graph.hpp graph.cpp common.h serial.hpp)

# target_link_libraries(openmp PRIVATE OpenMP::OpenMP_CXX)

# Copy the job scripts
# configure_file(job-serial job-serial COPYONLY)
# configure_file(job-openmp job-openmp COPYONLY)