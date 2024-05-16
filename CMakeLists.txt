cmake_minimum_required(VERSION 3.14)
project(finalproject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

find_package(MPI REQUIRED)
# find_package(UPCXX REQUIRED)

# We require the GNU compiler for this assignment.

#if (NOT "${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
#    set(Prg-Intel "PrgEnv-intel")
#    set(Prg-Clang "PrgEnv-cray")
#    message(WARNING
#            "Must use GNU Compiler for submission. Make sure you ran:\n"
#            "module swap ${Prg-${CMAKE_C_COMPILER_ID}} PrgEnv-gnu")
#endif ()

# Uncomment for MPI
add_executable(mpi main.cpp graph.hpp common.h mpi.hpp)

# Uncomment for UPC++
# add_executable(upcxx main.cpp graph.hpp common.h upcxx.hpp)

#add_executable(serial main.cpp graph.hpp graph.cpp common.h serial.hpp)

target_link_libraries(mpi PRIVATE MPI::MPI_CXX)
# target_link_libraries(upcxx PRIVATE UPCXX::upcxx)

# Copy the job scripts
# configure_file(job-serial job-serial COPYONLY)
