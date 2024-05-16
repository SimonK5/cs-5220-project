# CS 5220: Parallel A*

To run MPI implementation:
1. Uncomment `add_executable(mpi main.cpp graph.hpp common.h mpi.hpp)` in CMakeLists.txt
2. Comment out `add_executable(upcxx main.cpp graph.hpp common.h upcxx.hpp)` in CMakeLists.txt
3. `cd build`
4. `cmake ..`
5. From main.cpp, call `mpi_astar(argc, argv, MAP_SIZE, STARTX, STARTY, ENDX, ENDY);`
6. `srun -N 1 --ntasks-per-node=[NUM_TASKS_PER_NODE] ./mpi`

To run UPC++ implementation:
1. Uncomment `add_executable(upcxx main.cpp graph.hpp common.h upcxx.hpp)` in CMakeLists.txt
2. Comment out `add_executable(mpi main.cpp graph.hpp common.h mpi.hpp)` in CMakeLists.txt
3. `cd build`
4. `cmake ..`
5. From main.cpp, call:
```
upcxx::init();
upcxx_astar(MAP_SIZE, std::vector<Obstacle>(),STARTX,STARTY,ENDX,ENDY);
upcxx::finalize();
```
6. TODO: command to call UPC++