cmake_minimum_required(VERSION 3.16)
project(MeshBoolean)

#find_package(TBB REQUIRED)

add_executable(meshboolean
        meshboolean.cc
        bvh.hh
        common.hh
        meshio.hh
        vec3.hh
        non_copyable.hh
        timers.hh
        ball_tree2.hh
        statistics.hh
        )

target_compile_features(meshboolean PUBLIC cxx_std_14)
#target_link_libraries(meshboolean PUBLIC TBB::tbb)
set_target_properties(meshboolean PROPERTIES CXX_EXTENSIONS OFF)

add_executable(partition_playground
        partition_playground.cc
        )