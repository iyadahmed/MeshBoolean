cmake_minimum_required(VERSION 3.16)
project(MeshBoolean)

#find_package(TBB REQUIRED)

add_executable(
        boolean

        boolean.cc
        bvh2.hh
        common.hh
        meshio.hh
        vec3.hh
        non_copyable.hh
        timers.hh
        ball_tree2.hh
        statistics.hh
        segment_segment_intersection_2d.hh
        bvh_point_query.hh
)

target_compile_features(boolean PUBLIC cxx_std_11)
#target_link_libraries(boolean PUBLIC TBB::tbb)
set_target_properties(boolean PROPERTIES CXX_EXTENSIONS OFF)

add_executable(
        unique

        unique_points.cc
        meshio.hh
        vec3.hh
)
target_compile_features(unique PUBLIC cxx_std_11)
#target_link_libraries(unique PUBLIC TBB::tbb)
set_target_properties(unique PROPERTIES CXX_EXTENSIONS OFF)

add_executable(partition_playground partition_playground.cc)