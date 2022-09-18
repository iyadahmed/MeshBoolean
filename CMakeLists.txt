cmake_minimum_required(VERSION 3.16)
project(MeshBoolean)

#find_package(TBB REQUIRED)

add_executable(
        boolean

        ball_tree2.hh
        boolean2.cc
        common.hh
        non_copyable.hh
        statistics.hh
        timers.hh
        vec3.hh

        BVH/aabb.hh
        BVH/barycentric.hh
        BVH/bvh.cc
        BVH/bvh.hh
        BVH/common.hh
        BVH/node.hh
        BVH/query_point.cc
        BVH/query_point.hh
        BVH/segment_aabb_intersection.hh
        BVH/segment_bvh_intersection.hh
        BVH/segment_segment_intersection_2d.hh
        BVH/segment_triangle_intersection.hh
        BVH/triangle.hh
        meshio/stl/stl_binary_triangle.hh meshio/stl/stl_binary_reader.hh meshio/fread_e.hh meshio/stl/stl_binary_writer.hh
        meshio/tri_ascii_reader.hh
)

target_compile_features(boolean PUBLIC cxx_std_17)
#target_link_libraries(boolean PUBLIC TBB::tbb)
set_target_properties(boolean PROPERTIES CXX_EXTENSIONS OFF)

add_executable(
        unique

        meshio/stl/stl_binary_triangle.hh meshio/stl/stl_binary_reader.hh meshio/fread_e.hh meshio/stl/stl_binary_writer.hh
        unique_points.cc
        vec3.hh
)
target_compile_features(unique PUBLIC cxx_std_17)
#target_link_libraries(unique PUBLIC TBB::tbb)
set_target_properties(unique PROPERTIES CXX_EXTENSIONS OFF)

add_executable(partition_playground partition_playground.cc)