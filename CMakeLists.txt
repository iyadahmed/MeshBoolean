project(MeshBoolean)

add_executable(meshboolean
        bvh.cc
        bvh.hh
        ball_tree.hh
        common.hh
        meshio.hh
        vec3.hh
        non_copyable.hh
        timers.hh
        ball_tree2.hh
        )

add_executable(partition_playground
        partition_playground.cc
        )