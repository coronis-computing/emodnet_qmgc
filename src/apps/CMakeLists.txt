add_executable(qm_parser qm_parser.cpp
                         ../base/quantized_mesh_tile.cpp
                         ../base/gzip_file_reader.cpp
                         ../base/gzip_file_writer.cpp
                         ../base/quantized_mesh.cpp
                         ../base/cgal_utils.cpp)
target_link_libraries(qm_parser ${Boost_LIBRARIES}
                                ${ZLIB_LIBRARIES}
                                ${CTB_LIBRARY}
                                ${CGAL_LIBRARIES}
                                ${GDAL_LIBRARY})

add_executable(qm_tiler qm_tiler.cpp
                        ../base/quantized_mesh_tile.cpp
                        ../base/gzip_file_reader.cpp
                        ../base/gzip_file_writer.cpp
                        ../base/quantized_mesh.cpp
                        ../base/quantized_mesh_tiler.cpp
                        ../../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                        ../../3rdParty/meshoptimizer/vfetchoptimizer.cpp
                        ../base/cgal_utils.cpp
                        ../base/zoom_tiles_border_vertices_cache.cpp
                        ../base/quantized_mesh_tiles_pyramid_builder_parallel.cpp
                        ../base/surface_simplification_lindstrom_turk_strategy.cpp
                        ../base/surface_simplifier_remeshing_strategy.cpp
                        ../base/crs_conversions.h ../base/cgal_polyhedron_builder_from_c3t3_boundary.h)
target_link_libraries(qm_tiler ${Boost_LIBRARIES}
                               ${ZLIB_LIBRARIES}
                               ${CTB_LIBRARY}
                               ${CGAL_LIBRARIES}
                               ${GDAL_LIBRARY})
