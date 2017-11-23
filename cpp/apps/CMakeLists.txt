add_executable(qm_parser qm_parser.cpp
                         ../src/quantized_mesh_tile.cpp
                         ../src/gzip_file_reader.cpp
                         ../src/gzip_file_writer.cpp
                         ../src/quantized_mesh.cpp
                         ../src/cgal_utils.cpp)
target_link_libraries(qm_parser ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})

add_executable(qm_tiler qm_tiler.cpp ../src/quantized_mesh_tile.cpp
                                     ../src/gzip_file_reader.cpp
                                     ../src/gzip_file_writer.cpp
                                     ../src/quantized_mesh.cpp
                                     ../src/quantized_mesh_tiler.cpp
                                     ../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                                     ../3rdParty/meshoptimizer/vfetchoptimizer.cpp
                                     ../src/cgal_utils.cpp
                                     ../src/cgal_further_constrained_placement.h
                                     ../src/cgal_corner_vertices_are_constrained_vertex_map.h ../src/ellipsoid.h ../src/zoom_tiles_border_vertices_cache.cpp ../src/zoom_tiles_border_vertices_cache.h ../src/tile_border_vertices.h ../src/quantized_mesh_tiles_pyramid_builder.cpp ../src/quantized_mesh_tiles_pyramid_builder.h ../src/zoom_tiles_processing_scheduler.h ../src/cgal_cost_and_count_stop_predicate.h)
target_link_libraries(qm_tiler ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})
