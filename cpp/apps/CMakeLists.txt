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
                                     ../src/zoom_tiles_border_vertices_cache.cpp
                                     ../src/quantized_mesh_tiles_pyramid_builder_parallel.cpp
                                     ../src/surface_simplification_lindstrom_turk_strategy.cpp
#                                     ../src/surface_simplifier_remeshing_strategy.cpp
                                     )
target_link_libraries(qm_tiler ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})
