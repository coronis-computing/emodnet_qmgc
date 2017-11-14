add_executable(parse_quantized_mesh_terrain parse_quantized_mesh_terrain.cpp
                                            ../src/quantized_mesh_tile.cpp
                                            ../src/gzip_file_reader.cpp
                                            ../src/gzip_file_writer.cpp
                                            ../src/quantized_mesh.cpp
                                            ../src/cgal_utils.cpp)
target_link_libraries(parse_quantized_mesh_terrain ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})

add_executable(convert_to_qmt convert_to_qmt.cpp
        ../src/quantized_mesh_tile.cpp
        ../src/gzip_file_reader.cpp
        ../src/gzip_file_writer.cpp
        ../src/quantized_mesh.cpp
        ../src/cgal_utils.cpp)
target_link_libraries(convert_to_qmt ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})

add_executable(qm_tiler qm_tiler.cpp ../src/quantized_mesh_tile.cpp
                                     ../src/gzip_file_reader.cpp
                                     ../src/gzip_file_writer.cpp
                                     ../src/quantized_mesh.cpp
                                     ../src/quantized_mesh_tiler.cpp
                                     ../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                                     ../3rdParty/meshoptimizer/vfetchoptimizer.cpp
                                     ../src/cgal_utils.cpp ../src/cgal_further_constrained_placement.h "../src/cgal_corner_vertices_are_constrained_vertex_map.h")
target_link_libraries(qm_tiler ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})
