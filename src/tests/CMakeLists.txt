add_executable(test_read_write test_read_write.cpp
                               ../base/quantized_mesh_tile.cpp
                               ../base/gzip_file_reader.cpp
                               ../base/gzip_file_writer.cpp
                               ../base/quantized_mesh.cpp
                               ../base/cgal_utils.cpp)
target_link_libraries(test_read_write ${Boost_LIBRARIES}
                                      ${ZLIB_LIBRARIES}
                                      ${CTB_LIBRARY}
                                      ${CGAL_LIBRARIES}
                                      ${GDAL_LIBRARY}
                                      ${GeographicLib_LIBRARIES})

add_executable(simplify_maintaining_border_edges simplify_preserving_border_edges.cpp)
target_link_libraries(simplify_maintaining_border_edges ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(remesh_preserving_borders remesh_preserving_borders.cpp)
target_link_libraries(remesh_preserving_borders ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

