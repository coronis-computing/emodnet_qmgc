add_executable(test_read_write test_read_write.cpp
                               ../base/quantized_mesh_tile.cpp
                               ../base/gzip_file_reader.cpp
                               ../base/gzip_file_writer.cpp
                               ../base/quantized_mesh.cpp
        )
target_link_libraries(test_read_write ${Boost_LIBRARIES}
                                      ${ZLIB_LIBRARIES}
                                      ${CTB_LIBRARY}
                                      ${CGAL_LIBRARIES}
                                      ${GDAL_LIBRARY})

add_executable(simplify_maintaining_border_edges simplify_preserving_border_edges.cpp)
target_link_libraries(simplify_maintaining_border_edges ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(remesh_preserving_borders remesh_preserving_borders.cpp)
target_link_libraries(remesh_preserving_borders ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(hierarchy_simplification hierarchy_simplification.cpp )
target_link_libraries(hierarchy_simplification ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(wlop_simplification wlop_simplification.cpp)
target_link_libraries(wlop_simplification ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(test_check_borders test_check_borders.cpp
                                  ../base/quantized_mesh_tile.cpp
                                  ../base/gzip_file_reader.cpp
                                  ../base/gzip_file_writer.cpp
                                  ../base/quantized_mesh.cpp)
target_link_libraries(test_check_borders ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${GDAL_LIBRARY})
