add_executable(qm_parser qm_parser.cpp
               ../base/quantized_mesh_tile.cpp
               ../base/gzip_file_reader.cpp
               ../base/gzip_file_writer.cpp
               ../base/quantized_mesh.cpp)
target_link_libraries(qm_parser ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY})

add_executable(test_read_write test_read_write.cpp
                               ../base/quantized_mesh_tile.cpp
                               ../base/gzip_file_reader.cpp
                               ../base/gzip_file_writer.cpp
                               ../base/quantized_mesh.cpp)
target_link_libraries(test_read_write ${Boost_LIBRARIES}
                                      ${ZLIB_LIBRARIES}
                                      ${CTB_LIBRARY}
                                      ${CGAL_LIBRARIES}
                                      ${GDAL_LIBRARY})

add_executable(simplify_maintaining_border_edges simplify_preserving_border_edges.cpp)
target_link_libraries(simplify_maintaining_border_edges ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(remesh_preserving_borders remesh_preserving_borders.cpp)
target_link_libraries(remesh_preserving_borders ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(hierarchy_simplification hierarchy_simplification.cpp)
target_link_libraries(hierarchy_simplification ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(wlop_simplification wlop_simplification.cpp)
target_link_libraries(wlop_simplification ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(test_check_borders test_check_borders.cpp
                                  ../base/quantized_mesh_tile.cpp
                                  ../base/gzip_file_reader.cpp
                                  ../base/gzip_file_writer.cpp
                                  ../base/quantized_mesh.cpp)
target_link_libraries(test_check_borders ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${GDAL_LIBRARY})

add_executable(detect_features_without_borders detect_features_without_borders.cpp)
target_link_libraries(detect_features_without_borders ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

add_executable(compute_statistics compute_statistics.cpp
                                  ../base/quantized_mesh_tile.cpp
                                  ../base/quantized_mesh_tiler.cpp
                                  ../base/gzip_file_reader.cpp
                                  ../base/gzip_file_writer.cpp
                                  ../base/quantized_mesh.cpp
                                  ../../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                                  ../../3rdParty/meshoptimizer/vfetchoptimizer.cpp
                                  ../base/crs_conversions.cpp)
target_link_libraries(compute_statistics ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${GDAL_LIBRARY})