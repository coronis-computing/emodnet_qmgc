add_executable(qm_parser qm_parser.cpp
                         ../base/quantized_mesh_tile.cpp
                         ../base/gzip_file_reader.cpp
                         ../base/gzip_file_writer.cpp
                         ../base/quantized_mesh.cpp)
target_link_libraries(qm_parser ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY})

add_executable(qm_tiler qm_tiler.cpp
                        ../base/quantized_mesh_tile.cpp
                        ../base/gzip_file_reader.cpp
                        ../base/gzip_file_writer.cpp
                        ../base/quantized_mesh.cpp
                        ../base/quantized_mesh_tiler.cpp
                        ../../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                        ../../3rdParty/meshoptimizer/vfetchoptimizer.cpp
        ../base/zoom_tiles_border_vertices_cache.cpp
                        ../base/quantized_mesh_tiles_pyramid_builder_parallel.cpp
                        ../base/crs_conversions.h)
#        ../tin_creation/tin_creation_delaunay_strategy.cpp
#        ../tin_creation/tin_creation_simplification_lindstrom_turk_strategy.cpp
#        ../tin_creation/tin_creation_remeshing_strategy.cpp
#        ../tin_creation/tin_creation_greedy_insertion_strategy.cpp
#        ../tin_creation/tin_creation_simplification_point_set.cpp
#        ../tin_creation/tin_creation_simplification_point_set_hierarchy.cpp
#        ../tin_creation/tin_creation_simplification_point_set_wlop.cpp
#        ../tin_creation/tin_creation_simplification_point_set_random.cpp
#        ../tin_creation/tin_creation_simplification_point_set_grid.cpp)
target_link_libraries(qm_tiler TinCreation
                               ${Boost_LIBRARIES}
                               ${ZLIB_LIBRARIES}
                               ${CTB_LIBRARY}
                               ${CGAL_LIBRARIES}
                               ${GDAL_LIBRARY})

add_executable(dem2tin dem2tin.cpp ../cgal/detect_sharp_edges_without_borders.h ../cgal/extract_polylines_from_sharp_edges.h)
#        ../TinCreation/tin_creation_delaunay_strategy.cpp
#        ../TinCreation/tin_creation_simplification_lindstrom_turk_strategy.cpp
#        ../TinCreation/tin_creation_remeshing_strategy.cpp
#        ../TinCreation/tin_creation_greedy_insertion_strategy.cpp
#        ../TinCreation/tin_creation_simplification_point_set.cpp
#        ../TinCreation/tin_creation_simplification_point_set_hierarchy.cpp
#        ../TinCreation/tin_creation_simplification_point_set_wlop.cpp
#        ../TinCreation/tin_creation_simplification_point_set_random.cpp
#        ../TinCreation/tin_creation_simplification_point_set_grid.cpp)
target_link_libraries(dem2tin TinCreation ${Boost_LIBRARIES} ${CGAL_LIBRARIES} ${GDAL_LIBRARY})

install(TARGETS qm_parser qm_tiler dem2tin DESTINATION bin)