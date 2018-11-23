add_executable(qm_tiler qm_tiler.cpp
                        ../base/quantized_mesh_tile.cpp
                        ../base/gzip_file_reader.cpp
                        ../base/gzip_file_writer.cpp
                        ../base/quantized_mesh.cpp
                        ../base/quantized_mesh_tiler.cpp
                        ../../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                        ../../3rdParty/meshoptimizer/vfetchoptimizer.cpp
                        ../base/zoom_tiles_border_vertices_cache.cpp
                        ../base/quantized_mesh_tiles_pyramid_builder_parallel.cpp)
target_link_libraries(qm_tiler TinCreation
                               ${Boost_LIBRARIES}
                               ${ZLIB_LIBRARIES}
                               ${CTB_LIBRARY}
                               ${CGAL_LIBRARIES}
                               ${GDAL_LIBRARY})

add_executable(dem2tin dem2tin.cpp)
target_link_libraries(dem2tin TinCreation ${Boost_LIBRARIES} ${CGAL_LIBRARIES} ${GDAL_LIBRARY})

install(TARGETS qm_tiler dem2tin DESTINATION bin)