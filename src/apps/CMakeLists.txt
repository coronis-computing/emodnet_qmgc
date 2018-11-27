# qm_tiler app
add_executable(qm_tiler qm_tiler.cpp
                        ../base/quantized_mesh_tile.cpp
                        ../base/gzip_file_reader.cpp
                        ../base/gzip_file_writer.cpp
                        ../base/quantized_mesh.cpp
                        ../base/quantized_mesh_tiler.cpp
                        ../../3rdParty/meshoptimizer/vcacheoptimizer.cpp
                        ../../3rdParty/meshoptimizer/vfetchoptimizer.cpp
                        ../base/zoom_tiles_border_vertices_cache.cpp
                        ../base/quantized_mesh_tiles_pyramid_builder.cpp)
target_link_libraries(qm_tiler TinCreation
                               ${Boost_LIBRARIES}
                               ${ZLIB_LIBRARIES}
                               ${CTB_LIBRARY}
                               ${CGAL_LIBRARIES}
                               ${GDAL_LIBRARY})

if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(qm_tiler PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(qm_tiler "${CMAKE_THREAD_LIBS_INIT}")
endif()

# dem2tin app
add_executable(dem2tin dem2tin.cpp)
target_link_libraries(dem2tin TinCreation ${Boost_LIBRARIES} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(dem2tin PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(dem2tin "${CMAKE_THREAD_LIBS_INIT}")
endif()

install(TARGETS qm_tiler dem2tin DESTINATION bin)