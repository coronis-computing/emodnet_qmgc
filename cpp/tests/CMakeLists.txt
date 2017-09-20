

add_executable(test_read_write test_read_write.cpp
        ../src/quantized_mesh_tile.cpp
        ../src/gzip_file_reader.cpp
        ../src/gzip_file_writer.cpp
        ../src/quantized_mesh.cpp
        ../src/cgal_utils.cpp)
target_link_libraries(test_read_write ${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${CTB_LIBRARY} ${CGAL_LIBRARIES} ${GDAL_LIBRARY} ${GeographicLib_LIBRARIES})

include( CGAL_CreateSingleSourceCGALProgram )
create_single_source_cgal_program( simplify_maintaining_border_edges.cpp )
target_link_libraries(simplify_maintaining_border_edges ${Boost_LIBRARIES} ${CGAL_LIBRARIES})

