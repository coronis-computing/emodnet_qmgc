

add_executable(parse_quantized_mesh_terrain parse_quantized_mesh_terrain.cpp
                                            ../src/quantized_mesh_tile.cpp
                                            ../src/gzip_file_reader.cpp)
target_link_libraries(parse_quantized_mesh_terrain ${Boost_LIBRARIES} ${ZLIB_LIBRARIES})