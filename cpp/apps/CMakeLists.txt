

add_executable(parse_quantized_mesh_terrain parse_quantized_mesh_terrain.cpp
                                            ../src/quantized_mesh_tile.cpp)
target_link_libraries(parse_quantized_mesh_terrain ${Boost_LIBRARIES} ${ZLIB_LIBRARIES})