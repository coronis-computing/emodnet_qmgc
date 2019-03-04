add_library(TinCreation SHARED tin_creator.cpp
                               tin_creation_delaunay_strategy.cpp
                               tin_creation_greedy_insertion_strategy.cpp
                               tin_creation_remeshing_strategy.cpp
                               tin_creation_simplification_lindstrom_turk_strategy.cpp
                               tin_creation_simplification_point_set.cpp
                               tin_creation_simplification_point_set_grid.cpp
                               tin_creation_simplification_point_set_hierarchy.cpp
                               tin_creation_simplification_point_set_random.cpp
                               tin_creation_simplification_point_set_wlop.cpp)
                               #../base/crs_conversions.cpp)

set_target_properties(TinCreation PROPERTIES PUBLIC_HEADER tin_creator.h
                                                           tin_creation_cgal_types.h
                                                           tin_creation_delaunay_strategy.h
                                                           tin_creation_greedy_insertion_strategy.h
                                                           tin_creation_remeshing_strategy.h
                                                           tin_creation_simplification_lindstrom_turk_strategy.h
                                                           tin_creation_simplification_point_set.h
                                                           tin_creation_simplification_point_set_grid.h
                                                           tin_creation_simplification_point_set_hierarchy.h
                                                           tin_creation_simplification_point_set_random.h
                                                           tin_creation_simplification_point_set_wlop.h)

install(TARGETS TinCreation
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})