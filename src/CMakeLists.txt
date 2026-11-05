        include_directories(base)
        include_directories(apps)

        add_subdirectory(tin_creation)
        add_subdirectory(apps)
        if(QMGC_BUILD_DEV_TESTS)
            add_subdirectory(dev-tests)
        endif()
