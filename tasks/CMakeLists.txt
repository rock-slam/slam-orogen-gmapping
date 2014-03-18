# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(gmappingTaskLib)


add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME} -DLINUX)

include(FindPkgConfig)

pkg_check_modules(BASE REQUIRED "base-lib")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})



find_package(Boost REQUIRED COMPONENTS thread)
#find_package(gmapping REQUIRED)


include_directories(${gmapping_INCLUDE_DIRS}/gmapping ${gmapping_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

ADD_LIBRARY(${GMAPPING_TASKLIB_NAME} SHARED 
    ${GMAPPING_TASKLIB_SOURCES})
add_dependencies(${GMAPPING_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${GMAPPING_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${gmapping_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${GMAPPING_TASKLIB_DEPENDENT_LIBRARIES}
    ${BASE_LIBRARIES})
SET_TARGET_PROPERTIES(${GMAPPING_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${GMAPPING_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${GMAPPING_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${GMAPPING_TASKLIB_HEADERS}
    DESTINATION include/orocos/gmapping)

