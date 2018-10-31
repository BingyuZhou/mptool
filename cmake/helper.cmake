function(add_gunit_test SOURCE_FILE_NAME)
    get_filename_component(TEST_EXECUTABLE_NAME ${SOURCE_FILE_NAME} NAME_WE)

    get_filename_component(TEST_EXECUTABLE_DIRECTORY_NAME "${SOURCE_FILE_NAME}/.." ABSOLUTE)

    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_FILE_NAME})

    target_link_libraries(${TEST_EXECUTABLE_NAME}
        ${ARGN} GUnit gcov)

    install(TARGETS ${TEST_EXECUTABLE_NAME} DESTINATION test)
    add_test(NAME ${TEST_EXECUTABLE_NAME} COMMAND ${TEST_EXECUTABLE_NAME})
    set_tests_properties(${TEST_EXECUTABLE_NAME} PROPERTIES LABELS ${TEST_EXECUTABLE_DIRECTORY_NAME})
endfunction()
