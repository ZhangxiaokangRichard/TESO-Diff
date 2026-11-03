function(avanti_configure_runtime target export_macro runtime_root)
    target_compile_features(${target} PUBLIC cxx_std_11)
    target_compile_definitions(${target} PRIVATE ${export_macro})
    target_include_directories(${target} PUBLIC
        "$<BUILD_INTERFACE:${runtime_root}/include>" "$<INSTALL_INTERFACE:include>")
    set_target_properties(${target} PROPERTIES
        POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist")
    foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set_target_properties(${target} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/dist"
            LIBRARY_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/dist"
            ARCHIVE_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/dist")
    endforeach()
    if(WIN32)
        set_target_properties(${target} PROPERTIES PREFIX "")
    endif()
    if(MINGW)
        target_link_options(${target} PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
    if(AVANTI_STAGE_LIBS)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "${runtime_root}/lib"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:${target}>" "${runtime_root}/lib/$<TARGET_FILE_NAME:${target}>"
            VERBATIM)
    endif()
    include(GNUInstallDirs)
    install(TARGETS ${target} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endfunction()

function(avanti_add_test target source runtime)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE ${runtime})
    # Imported tests use assert: keep checks enabled in Release.
    if(MSVC)
        target_compile_options(${target} PRIVATE /UNDEBUG)
    else()
        target_compile_options(${target} PRIVATE -UNDEBUG)
    endif()
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist" BUILD_RPATH "$ORIGIN")
    foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/dist")
    endforeach()
    add_test(NAME ${target} COMMAND ${target})
endfunction()
