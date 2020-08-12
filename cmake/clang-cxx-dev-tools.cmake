# Additional targets to perform clang-format/clang-tidy

#Eclude files in build and Eigen directory
set(EXCLUDE_BUILD_DIR "/build/")
set(EXCLUDE_EIGEN_DIR "/Eigen/")

# Get all project files
file(GLOB_RECURSE
     ALL_CXX_SOURCE_FILES
     *.[chi]pp *.[chi]xx *.h *.c *.cc *.hh *.ii *.[CHI]
    )

# Remove files in build directory
foreach (TMP_PATH ${ALL_CXX_SOURCE_FILES})
    string (FIND ${TMP_PATH} ${EXCLUDE_BUILD_DIR} EXCLUDE_DIR_FOUND)
    if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
        list (REMOVE_ITEM ALL_CXX_SOURCE_FILES ${TMP_PATH})
    endif ()
endforeach(TMP_PATH)

# Remove files in Eigen directory
foreach (TMP_PATH ${ALL_CXX_SOURCE_FILES})
    string (FIND ${TMP_PATH} ${EXCLUDE_EIGEN_DIR} EXCLUDE_DIR_FOUND)
    if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
        list (REMOVE_ITEM ALL_CXX_SOURCE_FILES ${TMP_PATH})
    endif ()
endforeach(TMP_PATH)

# Adding clang-format target if executable is found
find_program(CLANG_FORMAT "clang-format")
if(CLANG_FORMAT)
  add_custom_target(
    format
    COMMAND clang-format
    -i
    -style=file
    --verbose 
    ${ALL_CXX_SOURCE_FILES}
    )
endif()
