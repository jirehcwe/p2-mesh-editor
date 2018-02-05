add_custom_target(docs ALL ${DOXYGEN_EXECUTABLE}
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/documentation
                  COMMENT "Generating HTML documentation" VERBATIM)

