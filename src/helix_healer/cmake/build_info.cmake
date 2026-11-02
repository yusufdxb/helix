# Runs at every build (not only at configure): writes the git commit and whether the
# package sources differed from it, so every binary can report what it was built from.
# Inputs: SRC (package source dir), OUT (header path).
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${SRC}
  OUTPUT_VARIABLE commit OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc ERROR_QUIET)
if(NOT rc EQUAL 0)
  set(commit "unknown")
endif()
execute_process(COMMAND git status --porcelain -- . ../helix_msgs WORKING_DIRECTORY ${SRC}
  OUTPUT_VARIABLE st OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(st STREQUAL "")
  set(dirty 0)
else()
  set(dirty 1)
endif()
set(text "#pragma once\n#define HELIX_BUILD_COMMIT \"${commit}\"\n#define HELIX_BUILD_DIRTY ${dirty}\n")
file(WRITE ${OUT}.tmp "${text}")
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OUT}.tmp ${OUT})
