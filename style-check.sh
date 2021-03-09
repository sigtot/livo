#!/usr/bin/env bash

# TODO: Does not recursively search directories
cat src/*.cpp include/livo/*.h | clang-format-10 --dry-run -Werror
