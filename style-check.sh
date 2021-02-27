#!/usr/bin/env bash

# TODO: Does not recursively search directories
cat *.{cpp,h} | clang-format-10 --dry-run -Werror
