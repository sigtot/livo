#!/usr/bin/env bash

echo "Setting kernel flags to enable perf profiling"
sudo sh -c "echo 0 > /proc/sys/kernel/kptr_restrict"
sudo sh -c "echo -1 > /proc/sys/kernel/perf_event_paranoid"

echo "Done setting flags"
