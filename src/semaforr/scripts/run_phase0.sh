#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${workspace_root}"

profile="${1:-normal}"
build_base="build"

case "${profile}" in
  normal)
    colcon build \
      --packages-select semaforr \
      --cmake-args -DBUILD_TESTING=ON
    ;;
  sanitizer)
    build_base="build-sanitizer"
    colcon build \
      --packages-select semaforr \
      --build-base build-sanitizer \
      --install-base install-sanitizer \
      --cmake-args \
        -DBUILD_TESTING=ON \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DSEMAFORR_ENABLE_SANITIZERS=ON
    ;;
  coverage)
    build_base="build-coverage"
    colcon build \
      --packages-select semaforr \
      --build-base build-coverage \
      --install-base install-coverage \
      --cmake-args \
        -DBUILD_TESTING=ON \
        -DCMAKE_BUILD_TYPE=Debug \
        -DSEMAFORR_ENABLE_COVERAGE=ON
    ;;
  *)
    echo "usage: $0 [normal|sanitizer|coverage]" >&2
    exit 2
    ;;
esac

if [[ "${profile}" == "sanitizer" ]]; then
  ASAN_OPTIONS="detect_leaks=1:halt_on_error=1" \
  UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1" \
    colcon test \
      --packages-select semaforr \
      --build-base "${build_base}"
else
  colcon test \
    --packages-select semaforr \
    --build-base "${build_base}"
fi
colcon test-result --verbose

if [[ "${profile}" == "coverage" ]]; then
  if command -v lcov >/dev/null 2>&1 && command -v genhtml >/dev/null 2>&1; then
    mkdir -p coverage
    lcov \
      --capture \
      --directory build-coverage/semaforr \
      --output-file coverage/semaforr.raw.info
    lcov \
      --extract coverage/semaforr.raw.info \
      "/workspace/src/semaforr/*" \
      --output-file coverage/semaforr.project.info
    lcov \
      --remove coverage/semaforr.project.info \
      "/workspace/src/semaforr/test/*" \
      --output-file coverage/semaforr.info
    genhtml \
      coverage/semaforr.info \
      --output-directory coverage/html
    echo "Coverage report: ${workspace_root}/coverage/html/index.html"
  else
    echo "Coverage was instrumented; install lcov to generate the HTML report."
  fi
fi
