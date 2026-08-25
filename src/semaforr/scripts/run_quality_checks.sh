#!/usr/bin/env bash
# File overview: This file implements run quality checks behavior for developer tooling and experiment automation. It records the declarations, settings, fixtures, or guidance needed by that responsibility. Its package-relative location is `scripts/run_quality_checks.sh`.
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${workspace_root}"

profile="${1:-normal}"
build_base="build"

case "${profile}" in
  normal)
    colcon build \
      --packages-up-to semaforr \
      --cmake-args \
        -DBUILD_TESTING=ON \
        -DSEMAFORR_STRICT_MODERN_CODE=ON
    ;;
  sanitizer)
    build_base="build-sanitizer"
    colcon build \
      --packages-up-to semaforr \
      --build-base build-sanitizer \
      --install-base install-sanitizer \
      --cmake-args \
        -DBUILD_TESTING=ON \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DSEMAFORR_ENABLE_SANITIZERS=ON \
        -DSEMAFORR_STRICT_MODERN_CODE=ON
    ;;
  coverage)
    build_base="build-coverage"
    colcon build \
      --packages-up-to semaforr \
      --build-base build-coverage \
      --install-base install-coverage \
      --cmake-args \
        -DBUILD_TESTING=ON \
        -DCMAKE_BUILD_TYPE=Debug \
        -DSEMAFORR_ENABLE_COVERAGE=ON \
        -DSEMAFORR_STRICT_MODERN_CODE=ON
    ;;
  *)
    echo "usage: $0 [normal|sanitizer|coverage]" >&2
    exit 2
    ;;
esac

if [[ "${profile}" == "coverage" ]]; then
  # A renamed translation unit can leave an orphaned counter in an incremental
  # build. Remove only coverage runtime data before executing this profile.
  find build-coverage/semaforr -type f -name '*.gcda' -delete
fi

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
      "*/src/semaforr/*" \
      --output-file coverage/semaforr.project.info
    lcov \
      --remove coverage/semaforr.project.info \
      "*/src/semaforr/test/*" \
      --output-file coverage/semaforr.info
    python3 \
      src/semaforr/scripts/check_coverage.py \
      coverage/semaforr.info
    genhtml \
      coverage/semaforr.info \
      --output-directory coverage/html
    echo "Coverage report: ${workspace_root}/coverage/html/index.html"
  else
    echo "Coverage was instrumented; install lcov to generate the HTML report."
  fi
fi
