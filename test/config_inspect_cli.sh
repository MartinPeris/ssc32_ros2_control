#!/usr/bin/env bash
set -euo pipefail

readonly executable="$1"
readonly source_dir="$2"
readonly urdf="${source_dir}/urdf/example.urdf"

expect_failure() {
  if "$@" >/dev/null 2>&1; then
    printf 'Expected command to fail:' >&2
    printf ' %q' "$@" >&2
    printf '\n' >&2
    exit 1
  fi
}

"${executable}" "${urdf}" >/dev/null
"${executable}" --json "${urdf}" >/dev/null
"${executable}" --hardware ssc32_example "${urdf}" >/dev/null
"${executable}" --help >/dev/null
expect_failure "${executable}"
expect_failure "${executable}" --hardware
expect_failure "${executable}" --unknown "${urdf}"
expect_failure "${executable}" "${urdf}" "${urdf}"
expect_failure "${executable}" /definitely/missing/ssc32.urdf

temporary_directory="$(mktemp -d "${TMPDIR:-/tmp}/ssc32-inspect-test.XXXXXX")"
trap 'rm -rf "${temporary_directory}"' EXIT
printf '%s\n' '<not-a-robot>' >"${temporary_directory}/invalid.urdf"
expect_failure "${executable}" "${temporary_directory}/invalid.urdf"
