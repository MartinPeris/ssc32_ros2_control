#!/usr/bin/env bash
set -euo pipefail

readonly dependency_commit="2d3004df425c1eddc0084e7f1e40ebdc3fa29836"
repository_root="$(git rev-parse --show-toplevel)"
readonly repository_root
readonly source_mode="${1:---working-tree}"

case "${source_mode}" in
  --staged | --working-tree) ;;
  *)
    printf 'Usage: %s [--staged|--working-tree]\n' "$0" >&2
    exit 2
    ;;
esac

for command in docker git tar; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    printf 'Required command not found: %s\n' "${command}" >&2
    exit 1
  fi
done

if ! docker info >/dev/null 2>&1; then
  printf 'Docker is required and must be running.\n' >&2
  exit 1
fi

if [[ "${source_mode}" == "--staged" ]]; then
  git -C "${repository_root}" diff --cached --check
fi

workspace="$(mktemp -d "${TMPDIR:-/tmp}/ssc32-quality.XXXXXX")"
cleanup() {
  rm -rf "${workspace}"
}
trap cleanup EXIT

mkdir -p "${workspace}/src/ssc32_ros2_control" "${workspace}/src/ssc32"
if [[ "${source_mode}" == "--staged" ]]; then
  tree="$(git -C "${repository_root}" write-tree)"
  git -C "${repository_root}" archive "${tree}" |
    tar -xf - -C "${workspace}/src/ssc32_ros2_control"
else
  tar -C "${repository_root}" \
    --exclude=.git --exclude=build --exclude=install --exclude=log \
    -cf - . | tar -xf - -C "${workspace}/src/ssc32_ros2_control"
fi

dependency_repository="${repository_root}/../ssc32"
if [[ -d "${dependency_repository}/.git" ]] &&
  git -C "${dependency_repository}" cat-file -e "${dependency_commit}^{commit}" 2>/dev/null; then
  git -C "${dependency_repository}" archive "${dependency_commit}" |
    tar -xf - -C "${workspace}/src/ssc32"
else
  dependency_cache="${XDG_CACHE_HOME:-${HOME}/.cache}/ssc32_ros2_control/ssc32"
  mkdir -p "$(dirname "${dependency_cache}")"
  if [[ ! -d "${dependency_cache}/.git" ]]; then
    git clone --quiet https://github.com/MartinPeris/ssc32.git "${dependency_cache}"
  fi
  if ! git -C "${dependency_cache}" cat-file -e "${dependency_commit}^{commit}" 2>/dev/null; then
    git -C "${dependency_cache}" fetch --quiet origin "${dependency_commit}"
  fi
  git -C "${dependency_cache}" archive "${dependency_commit}" |
    tar -xf - -C "${workspace}/src/ssc32"
fi

quality_source="${workspace}/src/ssc32_ros2_control"
dockerfile="${quality_source}/docker/Dockerfile.lyrical"
image_tag="ssc32-ros2-control-quality:$(git hash-object "${dockerfile}" | cut -c1-12)"
if ! docker image inspect "${image_tag}" >/dev/null 2>&1; then
  printf 'Building quality image %s (first run only for this toolchain revision)\n' "${image_tag}"
  docker build --tag "${image_tag}" --file "${dockerfile}" "${quality_source}"
fi

printf 'Running quality gate against the %s snapshot\n' "${source_mode#--}"
docker run --rm \
  --volume "${workspace}:/workspace" \
  --volume ssc32_ros2_control_ccache:/root/.cache/ccache \
  --env CCACHE_DIR=/root/.cache/ccache \
  "${image_tag}" \
  bash /workspace/src/ssc32_ros2_control/scripts/quality_gate_inner.sh
