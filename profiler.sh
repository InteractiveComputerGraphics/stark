#!/usr/bin/env bash
set -euo pipefail

# ./profiler.sh -n test3 -t examples/examples

# Repo-local perf profiler runner.
#
# Examples:
#   ./tools/profile.sh -b build -t myapp -n baseline -- --scene s.json --frames 200
#   ./tools/profile.sh -t bin/myapp -n "after_opt" -- --foo 1
#
# Notes:
# - Stores perf data in ./profile/<name>.data and metadata in ./profile/<name>.meta.txt
# - Uses RelWithDebInfo + -O2 -g + frame pointers for better stacks.

BUILD_DIR="build"
TARGET=""                  # either a CMake target name OR a path relative to build dir (see -t)
NAME=""                    # run name -> profile/<NAME>.data
GENERATOR=""               # optional: Ninja/Unix Makefiles etc (rarely needed)
CMAKE_EXTRA=()             # extra -D... args
OPEN_HOTSPOT=1

usage() {
  cat <<EOF
Usage:
  $0 [-b BUILD_DIR] [-t TARGET_OR_PATH] [-n NAME] [--no-hotspot] [--cmake -DKEY=VAL ...] -- [program args...]

Required:
  -t   CMake target name OR path to binary relative to build dir (e.g. "app/myapp")
  -n   Run name (used as filename under ./profile)

Options:
  -b   Build dir (default: build)
  --no-hotspot   Do not open Hotspot automatically
  --cmake ...    Additional CMake definitions (everything after --cmake until --)
  --             Separator before program args

Examples:
  $0 -b build -t myapp -n baseline -- --scene s.json --frames 200
  $0 -t bin/myapp -n test1 -- --foo bar
EOF
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -b) BUILD_DIR="$2"; shift 2;;
    -t) TARGET="$2"; shift 2;;
    -n) NAME="$2"; shift 2;;
    --no-hotspot) OPEN_HOTSPOT=0; shift 1;;
    --cmake)
      shift
      while [[ $# -gt 0 && "${1:-}" != "--" ]]; do
        CMAKE_EXTRA+=("$1")
        shift
      done
      ;;
    --) shift; break;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1"; usage; exit 2;;
  esac
done

if [[ -z "${TARGET}" || -z "${NAME}" ]]; then
  echo "Error: -t and -n are required."
  usage
  exit 2
fi

PROFILE_DIR="profile"
mkdir -p "${PROFILE_DIR}"

OUT_DATA="${PROFILE_DIR}/${NAME}.data"
OUT_META="${PROFILE_DIR}/${NAME}.meta.txt"

# CMake flags for profiling-friendly build
CMAKE_BUILD_TYPE="RelWithDebInfo"
C_FLAGS="-O2 -g -fno-omit-frame-pointer"
CXX_FLAGS="-O2 -g -fno-omit-frame-pointer"

# Configure (idempotent)
cmake -S . -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
  -DCMAKE_C_FLAGS_RELWITHDEBINFO="${C_FLAGS}" \
  -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="${CXX_FLAGS}" \
  "${CMAKE_EXTRA[@]}"

# Build
cmake --build "${BUILD_DIR}" -j

# Resolve binary:
# 1) If TARGET is an executable file under build dir -> use it
# 2) Else try cmake --build --target TARGET and locate via common heuristics:
#    - if build/<TARGET> exists
#    - else require user to pass relative path
BIN_CANDIDATE="${BUILD_DIR}/${TARGET}"
if [[ -x "${BIN_CANDIDATE}" ]]; then
  BIN="${BIN_CANDIDATE}"
else
  # Try building target explicitly (helps if -t is a target name)
  cmake --build "${BUILD_DIR}" --target "${TARGET}" -j8 || true

  if [[ -x "${BIN_CANDIDATE}" ]]; then
    BIN="${BIN_CANDIDATE}"
  else
    echo "Could not resolve executable:"
    echo "  Tried: ${BIN_CANDIDATE}"
    echo ""
    echo "Fix: pass -t as a path relative to build dir (e.g. -t app/myapp),"
    echo "or ensure the executable ends up at build/<target>."
    exit 1
  fi
fi

# Write metadata for reproducibility/comparison
{
  echo "name: ${NAME}"
  echo "date: $(date -Is)"
  echo "build_dir: ${BUILD_DIR}"
  echo "binary: ${BIN}"
  echo "args: $*"
  echo "cmake_build_type: ${CMAKE_BUILD_TYPE}"
  echo "c_flags_relwithdebinfo: ${C_FLAGS}"
  echo "cxx_flags_relwithdebinfo: ${CXX_FLAGS}"
  echo "cmake_extra: ${CMAKE_EXTRA[*]:-}"
  if command -v git >/dev/null 2>&1; then
    echo "git_commit: $(git rev-parse HEAD 2>/dev/null || true)"
    echo "git_status:"
    git status --porcelain 2>/dev/null || true
  fi
} > "${OUT_META}"

echo "Recording perf → ${OUT_DATA}"
echo "Command: ${BIN} $*"

# Record with call graphs
perf record -o "${OUT_DATA}" -g --call-graph fp -- "${BIN}" "$@"

echo "Wrote:"
echo "  ${OUT_DATA}"
echo "  ${OUT_META}"

if [[ ${OPEN_HOTSPOT} -eq 1 ]]; then
  echo "Opening Hotspot..."
  hotspot "${OUT_DATA}" >/dev/null 2>&1 & disown
fi
