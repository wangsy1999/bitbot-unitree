#!/bin/bash
# Bootstrap project-local tools, then build/test/run the macOS simulator.
set -euo pipefail

if [[ "$(uname -s)" != Darwin ]]; then
  echo "This script requires macOS." >&2
  exit 1
fi
mode="${1:-build}"
case "$mode" in
  build|test|run) ;;
  *) echo "Usage: $0 [build|test|run]" >&2; exit 1 ;;
esac
root="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="$root/build"
tools_dir="$build_dir/tools"
mkdir -p "$tools_dir"
xcrun --find clang++ >/dev/null

cmake_bin="${CMAKE:-}"
if [[ -z "$cmake_bin" ]]; then
  if command -v cmake >/dev/null 2>&1; then
    cmake_bin="$(command -v cmake)"
  else
    cmake_bin="$tools_dir/python/cmake/data/bin/cmake"
    if [[ ! -x "$cmake_bin" ]]; then
      python3 -m pip install --no-cache-dir --target "$tools_dir/python" 'cmake==3.31.6'
    fi
  fi
fi
arch="$(uname -m)"
case "$arch" in
  arm64|x86_64) ;;
  *) echo "Unsupported architecture: $arch" >&2; exit 1 ;;
esac
ort_version=1.23.2
ort_name="onnxruntime-osx-$arch-$ort_version"
ort_root="${ONNXRUNTIME_ROOT:-$tools_dir/$ort_name}"
if [[ ! -f "$ort_root/include/onnxruntime_cxx_api.h" ]]; then
  if [[ -n "${ONNXRUNTIME_ROOT:-}" ]]; then
    echo "ONNXRUNTIME_ROOT has no include/onnxruntime_cxx_api.h: $ort_root" >&2
    exit 1
  fi
  curl -fL --retry 3 --connect-timeout 20 \
    "https://github.com/microsoft/onnxruntime/releases/download/v$ort_version/$ort_name.tgz" \
    -o "$tools_dir/$ort_name.tgz"
  tar -xzf "$tools_dir/$ort_name.tgz" -C "$tools_dir"
fi

"$cmake_bin" -S "$root" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES="$arch" \
  -DBUILD_SIMULATION=ON -DBITBOT_MUJOCO_BUILD_TESTS=ON \
  -DONNXRUNTIME_ROOT="$ort_root"
"$cmake_bin" --build "$build_dir" --target main_app bitbot_mujoco_smoke_test \
  --parallel "${JOBS:-$(sysctl -n hw.logicalcpu)}"
if [[ "$mode" == test ]]; then
  "$(dirname "$cmake_bin")/ctest" --test-dir "$build_dir" --output-on-failure
elif [[ "$mode" == run ]]; then
  # Existing policy/trajectory paths are relative to build/bin.
  cd "$build_dir/bin"
  exec ./main_app
fi
