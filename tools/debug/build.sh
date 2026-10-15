#!/usr/bin/env bash
# Configure and build with the STM32CubeIDE toolchain and the existing CMake preset.
# Does not flash.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"

if [[ ! -x "${CMAKE_BIN}" || ! -x "${NINJA_BIN}" || ! -x "${GCC_BIN}" ]]; then
    echo "error: CubeIDE cmake, ninja, or gcc is missing" >&2
    exit 1
fi

cd "${ROOT}"
export PATH="$(dirname "${GCC_BIN}"):$(dirname "${NINJA_BIN}"):$(dirname "${CMAKE_BIN}"):${PATH}"

compiler_file="$(find "${BUILD_DIR}/CMakeFiles" -name CMakeCCompiler.cmake 2>/dev/null | head -1 || true)"
if [[ -n "${compiler_file}" ]] && ! grep -q "gnu-tools-for-stm32/14.3.1+st.2" "${compiler_file}"; then
    echo "${BUILD_DIR} was configured with a compiler other than GNU Tools for STM32 14.3.1."
    echo "Reconfiguring that gitignored build directory. Sources are not modified."
    rm -rf "${BUILD_DIR}/CMakeCache.txt" "${BUILD_DIR}/CMakeFiles"
fi

# The toolchain file forces -g0 for Release; -g as a compile option restores
# symbols for GDB/peek.sh and leaves the generated code identical.
"${CMAKE_BIN}" --preset "${BUILD_TYPE}" -S "${ROOT}" \
    -DCMAKE_PROJECT_INCLUDE="${DEBUG_DIR}/debug-info.cmake"
"${CMAKE_BIN}" --build --preset "${BUILD_TYPE}" --parallel
echo "ELF ${ELF}"
echo "BIN ${BIN}"
"${OBJDUMP_BIN}" -s -j .isr_vector "${ELF}" | head -6
