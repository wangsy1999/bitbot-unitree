# The bundled kernel's unqualified to_json<...> also instantiates nlohmann's
# unrelated overloads under Clang. Patch only the extracted build-tree header;
# keep the upstream ZIP intact. This replacement is idempotent on reconfigure.
get_target_property(_kernel_source BitbotKernel SOURCE_DIR)
set(_kernel_header "${_kernel_source}/include/bitbot_kernel/kernel/kernel.hpp")
file(READ "${_kernel_header}" _kernel_original)
string(REPLACE "to_json<DerivedT, BusManagerT, UserData>(j, header);"
  "j = Json{{\"kernel\", header.kernel}, {\"bus\", header.bus}, {\"extra\", header.extra}};"
  _kernel_patched "${_kernel_original}")
if(NOT _kernel_original STREQUAL _kernel_patched)
  file(WRITE "${_kernel_header}" "${_kernel_patched}")
endif()
