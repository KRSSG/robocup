# ssl_common
common headers (and maybe libs) required by all packages

things like field config, geometry.hpp etc.

requires the macro SIM_COMM or SSL_COMM to decide which field config to use. (ideally, we probably don't need a differentiation)
this change needs to be made during compile time in the CMakeLists.txt for now, will think of a better solution later, or remove the distinction entirely.