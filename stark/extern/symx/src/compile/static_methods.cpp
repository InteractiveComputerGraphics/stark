#include "static_methods.h"
#include "compiler_utils.h"
#include <mutex>

// Global state
#ifdef SYMX_COMPILER_PATH
static std::string _compiler_path_setting = SYMX_COMPILER_PATH;
#else
static std::string _compiler_path_setting = "AUTO";
#endif

static bool _suppress_compiler_output_global = true;
static bool _check_mode_ON_global = false;
static bool _is_load_compiled_enabled_global = true;
static bool _force_load_global = false;

// Mutex for thread safety
static std::mutex _compiler_mutex;

void symx::set_compiler_path(const std::string& path) { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	_compiler_path_setting = path; 
}
std::string symx::get_compiler_path() { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	return _compiler_path_setting; 
}

void symx::suppress_compiler_output(bool suppress) { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	_suppress_compiler_output_global = suppress; 
}
bool symx::get_suppress_compiler_output() { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	return _suppress_compiler_output_global; 
}

void symx::check_mode_ON(bool check) { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	_check_mode_ON_global = check; 
}
bool symx::get_check_mode_ON() { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	return _check_mode_ON_global; 
}

void symx::enable_load_compiled(bool enable) { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	_is_load_compiled_enabled_global = enable; 
}
bool symx::is_load_compiled_enabled() { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	return _is_load_compiled_enabled_global; 
}

void symx::force_load(bool force) { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	_force_load_global = force; 
}
bool symx::get_force_load() { 
	std::lock_guard<std::mutex> lock(_compiler_mutex);
	return _force_load_global; 
}

// Auto-resolve compiler path on load
namespace {
	struct AutoCompilerResolver {
		AutoCompilerResolver() {
			// This ensures the compiler path is resolved (if AUTO) when the library is loaded.
			// It avoids race conditions that might occur if multiple threads trigger resolution lazily.
			std::string path = symx::resolve_compiler_path();
			symx::set_compiler_path(path);
		}
	};
	static AutoCompilerResolver _auto_compiler_resolver;
}

