#pragma once
#include <string>

namespace symx
{
	// Compiler path management
	void set_compiler_path(const std::string& path);
	std::string get_compiler_path();

	// Global compiler output suppression state
	void suppress_compiler_output(bool suppress = true);
	bool get_suppress_compiler_output();
	
	// Global NaN checking state
	void check_mode_ON(bool check = true);
	bool get_check_mode_ON();
	
	// Global force compilation state. Will return false when trying to load a cached compilation.
	void enable_load_compiled(bool force = true);
	bool is_load_compiled_enabled();

	// Global force load state. Will always load a cached compilation if available, without checking the checksum.
	void force_load(bool force = true);
	bool get_force_load();
}
