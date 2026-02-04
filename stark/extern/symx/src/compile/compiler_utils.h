#pragma once
#include <string>
#include <vector>

namespace symx
{
	// Compiler information structure
	struct CompilerInfo {
		std::string path;
		bool is_available;
		bool could_compile;
		bool could_load;
		bool verified_result;
		bool supports_avx;
	};

	// Compiler path management
	std::string resolve_compiler_path(); // Returns the effective path (resolves AUTO)
	std::vector<std::string> list_auto_search_compiler_paths(bool return_one = false);
	CompilerInfo get_compiler_info(const std::string& path);
	void print_compiler_info(const CompilerInfo& info);
	void validate_auto_search_compiler_paths();

	// Diagnostics
	void print_compiler_diagnostics();
    void print_compiler_help();
}
