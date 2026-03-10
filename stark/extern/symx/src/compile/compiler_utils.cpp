#include "compiler_utils.h"
#include "static_methods.h"
#include <filesystem>
#include <iostream>
#include <sstream>
#include <array>
#include <memory>
#include <limits>
#include <vector>
#include <algorithm>
#include <mutex>
#include <fstream>
#include <random>
#include <cmath>

#ifdef _MSC_VER
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace symx
{
	// Forward declaration of internal helper
	static void _test_compile_simple_program(CompilerInfo& info, const std::string& compiler_path, bool check_avx = false);
}

std::vector<std::string> symx::list_auto_search_compiler_paths(bool return_one)
{
	std::vector<std::string> paths;

#ifdef _MSC_VER
	auto quote = [](const std::string& s) { return "\"" + s + "\""; };

	auto add_unique = [&](const std::string& p) {
		if (std::find(paths.begin(), paths.end(), p) == paths.end())
			paths.push_back(p);
	};

	auto trim = [](std::string s) {
		auto not_space = [](unsigned char c) { return !std::isspace(c); };
		s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
		s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
		return s;
	};

	auto run_capture = [&](const std::string& cmd) -> std::string {
		std::string out;
		FILE* pipe = _popen(cmd.c_str(), "r");
		if (!pipe) return out;
		char buf[4096];
		while (fgets(buf, (int)sizeof(buf), pipe)) out += buf;
		_pclose(pipe);
		return out;
	};

    auto getenv_str = [](const char* name) -> std::string {
        char* buf = nullptr;
        size_t len = 0;
        if (_dupenv_s(&buf, &len, name) != 0 || !buf) return {};
        std::string s(buf);
        free(buf);
        return s;
    };

	// 0) Try vswhere (latest VS instance with VC tools), if available
	{
		std::vector<std::string> vswhere_candidates;
        std::string pf86 = getenv_str("ProgramFiles(x86)");
		if (!pf86.empty()) {
			vswhere_candidates.push_back(std::string(pf86) + "\\Microsoft Visual Studio\\Installer\\vswhere.exe");
        }
        std::string pf = getenv_str("ProgramFiles");
		if (!pf.empty()) {
			vswhere_candidates.push_back(std::string(pf) + "\\Microsoft Visual Studio\\Installer\\vswhere.exe");
        }
		vswhere_candidates.push_back("C:\\Program Files (x86)\\Microsoft Visual Studio\\Installer\\vswhere.exe");

		for (const auto& vw : vswhere_candidates) {
			if (!std::filesystem::exists(vw)) continue;

			std::string cmd =
				quote(vw) +
				" -latest -products *"
				" -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64"
				" -property installationPath";

			std::string install = trim(run_capture(cmd));
			if (!install.empty()) {
				std::string vcvars64 = install + "\\VC\\Auxiliary\\Build\\vcvars64.bat";
				if (std::filesystem::exists(vcvars64)) {
					add_unique(quote(vcvars64));
					if (return_one) return paths;
				}
			}
			break; // don't try more vswhere locations if one existed
		}
	}

	// 1) Common places fallback
	std::vector<std::string> roots;
    
    std::string pf = getenv_str("ProgramFiles");
	if (!pf.empty()) roots.push_back(pf);
    std::string pf86 = getenv_str("ProgramFiles(x86)");
	if (!pf86.empty()) roots.push_back(pf86);
	roots.push_back("C:\\Program Files");
	roots.push_back("C:\\Program Files (x86)");

	std::vector<std::string> versions = { "18", "2022", "2019", "2017" };
	std::vector<std::string> editions = { "Community", "Professional", "Enterprise", "BuildTools", "Preview" };

	for (const auto& root : roots) {
		for (const auto& ver : versions) {
			for (const auto& ed : editions) {
				std::string p = root + "\\Microsoft Visual Studio\\" + ver + "\\" + ed +
					"\\VC\\Auxiliary\\Build\\vcvars64.bat";
				if (std::filesystem::exists(p)) {
					add_unique(quote(p));
					if (return_one) return paths;
				}
			}
		}
	}

	// (Optional) small legacy fallback
	for (const auto& root : roots) {
		std::string p = root + "\\Microsoft Visual Studio 14.0\\VC\\vcvarsall.bat";
		if (std::filesystem::exists(p)) {
			add_unique(quote(p));
			if (return_one) return paths;
		}
	}

	return paths;

#else
	// Linux logic (intentional: return a command string)
	if (const char* cxx = std::getenv("CXX")) {
		if (cxx && *cxx) {
			paths.push_back(std::string(cxx));
			if (return_one) return paths;
		}
	}

	std::vector<std::string> candidates = { "g++", "clang++", "c++", "/usr/bin/g++", "/usr/bin/clang++", "/usr/bin/c++" };
	for (const auto& p : candidates) {
		if (!p.empty() && p[0] == '/') {
			if (std::filesystem::exists(p)) {
				paths.push_back(p);
				if (return_one) return paths;
			}
		} else {
			paths.push_back(p);
			if (return_one) return paths;
		}
	}
	return paths;
#endif
}


symx::CompilerInfo symx::get_compiler_info(const std::string& path)
{
    // We try compiling simple programs
	CompilerInfo info;
	info.path = path;

    // Try first with AVX
    bool check_avx = true;
    _test_compile_simple_program(info, path, check_avx);

    if (info.is_available) { 
        if (!info.could_compile) {
            // Try without AVX
            check_avx = false;
            _test_compile_simple_program(info, path, check_avx);
        }
    }

    return info;
}

void symx::print_compiler_info(const CompilerInfo &info)
{
    auto print_bool = [](bool val) { return val ? "Yes" : "No"; };

	std::cout << "Compiler Path: " << info.path << std::endl;
    std::cout << "\t Available: " << print_bool(info.is_available) << std::endl;
    if (info.is_available) {
        std::cout << "\t Could Compile Test Program: " << print_bool(info.could_compile) << std::endl;
        std::cout << "\t Could Load Test Program: " << print_bool(info.could_load) << std::endl;
        std::cout << "\t Verified Result: " << print_bool(info.verified_result) << std::endl;
        std::cout << "\t Supports AVX: " << print_bool(info.supports_avx) << std::endl;
    }
}

void symx::validate_auto_search_compiler_paths()
{
	std::cout << "Validating auto-detected compilers..." << std::endl;
	auto paths = list_auto_search_compiler_paths();
	if (paths.empty()) {
		std::cout << "No compilers found in standard locations." << std::endl;
		return;
	}

	for (const auto& path : paths) {
		CompilerInfo info = get_compiler_info(path);
		print_compiler_info(info);
	}
}

std::string symx::resolve_compiler_path()
{
    // User specified in CMake
	std::string current_setting = symx::get_compiler_path();
	if (current_setting != "AUTO") {
		return current_setting;
	}

    // Check default auto-search paths
    std::vector<std::string> compiler_path = list_auto_search_compiler_paths(/*return_one=*/ true);
    if (!compiler_path.empty()) {
        return compiler_path[0];
    }

    // Error and exit
    std::cout << "symx error: Could not auto-resolve compiler path." << std::endl;
    std::cout << "No suitable compiler found in standard locations." << std::endl;
    symx::print_compiler_help();
    exit(-1);
}

void symx::print_compiler_diagnostics()
{
    std::string compiler_path = resolve_compiler_path();
    CompilerInfo info = get_compiler_info(compiler_path);
    print_compiler_info(info);
}

void symx::print_compiler_help()
{
    std::cout << "Compiler Help:" << std::endl;
    std::cout << "1. Ensure you have a C++ compiler installed (e.g., g++, clang++, MSVC)." << std::endl;
    std::cout << "2. Make sure the compiler supports C++17 or higher." << std::endl;
    std::cout << "3. If using SIMD features, ensure the compiler supports AVX instructions." << std::endl;
    std::cout << "4. You can set the compiler path in symx using symx::set_compiler_path(path)." << std::endl;
    std::cout << "5. Use symx::suppress_compiler_output(false) to see detailed compilation errors." << std::endl;
    std::cout << "6. For specific compiler diagnostics, run symx::print_compiler_diagnostics()." << std::endl;
    std::cout << "7. To check all auto-detected compilers, run symx::validate_auto_search_compiler_paths()." << std::endl;
    std::cout << "8. Try after manually deleting cached compiled shared objects in the codegen folder." << std::endl;
}

// =============================================================================
// INTERNAL HELPERS
// =============================================================================

void symx::_test_compile_simple_program(CompilerInfo& info, const std::string& compiler_path, bool check_avx)
{
    // Check availability
#ifdef _MSC_VER
    std::string check_cmd = compiler_path + " >nul 2>nul";
#else
    std::string check_cmd = compiler_path + " --version > /dev/null 2>&1";
#endif
    int avail_result = system(check_cmd.c_str());
    info.is_available = (avail_result == 0);
    if (!info.is_available) {
        return;
    }

	// Create a temporary folder for testing with a random name to avoid race conditions
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, 999999);
	std::string folder_name = "symx_compiler_test_" + std::to_string(dis(gen));
	
	std::filesystem::create_directories(folder_name);
	
	// Write a simple test program
	std::ofstream test_file(folder_name + "/test.cpp");
	test_file << "#include <cmath>\n";
	if (check_avx) {
		test_file << "#include <immintrin.h>\n";
	}
	test_file << "extern \"C\" {\n";
	test_file << "#ifdef _MSC_VER\n#define EXPORT __declspec(dllexport)\n#else\n#define EXPORT\n#endif\n";
	test_file << "EXPORT double test_func(double x) { \n";
	if (check_avx) {
		test_file << "    __m256d a = _mm256_set1_pd(x);\n";
		test_file << "    __m256d b = _mm256_add_pd(a, a);\n";
		test_file << "    double res[4];\n";
		test_file << "    _mm256_storeu_pd(res, b);\n";
		test_file << "    return res[0];\n";
	} else {
		test_file << "    return std::sin(x);\n";
	}
	test_file << "}\n";
	test_file << "}\n";
	test_file.close();
	
	// Try to compile it
	std::string compile_cmd;
	std::string lib_name;
#ifdef _MSC_VER
	lib_name = "test.dll";
	std::string avx_flag = check_avx ? "/arch:AVX2" : "";
	compile_cmd = compiler_path + " >nul 2>nul && cd " + folder_name + " && cl test.cpp /LD /Ox " + avx_flag + " >nul 2>nul";
#else
	lib_name = "test.so";
	std::string avx_flag = check_avx ? "-march=native -mavx2" : "";
	compile_cmd = "cd " + folder_name + " && " + compiler_path + " test.cpp -shared -O3 " + avx_flag + " -o " + lib_name + " > /dev/null 2>&1";
#endif
	
	int result = system(compile_cmd.c_str());
	info.could_compile = (result == 0);

	if (info.could_compile) {
		// Try to load and run
		std::filesystem::path lib_path = std::filesystem::absolute(folder_name);
		lib_path /= lib_name;
		
		typedef double (*FuncType)(double);
		FuncType func = nullptr;
		double input = 1.0;
		double expected = check_avx ? 2.0 : std::sin(1.0);
		double output = 0.0;

#ifdef _MSC_VER
		HMODULE hLib = LoadLibraryA(lib_path.string().c_str());
		info.could_load = (hLib != NULL);
		if (info.could_load) {
			func = (FuncType)GetProcAddress(hLib, "test_func");
			if (func) {
				output = func(input);
			} 
			FreeLibrary(hLib);
		} 
#else
		void* handle = dlopen(lib_path.string().c_str(), RTLD_LAZY);
		info.could_load = (handle != NULL);
		if (info.could_load) {
			func = (FuncType)dlsym(handle, "test_func");
			if (func) {
				output = func(input);
			} 
			dlclose(handle);
		}
#endif
		if (info.could_load) {
			info.verified_result = (std::abs(output - expected) <= 1e-6);
			info.supports_avx = check_avx && info.verified_result;
		}
	}
	
	// Cleanup
	try {
		std::filesystem::remove_all(folder_name);
	} catch (...) {}
}
