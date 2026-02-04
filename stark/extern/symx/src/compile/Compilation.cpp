#include "Compilation.h"
#include "compiler_utils.h"

#include <filesystem>
#include <iostream>
#include <sstream>
#include <array>
#include <memory>
#include <limits>
#include <vector>
#include <algorithm>
#include <picoSHA2/picosha2.h>
#include "omp.h"

#include "../symbol/utils.h"

#ifdef _MSC_VER
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif

// https://thispointer.com/find-and-replace-all-occurrences-of-a-sub-string-in-c/
void replace_all(std::string& data, std::string toSearch, std::string replaceStr)
{
	// Get the first occurrence
	size_t pos = data.find(toSearch);
	// Repeat till end is reached
	while (pos != std::string::npos)
	{
		// Replace this occurrence of Sub String
		data.replace(pos, toSearch.size(), replaceStr);
		// Get the next occurrence from the current position
		pos = data.find(toSearch, pos + replaceStr.size());
	}
}

std::string to_string_with_precision(const double a_value, const int n = 6)
{
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}


symx::Compilation::~Compilation()
{
	if (this->lib != nullptr) {
#ifdef _MSC_VER
		FreeLibrary(this->lib);
#else
		dlclose(this->lib);
#endif
	}
}
void symx::Compilation::compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id, FloatType float_type)
{
	std::string id = cache_id;
	if (id == "") {
		id = get_checksum(expr);
	}

	double t0 = omp_get_wtime();
	symx::Sequence seq(expr);
	this->_write_shared_object_code(seq, name, folder, id, float_type);
	double t1 = omp_get_wtime();
	this->runtime_codegen = t1 - t0;

	int err = -1;
	std::string command;
	std::string compiler_path = symx::resolve_compiler_path();

#ifdef _MSC_VER
	const std::string enable_output = (get_suppress_compiler_output()) ? " >nul 2>nul " : "";
	command = compiler_path + enable_output;
	command += " && cd " + folder;
	command += " && cl " + name + ".cpp /LD /Ox /arch:AVX2 /bigobj" + enable_output;  // Note: fast-math is not used because it can change the expected results (e.g. sqrt(pow(x, 2)) < 0)
	command += " && del " + name + ".exp";
	command += " && del " + name + ".lib";
	command += " && del " + name + ".obj";
#else
	const std::string enable_output = (get_suppress_compiler_output()) ? " > /dev/null 2>&1 " : "";
	command += "cd " + folder;
	command += " ; " + compiler_path + " " + name + ".cpp -shared -O3 -march=native -o " + name + ".so" + enable_output;  // Note: fast-math is not used because it can change the expected results (e.g. sqrt(pow(x, 2)) < 0)
#endif
	t0 = omp_get_wtime();
	err = system(command.c_str());

	// When compiling multiple units in parallel, sometimes it will not run successfully.
	// Trying it again solves the problem.
	if (err != 0) {
		err = system(command.c_str());
	}
	t1 = omp_get_wtime();
	this->runtime_compilation = t1 - t0;

	if (err != 0) {
		std::cout << "=== SYMX COMPILATION FAILED ===" << std::endl;
		std::cout << "Function: " << name << std::endl;
		std::cout << "Folder: " << folder << std::endl;
		std::cout << "Error code: " << err << std::endl;
		std::cout << "Command that failed:" << std::endl;
		std::cout << command << std::endl << std::endl;
		symx::print_compiler_help();
		exit(-1);
	}

	// Memory
	this->n_bytes_symbols = expr[0].get_expression_graph()->size() * sizeof(Expr);

	// Check if it can be loaded
	this->override_load = true; // Force load the compiled function
	bool success = this->load_if_cached(name, folder, id, float_type);
	this->override_load = false;
	this->cached = false;

	// If loading was not successful, there is a problem with the compilation
	if (!success) {
		std::cout << "symx error: Failed to load a freshly compiled shared object ('" << name << "')." << std::endl;
		std::cout << "This indicates a problem with the compilation process." << std::endl;
		std::cout << "Run symx::validate_auto_search_compiler_paths() for more info." << std::endl;
		exit(-1);
	}
}
bool symx::Compilation::load_if_cached(std::string name, std::string folder, std::string cache_id, FloatType float_type)
{
	if (!this->override_load && !symx::is_load_compiled_enabled()) {
		return false; // Global load is disabled, do not load
	}

	if (cache_id == "") {
		std::cout << "symx error: symx::Compilation::load_if_cached(). Cannot load with cache_id = \"\"" << std::endl;
		exit(-1);
	}

	// File exists?
	#ifdef _MSC_VER
	const std::string path = folder + "/" + name + ".dll";
	#else
	const std::string path = folder + "/" + name + ".so";
	#endif
	std::ifstream file(path.c_str());
	const bool file_exists = file.good();
	if (!file_exists) {
		return false;
	}

	// Load DLL
#ifdef _MSC_VER
	if (this->lib != nullptr) {
		FreeLibrary(this->lib);
		this->compiled_f = nullptr;
	}
	this->lib = LoadLibrary(TEXT(path.c_str()));
#else
	if (this->lib != nullptr) {
		dlclose(this->lib);
		this->compiled_f = nullptr;
	}
	this->lib = dlopen(path.c_str(), RTLD_LAZY);
#endif

	if (this->lib == NULL) {
		std::cout << "symx error: Failed to load compiled shared library for '" << name << "'." << std::endl;
		std::cout << "checksum matched but loading failed." << std::endl;
		std::cout << "File path: " << path << std::endl;
		symx::print_compiler_help();
		exit(-1);
	}

	bool loaded = false;
	if (!this->override_load && symx::get_force_load()) {
		loaded = true;
	}
	else {
		// Load the SHA256 in the DLL
		std::string dll_sha256_checksum;
		dll_sha256_checksum.resize(64);
#ifdef _MSC_VER
		void (*get_sha256)(char*) = reinterpret_cast<void(*)(char*)>(GetProcAddress(this->lib, "get_sha256"));
#else
		void (*get_sha256)(char*) = reinterpret_cast<void(*)(char*)>(dlsym(this->lib, "get_sha256"));
#endif
		get_sha256(dll_sha256_checksum.data());

		// Reconstruct SHA256
		std::string id = cache_id + std::to_string(static_cast<int>(float_type));
		std::vector<unsigned char> hash(picosha2::k_digest_size);
		picosha2::hash256(id.begin(), id.end(), hash.begin(), hash.end());
		std::string sha256_checksum = picosha2::bytes_to_hex_string(hash.begin(), hash.end());
		loaded = dll_sha256_checksum == sha256_checksum;
	}

	// checksums match?
	if (loaded) {
#ifdef _MSC_VER
		this->compiled_f = GetProcAddress(this->lib, name.c_str());
		this->n_inputs = reinterpret_cast<int(*)()>(GetProcAddress(this->lib, "get_n_inputs"))();
		this->n_outputs = reinterpret_cast<int(*)()>(GetProcAddress(this->lib, "get_n_outputs"))();
		this->compiled_type = static_cast<FloatType>(reinterpret_cast<int(*)()>(GetProcAddress(this->lib, "get_float_type"))());
#else
		this->compiled_f = dlsym(this->lib, name.c_str());
		this->n_inputs = reinterpret_cast<int(*)()>(dlsym(this->lib, "get_n_inputs"))();
		this->n_outputs = reinterpret_cast<int(*)()>(dlsym(this->lib, "get_n_outputs"))();
		this->compiled_type = static_cast<FloatType>(reinterpret_cast<int(*)()>(dlsym(this->lib, "get_float_type"))());
#endif
		this->compiled_type = float_type;
		this->cached = true;
		return true;
	}
	else {
		if (this->lib != nullptr) {
#ifdef _MSC_VER
			FreeLibrary(this->lib);
#else
			dlclose(this->lib);
#endif
			this->lib = nullptr;
		}
		this->cached = false;
		return false;
	}
}
void symx::Compilation::try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id, FloatType float_type)
{
	std::string id = cache_id;
	if (id == "") {
		id = get_checksum(expr);
	}

	// Check global force compilation state - if true, skip loading and force compilation
	if (symx::is_load_compiled_enabled()) {
		if (!this->load_if_cached(name, folder, id, float_type)) {
			this->compile(expr, name, folder, id, float_type);
		}
	} else {
		this->compile(expr, name, folder, id, float_type);
	}
}
void symx::Compilation::_write_shared_object_code(Sequence& seq, std::string name, std::string folder, std::string cache_id, FloatType float_type)
{
	std::string code;

	// SIMD mode?
	const bool is_simd = (float_type != FloatType::Double && float_type != FloatType::Float) ? true : false;

	// Type as text
	const std::string type = get_float_type_as_string(float_type);

	// Checksum
	std::string sha256_checksum;
	if (cache_id == "") {
		sha256_checksum = "0000000000000000000000000000000000000000000000000000000000000000";
	}
	else {
		// Add the operation type and rehash
		std::string id = cache_id + std::to_string(static_cast<int>(float_type));
		std::vector<unsigned char> hash(picosha2::k_digest_size);
		picosha2::hash256(id.begin(), id.end(), hash.begin(), hash.end());
		sha256_checksum = picosha2::bytes_to_hex_string(hash.begin(), hash.end());
	}

	// Includes
	code += "#include <cmath>\n";
	code += "#include <cstdio>\n";
	code += "#include <limits>\n";
	if (is_simd) {
		code += "#include <immintrin.h>\n";
	}
	code += "\n";

	// Extern for MSVC
	code += "#if _MSC_VER\n";
	code += "#define EXPORT __declspec(dllexport)\n";
	code += "#else\n";
	code += "#define EXPORT\n";
	code += "#endif\n";
	code += "\n";


	// SIMD core functions
	if (is_simd) {
		code += "// SIMD core functions\n";
		this->_add_core_simd_functions(code, float_type);
		code += "\n";
	}

	// Begin extern
	code += "extern \"C\"\n{\n\n";

	// Function info
	code += "EXPORT\n";
	code += "int get_n_inputs() { return " + std::to_string(seq.get_n_inputs()) + "; }\n\n";
	code += "EXPORT\n";
	code += "int get_n_outputs() { return " + std::to_string(seq.get_n_outputs()) + "; }\n\n";
	code += "EXPORT\n";
	code += "int get_float_type() { return " + std::to_string(static_cast<int>(float_type)) + "; }\n\n";

	// Checksum
	code += "EXPORT\n";
	code += "void get_sha256(char* str) {\n\tchar id[] = \"" + sha256_checksum + "\";\n\tfor(int i = 0; i < 64; i++) {\n\t\tstr[i] = id[i];\n\t}\n}\n\n";

	// Function
	code += "EXPORT\n";
	code += "void " + name + "(" + type + "* in, " + type + "* out)\n";
	code += "{\n";
	code += "\t/*\n";
	code += seq.count_ops_string("\t\t");
	code += "\t*/\n";

	if (is_simd) {
		this->_add_instructions_simd(code, seq, type);
	}
	else {
		this->_add_instructions_scalar(code, seq, type);
	}
	code += "}\n\n";

	// End extern
	code += "} // extern \"C\"\n";

	// Create folder if it doesn't exist
	std::filesystem::create_directories(folder);

	// Write file
	std::ofstream outfile(folder + "/" + name + ".cpp");
	if (!outfile) {
		std::cout << "symx error: Cannot open the file " + folder + "/" + name + ".cpp" << std::endl;
		exit(-1);
	}
	outfile << code;
	outfile.close();
}
int symx::Compilation::get_n_inputs() const
{
	return this->n_inputs;
}
int symx::Compilation::get_n_outputs() const
{
	return this->n_outputs;
}
bool symx::Compilation::is_valid() const
{
	return this->compiled_f != nullptr;
}
bool symx::Compilation::was_cached() const
{
	return this->cached;
}
symx::FloatType symx::Compilation::get_compiled_type() const
{
    return this->compiled_type;
}

symx::Compilation::Info symx::Compilation::get_info() const
{
    Info info;
    info.n_inputs = this->n_inputs;
    info.n_outputs = this->n_outputs;
    info.compiled_type = this->compiled_type;
    info.was_cached = this->cached;
    info.runtime_codegen = this->runtime_codegen;
    info.runtime_compilation = this->runtime_compilation;
	info.n_bytes_symbols = this->n_bytes_symbols;
    return info;
}
void symx::Compilation::Info::print() const
{
	std::cout << "=== Compilation Info ===" << std::endl;
	std::cout << "Number of inputs: " << this->n_inputs << std::endl;
	std::cout << "Number of outputs: " << this->n_outputs << std::endl;
	std::cout << "Compiled float type: " << get_float_type_as_string(this->compiled_type) << std::endl;
	std::cout << "Was cached: " << (this->was_cached ? "Yes" : "No") << std::endl;
	std::cout << "Runtime code generation time (s): " << this->runtime_codegen << std::endl;
	std::cout << "Runtime compilation time (s): " << this->runtime_compilation << std::endl;
	std::cout << "Number of bytes for symbols: " << this->n_bytes_symbols << " bytes" << std::endl;
	std::cout << "========================" << std::endl;
}

void symx::Compilation::_add_instructions_scalar(std::string& code, Sequence& seq, std::string type)
{
	auto idx = [&](const int32_t& i)
	{
		if (i < seq.n_inputs) {
			return "in[" + std::to_string(i) + "]";
		}
		else {
			return "v" + std::to_string(i);
		}
	};

	// Operations
	int indentation = 1;

	auto tab = [&indentation]() { 
		std::string ind = "";
		for (int i = 0; i < indentation; i++) {
			ind += "\t";
		}
		return ind;
	};

	for (auto& op : seq.ops) {
		switch (op.type)
		{
		case ExprType::Symbol: // Represents output
			code += tab() + "out[" + std::to_string(op.dst) + "] = " + idx(op.a) + ";\n"; break;
		case ExprType::ConstantFloat:
			code += tab() + type + " " + idx(op.dst) + " = " + to_string_with_precision(op.constant, 20) + ";\n"; break;
		case ExprType::Add:
			code += tab() + type + " " + idx(op.dst) + " = " + idx(op.a) + " + " + idx(op.b) + ";\n"; break;
		case ExprType::Sub:
			code += tab() + type + " " + idx(op.dst) + " = " + idx(op.a) + " - " + idx(op.b) + ";\n"; break;
		case ExprType::Mul:
			code += tab() + type + " " + idx(op.dst) + " = " + idx(op.a) + " * " + idx(op.b) + ";\n"; break;
		case ExprType::Reciprocal:
			code += tab() + type + " " + idx(op.dst) + " = static_cast<" + type + ">(1.0) / " + idx(op.a) + ";\n"; break;
		case ExprType::PowN:
			code += tab() + type + " " + idx(op.dst) + " = std::pow(" + idx(op.a) + ", " + std::to_string(op.b) + ");\n"; break;
		case ExprType::PowF:
			code += tab() + type + " " + idx(op.dst) + " = std::pow(" + idx(op.a) + ", " + idx(op.b) + ");\n"; break;
		case ExprType::Sqrt:
			code += tab() + type + " " + idx(op.dst) + " = std::sqrt(" + idx(op.a) + ");\n"; break;
		case ExprType::Ln:
			code += tab() + type + " " + idx(op.dst) + " = (" + idx(op.a) + " <= 0.0) ? -std::numeric_limits<" + type + ">::infinity() : std::log(" + idx(op.a) + ");\n"; break;
		case ExprType::Log10:
			code += tab() + type + " " + idx(op.dst) + " = (" + idx(op.a) + " <= 0.0) ? -std::numeric_limits<" + type + ">::infinity() : std::log10(" + idx(op.a) + ");\n"; break;
		case ExprType::Exp:
			code += tab() + type + " " + idx(op.dst) + " = std::exp(" + idx(op.a) + ");\n"; break;
		case ExprType::Sin:
			code += tab() + type + " " + idx(op.dst) + " = std::sin(" + idx(op.a) + ");\n"; break;
		case ExprType::Cos:
			code += tab() + type + " " + idx(op.dst) + " = std::cos(" + idx(op.a) + ");\n"; break;
		case ExprType::Tan:
			code += tab() + type + " " + idx(op.dst) + " = std::tan(" + idx(op.a) + ");\n"; break;
		case ExprType::ArcSin:
			code += tab() + type + " " + idx(op.dst) + " = std::asin(" + idx(op.a) + ");\n"; break;
		case ExprType::ArcCos:
			code += tab() + type + " " + idx(op.dst) + " = std::acos(" + idx(op.a) + ");\n"; break;
		case ExprType::ArcTan:
			code += tab() + type + " " + idx(op.dst) + " = std::atan(" + idx(op.a) + ");\n"; break;
		case ExprType::Print:
			code += tab() + type + " " + idx(op.dst) + " = 0.0; printf(\"val_" + idx(op.a) + "=%.10e\\n\"," + idx(op.a) + ");\n"; break;
		case ExprType::Branch:
			if (op.is_endif()) {
				indentation--;
				code += tab() + "}\n";
			}
			if (op.is_positive_branch()) {
				code += tab() + "if (" + idx(op.cond) + " >= 0.0)\n";
				code += tab() + "{\n";
				indentation++;
			}
			else if (op.is_negative_branch()) {
				indentation--;
				code += tab() + "}\n";
				code += tab() + "else\n";
				code += tab() + "{\n";
				indentation++;
			}
			break;
		default:
			std::cout << "symx error: Compilation::_add_instructions_scalar found a not handled ExprType." << std::endl;
			exit(-1);
			break;
		}
	}
}
void symx::Compilation::_add_instructions_simd(std::string& code, Sequence& eval, std::string type)
{
	auto idx = [&](const int32_t& i)
	{
		if (i < eval.n_inputs) {
			return "in[" + std::to_string(i) + "]";
		}
		else {
			return "v" + std::to_string(i);
		}
	};

	// Determine if we need float suffix for constants
	const bool is_float_simd = (type == "__m128" || type == "__m256");
	const std::string constant_suffix = is_float_simd ? "f" : "";

	// Operations
	const std::string begin_line = "\t" + type + " ";
	for (auto& op : eval.ops) {
		switch (op.type)
		{
		case ExprType::Symbol: // Represents output
			code += "\tout[" + std::to_string(op.dst) + "] = " + idx(op.a) + ";\n"; break;
		case ExprType::ConstantFloat:
			code += begin_line + idx(op.dst) + " = set1(" + to_string_with_precision(op.constant, 20) + constant_suffix + ");\n"; break;
		case ExprType::Add:
			code += begin_line + idx(op.dst) + " = add(" + idx(op.a) + ", " + idx(op.b) + ");\n"; break;
		case ExprType::Sub:
			code += begin_line + idx(op.dst) + " = sub(" + idx(op.a) + ", " + idx(op.b) + ");\n"; break;
		case ExprType::Mul:
			code += begin_line + idx(op.dst) + " = mul(" + idx(op.a) + ", " + idx(op.b) + ");\n"; break;
		case ExprType::Reciprocal:
			code += begin_line + idx(op.dst) + " = inv(" + idx(op.a) + ");\n"; break;
		case ExprType::PowN:
			code += begin_line + idx(op.dst) + " = powN<" + std::to_string(op.b) + ">(" + idx(op.a) + "); \n"; break;
		case ExprType::PowF:
			code += begin_line + idx(op.dst) + " = pow(" + idx(op.a) + ", " + idx(op.b) + ");\n";
			break;
		case ExprType::Sqrt:
			code += begin_line + idx(op.dst) + " = sqrt(" + idx(op.a) + ");\n"; break;
		case ExprType::Ln:
			code += begin_line + idx(op.dst) + " = log(" + idx(op.a) + ");\n";
			break;
		case ExprType::Log10:
			code += begin_line + idx(op.dst) + " = log10(" + idx(op.a) + ");\n";
			break;
		case ExprType::Exp:
			code += begin_line + idx(op.dst) + " = exp(" + idx(op.a) + ");\n";
			break;
		case ExprType::Sin:
			code += begin_line + idx(op.dst) + " = sin(" + idx(op.a) + ");\n";
			break;
		case ExprType::Cos:
			code += begin_line + idx(op.dst) + " = cos(" + idx(op.a) + ");\n";
			break;
		case ExprType::Tan:
			code += begin_line + idx(op.dst) + " = tan(" + idx(op.a) + ");\n";
			break;
		case ExprType::ArcSin:
			code += begin_line + idx(op.dst) + " = asin(" + idx(op.a) + ");\n";
			break;
		case ExprType::ArcCos:
			code += begin_line + idx(op.dst) + " = acos(" + idx(op.a) + ");\n";
			break;
		case ExprType::ArcTan:
			code += begin_line + idx(op.dst) + " = atan(" + idx(op.a) + ");\n";
			break;
		case ExprType::Branch:
			std::cout << "symx error: Cannot generate SIMD code with banches." << std::endl;
			exit(-1);
			break;

		default:
			break;
		}
	}
}
void symx::Compilation::_add_core_simd_functions(std::string& code, FloatType float_type)
{
#ifdef SYMX_ENABLE_AVX2

	std::string functions;
	
	// Determine the number of elements per SIMD vector and scalar type
	int simd_width = 4; // default for SIMD4d
	std::string scalar_type = "double"; // default for SIMD4d
	std::string constant_suffix = ""; // default for double constants
	if (float_type == FloatType::SIMD8f) {
		simd_width = 8;
		scalar_type = "float";
		constant_suffix = "f";
	}
	
	functions += "__m256d set1(" + scalar_type + " a) { return _mm256_set1_pd(a); }\n";
	functions += "__m256d add(__m256d& a, __m256d& b) { return _mm256_add_pd(a, b); }\n";
	functions += "__m256d sub(__m256d& a, __m256d& b) { return _mm256_sub_pd(a, b); }\n";
	functions += "__m256d mul(__m256d& a, __m256d& b) { return _mm256_mul_pd(a, b); }\n";
	functions += "__m256d inv(__m256d& a) { return _mm256_div_pd(_mm256_set1_pd(1.0" + constant_suffix + "), a); }\n";
	functions += "__m256d sqrt(__m256d& a) { return _mm256_sqrt_pd(a); }\n";

	functions += "__m256d pow(__m256d& a, __m256d& b) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* b_view = reinterpret_cast<" + scalar_type + "*>(&b); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::pow(a_view[i], b_view[i]); }; return s; }\n";
	functions += "__m256d log(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = (a_view[i] <= 0.0) ? -std::numeric_limits<" + scalar_type + ">::infinity() : std::log(a_view[i]); }; return s; }\n";
	functions += "__m256d log10(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = (a_view[i] <= 0.0) ? -std::numeric_limits<" + scalar_type + ">::infinity() : std::log10(a_view[i]); }; return s; }\n";
	functions += "__m256d exp(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::exp(a_view[i]); }; return s; }\n";
	functions += "__m256d sin(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::sin(a_view[i]); }; return s; }\n";
	functions += "__m256d cos(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::cos(a_view[i]); }; return s; }\n";
	functions += "__m256d tan(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::tan(a_view[i]); }; return s; }\n";
	functions += "__m256d asin(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::asin(a_view[i]); }; return s; }\n";
	functions += "__m256d acos(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::acos(a_view[i]); }; return s; }\n";
	functions += "__m256d atan(__m256d& a) { __m256d s; " + scalar_type + "* a_view = reinterpret_cast<" + scalar_type + "*>(&a); " + scalar_type + "* s_view = reinterpret_cast<" + scalar_type + "*>(&s); for (int i = 0; i < " + std::to_string(simd_width) + "; i++) { s_view[i] = std::atan(a_view[i]); }; return s; }\n";

	switch (float_type)
	{
	case symx::FloatType::Double:
		break;
	case symx::FloatType::Float:
		break;
	case symx::FloatType::SIMD4d:
		break;
	case symx::FloatType::SIMD8f:
		replace_all(functions, "_mm256_", "_mm256_");
		replace_all(functions, "__m256d", "__m256");
		replace_all(functions, "_pd", "_ps");
		break;
	default:
		break;
	}

	// powN
	functions += "\n";
	functions += "template<size_t N, typename FLOAT_TYPE>\n";
	functions += "FLOAT_TYPE powN(FLOAT_TYPE & a)\n";
	functions += "{\n";
	functions += "    FLOAT_TYPE b = a;\n";
	functions += "    for (size_t i = 1; i < N; i++) {\n";
	functions += "        b = mul(b, a);\n";
	functions += "    }\n";
	functions += "    return b;\n";
	functions += "}\n";

	code += functions;
#else
	std::cout << "symx error: SIMD compilation not available." << std::endl;
	exit(-1);
#endif
}
