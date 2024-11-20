#include "Compilation.h"

#include "omp.h"

#ifdef _MSC_VER
#define NOMINMAX
#include <windows.h>
std::string symx::compiler_command = "\"C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Auxiliary\\Build\\vcvarsx86_amd64.bat\"";
#else
#include <dlfcn.h>
std::string symx::compiler_command = "g++";
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
void symx::Compilation::compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, OpType op_type, bool suppress_compiler_output)
{
	double t0 = omp_get_wtime();
	symx::Sequence seq(expr);
	this->_write_shared_object_code(seq, name, folder, id, op_type);
	double t1 = omp_get_wtime();
	this->runtime_codegen = t1 - t0;

	int err = -1;
	std::string command;
#ifdef _MSC_VER
	const std::string enable_output = (suppress_compiler_output) ? " >nul 2>nul " : "";
	command = symx::compiler_command + enable_output;
	command += " && cd " + folder;
	command += " && cl " + name + ".cpp /LD /Ox /arch:AVX2 /bigobj" + enable_output;  // Note: fast-math is not used because it can change the expected results (e.g. sqrt(pow(x, 2)) < 0)
	command += " && del " + name + ".exp";
	command += " && del " + name + ".lib";
	command += " && del " + name + ".obj";
#else
	const std::string enable_output = (suppress_compiler_output) ? " > /dev/null " : "";
	command += "cd " + folder;
	command += " ; g++ " + name + ".cpp -shared -O3 -march=native -o " + name + ".so" + enable_output;  // Note: fast-math is not used because it can change the expected results (e.g. sqrt(pow(x, 2)) < 0)
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
		std::cout << "symx error: Compilation failed for " << name << ". Try not suppressing compilation output to see compiler error." << std::endl;
		std::cout << "compilation command: " << std::endl << std::endl;
		std::cout << command << std::endl;
		exit(-1);
	}

	bool success = this->load_if_cached(name, folder, id, op_type);
	this->was_cached = false;
}
bool symx::Compilation::load_if_cached(std::string name, std::string folder, std::string id, OpType op_type)
{
	if (id == "") {
		std::cout << "symx error: symx::Compilation::load_if_exists. Cannot load with id = \"\"" << std::endl;
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
		std::cout << "symx error: invalid shared object loaded for input (" << name << ", " << folder << ")." << std::endl;
		std::cout << "Probably the compilation didn't work. Troubleshoot:" << std::endl;
		std::cout << "\tTry to compile without supressing output." << std::endl;
		std::cout << "\tIs the compiler path set correctly? (" << symx::compiler_command << ")" << std::endl;
		std::cout << "\tIs the compiler producing the same bitness than the main program? (32-bit or 64-bit)" << std::endl;
		std::cout << "\tIf you used SIMD types, does the compiler supports them?" << std::endl;
		exit(-1);
	}

	bool load = false;
	if (id == "FORCE_LOAD") {
		load = true;
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
		id += std::to_string(static_cast<int>(op_type));
		std::vector<unsigned char> hash(picosha2::k_digest_size);
		picosha2::hash256(id.begin(), id.end(), hash.begin(), hash.end());
		std::string sha256_checksum = picosha2::bytes_to_hex_string(hash.begin(), hash.end());
		load = dll_sha256_checksum == sha256_checksum;
	}

	// checksums match?
	if (load) {
#ifdef _MSC_VER
		this->compiled_f = GetProcAddress(this->lib, name.c_str());
		this->n_inputs = reinterpret_cast<int(*)()>(GetProcAddress(this->lib, "get_n_inputs"))();
		this->n_outputs = reinterpret_cast<int(*)()>(GetProcAddress(this->lib, "get_n_outputs"))();
		this->compiled_type = static_cast<OpType>(reinterpret_cast<int(*)()>(GetProcAddress(this->lib, "get_op_type"))());
#else
		this->compiled_f = dlsym(this->lib, name.c_str());
		this->n_inputs = reinterpret_cast<int(*)()>(dlsym(this->lib, "get_n_inputs"))();
		this->n_outputs = reinterpret_cast<int(*)()>(dlsym(this->lib, "get_n_outputs"))();
		this->compiled_type = static_cast<OpType>(reinterpret_cast<int(*)()>(dlsym(this->lib, "get_op_type"))());
#endif
		this->compiled_type = op_type;
		this->was_cached = true;
		return true;
	}
	else {
		if (this->lib != nullptr) {
#ifdef _MSC_VER
			FreeLibrary(this->lib);
#else
			dlclose(this->lib);
#endif
		}
		this->was_cached = false;
		return false;
	}
}
void symx::Compilation::try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, OpType op_type, bool suppress_compiler_output)
{
	if (!this->load_if_cached(name, folder, id, op_type)) {
		this->compile(expr, name, folder, id, op_type, suppress_compiler_output);
	}
}

void symx::Compilation::_write_shared_object_code(Sequence& seq, std::string name, std::string folder, std::string id, OpType op_type)
{
	std::string code;

	// SIMD mode?
	const bool is_simd = (op_type != OpType::Double && op_type != OpType::Float) ? true : false;

	// Type as text
	const std::string type = this->_get_op_type_string(op_type);

	// Checksum
	std::string sha256_checksum;
	if (id == "") {
		sha256_checksum = "0000000000000000000000000000000000000000000000000000000000000000";
	}
	else {
		// Add the operation type and rehash
		id += std::to_string(static_cast<int>(op_type));
		std::vector<unsigned char> hash(picosha2::k_digest_size);
		picosha2::hash256(id.begin(), id.end(), hash.begin(), hash.end());
		sha256_checksum = picosha2::bytes_to_hex_string(hash.begin(), hash.end());
	}

	// Includes
	code += "#include <cmath>\n";
	code += "#include <cstdio>\n";
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
		this->_add_core_simd_functions(code, op_type);
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
	code += "int get_op_type() { return " + std::to_string(static_cast<int>(op_type)) + "; }\n\n";

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

	// Write file
	std::ofstream outfile(folder + "/" + name + ".cpp");
	if (!outfile) {
		std::cout << "symx error: Cannot open the file " + folder + "/" + name << std::endl;
		exit(-1);
	}
	outfile << code;
	outfile.close();
}
bool symx::Compilation::is_valid() const
{
	return this->compiled_f != nullptr;
}
std::string symx::Compilation::_get_op_type_string(OpType op_type)
{
	switch (op_type)
	{
	case symx::OpType::Double:
		return "double";
	case symx::OpType::Float:
		return "float";
#ifdef SYMX_ENABLE_AVX
	case symx::OpType::SIMD2d:
		return "__m128d";
	case symx::OpType::SIMD4f:
		return "__m128";
	case symx::OpType::SIMD4d:
		return "__m256d";
	case symx::OpType::SIMD8f:
		return "__m256";
	case symx::OpType::SIMD8d:
		return "__m512d";
	case symx::OpType::SIMD16f:
		return "__m512";
#endif
	default:
		return "";
		break;
	}
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
			code += tab() + type + " " + idx(op.dst) + " = std::log(" + idx(op.a) + ");\n"; break;
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

	// Operations
	const std::string begin_line = "\t" + type + " ";
	for (auto& op : eval.ops) {
		switch (op.type)
		{
		case ExprType::Symbol: // Represents output
			code += "\tout[" + std::to_string(op.dst) + "] = " + idx(op.a) + ";\n"; break;
		case ExprType::ConstantFloat:
			code += begin_line + idx(op.dst) + " = set1(" + to_string_with_precision(op.constant, 20) + ");\n"; break;
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
void symx::Compilation::_add_core_simd_functions(std::string& code, OpType op_type)
{
#ifdef SYMX_ENABLE_AVX
	if (op_type != OpType::SIMD4d) {
		std::cout << "symx error: SIMD compilation only possible with type __m256d." << std::endl;
		exit(-1);
	}


	std::string functions;
	functions += "__m256d set1(double a) { return _mm256_set1_pd(a); }\n";
	functions += "__m256d add(__m256d& a, __m256d& b) { return _mm256_add_pd(a, b); }\n";
	functions += "__m256d sub(__m256d& a, __m256d& b) { return _mm256_sub_pd(a, b); }\n";
	functions += "__m256d mul(__m256d& a, __m256d& b) { return _mm256_mul_pd(a, b); }\n";
	functions += "__m256d inv(__m256d& a) { return _mm256_div_pd(_mm256_set1_pd(1.0), a); }\n";
	functions += "__m256d sqrt(__m256d& a) { return _mm256_sqrt_pd(a); }\n";

	functions += "__m256d pow(__m256d& a, __m256d& b) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* b_view = reinterpret_cast<double*>(&b); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::pow(a_view[i], b_view[i]); }; return s; }\n";
	functions += "__m256d log(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::log(a_view[i]); }; return s; }\n";
	functions += "__m256d exp(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::exp(a_view[i]); }; return s; }\n";
	functions += "__m256d sin(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::sin(a_view[i]); }; return s; }\n";
	functions += "__m256d cos(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::cos(a_view[i]); }; return s; }\n";
	functions += "__m256d tan(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::tan(a_view[i]); }; return s; }\n";
	functions += "__m256d asin(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::asin(a_view[i]); }; return s; }\n";
	functions += "__m256d acos(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::acos(a_view[i]); }; return s; }\n";
	functions += "__m256d atan(__m256d& a) { __m256d s; double* a_view = reinterpret_cast<double*>(&a); double* s_view = reinterpret_cast<double*>(&s); for (int i = 0; i < 4; i++) { s_view[i] = std::atan(a_view[i]); }; return s; }\n";

	switch (op_type)
	{
	case symx::OpType::Double:
		break;
	case symx::OpType::Float:
		break;
	case symx::OpType::SIMD2d:
		replace_all(functions, "_mm256_", "_mm_");
		replace_all(functions, "__m256d", "__m128d");
		break;
	case symx::OpType::SIMD4f:
		replace_all(functions, "_mm256_", "_mm_");
		replace_all(functions, "__m256d", "__m128");
		replace_all(functions, "_pd", "_ps");
		break;
	case symx::OpType::SIMD4d:
		break;
	case symx::OpType::SIMD8f:
		replace_all(functions, "_mm256_", "_mm256_");
		replace_all(functions, "__m256d", "__m256");
		replace_all(functions, "_pd", "_ps");
		break;
	case symx::OpType::SIMD8d:
		replace_all(functions, "_mm256_", "_mm512_");
		replace_all(functions, "__m256d", "__m512d");
		break;
	case symx::OpType::SIMD16f:
		replace_all(functions, "_mm256_", "_mm512_");
		replace_all(functions, "__m256d", "__m512");
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
