#pragma once
#include <string>
#include <memory>


namespace stark
{
	class Output
	{
	public:
		static std::unique_ptr<stb::utils::Output> output;
		static std::unique_ptr<stb::utils::Logger> logger;

		static void initialize(const std::string& folder, const std::string& filename, const int user_verbosity);
		static stb::utils::Output* get();
	};
}
