#include "Output.h"

std::unique_ptr<stb::utils::Output> sym::Output::output = nullptr;
std::unique_ptr<stb::utils::Logger> sym::Output::logger = nullptr;

void stark::Output::initialize(const std::string& folder, const std::string& name, const int user_verbosity)
{
	if (!stb::utils::FileSystem::isDirectory(folder)) {
		stb::utils::FileSystem::makeDirs(folder);
	}

	output = std::make_unique<stb::utils::Output>();
	output->set_path(folder + "/" + name + "_console.txt");
	output->set_user_verbosity(user_verbosity);

	logger = std::make_unique<stb::utils::Logger>();
	logger->set_path(folder + "/" + name + "_logger.txt");
}

stb::utils::Output* sym::Output::get()
{
	return output.get();
}
