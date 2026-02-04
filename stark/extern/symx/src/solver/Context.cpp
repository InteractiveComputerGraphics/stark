#include "Context.h"

#include <iostream>
#include <omp.h>
#include "../macros.h"

symx::Context::Context()
{
    this->n_threads = omp_get_max_threads()/2;
    this->print = [](const std::string& s) { std::cout << s << std::flush; };
    this->compilation_directory = symx::get_codegen_dir() + "/codegen";
}

std::shared_ptr<symx::Context> symx::Context::create()
{
    return std::make_shared<Context>();
}
