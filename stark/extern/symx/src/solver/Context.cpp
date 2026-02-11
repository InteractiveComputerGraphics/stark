#include "Context.h"

#include <iostream>
#include <omp.h>
#include "../macros.h"

symx::Context::Context()
{
    this->n_threads = omp_get_max_threads()/2;
    this->compilation_directory = symx::get_codegen_dir() + "/codegen";
    this->output = std::make_shared<OutputSink>();
    this->logger = std::make_shared<Logger>();
}

std::shared_ptr<symx::Context> symx::Context::create()
{
    return std::make_shared<Context>();
}
