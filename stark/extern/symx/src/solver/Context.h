#pragma once
#include <functional>
#include <string>
#include <memory>

#include "solver_utils.h"
#include "OutputSink.h"

namespace symx
{
    class Context
    {
    public:
        int32_t n_threads = -1;
        std::function<void(const std::string&)> print = nullptr;
        std::string compilation_directory = "";
        bool is_silent = false;

        OutputSink sink;  // Public: everyone prints through this

        Context();
        static std::shared_ptr<Context> create();
    };
    using spContext = std::shared_ptr<Context>;
} // namespace symx