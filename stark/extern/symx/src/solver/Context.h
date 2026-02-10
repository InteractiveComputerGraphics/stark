#pragma once
#include <functional>
#include <string>
#include <memory>

#include "OutputSink.h"

namespace symx
{
    class Context
    {
    public:
        int32_t n_threads = -1;
        // std::function<void(const std::string&)> print = nullptr;
        std::string compilation_directory = "";
        spOutputSink output;

        Context();
        static std::shared_ptr<Context> create();
    };
    using spContext = std::shared_ptr<Context>;
} // namespace symx