#pragma once
#include <functional>
#include <string>
#include <memory>

#include "OutputSink.h"
#include "Logger.h"

namespace symx
{
    class Context
    {
    public:
        int32_t n_threads = -1;
        std::string compilation_directory = "";
        spOutputSink output;
        spLogger logger;

        Context();
        static std::shared_ptr<Context> create();
    };
    using spContext = std::shared_ptr<Context>;
} // namespace symx