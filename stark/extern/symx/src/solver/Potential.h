#pragma once

#include "../compile/MappedWorkspace.h"
#include "../compile/CompiledInLoop.h"

namespace symx
{
    class Potential
    {
    public:
        /* Fields */
        std::string name = "";
        spMWS<double> mws;
        std::unique_ptr<Scalar> expr = nullptr;
        std::unique_ptr<Scalar> cond = nullptr;

        /* Methods */
        Potential(const std::string& name, spMWS<double> mws, const Scalar& expr, const Scalar& cond);
        Potential(const std::string& name, spMWS<double> mws, const Scalar& expr);
        const std::string& get_name() const;
        std::string get_checksum(const std::string& pre_hash = "") const;

        Scalar get_expression() const;
        bool has_conditional() const;
        Scalar get_condition() const;

        std::vector<Scalar> get_symbols(const DataMap<double>& data_map) const;
    };
} // namespace symx
