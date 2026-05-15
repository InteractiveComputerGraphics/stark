#pragma once

#include "../compile/MappedWorkspace.h"
#include "../compile/CompiledInLoop.h"

namespace symx
{
    /*
     * Wraps a single symbolic energy expression defined in a MappedWorkspace.
     * Stores the scalar expression (and optional condition for conditional evaluation)
     * produced by the user's definition callback in GlobalPotential::add_potential().
     */
    class Potential
    {
    public:
        /* Methods */
        Potential(const std::string& name, spMWS<double> mws, const Scalar& expr, const Scalar& cond);
        Potential(const std::string& name, spMWS<double> mws, const Scalar& expr);
        const std::string& get_name() const;
        spMWS<double> get_mws() const;
        std::string get_checksum(const std::string& pre_hash = "") const;

        Scalar get_expression() const;
        bool has_conditional() const;
        Scalar get_condition() const;

        std::vector<Scalar> get_symbols(const DataMap<double>& data_map) const;

    private:
        /* Fields */
        std::string name = "";
        spMWS<double> mws;
        std::unique_ptr<Scalar> expr = nullptr;
        std::unique_ptr<Scalar> cond = nullptr;
    };
} // namespace symx
