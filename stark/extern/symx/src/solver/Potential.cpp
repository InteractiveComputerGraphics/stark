#include "Potential.h"

using namespace symx;
Potential::Potential(const std::string& name, spMWS<double> mws, const Scalar& expr, const Scalar& cond)
    : name(name), mws(mws), expr(std::make_unique<Scalar>(expr)), cond(std::make_unique<Scalar>(cond))
{
}

Potential::Potential(const std::string& name, spMWS<double> mws, const Scalar& expr)
    : name(name), mws(mws), expr(std::make_unique<Scalar>(expr))
{
}

const std::string& Potential::get_name() const
{
    return this->name;
}
std::string symx::Potential::get_checksum(const std::string &pre_hash) const
{
    return this->mws->get_workspace().get_expression_graph().get_checksum(pre_hash);
}
Scalar Potential::get_expression() const
{
    return *(this->expr);
}
bool Potential::has_conditional() const
{
    return this->cond != nullptr;
}
Scalar Potential::get_condition() const
{
    return *(this->cond);
}

std::vector<Scalar> Potential::get_symbols(const DataMap<double>& data_map) const
{
    return this->mws->get_symbols(data_map);
}
