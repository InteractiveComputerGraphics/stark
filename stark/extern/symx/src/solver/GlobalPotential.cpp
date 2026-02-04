#include "GlobalPotential.h"

using namespace symx;

// Helpers ====================================================================
void _check_unique_name(const std::vector<std::unique_ptr<Potential>>& potentials, const std::string& name)
{
    for (auto& potential : potentials) {
        if (potential->get_name() == name) {
            std::cout << "symx error: cannot add_energy with already existing name \"" + name + "\"" << std::endl;
            exit(-1);
        }
    }
}
// ============================================================================

std::shared_ptr<GlobalPotential> symx::GlobalPotential::create()
{
    return std::make_shared<GlobalPotential>();
}

void symx::GlobalPotential::add_potential(const std::string &name, spMWS<double> mws, Element &elem, DefFunc definition_callback)
{
    _check_unique_name(this->potentials, name);
    const Scalar P = definition_callback(*mws, elem);
    std::unique_ptr<Potential> energy_ptr = std::make_unique<Potential>(name, mws, P);
    this->potentials.push_back(std::move(energy_ptr));
}
void symx::GlobalPotential::add_potential(const std::string& name, spMWS<double> mws, Element& elem, DefWCondFunc definition_with_conditional_callback)
{
    _check_unique_name(this->potentials, name);
    auto [P, C] = definition_with_conditional_callback(*mws, elem);
    std::unique_ptr<Potential> energy_ptr = std::make_unique<Potential>(name, mws, P, C);
    this->potentials.push_back(std::move(energy_ptr));
}
void symx::GlobalPotential::add_potential(const std::string& name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t stride, 
    std::function<Scalar(MappedWorkspace<double>&, Element&)> definition_callback)
{
    auto [mws, elem] = MappedWorkspace<double>::create(data, n_elements, stride);
    this->add_potential(name, mws, elem, definition_callback);
}
void symx::GlobalPotential::add_potential(const std::string& name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t stride,
    std::function<std::pair<Scalar, Scalar>(MappedWorkspace<double>&, Element&)> definition_with_conditional_callback)
{
    auto [mws, elem] = MappedWorkspace<double>::create(data, n_elements, stride);
    this->add_potential(name, mws, elem, definition_with_conditional_callback);
}
void symx::GlobalPotential::add_potential(const std::string &name, const std::vector<int32_t> &arr, const int32_t n_items_per_element, DefFunc definition_callback)
{
    auto [mws, elem] = MappedWorkspace<double>::create(arr, n_items_per_element);
    this->add_potential(name, mws, elem, definition_callback);
}
void symx::GlobalPotential::add_potential(const std::string& name, const std::vector<int32_t>& arr, const int32_t n_items_per_element, DefWCondFunc definition_with_conditional_callback)
{
    auto [mws, elem] = MappedWorkspace<double>::create(arr, n_items_per_element);
    this->add_potential(name, mws, elem, definition_with_conditional_callback);
}

void symx::GlobalPotential::add_dof(std::function<double *()> data, std::function<int32_t()> flat_size, const std::string &name)
{
    this->dof_maps.emplace_back(data, flat_size, /*stride=*/ 1, /*connectivity_index=*/-1, /*first_symbol_idx=*/-1);
}
void symx::GlobalPotential::add_dof(EigenMatrixRM<double> &arr, const std::string &name)
{
    this->add_dof(
        [&arr]() { return arr.data(); }, 
        [&arr]() { return (int32_t)(arr.size()); }, 
        name);
}
void symx::GlobalPotential::add_dof(double &scalar, const std::string &name)
{
    this->add_dof(
        [&scalar]() { return &scalar; }, 
        []() { return 1; }, 
        name);
}

const std::vector<int32_t> &symx::GlobalPotential::get_dofs_offsets()
{
    this->dof_offsets.clear();
    this->dof_offsets.push_back(0);
    for (const DataMap<double>& dof_map : this->dof_maps) {
        this->dof_offsets.push_back(this->dof_offsets.back() + (int32_t)dof_map.size());
    }
    return this->dof_offsets;
}

int32_t symx::GlobalPotential::get_n_dof_sets() const
{
    return (int32_t)this->dof_maps.size();
}

int32_t symx::GlobalPotential::get_n_dofs(int32_t set_i) const
{
    return (int32_t)this->dof_maps[set_i].size();
}

int32_t symx::GlobalPotential::get_total_n_dofs() const
{
    int32_t total_n_dofs = 0;
    for (const DataMap<double>& dof_map : this->dof_maps) {
        total_n_dofs += (int32_t)dof_map.size();
    }
    return total_n_dofs;
}
void symx::GlobalPotential::get_dofs(double *u) const
{
    int32_t offset = 0;
    for (const DataMap<double>& dof_map : this->dof_maps) {
        const int32_t n_dofs = (int32_t)dof_map.size();
        std::memcpy(u + offset, dof_map.data(), n_dofs * sizeof(double));
        offset += n_dofs;
    }
}
void symx::GlobalPotential::apply_dof_increment(const double *du)
{
    int32_t offset = 0;
    for (const DataMap<double>& dof_map : this->dof_maps) {
        const int32_t n_dofs = (int32_t)dof_map.size();
        double* dof_data = dof_map.data();
        for (int32_t i = 0; i < n_dofs; i++) {
            dof_data[i] += du[offset + i];
        }
        offset += n_dofs;
    }
}
void symx::GlobalPotential::set_dofs(const double *u)
{
    int32_t offset = 0;
    for (const DataMap<double>& dof_map : this->dof_maps) {
        const int32_t n_dofs = (int32_t)dof_map.size();
        double* dof_data = dof_map.data();
        std::memcpy(dof_data, u + offset, n_dofs * sizeof(double));
        offset += n_dofs;
    }
}

const std::vector<std::unique_ptr<Potential>> &symx::GlobalPotential::get_potentials() const
{
    return this->potentials;
}

const std::vector<DataMap<double>> &symx::GlobalPotential::get_dof_maps() const
{
    return this->dof_maps;
}

int32_t symx::GlobalPotential::get_n_potentials() const
{
    return (int32_t)this->potentials.size();
}
