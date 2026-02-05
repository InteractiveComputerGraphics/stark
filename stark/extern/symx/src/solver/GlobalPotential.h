#pragma once
#include "Potential.h"


namespace symx
{
    class GlobalPotential
    {
    public:
        /* Definitions */
        using DefFunc = std::function<Scalar(MappedWorkspace<double>&, Element&)>;
        using DefWCondFunc = std::function<std::pair<Scalar, Scalar>(MappedWorkspace<double>&, Element&)>;

    private:
        /* Fields */
        std::vector<std::unique_ptr<Potential>> potentials;
        std::vector<DataMap<double>> dof_maps; // non-const because we may want to modify dofs. No connectivity, stride or idx needed here.
        std::vector<int32_t> dof_offsets;

    public:
        /* Methods */
        GlobalPotential() = default;
        ~GlobalPotential() = default;
        static std::shared_ptr<GlobalPotential> create();

        // Add Potential
        //// Base methods
        void add_potential(const std::string& name, spMWS<double> mws, Element& elem, DefFunc definition_callback);
        void add_potential(const std::string& name, spMWS<double> mws, Element& elem, DefWCondFunc definition_with_conditional_callback);

        //// Convenience methods
		void add_potential(const std::string& name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t stride, DefFunc definition_callback);
		void add_potential(const std::string& name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t stride, DefWCondFunc definition_with_conditional_callback);
        void add_potential(const std::string& name, const std::vector<int32_t>& arr, const int32_t n_items_per_element, DefFunc definition_callback);
        void add_potential(const std::string& name, const std::vector<int32_t>& arr, const int32_t n_items_per_element, DefWCondFunc definition_with_conditional_callback);
		template<std::size_t N>
		void add_potential(const std::string& name, const std::vector<std::array<int32_t, N>>& arr, DefFunc definition_callback);
		template<std::size_t N>
		void add_potential(const std::string& name, const std::vector<std::array<int32_t, N>>& arr, DefWCondFunc definition_with_conditional_callback);
		template<std::size_t N>
		void add_potential(const std::string& name, const LabelledConnectivity<N>& conn, DefFunc definition_callback);
		template<std::size_t N>
		void add_potential(const std::string& name, const LabelledConnectivity<N>& conn, DefWCondFunc definition_with_conditional_callback);

        // Add DoFs
        void add_dof(std::function<std::uintptr_t()> id, std::function<double*()> data, std::function<int32_t()> flat_size, const std::string& name = "");
        void add_dof(EigenMatrixRM<double>& arr, const std::string& name = "");
        template<typename STATIC_VECTOR>
        void add_dof(std::vector<STATIC_VECTOR>& arr, const std::string& name = "");
        template<typename DYNAMIC_VECTOR>
        void add_dof(DYNAMIC_VECTOR& arr, const std::string& name = "");
        void add_dof(double& scalar, const std::string& name = "" );

        // DoF operations
        const std::vector<int32_t>& get_dofs_offsets();
        int32_t get_n_dof_sets() const;
        int32_t get_n_dofs(int32_t set_i) const;
        int32_t get_total_n_dofs() const;
		void get_dofs(double* u) const;
		void apply_dof_increment(const double* du);
		void set_dofs(const double* u);

        // Misc
        const std::vector<std::unique_ptr<Potential>>& get_potentials() const;
        const std::vector<DataMap<double>>& get_dof_maps() const;
        int32_t get_n_potentials() const;
    };


    // ============================================================================
    // Implementations
    // ============================================================================
    template<std::size_t N>
    inline void GlobalPotential::add_potential(const std::string& name, const std::vector<std::array<int32_t, N>>& arr, DefFunc definition_callback)
    {
        auto [mws, elem] = MappedWorkspace<double>::create(arr);
        this->add_potential(name, mws, elem, definition_callback);
    }
    template<std::size_t N>
    inline void GlobalPotential::add_potential(const std::string& name, const std::vector<std::array<int32_t, N>>& arr, DefWCondFunc definition_with_conditional_callback)
    {
        auto [mws, elem] = MappedWorkspace<double>::create(arr);
        this->add_potential(name, mws, elem, definition_with_conditional_callback);
    }
    template<std::size_t N>
    inline void GlobalPotential::add_potential(const std::string& name, const LabelledConnectivity<N>& conn, DefFunc definition_callback)
    {
        auto [mws, elem] = MappedWorkspace<double>::create(conn);
        this->add_potential(name, mws, elem, definition_callback);
    }
    template<std::size_t N>
    inline void GlobalPotential::add_potential(const std::string& name, const LabelledConnectivity<N>& conn, DefWCondFunc definition_with_conditional_callback)
    {
        auto [mws, elem] = MappedWorkspace<double>::create(conn);
        this->add_potential(name, mws, elem, definition_with_conditional_callback);
    }

    template<typename STATIC_VECTOR>
    void symx::GlobalPotential::add_dof(std::vector<STATIC_VECTOR>& arr, const std::string& name)
    {
        constexpr int32_t stride = sizeof(STATIC_VECTOR) / sizeof(double);
        this->add_dof(
            [&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
            [&arr]() { return arr[0].data(); }, 
            [&arr]() { return (int32_t)(arr.size()*stride); }, 
            name);
    }
    template<typename DYNAMIC_VECTOR>
    void symx::GlobalPotential::add_dof(DYNAMIC_VECTOR& arr, const std::string& name)
    {
        this->add_dof(
            [&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
            [&arr]() { return arr.data(); }, 
            [&arr]() { return (int32_t)(arr.size()); }, 
            name);
    }

    using spGlobalPotential = std::shared_ptr<GlobalPotential>;
} // namespace symx
