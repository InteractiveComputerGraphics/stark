#pragma once
#include <array>

#include "IntervalVector.h"


namespace stark::models
{
    template<std::size_t N>
    class IntervalConnectivity : 
        public IntervalVector<std::array<int, N>>
    {
    private:
        using T = std::array<int, N>;
    public:

        // Disable these 
        int append(const std::vector<T>& input) = delete;
        void update(const int set_id, const std::vector<T>& input) = delete;

        template<typename T_It>
        int append(const T_It begin, const T_It end) = delete;
        template<typename T_It>
        void update(const int set_id, const T_It begin, const T_It end) = delete;

        // Methods with indices update
        int append(const std::vector<T>& input, const int offset);
        void update(const int set_id, const std::vector<T>& input, const int old_offset, const int new_offset);

        template<typename T_It>
        int append(const T_It begin, const T_It end, const int offset);
        template<typename T_It>
        void update(const int set_id, const T_It begin, const T_It end, const int old_offset, const int new_offset);

    };




    // DEFINITIONS ===========================================================================
    template<std::size_t N>
    inline int IntervalConnectivity<N>::append(const std::vector<T>& input, const int offset)
    {
        return this->append(input.begin(), input.end(), offset);
    }
    template<std::size_t N>
    inline void IntervalConnectivity<N>::update(const int set_id, const std::vector<T>& input, const int old_offset, const int new_offset)
    {
        this->update(set_id, input.begin(), input.end(), old_offset, new_offset);
    }
    template<std::size_t N>
    template<typename T_It>
    inline int IntervalConnectivity<N>::append(const T_It begin, const T_It end, const int offset)
    {
        const int set_id = IntervalVector<T>::append(begin, end);
        for (int i = this->get_begin(set_id); i < this->get_end(set_id); i++) {
            for (size_t j = 0; j < N; j++) {
                this->get(i)[j] += offset;
            }
        }
        return set_id;
    }
    template<std::size_t N>
    template<typename T_It>
    inline void IntervalConnectivity<N>::update(const int set_id, const T_It begin, const T_It end, const int old_offset, const int new_offset)
    {
        IntervalVector<T>::update(set_id, begin, end);
        for (int i = this->get_begin(set_id); i < this->get_end(set_id); i++) {
            for (size_t j = 0; j < N; j++) {
                this->get(i)[j] += new_offset;
            }
        }
        const int n_diff = new_offset - old_offset;
        for (int i = this->get_end(set_id); i < this->size(); i++) {
            for (size_t j = 0; j < N; j++) {
                this->get(i)[j] += n_diff;
            }
        }
    }
}