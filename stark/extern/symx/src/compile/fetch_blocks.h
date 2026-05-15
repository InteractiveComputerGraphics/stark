#pragma once
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "../macros.h"

namespace symx::detail 
{
    template<std::size_t BYTES, std::size_t LINES>
    SYMX_FORCE_INLINE void fetch_block_fixed_size(
        char* SYMX_RESTRICT input,
        const char* SYMX_RESTRICT array_begin,
        const int32_t* SYMX_RESTRICT group_elem_conn,
        int32_t conn_stride,
        int32_t connectivity_index,
        int32_t stride,
        int32_t offset,
        int32_t mult,
        int32_t input_byte_stride)
    {
        SYMX_IVDEP
        for (size_t line_i = 0; line_i < LINES; ++line_i) {
            const int32_t* elem_conn = &group_elem_conn[line_i * conn_stride];
            const int32_t  loc_idx   = elem_conn[connectivity_index];
            const int32_t  base_idx  = loc_idx * stride * mult;
            char*       SYMX_RESTRICT dst = input + line_i * input_byte_stride + offset;
            const char* SYMX_RESTRICT src = array_begin + base_idx;
            std::memcpy(dst, src, BYTES);
        }
    }

    template<std::size_t LINES>
    SYMX_FORCE_INLINE void fetch_block_dynamic(
        char* SYMX_RESTRICT input,
        const char* SYMX_RESTRICT array_begin,
        const int32_t* SYMX_RESTRICT group_elem_conn,
        int32_t conn_stride,
        int32_t connectivity_index,
        int32_t stride_in_bytes,
        int32_t offset,
        int32_t mult,
        int32_t input_byte_stride)
    {
        SYMX_IVDEP
        for (size_t line_i = 0; line_i < LINES; ++line_i) {
            const int32_t* elem_conn = &group_elem_conn[line_i * conn_stride];
            const int32_t  loc_idx   = elem_conn[connectivity_index];
            const int32_t  base_idx  = loc_idx * mult;
            char*       SYMX_RESTRICT dst = input + line_i * input_byte_stride + offset;
            const char* SYMX_RESTRICT src = array_begin + base_idx * stride_in_bytes;
            std::memcpy(dst, src, stride_in_bytes);
        }
    }

} // namespace symx::simd
