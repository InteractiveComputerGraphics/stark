#pragma once
#include "CompiledInLoop.h"

/*
================================================================================
                     DATA FLOW AND BUFFER MANAGEMENT
================================================================================

This file implements the main computation loop for compiled symbolic functions.
The key design constraints are:

1. SIMD LAYOUT TRANSFORMATION
   - Input data arrives in AoS (Array of Structures) format: [elem0: x,y,z], [elem1: x,y,z], ...
   - SIMD requires SoA (Structure of Arrays) format: [x0,x1,x2...], [y0,y1,y2...], ...
   - After computation, output must be converted back to AoS for the callback

2. FALSE SHARING PREVENTION
   - Thread buffers use std::unique_ptr to ensure independent heap allocations
   - This prevents false sharing when multiple threads access their buffers

3. BUFFER ROLES (4 buffers per thread, indexed 0-3):

   ┌─────────────┬──────────────────────────────────────────────────────────────┐
   │ Buffer      │ Role                                                         │
   ├─────────────┼──────────────────────────────────────────────────────────────┤
   │ buf[0]      │ INPUT_AOS:  Fetched input data (always AoS format)           │
   │ buf[1]      │ INPUT_SOA:  Shuffled input for SIMD / scratch for summation  │
   │ buf[2]      │ OUTPUT_SOA: Computation output / accumulator for summation   │
   │ buf[3]      │ OUTPUT_AOS: Final AoS output for callback (always used)      │
   └─────────────┴──────────────────────────────────────────────────────────────┘

4. DATA FLOW BY MODE:

   NON-SUMMATION:
   ┌─────────────────────────────────────────────────────────────────────────────┐
   │ SIMD:   fetch→buf[0] → shuffle→buf[1] → run→buf[2] → unshuffle→buf[3]       │
   │ Scalar: fetch→buf[0] ─────────────────→ run→buf[3]                          │
   └─────────────────────────────────────────────────────────────────────────────┘

   WITH SUMMATION (multiple iterations accumulated):
   ┌─────────────────────────────────────────────────────────────────────────────┐
   │ SIMD:   fetch→buf[0] → shuffle→buf[1] → for each iter:                      │
   │                                           update buf[1], run→buf[3](scratch)│
   │                                           buf[2] += buf[3]                  │
   │                                         → unshuffle buf[2]→buf[3]           │
   │                                                                             │
   │ Scalar: fetch→buf[0] → for each iter:                                       │
   │                          update buf[0], run→buf[1](scratch)                 │
   │                          buf[3] += buf[1]                                   │
   │         → output already in buf[3]                                          │
   └─────────────────────────────────────────────────────────────────────────────┘

5. CASTING (if needed)
   - Input data is cast from INPUT_FLOAT to COMPILED_FLOAT before computation
   - Output data is cast from COMPILED_FLOAT to OUTPUT_FLOAT before callback

   Key invariant: Callback ALWAYS receives data from buf[3] in AoS format.

================================================================================
*/

namespace symx
{
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename CALLBACK_T>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run(
		int32_t n_threads, 
		CALLBACK_T callback,
		const std::vector<uint8_t>& is_element_active)
	{

		// Buffer indices for clarity
		enum BufferIndex : int32_t {
			BUF_INPUT_AOS  = 0,  // Fetched input (AoS)
			BUF_INPUT_SOA  = 1,  // Shuffled input (SoA) or scratch
			BUF_OUTPUT_SOA = 2,  // Computation output or accumulator
			BUF_OUTPUT_AOS = 3   // Final output for callback (AoS)
		};
		constexpr int32_t N_BUFFERS_PER_THREAD = 4;

		// Parameters
		constexpr int32_t LINES = 8;  // Always load 8 lines of SIMD data to hide memory latency
		constexpr int32_t PADDING = 8;  // Fill every buffer to 8 for safe SIMD operations

		// Validate SIMD constraints
		constexpr int N_SIMD = get_n_items_in_simd<COMPILED_FLOAT>();
		static_assert(LINES % N_SIMD == 0, "LINES must be divisible by N_SIMD for proper SIMD processing");

		// =================================================================================================
		// ==========================================  PREPARATION  ========================================
		// =================================================================================================

		// Exit if there are no elements in the connectivity
		if (this->mws->conn.n_elements() == 0) {
			return;
		}

        // Validate data
        if (symx::get_check_mode_ON()) {
            this->mws->verify_data_maps();
            this->mws->verify_no_out_of_bounds();
        }

        // Check n_threads
		if (n_threads <= 0) {
			std::cout << "symx error: invalid n_threads (" << n_threads << ") in CompiledInLoop.run_new() for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		// Common stuff
		using UNDERLYING_FLOAT = UNDERLYING_TYPE<COMPILED_FLOAT>;
		FloatType input_float_type = get_float_type_as_enum<INPUT_FLOAT>();
		FloatType compiled_float_type = get_float_type_as_enum<COMPILED_FLOAT>();
		FloatType underlying_float_type = get_float_type_as_enum<UNDERLYING_FLOAT>();
		FloatType output_float_type = get_float_type_as_enum<OUTPUT_FLOAT>();
		const bool check_mode_ON = get_check_mode_ON();
		const int32_t n_inputs = this->compilation.get_n_inputs();
		const int32_t n_outputs = this->compilation.get_n_outputs();
		const int32_t padded_n_inputs = detail::round_up(n_inputs, PADDING);
		const int32_t padded_n_outputs = detail::round_up(n_outputs, PADDING);
		const bool has_summation = this->mws->has_summation();
		const int32_t* connectivity_data = this->mws->conn.data();
		const int32_t n_elements = this->mws->conn.n_elements();
		const int32_t connectivity_stride = this->mws->conn.stride;
		bool has_conditional_evaluation = is_element_active.size() > 0;
        auto symbol_number_as_str = [&](const int i) { return std::to_string(i) + "/" + std::to_string(n_inputs); };

		// Nothing to do
		if (n_elements == 0) { return; } // Early exit

		// Condition zero is no condition. Otherwise, we expect a condition vector of size n_elements
		if (is_element_active.size() > 0 && (int32_t)is_element_active.size() != n_elements) {
			std::cout << "symx error: invalid size of is_element_active (" << is_element_active.size() << ") in CompiledInLoop.run_new() for instance with name \"" + this->name + "\". Expected size is " << n_elements << "." << std::endl;
			exit(-1);
		}


		// Get the compiled function
		void(*f)(COMPILED_FLOAT*, COMPILED_FLOAT*) = this->compilation.template get_f<COMPILED_FLOAT>();

		// -----------------------------------------------------------------------------------------
		// Buffer allocation - using unique_ptr to ensure independent heap allocations per thread,
		// preventing false sharing (cache line contention) between threads.
		// -----------------------------------------------------------------------------------------
		const int32_t buffer_size = LINES * std::max(padded_n_inputs, padded_n_outputs);
		const int32_t buffer_size_bytes = buffer_size * sizeof(double);  // Size for doubles (largest) (stencil-dependent allocation, not global)
		const int32_t total_buffers = n_threads * N_BUFFERS_PER_THREAD;
		
		if ((int32_t)this->thread_buf.size() < total_buffers) {
			this->thread_buf.resize(total_buffers);
			for (auto& buf : this->thread_buf) {
				buf = std::make_unique<simd::avector<char, 64>>();
			}
		}
		for (auto& buf : this->thread_buf) {
			buf->resize(buffer_size_bytes);
		}
		
		this->thread_elem_buffer.resize(n_threads);
		for (std::vector<int32_t>& elem_buffer : this->thread_elem_buffer) {
			elem_buffer.resize(LINES*connectivity_stride);
		}

		// Helper lambda to access buffers with clear semantics
		auto get_buf = [&](int32_t tid, BufferIndex idx) -> char* {
			return this->thread_buf[tid * N_BUFFERS_PER_THREAD + idx]->data();
		};

		// Block fetch setup
		this->block_fetches.clear();
		constexpr uint32_t n_bytes_per_scalar = sizeof(INPUT_FLOAT);
		const int32_t n_blocks = (int32_t)this->mws->maps.size();
		for (int32_t block_i = 0; block_i < n_blocks; block_i++){
			const auto& map = this->mws->maps[block_i];
            BlockFetch block;
            block.array_begin = reinterpret_cast<const char*>(map.data());
            block.stride_in_bytes = map.stride*n_bytes_per_scalar;
            block.input_byte_offset = map.first_symbol_idx*n_bytes_per_scalar;

			if (map.connectivity_index == -1) { // Fixed value
				block.connectivity_index = 0; // Index the first but we won't use it
				block.mult = 0;
			} 
			else {  // Indexed value
				block.connectivity_index = map.connectivity_index;
				block.mult = 1;
			}
            
			this->block_fetches.push_back(block);
        }

		// No coloring - all elements in one color bin
		if (!this->use_coloring) {
			this->color_bins.resize(1);
			if ((int32_t)this->color_bins[0].size() != n_elements) {
				this->color_bins[0].resize(n_elements);
				std::iota(this->color_bins[0].begin(), this->color_bins[0].end(), 0);
			}
		}
		const int n_colors = (int)this->color_bins.size();


		// =================================================================================================
		// ==========================================  MAIN LOOP  ==========================================
		// =================================================================================================

		const std::vector<int32_t>* active_color_elements;

		// Loop over colors
		for (int color_i = 0; color_i < n_colors; color_i++) {
			const std::vector<int32_t>& color_bin = this->color_bins[color_i];
			const int n_elements_in_color = (int)color_bin.size();

			// Find active elements
			if (has_conditional_evaluation) {
				this->active_element_indices.clear();
				this->active_element_indices.reserve(n_elements);
				for (int i = 0; i < n_elements_in_color; i++) {
					if (is_element_active[color_bin[i]]) {
						this->active_element_indices.push_back(color_bin[i]);
					}
				}
				active_color_elements = &this->active_element_indices;
			} 
			else {
				active_color_elements = &color_bin;
			}

			// Padding for loading multiple lines
			const int n_active_color_elements = (int)active_color_elements->size();
			if (n_active_color_elements == 0) { continue; } // Skip empty color bins
			const int n_elem_groups = detail::ceil_div(n_active_color_elements, LINES);
			const int padded_n_elem_groups = n_elem_groups * LINES;
			
			#pragma omp parallel for schedule(static) num_threads(n_threads) if(n_elem_groups > 2*n_threads)
			for (int group_i = 0; group_i < n_elem_groups; group_i++) {
				const int thread_id = omp_get_thread_num();
				const int32_t input_size = LINES * padded_n_inputs;

				// Get buffer pointers for this thread (see BufferIndex enum for roles)
				char* buf_input_aos  = get_buf(thread_id, BUF_INPUT_AOS);
				char* buf_input_soa  = get_buf(thread_id, BUF_INPUT_SOA);
				char* buf_output_soa = get_buf(thread_id, BUF_OUTPUT_SOA);
				char* buf_output_aos = get_buf(thread_id, BUF_OUTPUT_AOS);

				// Gather elements
				std::array<uint8_t, LINES> is_pad_element;
				std::array<int32_t, LINES> element_indices;
				std::vector<int32_t>& group_elem_conn = this->thread_elem_buffer[thread_id];
				for (int line_i = 0; line_i < LINES; line_i++) {
					int color_idx = group_i * LINES + line_i;
					is_pad_element[line_i] = (uint8_t)(color_idx >= n_active_color_elements);
					color_idx = std::min(color_idx, n_active_color_elements - 1); // Use the last element for padding
					const int elem_idx = (*active_color_elements)[color_idx];
					element_indices[line_i] = elem_idx;
					const int32_t* elem_conn = connectivity_data + connectivity_stride * elem_idx;
					for (int j = 0; j < connectivity_stride; j++) {
						group_elem_conn[line_i*connectivity_stride + j] = elem_conn[j];
					}
				}

				// ========== STAGE 1: FETCH INPUT → buf_input_aos (AoS format) ==========
				{
					if (!check_mode_ON) {
						char* SYMX_RESTRICT input = buf_input_aos;
						const int32_t input_byte_stride = padded_n_inputs * sizeof(INPUT_FLOAT);
						for (const BlockFetch& block : this->block_fetches) {
							const char* SYMX_RESTRICT array_begin = block.array_begin;
							const int32_t stride_in_bytes = block.stride_in_bytes;

							switch (stride_in_bytes) {
								case 4:  detail::fetch_block_fixed_size<4,  LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 8:  detail::fetch_block_fixed_size<8,  LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 12: detail::fetch_block_fixed_size<12, LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 16: detail::fetch_block_fixed_size<16, LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 24: detail::fetch_block_fixed_size<24, LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 32: detail::fetch_block_fixed_size<32, LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 36: detail::fetch_block_fixed_size<36, LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								case 72: detail::fetch_block_fixed_size<72, LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, block.stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
								default: detail::fetch_block_dynamic<LINES>(input, array_begin, group_elem_conn.data(), connectivity_stride, block.connectivity_index, stride_in_bytes, block.input_byte_offset, block.mult, input_byte_stride); break;
							}
						}
					}
					else {
						// Scalar fetch with bounds checking
						INPUT_FLOAT* input = reinterpret_cast<INPUT_FLOAT*>(buf_input_aos);
						for (const BlockFetch& block : this->block_fetches) {
							const INPUT_FLOAT* array_begin = reinterpret_cast<const INPUT_FLOAT*>(block.array_begin);
							const int32_t stride = block.stride_in_bytes/n_bytes_per_scalar;
							const int32_t input_index = block.input_byte_offset / n_bytes_per_scalar;
							for (int line_i = 0; line_i < LINES; line_i++) {
								const int32_t* elem_conn = &group_elem_conn[line_i * connectivity_stride];
								const int32_t loc_idx = elem_conn[block.connectivity_index];
								for (int32_t offset = 0; offset < stride; offset++) {
									const int32_t value_idx = loc_idx * stride * block.mult + offset;
									input[line_i * padded_n_inputs + input_index + offset] = array_begin[value_idx];
								}
							}
						}
					}

					// Check for NaNs in input
					if (check_mode_ON) {
						INPUT_FLOAT* input = reinterpret_cast<INPUT_FLOAT*>(buf_input_aos);
						int32_t nan_index = detail::find_nan(input, LINES * padded_n_inputs);
						if (nan_index != -1) {
							const int input_i = nan_index % padded_n_inputs;
							std::cout << "symx error: NaN in input " << symbol_number_as_str(input_i) << " for \"" << this->name << "\"" << std::endl;
							exit(-1);
						}
					}
				}

				// ========== STAGE 2: CAST INPUT (if needed) ==========
				if (input_float_type == FloatType::Double && underlying_float_type == FloatType::Float) {
					detail::cast_inplace_f64_to_f32(buf_input_aos, input_size);
				}
				else if (input_float_type == FloatType::Float && underlying_float_type == FloatType::Double) {
					detail::cast_inplace_f32_to_f64(buf_input_aos, input_size);
				}

				// ========== STAGE 3: SHUFFLE AoS→SoA (SIMD only) → buf_input_soa ==========
				const bool is_simd = (N_SIMD > 1);
#ifdef SYMX_ENABLE_AVX2
				if (compiled_float_type == FloatType::SIMD8f) {
					detail::aos_to_soa_f32(
						reinterpret_cast<float*>(buf_input_aos),
						reinterpret_cast<float*>(buf_input_soa),
						padded_n_inputs);
				}
				else if (compiled_float_type == FloatType::SIMD4d) {
					double* src = reinterpret_cast<double*>(buf_input_aos);
					double* dst = reinterpret_cast<double*>(buf_input_soa);
					detail::aos_to_soa_f64(src, dst, padded_n_inputs);  // First 4 lines
					const int jump = 4*padded_n_inputs;
					detail::aos_to_soa_f64(src + jump, dst + jump, padded_n_inputs);  // Next 4 lines
				}
#endif
				// For scalar: data stays in buf_input_aos, no shuffle needed

				// ========== STAGE 4: COMPUTATION ==========
				// Determine computation input/output buffers based on mode:
				//   SIMD:   input from buf_input_soa,  output to buf_output_soa
				//   Scalar: input from buf_input_aos,  output to buf_output_aos
				UNDERLYING_FLOAT* comp_input  = is_simd ? reinterpret_cast<UNDERLYING_FLOAT*>(buf_input_soa)
				                                        : reinterpret_cast<UNDERLYING_FLOAT*>(buf_input_aos);
				UNDERLYING_FLOAT* comp_output = is_simd ? reinterpret_cast<UNDERLYING_FLOAT*>(buf_output_soa)
				                                        : reinterpret_cast<UNDERLYING_FLOAT*>(buf_output_aos);

				if (!has_summation) {
					// Direct computation: input → output
					const int n_runs = LINES/N_SIMD;
					const int input_stride = padded_n_inputs * N_SIMD;
					const int output_stride = padded_n_outputs * N_SIMD;
					for (int i = 0; i < n_runs; i++) {
						if (is_pad_element[i * N_SIMD]) { break; }
						f(reinterpret_cast<COMPILED_FLOAT*>(comp_input  + i * input_stride),
						  reinterpret_cast<COMPILED_FLOAT*>(comp_output + i * output_stride));
					}
				}
				else {
					// Summation: accumulate multiple iterations
					// Scratch buffer for each iteration's output:
					//   SIMD:   buf_output_aos (reused, will be overwritten during unshuffle)
					//   Scalar: buf_input_soa (unused in scalar path)
					UNDERLYING_FLOAT* scratch = is_simd ? reinterpret_cast<UNDERLYING_FLOAT*>(buf_output_aos)
					                                    : reinterpret_cast<UNDERLYING_FLOAT*>(buf_input_soa);
					UNDERLYING_FLOAT* accum   = comp_output;  // Accumulator

					// Zero accumulator
					for (int i = 0; i < padded_n_outputs * LINES; i++) {
						accum[i] = static_cast<UNDERLYING_FLOAT>(0.0);
					}

					const Summation<INPUT_FLOAT>& summation = this->mws->summation;
					for (int sum_it = 0; sum_it < summation.n_iterations; sum_it++) {
						// Update summation data in the SoA input buffer
						const int32_t begin = summation.first_symbol_idx;
						if (N_SIMD == 1) {
							for (int32_t line_i = 0; line_i < LINES; line_i++) {
								if (is_pad_element[line_i]) { break; }
								UNDERLYING_FLOAT* line_input = comp_input + line_i * padded_n_inputs;
								for (int i = 0; i < summation.stride; i++) {
									line_input[begin + i] = static_cast<UNDERLYING_FLOAT>(summation.data[sum_it * summation.stride + i]);
								}
							}
						}
						else if (N_SIMD == 8) {
							for (int i = 0; i < summation.stride; i++) {
								const UNDERLYING_FLOAT value = static_cast<UNDERLYING_FLOAT>(summation.data[sum_it * summation.stride + i]);
								UNDERLYING_FLOAT* dst = comp_input + LINES*(begin + i);
								for (int32_t j = 0; j < LINES; j++) { dst[j] = value; }
							}
						}
						else if (N_SIMD == 4) {
							const int jump = 4*padded_n_inputs;
							for (int i = 0; i < summation.stride; i++) {
								const UNDERLYING_FLOAT value = static_cast<UNDERLYING_FLOAT>(summation.data[sum_it * summation.stride + i]);
								UNDERLYING_FLOAT* dst = comp_input + N_SIMD*(begin + i);
								for (int32_t j = 0; j < N_SIMD; j++) { dst[j] = value; }
								dst += jump;
								for (int32_t j = 0; j < N_SIMD; j++) { dst[j] = value; }
							}
						}

						// Run and accumulate
						const int n_runs = LINES/N_SIMD;
						const int input_stride = padded_n_inputs * N_SIMD;
						const int output_stride = padded_n_outputs * N_SIMD;
						for (int32_t run_i = 0; run_i < n_runs; run_i++) {
							f(reinterpret_cast<COMPILED_FLOAT*>(comp_input + run_i * input_stride),
							  reinterpret_cast<COMPILED_FLOAT*>(scratch    + run_i * output_stride));
						}
						for (int i = 0; i < padded_n_outputs * LINES; i++) {
							accum[i] += scratch[i];
						}
					}
				}

				// ========== STAGE 5: UNSHUFFLE SoA→AoS (SIMD only) → buf_output_aos ==========
				// After this stage, output is ALWAYS in buf_output_aos (AoS format)
#ifdef SYMX_ENABLE_AVX2
				if (compiled_float_type == FloatType::SIMD8f) {
					detail::soa_to_aos_f32(
						reinterpret_cast<float*>(buf_output_soa),
						reinterpret_cast<float*>(buf_output_aos),
						padded_n_outputs);
				}
				else if (compiled_float_type == FloatType::SIMD4d) {
					double* src = reinterpret_cast<double*>(buf_output_soa);
					double* dst = reinterpret_cast<double*>(buf_output_aos);
					detail::soa_to_aos_f64(src, dst, padded_n_outputs);  // First 4 lines
					const int jump = 4*padded_n_outputs;
					detail::soa_to_aos_f64(src + jump, dst + jump, padded_n_outputs);  // Next 4 lines
				}
#endif
				// For scalar: output already in buf_output_aos

				// Final output pointer - always buf_output_aos after unshuffle
				OUTPUT_FLOAT* f_output = reinterpret_cast<OUTPUT_FLOAT*>(buf_output_aos);

				// ========== STAGE 6: CAST OUTPUT (if needed) ==========
				const int32_t output_size = LINES * padded_n_outputs;
				if (underlying_float_type == FloatType::Double && output_float_type == FloatType::Float) {
					detail::cast_inplace_f64_to_f32(reinterpret_cast<char*>(f_output), output_size);
				}
				else if (underlying_float_type == FloatType::Float && output_float_type == FloatType::Double) {
					detail::cast_inplace_f32_to_f64(reinterpret_cast<char*>(f_output), output_size);
				}

				// Check for NaNs in output
				if (check_mode_ON) {
					int32_t nan_index = detail::find_nan(f_output, LINES * padded_n_outputs);
					if (nan_index != -1) {
						const int output_i = nan_index % padded_n_outputs;
						std::cout << "symx error: NaN in output " << output_i << " for \"" << this->name << "\"" << std::endl;
						exit(-1);
					}
				}

				// ========== STAGE 7: CALLBACK ==========
				for (int line_i = 0; line_i < LINES; line_i++) {
					if (!is_pad_element[line_i]) {
						const int elem_idx = element_indices[line_i];
						int32_t* loc_conn = group_elem_conn.data() + line_i * connectivity_stride;
						OUTPUT_FLOAT* f_output_line = f_output + line_i * padded_n_outputs;
						callback(View<OUTPUT_FLOAT>(f_output_line, n_outputs), elem_idx, thread_id, View<int32_t>(loc_conn, connectivity_stride));
					}
				}
			} // End of group loop
		} // End of color loop
	}
}
