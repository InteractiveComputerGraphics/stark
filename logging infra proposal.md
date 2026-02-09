# Logging & Timing Infrastructure Proposal

## Executive Summary

This document proposes a unified logging, timing, and console-output infrastructure for the **symx + stark** ecosystem. SymX will own and provide the core infrastructure. Stark will consume it and extend it with simulation-specific semantics.

---

## 1. Current State Analysis

### 1.1 Reference Implementation (working, clean output)

The reference has **three separate systems** all living in `stark::core`:

| Class | Purpose | Lives in |
|-------|---------|----------|
| `Console` | Verbosity-gated output to stdout + file | stark |
| `Logger` | Key/value + typed series accumulator, timing, stats, save-to-disk | stark |
| `NewtonsMethod` | Receives Console + Logger as raw pointers, calls them directly | stark |

**Key design decisions in the reference:**
- `ConsoleVerbosity` has simulation-specific levels: `NoOutput`, `Frames`, `TimeSteps`, `NewtonIterations`, `LineSearch`.
- `Console::print(msg, verbosity)` checks `msg.verbosity <= this->verbosity` → print.
- `Logger` accumulates typed series with `get_statistics()` computing `sum | avg | [min, max]`.
- Newton receives `Console*` and `Logger*` and calls them deeply from within the solve loop.
- The **final summary** (`Stark::print()`) uses `Logger::get_statistics()` to print a structured table.
- Console writes **everything** to file (regardless of verbosity) and gates **console** output.
- Newton output is cleanly structured: indented per-iteration lines, projection sub-lines at the LineSearch verbosity, and a compact summary line at the TimeSteps level.

### 1.2 New Implementation (messy output, incomplete)

| Class | Purpose | Lives in |
|-------|---------|----------|
| `symx::Log` | Timer-only (insertion-ordered, `omp_get_wtime()`) | symx |
| `symx::NewtonsMethod::_print()` | Direct `std::cout` gated on `symx::Verbosity` | symx |
| `stark::core::Console` | Same as ref but **missing `LineSearch`** verbosity level | stark |
| `stark::core::Logger` | Simplified: no stats, no typed series, string-only series | stark |

**Problems identified:**

1. **symx prints directly to `std::cout`** — Newton's `_print()` bypasses Console entirely. Stark has no control over symx output (can't redirect, gate by stark-level verbosity, or file-log it).
2. **`symx::Verbosity`** has only `None`, `StepIterations`, `LineSearchIteration` — works for Newton but stark can't map its richer `Frames`/`TimeSteps` levels onto it.
3. **`symx::Log`** is timer-only. No series, no counters, no stats. The rich logging done by the reference Newton (`n_newton_iterations`, `projected_hessians_ratio`, residual trends, etc.) is **completely absent**.
4. **`stark::core::Logger`** was simplified too much — no typed series, no statistics, no description. The final `Stark::print()` is **stubbed out** with `"TODO: Final Stark::print() implementation"`.
5. **Console output formatting is messy**: Newton iterations blend with time-step summaries without proper indentation/newlines. The "Retrying with more projection" lines repeat without limit. No `#CG/newton` in summary. No `du_c` convergence percentage.
6. **Two incompatible verbosity enums** (`symx::Verbosity` vs `stark::core::ConsoleVerbosity`) with no mapping between them.
7. **Performance**: `Console::print` calls `std::flush` after every print. During a Newton step with many line-search iterations, this can be visibly slow on large simulations.
8. **`settings.as_string()` has bugs**: `max_time_step_size` and `time_step_size_success_multiplier` print `"true"` instead of their numeric values.

---

## 2. Design Requirements

1. **SymX is an independent library.** Its logging/output infra must not depend on stark. Other people may use symx to build non-simulation tools.
2. **Stark extends symx.** Stark must be able to intercept, redirect, and silence all symx output.
3. **Frequent output.** Prints must happen live during Newton iterations (not buffered until completion), since individual steps can take minutes in large simulations.
4. **Nullifiable for benchmarks.** All output and formatting must be trivially silenced (zero `fmt::format` calls, no virtual dispatch, no string allocation) when benchmarking.
5. **File logging.** All output should optionally go to file regardless of console verbosity.
6. **Final summary.** A structured summary (statistics over all time steps) must be printed at the end.
7. **Performance.** Avoid per-print virtual dispatch, mutex contention, and excessive `std::flush` calls.

---

## 3. Proposed Architecture

### 3.1 Overview

```
┌───────────────────────────────────────────────────────────────────┐
│                            STARK                                  │
│                                                                   │
│  ┌───────────────┐   ┌───────────────┐   ┌──────────────────────┐ │
│  │   Console     │   │    Logger     │   │   Stark::print()     │ │
│  │ (stark-level  │   │ (stark-level  │   │   final summary      │ │
│  │  verbosity,   │   │  series &     │   │   using Logger +     │ │
│  │  file + cout) │   │  timing)      │   │   Newton::Log stats  │ │
│  └──────┬────────┘   └───────────────┘   └──────────────────────┘ │
│         │ installs as OutputSink                                  │
│         ▼                                                         │
│  ┌───────────────────────────────────────────────────────────────┐│
│  │                        SYMX                                   ││
│  │                                                               ││
│  │  ┌────────────┐  ┌───────────────┐  ┌───────────────────────┐ ││
│  │  │ OutputSink │  │     Log       │  │    NewtonsMethod      │ ││
│  │  │ (callback  │  │ (timer +      │  │  + OutputFormatter    │ ││
│  │  │  + gate)   │  │  series +     │  │  (uses Sink + Log)    │ ││
│  │  │            │  │  stats)       │  │                       │ ││
│  │  └────────────┘  └───────────────┘  └───────────────────────┘ ││
│  └───────────────────────────────────────────────────────────────┘│
└───────────────────────────────────────────────────────────────────┘
```

### 3.2 `symx::OutputSink` — The print callback (in symx)

Replaces `NewtonsMethod::_print(msg, verbosity)`. This is the **only** way output flows out of symx.

```cpp
namespace symx {

    // Verbosity levels for symx — domain-agnostic names
    enum class Verbosity : int {
        Silent  = 0,   // No output at all
        Summary = 1,   // Per-solve summary (1 line per solve call)
        Step    = 2,   // Per Newton iteration line
        Detail  = 3,   // Line search iterations, projection retries
    };

    // Lightweight output sink — no virtual functions
    // Parent application installs a callback. When no callback, output → stdout.
    class OutputSink {
    public:
        using PrintFn = std::function<void(const std::string& msg, Verbosity level)>;

        void set_verbosity(Verbosity v) { verbosity_ = v; }
        Verbosity get_verbosity() const { return verbosity_; }

        void set_callback(PrintFn fn) { callback_ = std::move(fn); }
        void clear_callback() { callback_ = nullptr; }

        // Fast inline check — no formatting when silenced
        bool should_print(Verbosity level) const { return level <= verbosity_; }

        // Main print method
        void print(const std::string& msg, Verbosity level) const {
            if (level > verbosity_) return;
            if (callback_) {
                callback_(msg, level);
            } else {
                std::cout << msg;
            }
        }

    private:
        Verbosity verbosity_ = Verbosity::Step;
        PrintFn callback_ = nullptr;
    };
}
```

**Key properties:**
- **No virtual dispatch.** `std::function` with null check is one branch per call.
- **Caller-side gating.** `should_print()` is inline — when `Silent`, the caller never even formats the string. This is the critical performance feature.
- **Default to stdout.** If no callback is set, output goes directly to `std::cout` (for standalone symx use without stark).
- **Parent redirects everything.** Stark installs a callback that routes through `Console::print()` which handles file + verbosity gate.

### 3.3 `symx::Log` — Timer + Series + Stats (in symx)

Upgrade the current timer-only `Log` to also handle series and stats. This lets Newton log iteration counts, residuals, projection ratios, etc. without depending on a parent Logger.

```cpp
namespace symx {

    class Log {
    public:
        // === Scoped Timer (unchanged) ===
        struct ScopedTimer { /* same as now */ };
        ScopedTimer scope(const std::string& label);

        // === Accumulated Timers ===
        void start_timing(const std::string& label);
        void stop_timing(const std::string& label);

        // === Typed Series ===
        void append(const std::string& label, double v);
        void append(const std::string& label, int v);

        // === Single Values ===
        void set(const std::string& label, double v);
        void set(const std::string& label, int v);
        void add(const std::string& label, double v);
        void add(const std::string& label, int v);

        // === Getters ===
        double get_double(const std::string& label) const;
        int get_int(const std::string& label) const;
        const std::vector<double>& get_double_series(const std::string& label) const;
        const std::vector<int>& get_int_series(const std::string& label) const;

        // === Statistics ===
        // Returns "TOTAL | AVG | [MIN, MAX]" string for a series
        std::string get_statistics(const std::string& label) const;

        // === Output ===
        void print_timing_report(const OutputSink& sink) const;
        void print_statistics(const OutputSink& sink) const;

        // === Persistence ===
        void save_to_disk(const std::string& path) const;

        // === Control ===
        void clear();
        void set_enabled(bool enabled);  // When false, even timing calls become no-ops

        // === Access ===
        const auto& get_timers() const;

    private:
        bool enabled_ = true;

        // Timing
        struct Timer { double start = 0; double total = 0; int count = 0; };
        std::unordered_map<std::string, Timer> timers_;
        std::vector<std::string> timer_insertion_order_;

        // Series
        std::unordered_map<std::string, std::vector<double>> series_double_;
        std::unordered_map<std::string, std::vector<int>> series_int_;

        // Singles
        std::unordered_map<std::string, double> doubles_;
        std::unordered_map<std::string, int> ints_;
    };
}
```

### 3.4 `NewtonOutputFormatter` — Structured Newton Output (in symx)

This class encapsulates **all** formatting logic for Newton's Method output. It replaces the scattered `_print(fmt::format(...))` calls inside `NewtonsMethod::_solve_impl()`.

**The key insight:** Newton's output has a **fixed structure** (iteration headers, indented projection sub-lines, line search sub-lines, summary). Having a formatter class eliminates the current "stateful chain" problem where formatting of each print depends on what was printed previously.

```cpp
namespace symx {

    class NewtonOutputFormatter {
    public:
        NewtonOutputFormatter(const OutputSink& sink, const std::string& prefix = "");

        // === Newton iteration ===
        void begin_iteration(int iteration);
        void print_residual(double residual);
        void print_projection(double ratio_percent);
        void print_cg_iterations(int count);
        void print_step_size(double du_max);
        void print_step_cap(double capped_du);
        void print_max_step_hit(double clamped_du);

        // === Projection retry (inner loop) ===
        void begin_projection_retry();
        void end_projection_retry();  // Prints "Retrying..." once + newline

        // === Line search ===
        void begin_line_search_iteration(int it, double step);
        void print_invalid_state();
        void print_armijo(double E_over_E_threshold, double E_over_E0);

        // === Solve summary (printed at Summary verbosity → always visible at TimeSteps) ===
        void print_step_summary(SolverReturn result, int newton_its, int ls_its, int cg_its);

        // === Suffix hook (stark adds " | runtime: X ms | cr: Y\n") ===
        void print_suffix(const std::string& msg, Verbosity level);

    private:
        const OutputSink& sink_;
        std::string prefix_;
        bool iteration_line_open_ = false;  // Tracks whether we're mid-line in an iteration
    };
}
```

**Usage pattern in `NewtonsMethod::_solve_impl()`:**
```cpp
SolverReturn NewtonsMethod::_solve_impl() {
    NewtonOutputFormatter out(this->output_sink, this->settings.output_prefix);

    while (result == SolverReturn::Running) {
        out.begin_iteration(newton_iteration);     // Newline + prefix + "  1. "
        out.print_residual(residual_norm);          // "r = 4.5e-01 | "

        // Inner projection+solve loop
        while (!solved) {
            out.begin_projection_retry();           // (Detail level) projection sub-line
            out.print_projection(100.0 * ratio);    // "ph:  65.49% | "
            out.print_cg_iterations(n);             // "#CG    16 | "
            if (!success) {
                out.end_projection_retry();         // " -> failure reason\n"
                increase_projection();
                continue;
            }
            break;
        }

        out.print_step_size(du_max);                // "du = 3.8e-01 | "

        // Line search
        for (...) {
            out.begin_line_search_iteration(it, step);
            out.print_armijo(ratio1, ratio2);
        }
    }

    out.print_step_summary(result, n_its, ls_its, cg_its);
    // Stark appends: " | runtime: 2 ms | cr: 0.1\n"
}
```

The formatter knows the indentation state and handles newlines correctly, eliminating the current formatting mess.

### 3.5 Stark's Console & Logger (in stark)

#### Console (updated)

```cpp
namespace stark::core {

    enum class ConsoleVerbosity {
        NoOutput = 0,
        Frames = 1,
        TimeSteps = 2,
        NewtonIterations = 3,
        LineSearch = 4,          // Restored from reference
    };

    class Console {
    public:
        void initialize(...);
        void print(const std::string& msg, ConsoleVerbosity verbosity);
        // ... same interface ...

        // NEW: Returns a symx::OutputSink::PrintFn that routes through this Console
        symx::OutputSink::PrintFn make_symx_callback();
    };
}
```

**Verbosity mapping** (done once in `Stark::_initialize()`):
```
symx::Verbosity::Silent  →  (nothing prints)
symx::Verbosity::Summary →  ConsoleVerbosity::TimeSteps
symx::Verbosity::Step    →  ConsoleVerbosity::NewtonIterations
symx::Verbosity::Detail  →  ConsoleVerbosity::LineSearch
```

```cpp
void Stark::_initialize() {
    // ...
    auto callback = [this](const std::string& msg, symx::Verbosity level) {
        ConsoleVerbosity cv;
        switch (level) {
            case symx::Verbosity::Summary: cv = ConsoleVerbosity::TimeSteps; break;
            case symx::Verbosity::Step:    cv = ConsoleVerbosity::NewtonIterations; break;
            case symx::Verbosity::Detail:  cv = ConsoleVerbosity::LineSearch; break;
            default: return;
        }
        this->console.print(msg, cv);
    };
    this->newton->output_sink.set_callback(callback);

    // Map stark verbosity → symx verbosity
    switch (this->settings.output.console_verbosity) {
        case ConsoleVerbosity::NoOutput:
        case ConsoleVerbosity::Frames:
            this->newton->output_sink.set_verbosity(symx::Verbosity::Silent); break;
        case ConsoleVerbosity::TimeSteps:
            this->newton->output_sink.set_verbosity(symx::Verbosity::Summary); break;
        case ConsoleVerbosity::NewtonIterations:
            this->newton->output_sink.set_verbosity(symx::Verbosity::Step); break;
        case ConsoleVerbosity::LineSearch:
            this->newton->output_sink.set_verbosity(symx::Verbosity::Detail); break;
    }
}
```

This is the **only place** the mapping occurs. All of symx's output goes through the sink, and the Console handles the file+stdout routing.

#### Logger (upgraded back to reference quality)

Restore `stark::core::Logger` to match the reference implementation:
- **Typed series** (`vector<double>`, `vector<int>`, `vector<string>`)
- **`get_statistics()`** computing `sum | avg | [min, max]`
- **`description`** field for embedding settings in log files
- **Configurable float formats** (`file_float_format`, `stats_float_format`)
- **Accumulated-per-time-step** pattern (`append(label, value, idx)`)

Newton's `symx::Log` handles internal timing (energy eval, CG, assembly, etc.). After each time step, `Stark::run_one_step()` **pulls** aggregated data from `newton->log` and pushes it into the stark `Logger`:

```cpp
// After successful Newton solve
const auto& newton_log = this->newton->get_log();
this->logger.append("n_newton_iterations", newton_iteration);
this->logger.append("n_line_search_iterations", ls_iterations);
// ... Pull timing aggregates from newton_log for the stats table
```

#### `Stark::print()` (restored)

Restore the full final summary from the reference, pulling statistics from both `Logger` and `Newton::Log`:

```cpp
void Stark::print() {
    console.print("\nInfo\n", ConsoleVerbosity::Frames);
    console.print(fmt::format("\t Simulation name: {}\n", settings.output.simulation_name), ...);
    console.print(fmt::format("\t Simulation time: {:.3f} s\n", current_time), ...);
    console.print(fmt::format("\t ndofs: {:d}\n", ndofs), ...);
    console.print(fmt::format("\t Frames: {:d}\n", current_frame), ...);
    console.print(fmt::format("\t Time steps: {:d}\n", n_time_steps), ...);
    print_stats("dt:\t\t\t\t", "dt");
    print_stats("n_newton_iterations:\t\t", "n_newton_iterations");
    print_stats("projected_hessians_ratio:\t", "projected_hessians_ratio");

    console.print("\nRuntime\n", ConsoleVerbosity::Frames);
    print_stats("total:\t\t\t\t", "runtime_total");
    print_stats("Energy Evaluation:\t\t", "runtime_energy_eval");
    print_stats("CG:\t\t\t\t", "runtime_cg");
    print_stats("Project to PD:\t\t\t", "runtime_project_pd");
    // ...
}
```

---

## 4. Verbosity Levels — Complete Mapping

### symx levels (domain-agnostic)

| Level | What prints | Use case |
|-------|------------|----------|
| `Silent` | Nothing | Benchmarks, embedded in optimization loops |
| `Summary` | 1 line per `solve()`: `"#newton: 3 \| #ls/newton: 0.33 \| converged"` | Quick monitoring |
| `Step` | Per Newton iteration: `"  1. r = 4.5e-01 \| ph: 0.0% \| #CG 5 \| du = 3.8e-01"` | Normal development |
| `Detail` | + projection retries, line search steps, Armijo values | Debugging convergence |

### stark levels (simulation-aware)

| Level | What prints | Maps to symx |
|-------|------------|--------------|
| `NoOutput` | Nothing at all | `Silent` |
| `Frames` | Frame numbers, init info, settings, **final summary** | `Silent` (Newton is silent) |
| `TimeSteps` | + Per time step: `"dt: 33.33 ms \| #newton: 3 \| runtime: 2ms \| cr: 0.1"` | `Summary` |
| `NewtonIterations` | + Newton iteration details | `Step` |
| `LineSearch` | + Line search + projection retry details | `Detail` |

---

## 5. Performance Considerations

### 5.1 Avoid formatting when silenced

The `OutputSink::should_print()` check is inline and costs one integer comparison. Newton code should guard expensive formatting:

```cpp
if (sink.should_print(Verbosity::Step)) {
    sink.print(fmt::format("  {:3d}. r = {:.2e} | ", it, residual), Verbosity::Step);
}
```

When `Silent`, `fmt::format` is never called. This is the most important optimization.

### 5.2 Remove `std::flush` from every print

Current `Console::print` calls `std::flush` after every write. This is expensive and pointless for line-buffered console output.

**Change to:**
- **Console**: `std::cout << msg;` (no flush). Rely on `\n`-triggered flush or periodic explicit flush.
- **File**: Buffer in `std::stringstream`, flush once per time step (in `_write_frame()` or end of `run_one_step()`).

### 5.3 Avoid `std::function` overhead for silenced sinks

When `Verbosity::Silent`, `OutputSink::print()` returns at the integer comparison before touching `std::function`. Cost: one int compare per call.

### 5.4 Mutex scope

`Console` takes a `std::mutex` lock on every `print()`. This is needed when multiple threads might print (rare — usually only the main thread prints). Consider:
- Skip the lock for console-only output (stdout is OS-serialized).
- Only lock for file writes.
- Or: accept the lock cost (it's uncontended and fast).

### 5.5 Log timer overhead

`omp_get_wtime()` is cheap (~20 ns). The timer overhead is negligible for per-Newton-iteration timing. For the paranoid benchmark case, `Log::set_enabled(false)` skips all timing calls.

---

## 6. Nullification for Benchmarks

```cpp
// Option A: Settings (recommended)
settings.output.console_verbosity = ConsoleVerbosity::NoOutput;
settings.output.console_output_to = ConsoleOutputTo::NoOutput;
// → Newton's OutputSink automatically set to Silent
// → No fmt::format calls, no file writes, no stdout writes

// Option B: Programmatic (standalone symx)
newton->output_sink.set_verbosity(symx::Verbosity::Silent);
// symx::Log still collects timers—cheap. Read them after the fact.
// If even that is too much:
newton->log.set_enabled(false);  // Skips omp_get_wtime() calls
```

---

## 7. Target Output Format

### Per time step at `TimeSteps` verbosity:
```
	 dt: 33.333333 ms | #newton: 4  | #CG/newton: 12 | #ls/newton: 0.00 | converged | runtime: 2 ms | cr: 0.1
```

### Per time step at `NewtonIterations` verbosity:
```
	 dt: 33.333333 ms | 
		   1. r = 1.26e+01 | ph:   0.00% | #CG     1 | du = 3.8e-01 | 
		   2. r = 5.18e-03 | ph:   0.00% | #CG    16 | du = 2.4e-01 | 
		   3. r = 2.20e-04 | ph:   0.00% | #CG    15 | du = 1.1e-02 | 
		   4. r = 9.55e-07 | ph:   0.00% | #CG    16 | du = 2.6e-05 | 
		#newton: 4  | #CG/newton: 12 | #ls/newton: 0.00 | converged | runtime: 2 ms | cr: 0.1
```

### Per time step at `LineSearch` verbosity:
```
	 dt: 33.333333 ms | 
		   1. r = 5.45e-03 | 
			 0. p-tol:         | ph:   0.00% | #CG     8 |  -> Linear system failure
			 1. p-tol: 2.7e-03 | ph:  38.52% | #CG     9 |  -> Linear system failure
			 2. p-tol: 1.4e-03 | ph:  65.81% | #CG    16 |
			    ph:  65.81% | #CG    16 | du = 2.7e-01 | 
		   2. r = 3.43e-04 | ...
		#newton: 7  | #CG/newton: 15 | #ls/newton: 0.10 | converged | runtime: 3 ms | cr: 0.1
```

### Final summary (always at `Frames` level):
```
Info
	 Simulation name: twisting_cloth
	 Simulation time: 1.033 s
	 ndofs: 363
	 Frames: 31
	 Time steps: 31
	 dt:				1.033333	| 0.033333	| [0.033333, 0.033333]
	 n_newton_iterations:		137	| 4.419355	| [1, 20]
	 projected_hessians_ratio:	4.817014	| 0.035419	| [0.000000, 0.956661]

Runtime
	 total:				0.079398	| 0.002561	| [0.000698, 0.013471]
	 CG:				0.017144	| 0.000126	| [0.000017, 0.001147]
	 Energy Evaluation:		0.010394	| 0.000076	| [0.000059, 0.000275]
	 Project to PD:			0.001591	| 0.000033	| [0.000004, 0.000176]
	 Hessian Assembly:		0.011691	| 0.000086	| [0.000056, 0.000345]
	 before_energy_evaluation:	0.018020	| 0.000074	| [0.000063, 0.000204]
	 write_frame:			0.001477	| 0.000046	| [0.000000, 0.000099]
```

---

## 8. Migration Plan

### Phase 1: symx infrastructure
1. Rename/expand `symx::Verbosity` to the 4-level enum (`Silent`, `Summary`, `Step`, `Detail`).
2. Add `symx::OutputSink` class in `symx/src/solver/OutputSink.h`.
3. Upgrade `symx::Log` to include typed series, single values, `get_statistics()`, `set_enabled()`.
4. Create `symx::NewtonOutputFormatter` in `symx/src/solver/NewtonOutputFormatter.h`.
5. Add `OutputSink output_sink;` as public field on `NewtonsMethod`.
6. Refactor `NewtonsMethod::_solve_impl()` to use `OutputSink` + `Log` + `NewtonOutputFormatter`.
7. Remove `NewtonsMethod::_print()`.
8. Migrate compilation-stage prints from `Context::print` to `OutputSink` at `Summary` level.

### Phase 2: stark integration
1. Restore `ConsoleVerbosity::LineSearch` in the stark enum.
2. Add `Console::make_symx_callback()` for routing.
3. In `Stark::_initialize()`, install the callback and map verbosity levels.
4. Upgrade `stark::core::Logger` back to reference quality (typed series, stats, description).
5. In `Stark::run_one_step()`, after Newton solve, pull aggregated data from `newton->log` into `Logger`.
6. Restore `Stark::print()` with the full summary table.
7. Remove `std::flush` from `Console::print()` (use line-buffered output).

### Phase 3: Cleanup
1. Remove `symx::Context::print` callback (replaced by `OutputSink`).
2. Fix `Settings::as_string()` numeric formatting bugs.
3. Remove all debug `std::cout` scattered through the codebase.
4. Ensure Newton logs match reference: `n_newton_iterations`, `n_line_search_iterations`, `projected_hessians_ratio`, `residual_max`, `max_du`, etc.

---

## 9. Ownership Summary

| Component | Owner | File Location |
|-----------|-------|---------------|
| `OutputSink` | symx | `symx/src/solver/OutputSink.h` |
| `Verbosity` enum (4-level) | symx | `symx/src/solver/solver_utils.h` |
| `Log` (timer + series + stats) | symx | `symx/src/solver/Log.h` |
| `NewtonOutputFormatter` | symx | `symx/src/solver/NewtonOutputFormatter.h` |
| `NewtonsMethod.output_sink` | symx | `symx/src/solver/NewtonsMethod.h` (public field) |
| `Console` (file + stdout routing) | stark | `stark/src/core/Console.h` |
| `ConsoleVerbosity` (5-level enum) | stark | `stark/src/core/Console.h` |
| `Logger` (per-simulation accumulation) | stark | `stark/src/core/Logger.h` |
| Verbosity mapping (symx↔stark) | stark | `Stark::_initialize()` |
| Final summary | stark | `Stark::print()` |

---

## 10. Open Questions / Decisions

1. **Should `symx::Log` be shared or per-solve?**
   Currently it's a field on `NewtonsMethod` and accumulates across `solve()` calls. For stark (one Newton per lifetime) this is fine. Standalone symx users might want to reset between solve calls. **Proposal: keep accumulated, provide `clear()`.** Users call it when they want.

2. **Should compilation output migrate from `Context::print` to `OutputSink`?**
   Yes. The compilation messages ("EnergyLumpedInertia.........queued") should go through `OutputSink` at `Summary` level. `Context::print` becomes redundant and can be removed. The `OutputSink` can be stored on `Context` or passed as a parameter to compilation.

3. **Should `OutputSink` support multiple sinks?** (e.g., console + external listener)
   **No, for now.** Keep it simple: one callback. Stark's Console already handles both file + console. If a user wants to add their own listener on top, they can wrap the callback.

4. **Thread safety of `OutputSink`?**
   Newton is single-threaded at the print level (parallelism is within CG/assembly, which don't print). No mutex needed inside the sink. If a parent app forwards to a thread-unsafe target, they handle locking in their callback.

5. **`fmt` dependency?**
   symx already uses fmt. The formatter will use `fmt::format` internally. This is fine.

6. **What about DOF labels in the summary?**
   The reference prints DOF labels from `global_energy.dof_labels`. The new `GlobalPotential` doesn't have labels yet. This is a separate TODO (mentioned in symx port TODO.md) and orthogonal to the logging infra. The formatter just prints what it gets.

7. **Newton log timer labels: should they match the reference?**
   Not necessarily — the internal structure changed. But the **final summary** should show the same meaningful categories: total, energy eval, CG, assembly, projection, write_frame. Some reference labels (like `runtime_CG_success` vs `runtime_CG_fail`) can be simplified.
