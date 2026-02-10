# Logging & Timing Infrastructure — Architecture Document (v4)

## Executive Summary

This document describes the implemented logging and console-output infrastructure for the **symx + stark** ecosystem.
Everything lives in **symx**. stark is a consumer — it sets verbosity, opens the file, and prints per-time-step summaries.
There is **one verbosity enum** and **one shared `OutputSink`** (with PrintOnly/FileOnly/PrintAndFile modes) on `symx::Context`. Newton and Stark both hold a `shared_ptr` to the same sink.

**Status:** OutputSink, Verbosity enum, Newton/Stark print routing — all **implemented and working**. Logger upgrade and `Stark::print()` summary — **TODO**.

---

## 1. What Changed From the Old State

### 1.1 Old State (before this work)

| Class | Purpose | Lives in |
|-------|---------|----------|
| `symx::Verbosity` | 3-level enum (`None`, `StepIterations`, `LineSearchIteration`) | symx |
| `symx::Log` | Timer-only (insertion-ordered, `omp_get_wtime()`) | symx |
| `symx::NewtonsMethod::_print()` | Direct `std::cout`, gated on old `symx::Verbosity` | symx |
| `stark::core::Console` | Mutex-guarded, verbosity-gated output to stdout + file | stark |
| `stark::core::Logger` | Simplified: no stats, no typed series, string-only series | stark |

**Problems:** symx printed to `std::cout` directly (stark couldn't gate or redirect it), two incompatible verbosity enums, Console had per-print mutex + flush, Logger was too simple for a final summary.

### 1.2 New State (implemented)

| Component | Purpose | Lives in |
|-----------|---------|----------|
| `symx::Verbosity` | 4-level enum (`Silent`, `Summary`, `Step`, `Full`) | `OutputSink.h` |
| `symx::OutputTo` | 3-mode enum (`PrintOnly`, `FileOnly`, `PrintAndFile`) | `OutputSink.h` |
| `symx::OutputSink` | Verbosity-gated print + file output, shared via `spOutputSink` | `OutputSink.h` |
| `symx::Log` | Timer-only (unchanged, still `omp_get_wtime()` RAII scopes) | `Log.h` |
| `symx::NewtonsMethod` | All prints go through `this->output` (shared sink) | `NewtonsMethod.cpp` |
| `stark::core::Logger` | String-only series + timing (unchanged, still used for series data) | `Logger.h` |
| `stark::core::Console` | **Dead code** — still included but no longer called | `Console.h` |

---

## 2. Design Requirements

1. **One verbosity enum.** `symx::Verbosity` is used everywhere. No stark-level mapping, no second enum.
2. **SymX is independent.** The infra must not depend on stark. Standalone symx users get sane defaults (prints to stdout).
3. **Frequent output.** Prints happen live during Newton iterations (not buffered). Individual steps on large meshes can take minutes.
4. **Nullifiable for benchmarks.** `Verbosity::Silent` → Newton produces no output. `set_enabled(false)` disables the sink entirely. We accept the cost of `fmt::format` even for gated messages.
5. **File logging built into OutputSink.** Modes: `PrintOnly`, `FileOnly`, `PrintAndFile`. No separate Console class for file routing. File opened explicitly via `open_file()`.
6. **OutputSink on Context as shared_ptr.** Everybody that touches `symx::Context` has access. Newton copies the `shared_ptr` in its constructor — same object, single source of truth.
7. **No mutex on prints.** Single-threaded print path. OpenMP parallel sections use `#pragma omp critical` when they need to print.
8. **Timer ergonomics.** Scoped RAII timers via `Log::scope()` — one-liners.
9. **Customizable indentation.** The sink has `root_tab` and `tab_size`. stark sets `root_tab=1` so Newton's output is indented one level deeper than stark's headers. Auto-indent via `_indent(Verbosity)` uses `max(0, root_tab + level - 1)`.

---

## 3. Proposed Architecture

### 3.1 Overview

```
┌────────────────────────────────────────────────────────────────────┐
│                             STARK                                  │
│                                                                    │
│  Stark::_initialize()       Stark::run_one_step()   Stark::print() │
│  ┌─────────────────────┐    ┌──────────────────┐    ┌───────────┐  │
│  │ Sets verbosity,     │    │ Appends dt, cr,  │    │ Prints    │  │
│  │ opens log file,     │    │ runtime, frame#  │    │ stark     │  │
│  │ disable_console     │    │ to context.logger│    │ globals + │  │
│  │ if needed           │    │                  │    │ log stats │  │
│  └─────────┬───────────┘    └────────┬─────────┘    └───────────┘  │
│            │                         │                              │
│            ▼                         ▼                              │
│  ┌─────────────────────────────────────────────────────────────────┐│
│  │                    symx::Context                                ││
│  │                                                                 ││
│  │  ┌──────────────┐    ┌──────────────┐                           ││
│  │  │  OutputSink   │    │    Logger    │                           ││
│  │  │  verbosity    │    │  typed series│                           ││
│  │  │  print/file/  │    │  timers      │                           ││
│  │  │  both modes   │    │  statistics  │                           ││
│  │  └──────┬────────┘    └──────┬───────┘                           ││
│  │         │                    │                                   ││
│  │         ▼                    ▼                                   ││
│  │  ┌──────────────────────────────────────────┐                   ││
│  │  │           NewtonsMethod                   │                   ││
│  │  │  uses context.sink to print               │                   ││
│  │  │  uses context.logger for series/timers    │                   ││
│  │  │  verbosity = indentation depth            │                   ││
│  │  └──────────────────────────────────────────┘                   ││
│  └─────────────────────────────────────────────────────────────────┘│
└────────────────────────────────────────────────────────────────────┘
```

### 3.2 `symx::Verbosity` — Single enum (in symx)

```cpp
namespace symx {
    enum class Verbosity : int {
        Silent  = 0,   // No output at all
        Summary = 1,   // Per-solve summary (1 line per solve call)
        Step    = 2,   // Per Newton iteration line
        Full  = 3,   // Line search iterations, projection retries
    };
}
```

stark uses this directly. If set to `Silent`, Newton produces no output. stark itself can still print time step headers and frame numbers through the ungated `sink.print(msg)` overload (no verbosity argument). That overload still respects `console_enabled_` — so `enable_console_output = false` silences everything.

**From the stark user's perspective:**
- `Verbosity::Silent` → user sees time step header + frame writes (if console output is enabled)
- `settings.output.enable_console_output = false` → truly nothing on stdout
- `Verbosity::Summary` → + Newton summary line per time step
- `Verbosity::Step` → + per-iteration lines
- `Verbosity::Full` → + line search / projection details

### 3.3 `symx::OutputSink` — Print + File output (in symx)

The OutputSink replaces both `NewtonsMethod::_print()` and `stark::core::Console`. It owns file output directly.

```cpp
namespace symx {

    enum class OutputMode {
        PrintOnly,      // stdout only
        FileOnly,       // file only
        PrintAndFile,   // both
    };

    class OutputSink {
    public:
        // --- Verbosity ---
        void set_verbosity(Verbosity v) { verbosity_ = v; }
        Verbosity get_verbosity() const { return verbosity_; }

        // --- Indentation ---
        // root_tab: base indentation level added to every verbosity-gated print.
        //   E.g. stark sets root_tab=1 so Newton's Summary (level 1) prints at depth 2,
        //   leaving depth 0-1 for stark's own output.
        // tab_size: number of spaces per tab level.
        void set_root_tab(int level) { root_tab_ = level; }
        void set_tab_size(int spaces) { tab_size_ = spaces; }

        // --- Output mode & file ---
        void set_mode(OutputMode m) { mode_ = m; }
        void open_file(const std::string& path);  // File created lazily here
        void close_file();

        // --- Console enable (stark sets this to implement "no console at all") ---
        void set_console_enabled(bool v) { console_enabled_ = v; }

        // --- Core API ---

        // Print with verbosity gate + auto-indent.
        // Indentation = (root_tab + level) * tab_size spaces, prepended automatically.
        // Caller just provides the message content — no manual tabs.
        void print(const std::string& msg, Verbosity level) const {
            if (level > verbosity_) return;
            _emit(_indent(level) + msg);
        }

        // Ungated print — no verbosity check, no auto-indent.
        // For the parent lib's own output (time step headers, frame lines, final summary).
        // If console is disabled, this is also silent.
        void print(const std::string& msg) const {
            _emit(msg);
        }

        void flush_file() {
            if (file_.is_open()) file_.flush();
        }

    private:
        void _emit(const std::string& msg) const {
            if (console_enabled_ && mode_ != OutputMode::FileOnly) {
                std::cout << msg;
            }
            if (file_.is_open() && mode_ != OutputMode::PrintOnly) {
                file_ << msg;
            }
        }

        std::string _indent(Verbosity level) const {
            int depth = root_tab_ + static_cast<int>(level);
            return std::string(depth * tab_size_, ' ');
        }

        Verbosity verbosity_ = Verbosity::Step;
        OutputMode mode_ = OutputMode::PrintOnly;
        bool console_enabled_ = true;
        int root_tab_ = 0;    // symx default: 0 (standalone). stark sets to 1.
        int tab_size_ = 2;    // 2 spaces per level
        mutable std::ofstream file_;
    };
}
```

**Key properties:**
- **No mutex.** Single-threaded print path. Done.
- **No `std::flush` on stdout.** Line-buffered is fine.
- **File flush is explicit.** stark calls `flush_file()` once per time step.
- **`print(msg)`** (no verbosity) is the ungated path for the parent lib's own output (time step headers, frame lines, final summary). It respects `console_enabled_` — if the user set that to false, truly nothing prints.
- **`print(msg, level)`** is the verbosity-gated path. The sink **auto-indents** by `(root_tab + level) * tab_size` spaces. The caller never manually inserts `\t` or spaces — just the message content.
- **`set_console_enabled(false)`** silences stdout entirely. File output continues if configured.
- **Customizable indentation.** stark calls `sink.set_root_tab(1)` so that symx's `Summary` (level 1) prints at indent depth 2 (= `(1+1)*2 = 4 spaces`), leaving indent depth 0–1 for stark's own headers. `tab_size` defaults to 2 spaces but is configurable.
- **Lazy file.** No file is opened unless `open_file()` is called. Default is `PrintOnly` + `Verbosity::Step` → sane out of the box for standalone symx.

### 3.4 `symx::Logger` — Timer + Series + Stats (in symx)

Upgrade the current timer-only `Log` and absorb `stark::core::Logger`. This becomes the **single logger** used by everybody (Newton, stark, etc.) through `Context`.

```cpp
namespace symx {

    class Logger {
    public:
        // =============================================================
        // Scoped Timer — the ergonomic timing API
        // =============================================================
        // Usage: { auto t = logger.time("CG"); ... } // Accumulated automatically
        struct ScopedTimer {
            Logger& log;
            std::string label;
            double start;
            ScopedTimer(Logger& l, const std::string& label);
            ~ScopedTimer();
            ScopedTimer(ScopedTimer&&);
        };
        [[nodiscard]] ScopedTimer time(const std::string& label);

        // =============================================================
        // Typed Series (per time step data, appended over the simulation)
        // =============================================================
        void append(const std::string& label, double v);
        void append(const std::string& label, int v);
        void append(const std::string& label, const std::string& v);

        // =============================================================
        // Accumulators (for within-step aggregation, e.g., total CG iters)
        // =============================================================
        void add(const std::string& label, double v);
        void add(const std::string& label, int v);
        void set(const std::string& label, double v);
        void set(const std::string& label, int v);

        // =============================================================
        // Getters
        // =============================================================
        double get_double(const std::string& label) const;
        int get_int(const std::string& label) const;
        const std::vector<double>& get_double_series(const std::string& label) const;
        const std::vector<int>& get_int_series(const std::string& label) const;

        // =============================================================
        // Statistics — "SUM | AVG | [MIN, MAX]"
        // =============================================================
        std::string get_statistics(const std::string& label) const;

        // =============================================================
        // Timer Queries
        // =============================================================
        double get_timer_total(const std::string& label) const;
        int get_timer_count(const std::string& label) const;
        const std::vector<std::string>& get_timer_labels() const; // insertion order

        // =============================================================
        // Persistence — YAML format, machine-readable
        // =============================================================
        // Writes the log to disk. Summary block at top, then raw series.
        void save_to_yaml(const std::string& path) const;

        // =============================================================
        // Control
        // =============================================================
        void clear();
        void set_enabled(bool enabled);  // When false, timing + series become no-ops
        bool is_enabled() const;

        // Description block (written into log file header)
        void set_description(const std::string& desc);

    private:
        bool enabled_ = true;
        std::string description_;

        // Timing
        struct Timer { double start = 0; double total = 0; int count = 0; };
        std::unordered_map<std::string, Timer> timers_;
        std::vector<std::string> timer_insertion_order_;

        // Series
        std::unordered_map<std::string, std::vector<double>> series_double_;
        std::unordered_map<std::string, std::vector<int>> series_int_;
        std::unordered_map<std::string, std::vector<std::string>> series_string_;

        // Accumulators (reset-per-step by the user)
        std::unordered_map<std::string, double> acc_double_;
        std::unordered_map<std::string, int> acc_int_;
    };
}
```

**Timer ergonomics inside Newton:**
```cpp
// One-liner RAII timing — no start/stop boilerplate
{
    auto t = context.logger.time("CG");
    cg_solver.solve(...);
}  // automatically accumulated

{
    auto t = context.logger.time("energy_eval");
    evaluate_energies(...);
}

{
    auto t = context.logger.time("project_to_pd");
    project_hessian_to_pd(...);
}
```

No manual `start_timing` / `stop_timing` pairs. No risk of forgetting to stop. The `[[nodiscard]]` attribute ensures the user assigns it (otherwise the timer dies immediately).

**stark adds its own series after each step:**
```cpp
// In Stark::run_one_step(), after Newton solve:
context.logger.append("dt", dt);
context.logger.append("runtime_total", step_runtime_ms);
context.logger.append("convergence_ratio", cr);
context.logger.append("n_frames_written", frame_written ? 1 : 0);
```

### 3.5 `symx::Context` — Owns Sink + Logger

```cpp
namespace symx {
    class Context {
    public:
        OutputSink sink;      // Public, everyone prints through this
        Logger logger;        // Public, everyone logs through this

        // ... existing fields (energies, compilation, etc.) ...
    };
}
```

That's it. No getters, no indirection. Newton already has a `Context&` — it just uses `context.sink` and `context.logger` directly. stark has the same `Context&` and does the same.

### 3.6 Newton Printing — Verbosity as Indentation, Sink Does the Work

Instead of a dedicated `NewtonOutputFormatter`, Newton's `_solve_impl()` uses direct `context.sink.print(msg, level)` calls. The sink automatically prepends indentation based on the verbosity level. Callers **never embed manual tabs or spaces** for indentation — they just provide the message content and the verbosity tag.

**Example in `_solve_impl()`:**
```cpp
// Newton iteration line (Step level — sink auto-indents)
sink.print(fmt::format("{:3d}. r = {:.2e} | ph: {:6.2f}% | #CG {:5d} | du = {:.2e}\n",
    it, residual, 100.0 * ratio, n_cg, du_max), Verbosity::Step);

// Projection retry (Full level — deeper indent)
sink.print(fmt::format("{:2d}. p-tol: {:.1e} | ph: {:6.2f}% | #CG {:5d} | -> {}\n",
    retry, p_tol, 100.0 * ratio, n_cg, failure_reason), Verbosity::Full);

// Summary line after solve (Summary level)
sink.print(fmt::format("#newton: {:2d} | #CG/newton: {:2d} | #ls/newton: {:.2f} | {}\n",
    n_its, avg_cg, avg_ls, result_str), Verbosity::Summary);
```

No `should_print()` guards needed for most prints. We accept the `fmt::format` cost even for gated messages — the real performance concern is I/O and flushes, not string formatting. The sink's `print(msg, level)` returns immediately if `level > verbosity_`, so the formatted string is simply discarded. This keeps the Newton code clean.

For **truly hot paths** where formatting cost matters (rare), a guard is still available:
```cpp
if (sink.get_verbosity() >= Verbosity::Full) {
    // expensive formatting only when Full is active
}
```

**Incremental line building** — two approaches under consideration:

**Option A: Local string accumulation (simple)**
```cpp
std::string line = fmt::format("{:3d}. r = {:.2e}", it, residual);
// ... inner loop determines ph, CG, du ...
line += fmt::format(" | ph: {:6.2f}% | #CG {:5d} | du = {:.2e}", 100*ratio, n_cg, du_max);
line += "\n";
sink.print(line, Verbosity::Step);
```

**Option B: `append` + `flush_line` on the sink**
```cpp
sink.append(fmt::format("{:3d}. r = {:.2e}", it, residual), Verbosity::Step);
// ... inner loop ...
sink.append(fmt::format(" | ph: {:6.2f}% | #CG {:5d} | du = {:.2e}", 100*ratio, n_cg, du_max), Verbosity::Step);
sink.flush_line(Verbosity::Step);  // prints accumulated line with auto-indent + "\n"
```

Option B gives the iconic `"AAAA | BBBB | CCCC\n"` pattern without the caller managing a local string, and lets the sink handle the `" | "` separators if we want. Option A is simpler and has no new API surface.

**Decision deferred to implementation.** The target output format is correct and is the goal. We will determine the best ergonomics once we see the actual Newton code being refactored. Both options are compatible with the architecture.

### 3.7 Machine-Readable Log Files (YAML)

The `.log` file written by `Logger::save_to_yaml()` is valid YAML. Summary at the top for human readability, raw series below for Python processing.

**Example output (`twisting_cloth.yaml`):**
```yaml
# Simulation Log — twisting_cloth
# Generated: 2026-02-09 14:30:00

description: |
  twisting_cloth | n=10 | Progressive projection | step_tolerance=0.001

summary:
  simulation_time: 1.033
  n_frames: 31
  n_time_steps: 31
  ndofs: 363

  # label:                  SUM         AVG         MIN         MAX
  dt:                      [1.033333,   0.033333,   0.033333,   0.033333]
  n_newton_iterations:     [137,        4.419355,   1,          20]
  projected_hessians_ratio:[4.817014,   0.035419,   0.000000,   0.956661]

  runtime:
    total:                 [0.079398,   0.002561,   0.000698,   0.013471]
    CG:                    [0.017144,   0.000126,   0.000017,   0.001147]
    energy_eval:           [0.010394,   0.000076,   0.000059,   0.000275]
    project_to_pd:         [0.001591,   0.000033,   0.000004,   0.000176]
    hessian_assembly:      [0.011691,   0.000086,   0.000056,   0.000345]

series:
  dt:                      [0.033333, 0.033333, 0.033333, ...]
  n_newton_iterations:     [4, 3, 5, 7, 4, ...]
  projected_hessians_ratio:[0.0, 0.0, 0.012, 0.956, ...]
  runtime_total:           [0.013, 0.002, 0.003, ...]
  runtime_CG:              [0.001, 0.000, 0.001, ...]
  # ... all series
```

**Python loading:**
```python
import yaml

with open("twisting_cloth.yaml") as f:
    log = yaml.safe_load(f)

# Quick stats
print(log["summary"]["n_time_steps"])  # 31
print(log["summary"]["dt"])            # [1.033, 0.033, 0.033, 0.033]

# Full series for plotting
import matplotlib.pyplot as plt
plt.plot(log["series"]["n_newton_iterations"])
plt.ylabel("Newton iterations per step")
plt.show()
```

**Considerations:**
- YAML is human-readable with an editor and machine-readable with one `yaml.safe_load()` call.
- The summary block at the top gives a quick overview without scrolling through 10,000 data points.
- Series arrays can be long but YAML handles them fine. For very large simulations, we could also offer flow style `[...]` for compactness.
- `description` is a free-form string where stark dumps its settings.

---

## 4. Verbosity Levels

Single enum. Everything uses `symx::Verbosity`.

| Level | Newton output | stark output (if console enabled) | Use case |
|-------|-------------|------------------------------|----------|
| `Silent` | Nothing | Time step header, frame writes | Production monitoring |
| `Summary` | 1 summary line per `solve()` | + Newton summary appended to time step line | Quick dev feedback |
| `Step` | + Per Newton iteration lines | (same) | Normal development |
| `Full` | + Line search, projection retries | (same) | Debugging convergence |

To suppress **all** console output (including stark's own time step headers):
```cpp
settings.output.enable_console_output = false;
// This calls context.sink.set_console_enabled(false)
```

---

## 5. Performance Considerations

### 5.1 Formatting cost is acceptable

We accept the cost of `fmt::format` even when the message will be discarded by the verbosity gate. The sink's `print(msg, level)` is a cheap integer comparison that returns immediately when gated. The real performance concern is **I/O**: flushes, newlines hitting the terminal, and file writes. String formatting on the stack is fast and keeps the calling code clean.

For the rare case of truly expensive formatting (e.g., dumping a matrix), a manual guard is available:
```cpp
if (context.sink.get_verbosity() >= Verbosity::Full) {
    // expensive formatting only when needed
}
```

### 5.2 No flush, no mutex

- **No `std::flush`** on stdout. Line-buffered (`\n`-triggered) is sufficient.
- **No mutex.** The print path is single-threaded. OpenMP parallelism is inside CG/assembly where nobody prints.
- **File flush** is explicit: `context.sink.flush_file()` once per time step.

### 5.3 Timer overhead

`omp_get_wtime()` is ~20 ns. Negligible. For extreme benchmark paranoia: `context.logger.set_enabled(false)` skips everything including timing.

### 5.4 String building

When a line must be built incrementally (e.g., step line where CG count isn't known upfront), either build a local `std::string` and print once, or use `append` + `flush_line` (see §3.6). Either way: **one actual I/O operation per line**. No repeated stdout writes for partial lines.

---

## 6. Nullification for Benchmarks

```cpp
// Silent Newton, but stark still prints time step headers
context.sink.set_verbosity(symx::Verbosity::Silent);

// No console output at all (file logging continues if configured)
settings.output.enable_console_output = false;
// → context.sink.set_console_enabled(false)

// No file logging
context.sink.set_mode(OutputMode::PrintOnly);
// (or just don't call open_file())

// Skip even timer overhead
context.logger.set_enabled(false);
```

---

## 7. Target Output Format

### `Silent` verbosity (stark console enabled):
```
  dt: 33.33 ms | runtime: 2 ms | cr: 0.1
  dt: 33.33 ms | runtime: 3 ms | cr: 0.2
  Writing frame 1...
  dt: 33.33 ms | runtime: 2 ms | cr: 0.1
```

### `Summary` verbosity:
```
  dt: 33.33 ms | #newton: 4 | #CG/newton: 12 | #ls/newton: 0.00 | converged | runtime: 2 ms | cr: 0.1
```

### `Step` verbosity:
```
  dt: 33.33 ms |
    1. r = 1.26e+01 | ph:   0.00% | #CG     1 | du = 3.8e-01
    2. r = 5.18e-03 | ph:   0.00% | #CG    16 | du = 2.4e-01
    3. r = 2.20e-04 | ph:   0.00% | #CG    15 | du = 1.1e-02
    4. r = 9.55e-07 | ph:   0.00% | #CG    16 | du = 2.6e-05
  #newton: 4 | #CG/newton: 12 | #ls/newton: 0.00 | converged | runtime: 2 ms | cr: 0.1
```

### `Full` verbosity:
```
  dt: 33.33 ms |
    1. r = 5.45e-03 |
      0. p-tol:         | ph:   0.00% | #CG     8 | -> Linear system failure
      1. p-tol: 2.7e-03 | ph:  38.52% | #CG     9 | -> Linear system failure
      2. p-tol: 1.4e-03 | ph:  65.81% | #CG    16 |
      ph:  65.81% | #CG    16 | du = 2.7e-01
    2. r = 3.43e-04 | ...
  #newton: 7 | #CG/newton: 15 | #ls/newton: 0.10 | converged | runtime: 3 ms | cr: 0.1
```

### Final summary (stark prints this, always — unless console is disabled):
```
─── Simulation Summary ───
  Name:            twisting_cloth
  Simulation time: 1.033 s
  ndofs:           363
  Frames:          31
  Time steps:      31

  Newton iterations:    137 total | 4.4 avg | [1, 20]
  Projected Hessian:    0.04 avg  | [0.00, 0.96]

  Runtime (s):          0.079 total | 0.003 avg | [0.001, 0.013]
    CG:                 0.017 total | 22% of runtime
    Energy eval:        0.010 total | 13%
    Hessian assembly:   0.012 total | 15%
    Project to PD:      0.002 total |  2%
```

---

## 8. Migration Plan

### Phase 1: symx infrastructure
1. Rename/expand `symx::Verbosity` to the 4-level enum (`Silent`, `Summary`, `Step`, `Full`).
2. Create `symx::OutputSink` in `symx/src/solver/OutputSink.h` with `PrintOnly`/`FileOnly`/`PrintAndFile` modes.
3. Upgrade `symx::Log` → `symx::Logger`: typed series, accumulators, `ScopedTimer time()`, `get_statistics()`, `save_to_yaml()`.
4. Add `OutputSink sink;` and `Logger logger;` as public fields on `symx::Context`.
5. Refactor `NewtonsMethod::_solve_impl()`: replace `_print()` with direct `context.sink.print()` calls using verbosity-as-indentation convention.
6. Replace `NewtonsMethod::_print()` entirely.
7. Add scoped timers inside Newton at key points (CG, energy eval, assembly, projection).
8. **Leave internal symx error/warning/info prints as-is for now.** Focus on simulation output only.

### Phase 2: stark integration
1. Remove `stark::core::Console` class entirely.
2. Remove `stark::core::Logger` class entirely.
3. stark uses `context.sink` for all its own output (time step headers, frame writes, final summary).
4. stark uses `context.logger` for all its own series (dt, runtime, convergence ratio, etc.).
5. `Stark::_initialize()` configures `context.sink` (verbosity, mode, file path, console enable).
6. `Stark::run_one_step()` appends time-step-level data to `context.logger` after Newton solve.
7. Implement `Stark::print()` — a few stark global numbers + key Newton stats from `context.logger`.
8. `Stark::finalize()` calls `context.logger.save_to_yaml(path)`.

### Phase 3: Cleanup
1. Fix `Settings::as_string()` numeric formatting bugs.
2. Remove `symx::Context::print` callback (replaced by `OutputSink`).
3. Remove all stray `std::cout` debug prints.
4. Verify `.yaml` log files load cleanly in Python with `yaml.safe_load()`.

---

## 9. Ownership Summary

| Component | Owner | Location |
|-----------|-------|----------|
| `Verbosity` enum (4-level) | symx | `symx/src/solver/solver_utils.h` |
| `OutputSink` (print + file) | symx | `symx/src/solver/OutputSink.h` |
| `Logger` (timer + series + stats + yaml) | symx | `symx/src/solver/Logger.h` |
| `Context.sink` | symx | `symx/src/solver/Context.h` (public field) |
| `Context.logger` | symx | `symx/src/solver/Context.h` (public field) |
| Newton print calls | symx | `NewtonsMethod.cpp` (direct `context.sink.print()`) |
| Newton timing | symx | `NewtonsMethod.cpp` (scoped `context.logger.time()`) |
| Sink configuration | stark | `Stark::_initialize()` |
| Simulation-level series | stark | `Stark::run_one_step()` |
| Final summary | stark | `Stark::print()` |
| Log file save | stark | `Stark::finalize()` → `context.logger.save_to_yaml()` |

`stark::core::Console` — **deleted**
`stark::core::Logger` — **deleted**

---

## 10. Decisions / Notes

1. **Internal symx prints (errors, warnings, info):** Left as-is for now. They are mostly `std::cerr` or `Context::print` for compilation status. We will deal with them in a separate pass.

3. **Logger is shared.** Newton appends its series (n_iterations, residuals, etc.) and stark appends its own (dt, runtime, frames). They coexist in the same `context.logger`. No pull/push between separate loggers.

4. **`save_to_yaml()`:** Summary statistics at the top (human-readable), raw series at the bottom (machine-readable). Python loads with `yaml.safe_load()`. Description block includes simulation settings for provenance.

5. **No backwards compatibility constraints.** This is a clean build. Old stark::Console/Logger are deleted, not wrapped.

6. **`enable_console_output = false`** maps to `context.sink.set_console_enabled(false)`. File output continues if configured. This is the "truly silent" mode.
