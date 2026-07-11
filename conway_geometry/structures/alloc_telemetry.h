#pragma once

/*
 * Opt-in allocation telemetry for the AFTP (Allocation-Free Tessellation
 * Pipeline) sizing pass — see design/new/emsdk-upgrade-scalable-allocator.md
 * (Phase 4) in bldrs-ai/conway.
 *
 * Compiled in only when CONWAY_ALLOC_TELEMETRY is defined, and the module is
 * linked with:
 *
 *   -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=free
 *
 * (genie.lua adds both when the CONWAY_ALLOC_TELEMETRY env var is set at
 * project generation time). Release builds are untouched.
 *
 * What it measures: for every face tessellation (an AllocTelemetryScope
 * around AddFaceToGeometry / AddFaceToGeometrySimple), the number of
 * system-allocator calls and the peak live bytes allocated inside the scope.
 * This is exactly the per-face transient footprint a per-thread bump arena
 * must hold, so the aggregate histogram picks the arena size and predicts the
 * spill rate. Counters are thread-local during collection and merged into
 * process-wide atomics on scope exit, so the instrument works identically on
 * the MT build.
 */

#include <cstddef>

namespace conway {

#ifdef CONWAY_ALLOC_TELEMETRY

/** RAII scope marking one face tessellation; nesting is ignored (outermost
 *  scope wins) so helper paths that recurse don't double-count. */
class AllocTelemetryScope {
 public:
  AllocTelemetryScope();
  ~AllocTelemetryScope();

  AllocTelemetryScope(const AllocTelemetryScope&) = delete;
  AllocTelemetryScope& operator=(const AllocTelemetryScope&) = delete;

 private:
  bool outermost_ = false;
};

/** Coarse callsite buckets for attributing where in-scope allocations happen.
 *  Extend as needed; keep Count last. */
enum class AllocSite {
  Other = 0,
  Earcut,
  Cdt,
  SurfaceEval,
  NurbsInverse,
  TriBounds,
  TriBspline,
  TriCylinder,
  TriSphere,
  TriToroidal,
  TriConical,
  TriRevolution,
  TriExtrusion,
  Count
};

/** RAII: set the active allocation-attribution site for the current thread,
 *  restoring the previous one on exit (nests). Allocations made while active
 *  are counted against this site in the telemetry breakdown. */
class AllocTagScope {
 public:
  explicit AllocTagScope(AllocSite site);
  ~AllocTagScope();

  AllocTagScope(const AllocTagScope&) = delete;
  AllocTagScope& operator=(const AllocTagScope&) = delete;

 private:
  AllocSite previous_;
};

/** Print the aggregate histogram/summary to stderr (typically once per model,
 *  from the processor destructor or an explicit binding). */
void DumpAllocTelemetry(const char* label);

/** Reset all aggregate counters (e.g. between models in one process). */
void ResetAllocTelemetry();

#else  // !CONWAY_ALLOC_TELEMETRY — zero-cost stubs

class AllocTelemetryScope {
 public:
  AllocTelemetryScope() {}
};

enum class AllocSite {
  Other = 0,
  Earcut,
  Cdt,
  SurfaceEval,
  NurbsInverse,
  TriBounds,
  TriBspline,
  TriCylinder,
  TriSphere,
  TriToroidal,
  TriConical,
  TriRevolution,
  TriExtrusion,
  Count
};

class AllocTagScope {
 public:
  explicit AllocTagScope(AllocSite) {}
};

inline void DumpAllocTelemetry(const char*) {}
inline void ResetAllocTelemetry() {}

#endif

}  // namespace conway
