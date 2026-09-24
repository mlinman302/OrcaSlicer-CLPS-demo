#pragma once

#include "Print.hpp"

#include <functional>
#include <memory>

namespace Slic3r {

// What an engine leaves behind besides the G-code and GCodeProcessorResult.
// Consumers check these instead of assuming Classic's intermediate data exists.
struct SlicingEngineCaps {
    bool layers         = false; // fills PrintObject::layers()/support_layers(); marks pos* steps done
    bool tool_ordering  = false; // fills Print::tool_ordering() / wipe tower data
    bool conflict_check = false; // fills Print::get_conflict_result()
    bool cache          = false; // supports Print::export_cached_data / load_cached_data
    bool pipeline_hooks = false; // fires SlicingPipelineStepPlugin hooks
};

// Turns an applied Print (PrintObjects, regions, configs) into G-code. Selected per print by
// the slicing_engine option; Print::process() and Print::export_gcode() dispatch to it.
class SlicingEngine {
public:
    virtual ~SlicingEngine() = default;
    virtual SlicingEngineType type() const = 0;
    virtual SlicingEngineCaps caps() const = 0;

    // Worker thread. Must reach psSlicingFinished; must call throw_if_canceled() often.
    virtual void process(Print &print, long long *time_cost_with_cache, bool use_cache) = 0;

    // Worker thread. Writes G-code to `path`, fills `result`, marks psGCodeExport done.
    virtual void export_gcode(Print &print, const std::string &path, GCodeProcessorResult *result,
                              ThumbnailsGeneratorCallback thumbnail_cb) = 0;

protected:
    // Step-state and cancel access, so each engine does not need friendship with Print.
    static bool set_started(Print &print, PrintStep step) { return print.set_started(step); }
    static void set_done(Print &print, PrintStep step) { print.set_done(step); }
    static bool set_started(PrintObject &object, PrintObjectStep step) { return object.set_started(step); }
    static void set_done(PrintObject &object, PrintObjectStep step) { object.set_done(step); }
    static void throw_if_canceled(const Print &print) { print.throw_if_canceled(); }
    static void set_status(const Print &print, int percent, const std::string &message) { print.set_status(percent, message); }
};

// The existing OrcaSlicer pipeline. Forwards to the bodies that stay in Print.cpp.
class ClassicSlicingEngine : public SlicingEngine {
public:
    SlicingEngineType type() const override { return SlicingEngineType::Classic; }
    SlicingEngineCaps caps() const override { return { true, true, true, true, true }; }
    void process(Print &print, long long *time_cost_with_cache, bool use_cache) override;
    void export_gcode(Print &print, const std::string &path, GCodeProcessorResult *result,
                      ThumbnailsGeneratorCallback thumbnail_cb) override;
};

using SlicingEngineFactory = std::function<std::unique_ptr<SlicingEngine>()>;

// Engines register themselves by type; Classic is registered by libslic3r. An engine living in
// another layer (GUI, plugin, external-process wrapper) registers at startup, the same way
// Print::set_slicing_pipeline_hook_fn is injected, keeping libslic3r free of those dependencies.
void register_slicing_engine(SlicingEngineType type, SlicingEngineFactory factory);
// Falls back to Classic when nothing is registered for `type`.
std::unique_ptr<SlicingEngine> make_slicing_engine(SlicingEngineType type);

} // namespace Slic3r
