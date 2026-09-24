#include <catch2/catch_all.hpp>

#include "libslic3r/SlicingEngine.hpp"

#include "test_helpers.hpp"

#include <fstream>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// Records the calls Print dispatches to it and completes the steps an engine must complete.
class RecordingEngine : public SlicingEngine
{
public:
    struct Calls { int process = 0; int export_gcode = 0; std::string path; };
    explicit RecordingEngine(Calls &calls) : m_calls(calls) {}

    SlicingEngineType type() const override { return SlicingEngineType::Classic; }
    SlicingEngineCaps caps() const override { return {}; }

    void process(Print &print, long long *, bool) override
    {
        ++m_calls.process;
        throw_if_canceled(print);
        if (set_started(print, psWipeTower))
            set_done(print, psWipeTower);
        if (set_started(print, psSkirtBrim))
            set_done(print, psSkirtBrim);
    }

    void export_gcode(Print &print, const std::string &path, GCodeProcessorResult *, ThumbnailsGeneratorCallback) override
    {
        ++m_calls.export_gcode;
        m_calls.path = path;
        if (!set_started(print, psGCodeExport))
            return;
        std::ofstream(path) << "; recording engine\n";
        set_done(print, psGCodeExport);
    }

private:
    Calls &m_calls;
};

// Registers `factory` for Classic for the scope, then restores the real Classic engine.
struct ScopedClassicOverride
{
    explicit ScopedClassicOverride(SlicingEngineFactory factory) { register_slicing_engine(SlicingEngineType::Classic, std::move(factory)); }
    ~ScopedClassicOverride()
    {
        register_slicing_engine(SlicingEngineType::Classic, [] { return std::make_unique<ClassicSlicingEngine>(); });
    }
};

} // namespace

TEST_CASE("The Classic engine is registered and has every capability", "[SlicingEngine]")
{
    const std::unique_ptr<SlicingEngine> engine = make_slicing_engine(SlicingEngineType::Classic);
    REQUIRE(engine != nullptr);
    CHECK(dynamic_cast<ClassicSlicingEngine *>(engine.get()) != nullptr);
    CHECK(engine->type() == SlicingEngineType::Classic);
    const SlicingEngineCaps caps = engine->caps();
    CHECK(caps.layers);
    CHECK(caps.tool_ordering);
    CHECK(caps.conflict_check);
    CHECK(caps.cache);
    CHECK(caps.pipeline_hooks);
}

TEST_CASE("An engine type with nothing registered falls back to Classic", "[SlicingEngine]")
{
    // A value past the last enumerator stands in for an engine this build does not register.
    const std::unique_ptr<SlicingEngine> engine = make_slicing_engine(static_cast<SlicingEngineType>(42));
    REQUIRE(engine != nullptr);
    CHECK(engine->type() == SlicingEngineType::Classic);
}

TEST_CASE("Print dispatches process and export_gcode to the engine made at apply", "[SlicingEngine]")
{
    RecordingEngine::Calls calls;
    ScopedClassicOverride override_classic([&calls] { return std::make_unique<RecordingEngine>(calls); });

    Print print;
    Model model;
    init_print({ cube(20) }, print, model, DynamicPrintConfig::full_print_config());
    REQUIRE(dynamic_cast<RecordingEngine *>(&print.engine()) != nullptr);

    const std::string gcode = Slic3r::Test::gcode(print);
    CHECK(calls.process == 1);
    CHECK(calls.export_gcode == 1);
    CHECK_FALSE(calls.path.empty());
    CHECK(gcode.find("; recording engine") != std::string::npos);
    CHECK(print.is_step_done(psSlicingFinished));
    CHECK(print.is_step_done(psGCodeExport));
    // The engine produced no layers; nothing of Classic ran.
    CHECK_FALSE(print.is_step_done(posSlice));
}

TEST_CASE("Clearing a Print drops its engine and the next apply makes a new one", "[SlicingEngine]")
{
    int made = 0;
    RecordingEngine::Calls calls;
    ScopedClassicOverride override_classic([&] { ++made; return std::make_unique<RecordingEngine>(calls); });

    Print print;
    Model model;
    init_print({ cube(20) }, print, model, DynamicPrintConfig::full_print_config());
    CHECK(made == 1);

    // Re-applying the same engine keeps the instance.
    print.apply(model, DynamicPrintConfig::full_print_config());
    CHECK(made == 1);

    print.clear();
    print.apply(model, DynamicPrintConfig::full_print_config());
    CHECK(made == 2);
}
