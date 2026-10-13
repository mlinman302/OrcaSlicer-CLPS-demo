#include <catch2/catch_all.hpp>

#include "libslic3r/SlicingEngine.hpp"

#include "test_helpers.hpp"
#include "test_utils.hpp"

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Utils.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <set>
#include <thread>

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

// A stand-in second engine with no Classic intermediate data: one square outer wall around each
// instance's footprint per layer, written through the GCodeProcessor tags and parsed back like Classic.
class SquareEngine : public SlicingEngine
{
public:
    std::string       refusal;              // returned by validate() when not empty
    int               layer_delay_ms = 0;   // slows process() down for the cancel test
    std::atomic<bool> started { false };

    SlicingEngineType type() const override { return SlicingEngineType::Classic; }
    SlicingEngineCaps caps() const override { return {}; }

    StringObjectException validate(const Print &, std::vector<StringObjectException> *) const override { return { refusal }; }

    void process(Print &print, long long *, bool) override
    {
        started = true;
        set_status(print, 10, "Square engine");
        m_squares.clear();
        double top = 0.;
        for (const PrintObject *object : print.objects())
            for (size_t i = 0; i < object->model_object()->instances.size(); ++i) {
                const BoundingBoxf3 bb = object->model_object()->instance_bounding_box(i);
                m_squares.push_back(bb);
                top = std::max(top, bb.max.z());
            }
        m_layer_height = print.objects().front()->config().layer_height.value;
        m_layers       = int(std::round(top / m_layer_height));
        for (int i = 0; i < m_layers; ++i) {
            throw_if_canceled(print);
            if (layer_delay_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(layer_delay_ms));
        }
        if (set_started(print, psWipeTower))
            set_done(print, psWipeTower);
        if (set_started(print, psSkirtBrim))
            set_done(print, psSkirtBrim);
    }

    void export_gcode(Print &print, const std::string &path, GCodeProcessorResult *result, ThumbnailsGeneratorCallback) override
    {
        if (!set_started(print, psGCodeExport))
            return;
        {
            std::ofstream out(path);
            out << "G90\nM83\nG1 F1800\n";
            for (int layer = 1; layer <= m_layers; ++layer) {
                out << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) << "\n";
                out << "G1 Z" << layer * m_layer_height << "\n";
                out << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) << ExtrusionEntity::role_to_string(erExternalPerimeter) << "\n";
                out << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) << m_layer_height << "\n";
                out << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) << 0.45 << "\n";
                for (const BoundingBoxf3 &bb : m_squares) {
                    out << "G1 X" << bb.min.x() << " Y" << bb.min.y() << "\n";
                    out << "G1 X" << bb.max.x() << " Y" << bb.min.y() << " E1\n";
                    out << "G1 X" << bb.max.x() << " Y" << bb.max.y() << " E1\n";
                    out << "G1 X" << bb.min.x() << " Y" << bb.max.y() << " E1\n";
                    out << "G1 X" << bb.min.x() << " Y" << bb.min.y() << " E1\n";
                }
            }
        }
        if (result) {
            GCodeProcessor processor;
            const Vec3d origin = print.get_plate_origin();
            processor.set_xy_offset(origin.x(), origin.y());
            processor.apply_config(print.config());
            processor.process_file(path);
            *result = std::move(processor.extract_result());
        }
        set_done(print, psGCodeExport);
    }

    int layers() const { return m_layers; }

private:
    std::vector<BoundingBoxf3> m_squares;
    double                     m_layer_height = 0.;
    int                        m_layers       = 0;
};

// Builds a SquareEngine per factory call and keeps a pointer to the last one.
struct SquareEngineFactory
{
    SquareEngine *last = nullptr;
    std::string   refusal;
    int           layer_delay_ms = 0;
    SlicingEngineFactory factory()
    {
        return [this] {
            auto engine            = std::make_unique<SquareEngine>();
            engine->refusal        = refusal;
            engine->layer_delay_ms = layer_delay_ms;
            last                   = engine.get();
            return engine;
        };
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

TEST_CASE("An engine without layers reaches the end of slicing and fills the preview data", "[SlicingEngine]")
{
    SquareEngineFactory square;
    ScopedClassicOverride override_classic(square.factory());

    // A 20 mm cube at 0.2 mm layers is 100 layers.
    Print print;
    Model model;
    init_print({ cube(20) }, print, model, { { "layer_height", 0.2 }, { "initial_layer_print_height", 0.2 } });
    REQUIRE(square.last != nullptr);

    print.set_status_silent();
    print.process();
    CHECK(print.is_step_done(psSlicingFinished));
    CHECK_FALSE(print.is_step_done(posSlice));
    REQUIRE(square.last->layers() == 100);

    ScopedTemporaryFile temp(".gcode");
    GCodeProcessorResult result;
    print.export_gcode(temp.string(), &result, nullptr);
    CHECK(print.is_step_done(psGCodeExport));
    REQUIRE_FALSE(result.moves.empty());

    std::set<int> extrusion_layers_z; // in 0.01 mm, so Z compares exactly
    bool          outer_wall = false;
    for (const GCodeProcessorResult::MoveVertex &move : result.moves)
        if (move.type == EMoveType::Extrude) {
            extrusion_layers_z.insert(int(std::lround(move.position.z() * 100.)));
            outer_wall |= move.extrusion_role == erExternalPerimeter;
        }
    CHECK(outer_wall);
    REQUIRE(extrusion_layers_z.size() == 100);
    CHECK(*extrusion_layers_z.begin() == 20);   // 0.2 mm
    CHECK(*extrusion_layers_z.rbegin() == 2000); // 20 mm

    const float time = result.print_statistics.modes[size_t(PrintEstimatedStatistics::ETimeMode::Normal)].time;
    CHECK(time > 0.f);
}

TEST_CASE("Canceling a print stops the engine with a CanceledException", "[SlicingEngine]")
{
    SquareEngineFactory square;
    square.layer_delay_ms = 20; // 100 layers: two seconds unless canceled
    ScopedClassicOverride override_classic(square.factory());

    Print print;
    Model model;
    init_print({ cube(20) }, print, model, { { "layer_height", 0.2 }, { "initial_layer_print_height", 0.2 } });
    REQUIRE(square.last != nullptr);
    print.set_status_silent();

    bool canceled = false;
    std::thread worker([&] {
        try {
            print.process();
        } catch (const CanceledException &) {
            canceled = true;
        }
    });
    while (!square.last->started)
        std::this_thread::yield();
    print.cancel();
    worker.join();

    CHECK(canceled);
    CHECK_FALSE(print.is_step_done(psSlicingFinished));
}

TEST_CASE("Print::validate reports the engine's refusal before its own checks", "[SlicingEngine]")
{
    SquareEngineFactory square;
    square.refusal = "The square engine prints one filament only.";
    ScopedClassicOverride override_classic(square.factory());

    Print print;
    Model model;
    init_print({ cube(20) }, print, model, DynamicPrintConfig::full_print_config());
    CHECK(print.validate().string == square.refusal);
}

TEST_CASE("Consumers of Classic data see what an engine without it leaves", "[SlicingEngine]")
{
    // Two cubes on filaments 1 and 2 with the prime tower on: Classic prints a wipe tower here.
    DynamicPrintConfig config = Slic3r::Test::multifilament_config(2, {
        { "enable_prime_tower", true },
        { "wipe_tower_x", "50" }, // inside the 200x200 test bed
        { "wipe_tower_y", "50" },
    });
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ { { "extruder", "1" } }, { { "extruder", "2" } } };
    {
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{ cube(20), cube(20) }, print, model, config, &overrides);
        REQUIRE(print.has_wipe_tower());
    }

    SquareEngineFactory square;
    ScopedClassicOverride override_classic(square.factory());
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{ cube(20), cube(20) }, print, model, config, &overrides);
    const SlicingEngineCaps caps = print.engine_caps();
    CHECK_FALSE(caps.layers);
    CHECK_FALSE(caps.tool_ordering);
    CHECK_FALSE(caps.cache);
    CHECK_FALSE(print.has_wipe_tower());

    ScopedTemporaryDir cache;
    CHECK(print.export_cached_data(cache.string()) == 0);
    CHECK(boost::filesystem::is_empty(cache.path()));
    CHECK(print.load_cached_data(cache.string()) == CLI_IMPORT_CACHE_NOT_FOUND);
}
