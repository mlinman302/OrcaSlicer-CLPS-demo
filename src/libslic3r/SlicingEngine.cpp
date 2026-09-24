#include "SlicingEngine.hpp"

#include <map>
#include <mutex>

namespace Slic3r {

void ClassicSlicingEngine::process(Print &print, long long *time_cost_with_cache, bool use_cache)
{
    print.process_classic(time_cost_with_cache, use_cache);
}

void ClassicSlicingEngine::export_gcode(Print &print, const std::string &path, GCodeProcessorResult *result,
                                        ThumbnailsGeneratorCallback thumbnail_cb)
{
    print.export_gcode_classic(path, result, thumbnail_cb);
}

namespace {
struct Registry {
    std::mutex                                       mutex;
    std::map<SlicingEngineType, SlicingEngineFactory> factories{
        { SlicingEngineType::Classic, [] { return std::make_unique<ClassicSlicingEngine>(); } }
    };
};

Registry &registry()
{
    static Registry r;
    return r;
}
} // namespace

void register_slicing_engine(SlicingEngineType type, SlicingEngineFactory factory)
{
    Registry &r = registry();
    std::scoped_lock lock(r.mutex);
    r.factories[type] = std::move(factory);
}

std::unique_ptr<SlicingEngine> make_slicing_engine(SlicingEngineType type)
{
    Registry &r = registry();
    std::scoped_lock lock(r.mutex);
    auto it = r.factories.find(type);
    if (it != r.factories.end() && it->second)
        if (std::unique_ptr<SlicingEngine> engine = it->second())
            return engine;
    return std::make_unique<ClassicSlicingEngine>();
}

} // namespace Slic3r
