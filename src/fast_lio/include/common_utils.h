#include <filesystem>
#include <string>

namespace fs = std::filesystem;

bool create_directory(const std::string& path) {
    if (fs::exists(path)) {
        return fs::is_directory(path);
    }
    return fs::create_directories(path);
}

// Keep a sensor stream monotonic across a device-clock rollback. The
// correction is expressed as an accumulated offset so consumers can continue
// using the repaired timestamp without resetting filter state.
inline bool repair_timestamp(const double raw_ts, const double expected_dt,
                             double &offset, double &last_raw_ts,
                             double &last_ts, double &corrected_ts)
{
    corrected_ts = raw_ts + offset;

    if (last_ts < 0.0) {
        last_raw_ts = raw_ts;
        last_ts = corrected_ts;
        return true;
    }

    if (raw_ts < last_raw_ts) {
        offset += (last_ts + expected_dt) - corrected_ts;
        corrected_ts = raw_ts + offset;
    } else if (offset != 0.0 && raw_ts >= last_ts) {
        // The source clock has caught up with the repaired time axis.
        offset = 0.0;
        corrected_ts = raw_ts;
    }

    if (corrected_ts <= last_ts)
        return false;

    last_raw_ts = raw_ts;
    last_ts = corrected_ts;
    return true;
}
