/**
 * @file SuperMimicMotion.hpp
 * @brief 按 Uni-SuperMimic MotionLib 约定读取 CSV、重采样和构造未来目标。
 */
#pragma once

#include "Utils/CSVReader.hpp"
#include <array>
#include <cmath>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace z
{
    template<size_t JointNumber>
    class SuperMimicMotion
    {
    public:
        using Rotation = std::array<float, 9>;
        using Vector3 = std::array<float, 3>;
        using Quaternion = std::array<float, 4>; // wxyz
        using JointVector = std::array<float, JointNumber>;
        struct Frame
        {
            Vector3 Position{}, LinearVelocity{}, AngularVelocity{};
            Rotation Orientation{};
            JointVector JointPosition{}, JointVelocity{};
        };
        struct Clip
        {
            std::string Name;
            size_t Start, Count;
        };
        std::vector<Frame> Frames;
        std::vector<Clip> Clips;

        void Load(const nlohmann::json& manifest, const std::filesystem::path& directory, double dt)
        {
            Frames.clear(); Clips.clear();
            for (const auto& entry : manifest.at("Motions"))
            {
                const auto name = entry.at("Name").template get<std::string>();
                if (name.empty() || std::any_of(Clips.begin(), Clips.end(), [&name](const Clip& clip) { return clip.Name == name; }))
                    throw std::runtime_error("Invalid/duplicate SuperMimic motion name: " + name);
                const double fps = entry.at("SourceFPS").template get<double>();
                if (!std::isfinite(fps) || fps <= 0) throw std::runtime_error("Invalid SuperMimic source FPS");
                const auto path = directory / entry.at("Path").template get<std::string>();
                auto csv = CSVReader<float>::Create(path.string(), false);
                if (csv->RowSize() < 2 || csv->ColumnSize() != 7 + JointNumber)
                    throw std::runtime_error("SuperMimic CSV requires at least two rows and 36 columns: " + path.string());
                struct RawFrame { Vector3 Position; Quaternion Orientation; JointVector JointPosition; };
                std::vector<RawFrame> raw;
                raw.reserve(csv->RowSize());
                for (size_t i = 0; i < csv->RowSize(); ++i)
                {
                    const auto row = csv->getRow(i);
                    if (row.size() != 7 + JointNumber || !std::all_of(row.begin(), row.end(), [](float v) { return std::isfinite(v); }))
                        throw std::runtime_error("Invalid SuperMimic CSV row: " + path.string());
                    RawFrame frame;
                    std::copy_n(row.begin(), 3, frame.Position.begin());
                    frame.Orientation = {row[6], row[3], row[4], row[5]};
                    Normalize(frame.Orientation); // Validate without changing the source values.
                    std::copy_n(row.begin() + 7, JointNumber, frame.JointPosition.begin());
                    raw.push_back(frame);
                }
                const size_t count = static_cast<size_t>(std::floor((raw.size() - 1) / fps / dt)) + 1;
                if (count < 2) throw std::runtime_error("SuperMimic clip is shorter than one control step");
                std::vector<Frame> sampled(count);
                for (size_t i = 0; i < count; ++i)
                {
                    const float sample = static_cast<float>(i) * static_cast<float>(dt) * static_cast<float>(fps);
                    const size_t lower = std::min(static_cast<size_t>(std::floor(sample)), raw.size() - 2);
                    const float alpha = std::clamp(sample - static_cast<float>(lower), 0.f, 1.f);
                    for (size_t j = 0; j < 3; ++j)
                        sampled[i].Position[j] = raw[lower].Position[j] * (1 - alpha) + raw[lower + 1].Position[j] * alpha;
                    for (size_t j = 0; j < JointNumber; ++j)
                        sampled[i].JointPosition[j] = raw[lower].JointPosition[j] * (1 - alpha) + raw[lower + 1].JointPosition[j] * alpha;
                    sampled[i].Orientation = QuaternionMatrix(Slerp(raw[lower].Orientation, raw[lower + 1].Orientation, alpha));
                }
                // Differentiate before cropping, retaining the original boundary velocities.
                for (size_t i = 0; i < count; ++i)
                {
                    const size_t first = i ? i - 1 : 0, last = std::min(i + 1, count - 1);
                    const float interval = static_cast<float>((last - first) * dt);
                    Vector3 velocity;
                    for (size_t j = 0; j < 3; ++j)
                        velocity[j] = (sampled[last].Position[j] - sampled[first].Position[j]) / interval;
                    sampled[i].LinearVelocity = InverseRotate(sampled[i].Orientation, velocity);
                    const auto increment = MultiplyTranspose(sampled[last].Orientation, sampled[first].Orientation);
                    auto angular = RotationLog(increment);
                    for (auto& value : angular) value /= interval;
                    sampled[i].AngularVelocity = InverseRotate(sampled[i].Orientation, angular);
                    for (size_t j = 0; j < JointNumber; ++j)
                        sampled[i].JointVelocity[j] = (sampled[last].JointPosition[j] - sampled[first].JointPosition[j]) / interval;
                }
                size_t first = 0, last = count - 1;
                if (entry.contains("Range"))
                {
                    const auto& range = entry.at("Range");
                    const double start = range.value("StartTime", 0.0), end = range.value("EndTime", (count - 1) * dt);
                    if (!std::isfinite(start) || !std::isfinite(end) || start < 0 || end < start || start > (count - 1) * dt)
                        throw std::runtime_error("Invalid SuperMimic crop range: " + name);
                    first = std::min(static_cast<size_t>(std::llround(start / dt)), count - 1);
                    last = std::min(static_cast<size_t>(std::llround(end / dt)), count - 1);
                }
                Clips.push_back({name, Frames.size(), last - first + 1});
                Frames.insert(Frames.end(), sampled.begin() + first, sampled.begin() + last + 1);
            }
            if (Clips.empty()) throw std::runtime_error("SuperMimic CSV manifest has no motions");
        }

        void WriteGoal(size_t index, size_t end, std::vector<float>& output) const
        {
            constexpr size_t width = 2 * JointNumber + 15;
            const auto& current = Frames[index];
            const float norm = std::sqrt(current.Orientation[0] * current.Orientation[0] + current.Orientation[3] * current.Orientation[3] + 1e-9f);
            const float c = current.Orientation[0] / norm, s = current.Orientation[3] / norm;
            for (size_t i = 0; i < output.size() / width; ++i)
            {
                const auto& future = Frames[std::min(index + i + 1, end)];
                const auto& r = future.Orientation;
                const float x = future.Position[0] - current.Position[0], y = future.Position[1] - current.Position[1];
                size_t offset = i * width;
                output[offset++] = c * x + s * y;
                output[offset++] = -s * x + c * y;
                output[offset++] = future.Position[2] - current.Position[2];
                for (float value : {c*r[0]+s*r[3], c*r[1]+s*r[4], -s*r[0]+c*r[3], -s*r[1]+c*r[4], r[6], r[7]})
                    output[offset++] = value;
                for (float value : future.LinearVelocity) output[offset++] = value;
                for (float value : future.AngularVelocity) output[offset++] = value;
                for (float value : future.JointPosition) output[offset++] = value;
                for (float value : future.JointVelocity) output[offset++] = value * 0.1f;
            }
        }

    private:
        static Quaternion Normalize(Quaternion q)
        {
            float norm = 0;
            for (float v : q) norm += v * v;
            if (!std::isfinite(norm) || norm < 1e-12f) throw std::runtime_error("Invalid SuperMimic reference quaternion");
            norm = std::sqrt(norm);
            for (auto& value : q) value /= norm;
            return q;
        }
        static Quaternion Slerp(Quaternion first, Quaternion second, float t)
        {
            float dot = 0;
            for (size_t i = 0; i < 4; ++i) dot += first[i] * second[i];
            if (dot < 0) for (auto& value : second) value = -value;
            dot = std::min(std::abs(dot), 1.f);
            const float theta = std::acos(dot), sine = std::sin(theta);
            const float a = sine < 1e-6f ? 1-t : std::sin((1-t)*theta)/(sine+1e-12f);
            const float b = sine < 1e-6f ? t : std::sin(t*theta)/(sine+1e-12f);
            Quaternion result;
            for (size_t i = 0; i < 4; ++i) result[i] = a * first[i] + b * second[i];
            return Normalize(result);
        }
        static Rotation QuaternionMatrix(Quaternion q)
        {
            q = Normalize(q);
            const auto [w,x,y,z] = q;
            return {1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)};
        }
        static Vector3 InverseRotate(const Rotation& r, const Vector3& value)
        {
            Vector3 result{};
            for (size_t i = 0; i < 3; ++i)
                for (size_t j = 0; j < 3; ++j) result[i] += r[3*j+i] * value[j];
            return result;
        }
        static Rotation MultiplyTranspose(const Rotation& a, const Rotation& b)
        {
            Rotation result{};
            for (size_t i = 0; i < 3; ++i)
                for (size_t j = 0; j < 3; ++j)
                    for (size_t k = 0; k < 3; ++k) result[3*i+j] += a[3*i+k] * b[3*j+k];
            return result;
        }
        static Vector3 RotationLog(const Rotation& m)
        {
            const std::array<float,4> diagonal{1+m[0]+m[4]+m[8], 1+m[0]-m[4]-m[8], 1-m[0]+m[4]-m[8], 1-m[0]-m[4]+m[8]};
            const size_t largest = std::distance(diagonal.begin(), std::max_element(diagonal.begin(), diagonal.end()));
            const float v = std::sqrt(std::max(diagonal[largest],1e-10f))/2, d = 4*v;
            Quaternion q;
            switch (largest)
            {
            case 0: q={v,(m[7]-m[5])/d,(m[2]-m[6])/d,(m[3]-m[1])/d}; break;
            case 1: q={(m[7]-m[5])/d,v,(m[1]+m[3])/d,(m[2]+m[6])/d}; break;
            case 2: q={(m[2]-m[6])/d,(m[1]+m[3])/d,v,(m[5]+m[7])/d}; break;
            default:q={(m[3]-m[1])/d,(m[2]+m[6])/d,(m[5]+m[7])/d,v}; break;
            }
            q=Normalize(q);
            if(q[0]<0) for(auto& value:q) value=-value;
            const float sine=std::sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
            const float scale=sine<1e-6f ? 2/std::max(q[0],1e-6f) : 2*std::atan2(sine,q[0])/(sine+1e-12f);
            return {q[1]*scale,q[2]*scale,q[3]*scale};
        }
    };
}
