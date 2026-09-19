#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace exploration {

// Borrow the payload without copying it. Validate before reading, and respect
// row padding, field offsets and byte order instead of walking to data.end().
class CloudXYZ {
public:
    explicit CloudXYZ(const sensor_msgs::msg::PointCloud2& cloud) : cloud_(cloud)
    {
        if (cloud.width == 0 || cloud.height == 0) {
            valid_ = cloud.data.empty();
            return;
        }
        if (cloud.point_step < sizeof(float) ||
            static_cast<std::uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
            static_cast<std::uint64_t>(cloud.height) * cloud.row_step != cloud.data.size()) return;
        constexpr std::array<const char*, 3> names{"x", "y", "z"};
        std::array<bool, 3> found{};
        for (const auto& field : cloud.fields) {
            for (std::size_t axis = 0; axis < names.size(); ++axis) {
                if (field.name != names[axis]) continue;
                if (found[axis] || field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
                    field.count != 1 || field.offset > cloud.point_step - sizeof(float)) return;
                found[axis] = true;
                offsets_[axis] = field.offset;
            }
        }
        valid_ = found[0] && found[1] && found[2];
        const std::uint16_t native = 1;
        const bool native_big = reinterpret_cast<const unsigned char*>(&native)[0] == 0;
        swap_ = cloud.is_bigendian != native_big;
    }

    bool valid() const { return valid_; }

    template<class Function>
    void for_each(Function&& function) const
    {
        if (!valid_ || cloud_.width == 0 || cloud_.height == 0) return;
        static_assert(sizeof(float) == sizeof(std::uint32_t), "XYZ requires 32-bit floats");
        for (std::size_t row = 0; row < cloud_.height; ++row) {
            for (std::size_t col = 0; col < cloud_.width; ++col) {
                const auto* point = cloud_.data.data() + row * cloud_.row_step + col * cloud_.point_step;
                std::array<float, 3> xyz;
                for (std::size_t axis = 0; axis < xyz.size(); ++axis) {
                    std::uint32_t bits;
                    std::memcpy(&bits, point + offsets_[axis], sizeof(bits));
                    if (swap_) bits = (bits << 24) | ((bits & 0xff00) << 8) |
                                      ((bits >> 8) & 0xff00) | (bits >> 24);
                    std::memcpy(&xyz[axis], &bits, sizeof(bits));
                }
                function(xyz[0], xyz[1], xyz[2]);
            }
        }
    }

private:
    const sensor_msgs::msg::PointCloud2& cloud_;
    std::array<std::uint32_t, 3> offsets_{};
    bool valid_ = false;
    bool swap_ = false;
};

}  // namespace exploration
