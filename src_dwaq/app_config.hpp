/**
 * @file app_config.hpp
 * @brief 按构建目标选择配置文件，路径统一相对项目根目录解析。
 */
#pragma once

#include <filesystem>
#include <string>

namespace bitbot_app
{
    inline std::string ResolveConfigPath(const std::filesystem::path& configured_path)
    {
        const auto path = configured_path.is_absolute() ? configured_path
            : std::filesystem::path(PROJECT_ROOT_DIR) / configured_path;
        return path.lexically_normal().string();
    }
}
