#pragma once
#include "nlohmann/json.hpp"
#include "iostream"
#include "map"
#include "unordered_map"
#include "fstream"
#include "string"
#include "bitbot_kernel/utils/logger.h"

namespace bitbot
{
    class JsonParser
    {
    public:
        static std::unordered_map<std::string, std::string> ParseKeyEvent(const std::string& path, Logger::Console logger)
        {
            std::ifstream file(path);
            if (!file.is_open()) {
                logger->error("Error: Could not open {}", path);
                return std::unordered_map<std::string, std::string>();
            }

            nlohmann::json json;
            file >> json;

            std::unordered_map<std::string, std::string> key_event_map;

            for (const auto& item : json["control"]) {
                std::string kb_key = item["kb_key"];
                std::string event = item["event"];
                key_event_map[kb_key] = event;
            }

            return key_event_map;
        }
    };
};