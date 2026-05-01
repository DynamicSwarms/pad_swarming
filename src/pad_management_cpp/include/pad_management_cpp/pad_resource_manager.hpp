#pragma once

#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include <unordered_map>

class PadResourceManager : public IPadResourceManager
{
    bool is_allowed(std::string name) override;

    void _release(std::string name) override;
private: 

    std::unordered_map<std::string, std::unique_lock<std::mutex>> m_locks;

    std::mutex m_mutex;
};