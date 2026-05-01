#pragma once

#include <mutex>

class IPadResourceManager
{
public:
    virtual ~IPadResourceManager() = default;

    bool can_do_stuff(std::string name)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return is_allowed(name);
    };

    void release(std::string name)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        _release(name);
    };

private: 
    virtual bool is_allowed(std::string name) = 0; 
    virtual void _release(std::string name) = 0;

    std::mutex m_mutex;
};