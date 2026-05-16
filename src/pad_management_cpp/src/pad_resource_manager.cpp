#include "pad_management_cpp/pad_resource_manager.hpp"

#include <iostream>
#include <ostream>
bool 
PadResourceManager::is_allowed(std::string name)
{
    if (m_locks.find(name) != m_locks.end()) {
        std::cerr << "should not happen!!! only call once per name" << std::endl;
        return false;
    }
    std::unique_lock<std::mutex> lock(m_mutex, std::defer_lock);
    if (lock.try_lock()) {
        m_locks.emplace(name, std::move(lock));
        return true;
    } else {
        return false;
    }
}

void 
PadResourceManager::_release(std::string name)
{
    auto it = m_locks.find(name);
    if (it != m_locks.end()) {
        m_locks.erase(it); // Should release
    } else {
        std::cerr << "should not happen!!! trying to release unowned lock" << std::endl;
    }
}