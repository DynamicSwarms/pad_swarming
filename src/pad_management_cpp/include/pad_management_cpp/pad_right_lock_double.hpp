#pragma once

#include "pad_management_cpp/I_pad_right_lock.hpp"
#include <mutex>


class PadRightLockDouble : public IPadRightLock
{
    void lock() override;
    void unlock() override;
    bool try_lock() override;

private: 
    std::mutex m_mutex_A;
    std::mutex m_mutex_B;
};