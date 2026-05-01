#include "pad_management_cpp/pad_right_lock_base.hpp"

void PadRightLockBase::lock()
{
    m_mutex.lock();
}

void PadRightLockBase::unlock()
{
    m_mutex.unlock();
}

bool PadRightLockBase::try_lock()
{
    return m_mutex.try_lock();
}