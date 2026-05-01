#include "pad_management_cpp/pad_right_lock_base.hpp"

void PadRightLockDouble::lock()
{
    m_mutex_A.lock();
}

void PadRightLockBase::unlock()
{
    m_mutex_A.unlock();
}

bool PadRightLockBase::try_lock()
{
    return m_mutex.try_lock();
}