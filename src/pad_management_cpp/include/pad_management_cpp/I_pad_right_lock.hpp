#pragma once

class IPadRightLock
{
public:
    virtual ~IPadRightLock() = default;

    virtual void lock() = 0;

    virtual void unlock() = 0;

    virtual bool try_lock() = 0;
};