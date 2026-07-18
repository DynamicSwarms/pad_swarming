#pragma once

class ICommandContext
{
public:
    virtual ~ICommandContext() = default;

    virtual bool is_healthy() const = 0;

    virtual bool can_takeoff() const = 0;
    virtual bool can_land() const = 0;
    virtual bool is_flying() const = 0;
};