#pragma once

#include <functional>
#include <utility>

namespace rqt_padflies
{
  class PadflieMessageParser
  {
    public:
      PadflieMessageParser() = default;
      ~PadflieMessageParser() = default;

      void set_availability_callback(std::function<void()> callback)
      {
        m_availability_callback = std::move(callback);
      }

      void parse_availability()
      {
        if (m_availability_callback) {
          m_availability_callback();
        }
      }

    private: 
      std::function<void()> m_availability_callback;
  };
}