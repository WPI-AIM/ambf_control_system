#pragma once

// #include "Sway/Core.h"

namespace Sway {
  class Application
  {
  public:
    Application();
    virtual ~Application();

    void Run();
  };
  
  // To be defined in CLIENT
  Application* CreateApplication();
} // namespace Sway
