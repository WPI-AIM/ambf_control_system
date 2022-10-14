#pragma once

extern Sway::Application* Sway::CreateApplication();

int main(int argc, char** argv)
{
  printf("Sway engine\n");
  Sway::Log::Init();

  // Sway::Log::GetCoreLogger()->warn("Initialized Core");
  SW_CORE_TRACE("Initialized Core");
  SW_CORE_DEBUG("Initialized Core");
  SW_CORE_INFO("Initialized Core");
  SW_CORE_WARN("Initialized Core");
  SW_CORE_ERROR("Initialized Core");

  SW_CLIENT_TRACE("Initialized Core");
  SW_CLIENT_DEBUG("Initialized Core");
  SW_CLIENT_INFO("Initialized Core");
  SW_CLIENT_WARN("Initialized Core");
  SW_CLIENT_ERROR("Initialized Core");

  Sway::Log::GetClientLogger()->info("Hello client");
  auto app = Sway::CreateApplication();
  app->Run();
  delete app;

  return 0;
}