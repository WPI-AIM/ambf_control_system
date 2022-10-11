#include <Sway.h>

class Sandbox : public Sway::Application
{
public:
  Sandbox() 
  {

  }

  ~Sandbox()
  {

  }
};


Sway::Application* Sway::CreateApplication()
{
  return new Sandbox();
}