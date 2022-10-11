
#include <stdio.h>

extern Sway::Application* Sway::CreateApplication();

int main(int argc, char** argv)
{
  printf("Sway engine\n");
  auto app = Sway::CreateApplication();
  app->Run();
  delete app;

  return 0;
}