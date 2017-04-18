#include <cpp_api/navigation_bridge.h>
#include <iostream>

int main(int argc, char **argv)
{
   Navigation nav;
   
   std::cout << nav.position_set(0.0, 0.6, 1.0, 0.12, 0.5, true, false, true, false);
  
   return 0;
}
