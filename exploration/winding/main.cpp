#include <iostream>

#include "exploration/winding/library_winding.hpp"

int32_t main(int32_t ArgumentCount, char** Arguments) {
  idVec2 Start { +1.0f, -1.0f };
  idVec2 End { +1.0f, +1.0f };

  idVec3 Plane = idWinding2D::Plane2DFromPoints(Start, End, true);

  std::cout << "X : " << Plane.x << std::endl;
  std::cout << "Y : " << Plane.y << std::endl;
  std::cout << "Z : " << Plane.z << std::endl;

  return 0;
}
