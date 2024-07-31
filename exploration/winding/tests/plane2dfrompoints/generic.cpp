#include "exploration/winding/tests/plane2dfrompoints/shared.hpp"

struct Callback {
    static void Compute(
      float*       OutputX,
      float*       OutputY,
      float*       OutputZ,
      int32_t      Count,
      float const* StartX,
      float const* StartY,
      float const* EndX,
      float const* EndY,
      bool         Normalize
    ) {
      Plane2DFromPoints_Generic(
        OutputX, OutputY, OutputZ, Count, StartX, StartY, EndX, EndY, Normalize
      );
    }
};

INSTANTIATE_TYPED_TEST_SUITE_P(
  Generic, Plane2DFromPoints_Implementation, Callback
);
