#include "exploration/winding/tests/plane2dfrompoints/shared.hpp"

#if defined(__SSE__)

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
      Plane2DFromPoints_SSE(
        OutputX, OutputY, OutputZ, Count, StartX, StartY, EndX, EndY, Normalize
      );
    }
};

INSTANTIATE_TYPED_TEST_SUITE_P(
  X86_SSE_1, Plane2DFromPoints_Implementation, Callback
);

INSTANTIATE_TYPED_TEST_SUITE_P(
  X86_SSE_1, Plane2DFromPoints_WithinError, Callback
);

#endif
