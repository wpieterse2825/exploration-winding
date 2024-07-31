#include <vector>

#include <celero/Celero.h>

#include "exploration/winding/library_winding.hpp"

CELERO_MAIN

class Plane2DIntersectionFixture : public celero::TestFixture {
  public:
    Plane2DIntersectionFixture()
      : Count { 0 }
      , DidIntersect {}
      , IntersectionX {}
      , IntersectionY {}
      , Plane1X {}
      , Plane1Y {}
      , Plane1Z {}
      , Plane2X {}
      , Plane2Y {}
      , Plane2Z {} {
    }

    virtual std::vector<std::shared_ptr<celero::TestFixture::ExperimentValue>>
    getExperimentValues() const override {
      std::vector<std::shared_ptr<celero::TestFixture::ExperimentValue>>
        ExperimentValues {
          std::make_shared<celero::TestFixture::ExperimentValue>(128),
          std::make_shared<celero::TestFixture::ExperimentValue>(256),
          std::make_shared<celero::TestFixture::ExperimentValue>(512),
          std::make_shared<celero::TestFixture::ExperimentValue>(1024)
        };

      return ExperimentValues;
    }

    virtual void
    setUp(celero::TestFixture::ExperimentValue const* ExperimentValue
    ) override {
      Count = ExperimentValue->Value;

      DidIntersect.resize(Count);
      IntersectionX.resize(Count);
      IntersectionY.resize(Count);
      Plane1X.resize(Count);
      Plane1Y.resize(Count);
      Plane1Z.resize(Count);
      Plane2X.resize(Count);
      Plane2Y.resize(Count);
      Plane2Z.resize(Count);

      for(int32_t Index = 0; Index < Count; Index++) {
        idVec2 P1Start { -1.0f, 0.0f };
        idVec2 P1End { +1.0f, 0.0f };

        idVec2 P2Start { 0.0f, -1.0f };
        idVec2 P2End { 0.0f, +1.0f };

        idVec3 P1Plane = idWinding2D::Plane2DFromPoints(P1Start, P1End, true);
        idVec3 P2Plane = idWinding2D::Plane2DFromPoints(P2Start, P2End, true);

        Plane1X[Index] = P1Plane.x;
        Plane1Y[Index] = P1Plane.y;
        Plane1Z[Index] = P1Plane.z;

        Plane2X[Index] = P2Plane.x;
        Plane2Y[Index] = P2Plane.y;
        Plane2Z[Index] = P2Plane.z;
      }
    }

    virtual void tearDown() override {
      Plane2Z.clear();
      Plane2Y.clear();
      Plane2X.clear();
      Plane1Z.clear();
      Plane1Y.clear();
      Plane1X.clear();
      IntersectionY.clear();
      IntersectionX.clear();
      DidIntersect.clear();

      Count = 0;
    }

  protected:
    int32_t Count;

    std::vector<int8_t> DidIntersect;
    std::vector<float>  IntersectionX;
    std::vector<float>  IntersectionY;
    std::vector<float>  Plane1X;
    std::vector<float>  Plane1Y;
    std::vector<float>  Plane1Z;
    std::vector<float>  Plane2X;
    std::vector<float>  Plane2Y;
    std::vector<float>  Plane2Z;
};

static constexpr auto SAMPLE_COUNT    = 100;
static constexpr auto ITERATION_COUNT = 1000;

BASELINE_F(
  Plane2DIntersection,
  Generic,
  Plane2DIntersectionFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DIntersection_Generic(
    DidIntersect.data(),
    IntersectionX.data(),
    IntersectionY.data(),
    Count,
    Plane1X.data(),
    Plane1Y.data(),
    Plane1Z.data(),
    Plane2X.data(),
    Plane2Y.data(),
    Plane2Z.data()
  );

  celero::DoNotOptimizeAway(DidIntersect[0] == 1);
}

BENCHMARK_F(
  Plane2DIntersection,
  Optimized,
  Plane2DIntersectionFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DIntersection_Optimized(
    DidIntersect.data(),
    IntersectionX.data(),
    IntersectionY.data(),
    Count,
    Plane1X.data(),
    Plane1Y.data(),
    Plane1Z.data(),
    Plane2X.data(),
    Plane2Y.data(),
    Plane2Z.data()
  );

  celero::DoNotOptimizeAway(DidIntersect[0] == 1);
}

#if defined(__SSE__)

BENCHMARK_F(
  Plane2DIntersection,
  SSE,
  Plane2DIntersectionFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DIntersection_SSE(
    DidIntersect.data(),
    IntersectionX.data(),
    IntersectionY.data(),
    Count,
    Plane1X.data(),
    Plane1Y.data(),
    Plane1Z.data(),
    Plane2X.data(),
    Plane2Y.data(),
    Plane2Z.data()
  );

  celero::DoNotOptimizeAway(DidIntersect[0] == 1);
}

#endif

#if defined(__AVX__)

BENCHMARK_F(
  Plane2DIntersection,
  AVX,
  Plane2DIntersectionFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DIntersection_AVX(
    DidIntersect.data(),
    IntersectionX.data(),
    IntersectionY.data(),
    Count,
    Plane1X.data(),
    Plane1Y.data(),
    Plane1Z.data(),
    Plane2X.data(),
    Plane2Y.data(),
    Plane2Z.data()
  );

  celero::DoNotOptimizeAway(DidIntersect[0] == 1);
}

#endif

#if defined(__ARM_FEATURE_SVE)

BENCHMARK_F(
  Plane2DIntersection,
  SVE,
  Plane2DIntersectionFixture,
  SAMPLE_COUNT,
  ITERATION_COUNT
) {
  Plane2DIntersection_SVE(
    DidIntersect.data(),
    IntersectionX.data(),
    IntersectionY.data(),
    Count,
    Plane1X.data(),
    Plane1Y.data(),
    Plane1Z.data(),
    Plane2X.data(),
    Plane2Y.data(),
    Plane2Z.data()
  );

  celero::DoNotOptimizeAway(DidIntersect[0] == 1);
}

#endif
