#include <catch2/catch_all.hpp>
#include <utility>

#include <math.h>

#include "BTrack.h"

using namespace BTrackOriginal;

TEST_CASE("BTrack ", "[BTrack]") {
  SECTION("Constructors") {
    //======================================================================
    SECTION("constructorWithNoArguments") {
      BTrack b;

      REQUIRE(b.getHopSize() == 512);
    }

    //======================================================================
    SECTION("constructorWithHopSize") {
      BTrack b(1024);

      REQUIRE(b.getHopSize() == 1024);
    }

    //======================================================================
    SECTION("constructorWithHopSizeAndFrameSize") {
      BTrack b(256, 512);

      REQUIRE(b.getHopSize() == 256);
    }
  }
  SECTION("processingSimpleValues") {

    //======================================================================
    SECTION("processZeroValuedOnsetDetectionFunctionSamples") {
      BTrack b(512);

      long numSamples = 20000;

      std::vector<double> odfSamples;

      int maxInterval = 0;
      int currentInterval = 0;
      int numBeats = 0;

      for (int i = 0; i < numSamples; i++) {
        b.processOnsetDetectionFunctionSample(0.0);

        currentInterval++;

        if (b.beatDueInCurrentFrame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      // check that the maximum interval between beats does not
      // exceed 100 onset detection function samples (~ 1.3 seconds)
      REQUIRE(maxInterval < 100);

      // check that we have at least a beat for every 100 samples
      REQUIRE(numBeats > (numSamples / 100));
    }

    //======================================================================
    SECTION("processRandomOnsetDetectionFunctionSamples") {
      BTrack b(512);

      long numSamples = 20000;

      std::vector<double> odfSamples;

      int maxInterval = 0;
      int currentInterval = 0;
      int numBeats = 0;

      for (int i = 0; i < numSamples; i++) {
        odfSamples.push_back(random() % 1000);
      }

      for (int i = 0; i < numSamples; i++) {
        b.processOnsetDetectionFunctionSample(odfSamples[i]);

        currentInterval++;

        if (b.beatDueInCurrentFrame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      // check that the maximum interval between beats does not
      // exceed 100 onset detection function samples (~ 1.3 seconds)
      REQUIRE(maxInterval < 100);

      // check that we have at least a beat for every 100 samples
      REQUIRE(numBeats > (numSamples / 100));
    }

    //======================================================================
    SECTION("processNegativeOnsetDetectionFunctionSamples") {
      BTrack b(512);

      long numSamples = 20000;

      std::vector<double> odfSamples;

      int maxInterval = 0;
      int currentInterval = 0;
      int numBeats = 0;

      for (int i = 0; i < numSamples; i++) {
        odfSamples.push_back(-1.0 * (random() % 1000));
      }

      for (int i = 0; i < numSamples; i++) {
        b.processOnsetDetectionFunctionSample(odfSamples[i]);

        currentInterval++;

        if (b.beatDueInCurrentFrame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      // check that the maximum interval between beats does not
      // exceed 100 onset detection function samples (~ 1.3 seconds)
      REQUIRE(maxInterval < 100);

      // check that we have at least a beat for every 100 samples
      REQUIRE(numBeats > (numSamples / 100));
    }

    //======================================================================
    SECTION("processSeriesOfDeltaFunctions") {
      BTrack b(512);

      long numSamples = 20000;
      int beatPeriod = 43;

      std::vector<double> odfSamples;

      int maxInterval = 0;
      int currentInterval = 0;
      int numBeats = 0;
      int correct = 0;

      for (int i = 0; i < numSamples; i++) {
        if (i % beatPeriod == 0) {
          odfSamples.push_back(1000);
          correct++;
        } else {
          odfSamples.push_back(0.0);
        }
      }

      for (int i = 0; i < numSamples; i++) {
        b.processOnsetDetectionFunctionSample(odfSamples[i]);

        currentInterval++;

        if (b.beatDueInCurrentFrame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      // check that the maximum interval between beats does not
      // exceed 100 onset detection function samples (~ 1.3 seconds)
      REQUIRE(maxInterval < 100);

      // check that we have at least a beat for every 100 samples
      REQUIRE(numBeats > (numSamples / 100));

      // check that the number of correct beats is larger than 99%
      // of the total number of beats
      REQUIRE(((double)correct) > (((double)numBeats) * 0.99));
    }
  }
}