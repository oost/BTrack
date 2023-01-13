#include <catch2/catch_all.hpp>
#include <utility>

#include <math.h>

#include "beat_tracker.hpp"
#include "utils.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinULP;

TEST_CASE("BTrack ", "[BTrack]") {
  SECTION("Constructors") {
    //======================================================================
    SECTION("constructorWithNoArguments") {
      BTrack b;

      REQUIRE(b.get_hop_size() == 512);
    }

    //======================================================================
    SECTION("constructorWithHopSize") {
      BTrack b(1024);

      REQUIRE(b.get_hop_size() == 1024);
    }

    //======================================================================
    SECTION("constructorWithHopSizeAndFrameSize") {
      BTrack b(256, 512);

      REQUIRE(b.get_hop_size() == 256);
    }
  }

  SECTION("Utils") {
    std::vector<double> v{1, 2, 3};

    SECTION("normalizeArray") {
      normalize_array(v);
      REQUIRE_THAT(v[0], WithinAbs(1. / 6, 0.00001));
      REQUIRE_THAT(v[1], WithinAbs(2. / 6, 0.00001));
      REQUIRE_THAT(v[2], WithinAbs(3. / 6, 0.00001));
    }

    SECTION("calculateMeanOfArray") {
      double m = calculate_mean_of_array(v.begin(), v.end());
      REQUIRE_THAT(m, WithinAbs(2., 0.00001));
    }
  }

  SECTION("Onset detection function") {
    int hopSize = 512;
    int frameSize = 1024;
    OnsetDetectionFunction odf(hopSize, frameSize);

    SECTION("Constant") {
      std::vector<double> data(hopSize, 1.0);
      for (int i = 0; i < hopSize; i++) {
      }
      double res = odf.calculate_onset_detection_function_sample(data);
      REQUIRE_THAT(res, WithinAbs(2707, 1));
      res = odf.calculate_onset_detection_function_sample(data);
      REQUIRE_THAT(res, WithinAbs(985, 1));
    }

    SECTION("Linear") {
      std::vector<double> data(hopSize, 1.0);
      for (int i = 0; i < hopSize; i++) {
        data[i] = i;
      }
      double res = odf.calculate_onset_detection_function_sample(data);
      REQUIRE_THAT(res, WithinAbs(158214, 1));
      res = odf.calculate_onset_detection_function_sample(data);
      REQUIRE_THAT(res, WithinAbs(1296727, 1));
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
        b.process_onset_detection_function_sample(0.0);

        currentInterval++;

        if (b.beat_due_in_current_frame()) {
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
        b.process_onset_detection_function_sample(odfSamples[i]);

        currentInterval++;

        if (b.beat_due_in_current_frame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      // check that the maximum interval between beats does not
      // exceed 100 onset detection function samples (~ 1.3 seconds)
      REQUIRE(maxInterval == 68);

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
        b.process_onset_detection_function_sample(odfSamples[i]);

        currentInterval++;

        if (b.beat_due_in_current_frame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      // check that the maximum interval between beats does not
      // exceed 100 onset detection function samples (~ 1.3 seconds)
      REQUIRE(maxInterval == 62);

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
        b.process_onset_detection_function_sample(odfSamples[i]);

        currentInterval++;

        if (b.beat_due_in_current_frame()) {
          numBeats++;

          if (currentInterval > maxInterval) {
            maxInterval = currentInterval;
          }

          currentInterval = 0;
        }
      }

      REQUIRE(maxInterval == 49);

      // check that the number of correct beats is larger than 99%
      // of the total number of beats
      REQUIRE(std::abs(static_cast<double>(numBeats) / correct - 1) < 0.01);
    }
  }
}