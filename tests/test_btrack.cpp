#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <math.h>
#include <utility>

#include "beat_tracker.hpp"
#include "utils.h"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinULP;

using namespace btrack;

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

    RealArrayBuffer::Ptr input_buffer =
        std::make_shared<RealArrayBuffer>(hopSize);
    odf.set_input(input_buffer);

    SingleValueBuffer<double>::Ptr output =
        odf.output_cast<SingleValueBuffer<double>>();

    std::vector<double> &data = input_buffer->data();

    SECTION("Constant") {

      for (int i = 0; i < data.size(); i++) {
        data[i] = 1.0;
      }
      odf.execute();
      double res = output->value();
      REQUIRE_THAT(res, WithinAbs(2707, 1));
      odf.execute();

      res = output->value();
      REQUIRE_THAT(res, WithinAbs(985, 1));
    }

    SECTION("Linear") {
      for (int i = 0; i < data.size(); i++) {
        data[i] = i;
      }
      odf.execute();
      double res = output->value();
      REQUIRE_THAT(res, WithinAbs(158214, 1));
      odf.execute();
      res = output->value();
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

      REQUIRE_THAT(b.recent_average_tempo(), WithinAbs(156.605, 0.001));
    }

    //======================================================================
    SECTION("processRandomOnsetDetectionFunctionSamples") {
      BTrack b(512);
      std::srand(0);
      long numSamples = 20000;

      std::vector<double> odfSamples;

      int maxInterval = 0;
      int currentInterval = 0;
      int numBeats = 0;

      for (int i = 0; i < numSamples; i++) {
        odfSamples.push_back(rand() % 1000);
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
      REQUIRE(maxInterval == 60);

      // check that we have at least a beat for every 100 samples
      REQUIRE(numBeats > (numSamples / 100));

      REQUIRE_THAT(b.recent_average_tempo(), WithinAbs(119.654, 0.001));
    }

    //======================================================================
    SECTION("processNegativeOnsetDetectionFunctionSamples") {
      BTrack b(512);
      std::srand(0);
      long numSamples = 20000;

      std::vector<double> odfSamples;

      int maxInterval = 0;
      int currentInterval = 0;
      int numBeats = 0;

      for (int i = 0; i < numSamples; i++) {
        odfSamples.push_back(-1.0 * (std::rand() % 1000));
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
      REQUIRE(maxInterval == 60);

      // check that we have at least a beat for every 100 samples
      REQUIRE(numBeats > (numSamples / 100));

      REQUIRE_THAT(b.recent_average_tempo(), WithinAbs(119.654, 0.001));
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

      REQUIRE_THAT(b.recent_average_tempo(), WithinAbs(120.185, 0.001));
    }
  }
}