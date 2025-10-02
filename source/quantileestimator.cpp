// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#include <algorithm>

#include "quantileestimator.h"

namespace Evaluator
{
  void QuantileEstimator::AddInitialObservation(const double observation)
  {
    markerHeights[numObservations] = observation;
    if (++numObservations == NUM_MARKERS)
    {
      std::sort(markerHeights.begin(), markerHeights.end());
    }
  }
  
  void QuantileEstimator::AdjustMarkerPositions(const double observation)
  {
    int increment_bound;
    if      (observation < markerHeights[0])   { increment_bound = 0; markerHeights[0] = observation; }
    else if (observation < markerHeights[1])   { increment_bound = 0; }
    else if (observation < markerHeights[2])   { increment_bound = 1; }
    else if (observation < markerHeights[3])   { increment_bound = 2; }
    else if (observation <= markerHeights[4])  { increment_bound = 3; }
    else                                        { increment_bound = 3; markerHeights[4] = observation; }
  
    // Increment the appropriate markers that the given observation falls between.
    // Markers for segments of the dataset that are greater than the current observation are also affected.
    for (int index = increment_bound + 1; index < NUM_MARKERS; index++ ) { ++markerPositions[index]; }
  
    // Updated the desired marker positions.
    for (int index = 0; index < NUM_MARKERS; index++)
    { desiredMarkerPositions.at(index) += desiredMarkerPositionIncrements[index]; }
  }
  
  void QuantileEstimator::AdjustMarkerHeights()
  {
    for (int index = 1; index <= 3; index++)
    {
      double& marker_position = markerPositions[index];
      double difference = desiredMarkerPositions[index] - marker_position;
  
      // If the difference between the current marker position and the desired marker position is greater than one and
      // If distance to the adjacent markers is also greater than one.
      if ((difference >= 1 && markerPositions[index + 1] - marker_position > 1) || 
          (difference <= -1 && markerPositions[index - 1] - marker_position < -1))
      {
        int increment = difference > 0 ? 1 : -1;
        double candidate = Parabolic(index, increment);
  
        // If the candidate is between the adjacent marker heights
        if (markerHeights[index - 1] < candidate && candidate < markerHeights[index + 1])
        {
          // use the candidate calculated by the parabolic function
          markerHeights[index] = candidate;
        }
        else
        {
          // otherwise use a value calculated by a linear function
          markerHeights[index] = Linear(index, increment);
        }
        // increment the marker's position
        marker_position += increment;
      }
    }
  }
  
  double QuantileEstimator::Parabolic(const int index, const int increment)
  {
    double height = markerHeights[index];
    const int next = index+1, prev = index-1;
    const double prev_position = markerPositions[prev];
    const double next_position = markerPositions[next];
    const double factor = increment / (next_position - prev_position);
  
    const double cur_position = markerPositions[index];
    const double addend1 = (cur_position - prev_position + increment) * (markerHeights[next] - height) / (next_position - cur_position);
    const double addend2 = (next_position - cur_position - increment) * (height - markerHeights[prev]) / (cur_position - prev_position);
    const double parabolic = height + factor * (addend1 + addend2);
    return parabolic;
  }
  
  double QuantileEstimator::Linear(const int index, const int increment)
  {
    double height = markerHeights[index];
    return height + increment * (markerHeights[index + increment] - height) / (markerPositions[index + increment] - markerPositions[index]);
  }
  
  QuantileEstimator::QuantileEstimator(const double _quantile)
    : numObservations(0)
    , quantile(_quantile)
    , markerHeights{0, 0, 0, 0, 0}
    , markerPositions{0, 1, 2, 3, 4}
    , desiredMarkerPositions{0, 1, 2, 3, 4}
    , desiredMarkerPositionIncrements{0, quantile / 2, quantile, (1+quantile) / 2, 1}
  {}
  
  double QuantileEstimator::GetQuantile() const { return markerHeights[2]; }
  
  void QuantileEstimator::AddObservation(const double observation)
  {
    if (numObservations < NUM_MARKERS)
    {
      AddInitialObservation(observation);
      return;
    }
  
    AdjustMarkerPositions(observation);
  
    numObservations++;
  
    AdjustMarkerHeights();
  }
} // end namespace Evaluator
