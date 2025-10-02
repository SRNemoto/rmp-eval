// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#pragma once

#ifndef RMP_EVAL_QUANTILEESTIMATOR_H
#define RMP_EVAL_QUANTILEESTIMATOR_H

#include <array>

namespace Evaluator
{
  // This class is an implementation of the P^2 algorithm in the paper 
  // "The P^2 Algorithm for Dynamic Calculation of Quantiles and Histograms Without Storing Observations" by 
  // Raj Jain and Imrich Chlamtac. cse.wustl.edu/~jain/papers/ftp/psqr.pdf
  class QuantileEstimator
  {
  private:
    int numObservations;
    const double quantile;
    constexpr static int NUM_MARKERS = 5;
    std::array<double, NUM_MARKERS> markerHeights, markerPositions, desiredMarkerPositions, desiredMarkerPositionIncrements;
  
    void AddInitialObservation(const double observation);
    void AdjustMarkerPositions(const double observation);
    void AdjustMarkerHeights();
    double Parabolic(const int index, const int increment);
    double Linear(const int index, const int increment);
  
  public:
    QuantileEstimator(const double _quantile);
  
    double GetQuantile() const;
  
    void AddObservation(const double observation);
  };
} // end namespace Evaluator

#endif // !defined(RMP_EVAL_QUANTILEESTIMATOR_H)
