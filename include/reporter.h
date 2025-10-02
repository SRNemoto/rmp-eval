// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#pragma once

#ifndef RMP_EVAL_TIMER_H
#define RMP_EVAL_TIMER_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <time.h>

#include "quantileestimator.h"


namespace Evaluator
{
  inline constexpr uint64_t NanoPerSec = 1e9;
  inline constexpr size_t BucketCount = 5; // 5 buckets

  struct ReportData
  {
    uint64_t min = std::numeric_limits<uint64_t>::max();
    uint64_t max = 0;
    uint64_t sum = 0;
    int minIndex = -1;
    int maxIndex = -1;
    uint64_t observations = 0;
    double median = 0;
    uint64_t target = 0;

    // The "base bucket width". Bucket widths don't scale linearly
    // Example for 4khz bucketWidth = 31250 ns -> buckets are:
    //   [0] [0, 281'250)
    //   [1] [281'250, 312'500)
    //   [2] [312'500, 375'000)
    //   [3] [375'000, 500'000)
    //   [4] [500'000, +inf)
    uint64_t bucketWidth = 0;
    uint64_t buckets[BucketCount] = {};
  };

  struct TableColumn
  {
    std::string Label;
    static constexpr int DefaultColumnWidth = 10;
    static constexpr int PrintPrecision = 0;
    int Width = DefaultColumnWidth;
    using ValueGetterType = std::function<uint64_t(ReportData&)>;
    ValueGetterType ValueGetter;
    static void DefaultFormatter(std::ostream& stream, uint64_t value, int width);
    using ValueFormatterType = std::function<void(std::ostream&, uint64_t, int)>;
    ValueFormatterType ValueFormatter = DefaultFormatter;


    TableColumn(const std::string_view label, int width,
      ValueGetterType valueGetter,
      ValueFormatterType valueFormatter = DefaultFormatter);

    void PrintLabel(std::ostream& stream) const;
    void PrintValue(ReportData& data, std::ostream& stream) const;
  };

  class TableMaker
  {
  public:
    static constexpr char BeginRow[] = "| ";
    static constexpr char Separator[] = " | ";
    static constexpr char Dash = '-';
    static constexpr char DashJoint = '+';
    static constexpr size_t RowLabelWidth = 16;
    static TableMaker CreateTableMaker(uint64_t bucketWidth, bool isVerbose = false);
    TableMaker();
    void AddColumn(TableColumn&& column);
    int PrintLabels(std::ostream& stream) const;
    int PrintRow(std::string_view rowLabel, ReportData& data, std::ostream& stream) const;
  private:
    std::vector<TableColumn> columns;
    static int AddNewLine(std::ostream& stream, int count);
  };

  class TimerReport
  {
  public:
    TimerReport(uint64_t argTarget, uint64_t argBucketWidth, ReportData* argUpload = nullptr);
    void AddObservation(uint64_t observation, int index);

    void PrintReport(bool isVerbose = false, std::ostream& stream = std::cout) const;

    ReportData Snapshot() const;

  private:
    uint64_t min = std::numeric_limits<uint64_t>::max();
    uint64_t max = 0;
    uint64_t sum = 0;
    int minIndex = -1;
    int maxIndex = -1;
    uint64_t observations = 0;
    QuantileEstimator median{0.50};
    ReportData* uploadLocation = nullptr;
    uint64_t target = 0;
    uint64_t bucketWidth = 0;
    uint64_t buckets[BucketCount] = {};
  };

  inline uint64_t ToEpoch(const timespec& time)
  {
    return static_cast<uint64_t>(time.tv_sec) * NanoPerSec + static_cast<uint64_t>(time.tv_nsec);
  }

  inline uint64_t GetCurrentTime(int clockId = CLOCK_MONOTONIC)
  {
    timespec current;
    clock_gettime(clockId, &current);
    return ToEpoch(current);
  }

  class ScopedTimer
  {
  public:
    ScopedTimer(TimerReport& report,  bool& recordTime, int index, const int clockId = CLOCK_MONOTONIC)
      : report(report)
      , recordTime(recordTime)
      , index(index)
      , startTime(GetCurrentTime(clockId))
      , clockId(clockId)
    {}

    ~ScopedTimer()
    {
      if(recordTime)
        report.AddObservation(GetCurrentTime(clockId) - startTime, index);
    }

  private:
    TimerReport& report;
    bool& recordTime;
    int index;
    uint64_t startTime;
    int clockId;
  };

  int FormatDuration(std::chrono::milliseconds startTime, std::ostream& stream = std::cout);
  int FormatDuration(std::chrono::steady_clock::time_point startTime,
    std::chrono::steady_clock::time_point endTime, std::ostream& stream = std::cout);

  // print the duration of the scope
  class DurationReporter
  {
  public:
    DurationReporter(const std::string& msg);

    ~DurationReporter();

  private:
    std::string msg_;
    std::chrono::steady_clock::time_point start_;
  };

} // end namespace Evaluator

#endif // !defined(RMP_EVAL_TIMER_H)
