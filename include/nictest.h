// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#pragma once

#ifndef RMP_EVAL_NICTEST_H
#define RMP_EVAL_NICTEST_H

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <limits>

#include "reporter.h"

namespace Evaluator
{
  class INicTest
  {
  public:
    virtual ~INicTest() {}
    virtual void Send() = 0;

    // return true if a packet is recevied (helpful for nonblocking sockets)
    virtual bool Receive() = 0;
  };

  struct RunningStats
  {
    // min/max value and the iteration at which they occurred
    int64_t MinValue = std::numeric_limits<int64_t>::max();
    int64_t MaxValue = std::numeric_limits<int64_t>::min();
    size_t  MinIndex = 0;
    size_t  MaxIndex = 0;

    // Welford running mean (store as long double for precision)
    size_t count = 0;
    double Mean = 0.0L;

    void update(int64_t value, size_t idx) {
      // min/max
      if (value < MinValue) { MinValue = value; MinIndex = idx; }
      if (value > MaxValue) { MaxValue = value; MaxIndex = idx; }

      // mean (Welford)
      ++count;
      double delta = static_cast<double>(value) - Mean;
      Mean += delta / static_cast<double>(count);
    }
  };

  struct CadenceStats
  {
    RunningStats HardwareDeltaNanoseconds; // inter-arrival using hardware timestamps
    RunningStats SoftwareDeltaNanoseconds; // inter-arrival using software timestamps
  };

  struct TestParameters
  {
    std::string NicName;
    uint64_t Iterations = 0;
    int SendSleep = 0;
    int SendPriority = 0;
    int ReceivePriority = 0;
    int SendCpu = 0;
    int ReceiveCpu = 0;
    ReportData* SendData = nullptr;
    ReportData* ReceiveData = nullptr;
    bool IsVerbose = false;
    uint64_t BucketWidth = 0;
  };

  class EthercatNicTest : public INicTest
  {
    int socketDescriptor;
    std::mutex mutex;
    std::condition_variable condition;
    CadenceStats stats;
    uint64_t sendIteration = 0;
    uint64_t receiveIteration = 0;
    TestParameters params;
    TimerReport hardwareReport;
    TimerReport softwareReport;

    struct PrevStats
    {
      int64_t HardwareNanoseconds = 0;
      int64_t SoftwareNanoseconds = 0;
      bool HaveHardware = false;
      bool HaveSoftware = false;
    } prev;

    static constexpr std::chrono::seconds SocketTimeout{1};
  public:
    EthercatNicTest(const TestParameters& argParams, TimerReport&& hardwareReport, TimerReport&& softwareReport);
    ~EthercatNicTest() override;
    void Send() override;
    bool Receive() override;
  };

  std::string AppendErrorCode(const std::string_view message);
}

#endif // !defined(RMP_EVAL_NICTEST_H)
