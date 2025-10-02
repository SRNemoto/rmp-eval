// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#pragma once

#ifndef RMP_EVALCONFIG_H
#define RMP_EVAL_CONFIG_H

#include <optional>
#include <string>
#include <string_view>

namespace Evaluator
{
  // Configuration constants

  inline constexpr const char* DefaultNicName = "";
  inline constexpr size_t MaxFileSize = 1 << 20;  // 1MB max file read
  inline constexpr size_t ReadBufferSize = 4096;
  inline constexpr size_t MaxOutputSize = 65536;  // Max command output size
  inline constexpr int MaxIrqsToShow = 6;
  inline constexpr const char* CpuPrefix = "CPU";  // CPU column prefix in /proc/interrupts

  // Result modeling

  enum class CheckKind
  {
    PreemptRTActive,
    CoreIsolated,
    NohzFull,
    CpuGovernor,
    CpuFrequency,
    RcuNoCbs,
    IrqAffinityDefaultAvoidsRt,
    NoUnrelatedIrqsOnRt,
    NicPresent,
    NicIrqsPinned,
    RpsDisabled,
    NicLinkUp,
    NicQuiet,
    RtThrottlingDisabled,
    SwapDisabled,
    DeepCStatesCapped,
    TurboBoostPolicy,
    ClocksourceStable,
    SmtSiblingIsolated,
    TimerMigration,
  };
  
  enum class Status
  {
    Pass,
    Fail,
    Unknown
  };
  
  struct CheckResult
  {
    CheckKind kind;
    Status status;
    std::string name;   // pretty name
    std::string reason; // terse why/value
  };
  
  struct CheckContext
  {
    std::optional<int> cpu;
    std::optional<std::string> nic;
  };

  // Core interfaces

  enum class Domain { Cpu, Nic, System };
  
  class IDataSource
  {
  public:
    virtual ~IDataSource() = default;
    [[nodiscard]] virtual std::optional<std::string> Read(const std::string &path) const = 0;
    [[nodiscard]] virtual std::optional<std::string> CmdLineParam(std::string_view key) const = 0;
  };
  
  class ICheck
  {
  public:
    virtual ~ICheck() = default;
    virtual CheckKind Kind() const noexcept = 0;
    virtual const std::string& Name() const noexcept = 0;
    virtual Domain GetDomain() const noexcept = 0;
    [[nodiscard]] virtual CheckResult Evaluate(const CheckContext&, const IDataSource&) const = 0;
  };

  // Public functions

  std::string GetCpuInfo();
  std::string GetKernelInfo();

  void ReportSystemConfiguration(int cpu, std::string_view nicName = DefaultNicName);
}


#endif // !defined(RMP_EVAL_CONFIG_H)
