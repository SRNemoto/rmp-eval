// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#include "config.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ifaddrs.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <net/if.h>
#include <netinet/in.h>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

// Small utility and helper functions
namespace
{
  [[nodiscard]] std::string Trim(std::string str)
  {
    auto notspace = [](unsigned char ch) { return !std::isspace(ch); };
    str.erase(str.begin(), std::find_if(str.begin(), str.end(), notspace));
    str.erase(std::find_if(str.rbegin(), str.rend(), notspace).base(), str.end());
    return str;
  }
  
  [[nodiscard]] std::optional<std::string> Slurp(const fs::path &path, size_t max_bytes = Evaluator::MaxFileSize)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return std::nullopt;
    std::string out;
    out.reserve(1024);
    char buf[Evaluator::ReadBufferSize];
    size_t total = 0;
    while (ifs.good())
    {
      ifs.read(buf, sizeof(buf));
      std::streamsize got = ifs.gcount();
      if (got <= 0) break;
      if (total + static_cast<size_t>(got) > max_bytes) got = static_cast<std::streamsize>(max_bytes - total);
      out.append(buf, static_cast<size_t>(got));
      total += static_cast<size_t>(got);
      if (total >= max_bytes) break;
    }
    return out;
  }
  
  [[nodiscard]] bool PathExists(const fs::path &path) noexcept
  {
    std::error_code error_code;
    return fs::exists(path, error_code);
  }
  
  // Parse CPU list strings like "1-3,5,7-8"
  [[nodiscard]] std::set<int> ParseCpuList(const std::string &str)
  {
    std::set<int> out;
    std::string trimmed = Trim(str);
    if (trimmed.empty()) return out;
    std::stringstream stream(trimmed);
    std::string token;
    while (std::getline(stream, token, ','))
    {
      token = Trim(token);
      if (token.empty()) continue;
      auto dash = token.find('-');
      try
      {
        if (dash == std::string::npos)
        {
          int value = std::stoi(token);
          out.insert(value);
        }
        else
        {
          int start = std::stoi(token.substr(0, dash));
          int end = std::stoi(token.substr(dash + 1));
          if (start > end) std::swap(start, end);
          for (int i = start; i <= end; ++i) out.insert(i);
        }
      }
      catch (...) { /* ignore malformed tokens */ }
    }
    return out;
  }
  
  [[nodiscard]] std::optional<std::string> ReadCmdLine()
  {
    return Slurp("/proc/cmdline");
  }
  
  [[nodiscard]] std::optional<std::string> GetCmdLineParam(std::string_view key)
  {
    auto cmd = ReadCmdLine();
    if (!cmd) return std::nullopt;
    std::istringstream iss(*cmd);
    std::string tok;
    while (iss >> tok)
    {
      auto eq = tok.find('=');
      if (eq == std::string::npos)
      {
        if (tok == key) return std::string{}; // boolean-like flag
        continue;
      }
      if (tok.substr(0, eq) == key)
      {
        return tok.substr(eq + 1);
      }
    }
    return std::nullopt;
  }

  [[nodiscard]] std::string CpuModelString()
  {
    // Try lscpu first (if available)
    if (FILE *pipe = popen("LC_ALL=C lscpu 2>/dev/null", "r"))
    {
      std::string output;
      char buffer[512];
      while (fgets(buffer, sizeof(buffer), pipe))
      {
        output.append(buffer);
    if (output.size() > static_cast<size_t>(Evaluator::MaxOutputSize)) break;
      }
      pclose(pipe);
      if (!output.empty())
      {
        std::istringstream lscpu_stream(output);
        std::string line;
        auto take_after_colon = [](const std::string &s) {
          auto pos = s.find(":");
          return pos == std::string::npos ? std::string() : Trim(s.substr(pos + 1));
        };
        // 1) Model name
        lscpu_stream.clear(); lscpu_stream.seekg(0);
        while (std::getline(lscpu_stream, line)) {
          if (line.rfind("Model name:", 0) == 0) {
            auto value = take_after_colon(line);
            if (!value.empty())
              return value;
          }
        }
        // 2) Hardware
        lscpu_stream.clear(); lscpu_stream.seekg(0);
        while (std::getline(lscpu_stream, line)) {
          if (line.rfind("Hardware:", 0) == 0) {
            auto value = take_after_colon(line);
            if (!value.empty())
              return value;
          }
        }
        // 3) Architecture (fallback)
        lscpu_stream.clear(); lscpu_stream.seekg(0);
        while (std::getline(lscpu_stream, line)) {
          if (line.rfind("Architecture:", 0) == 0) {
            auto value = take_after_colon(line);
            if (!value.empty())
              return value;
          }
        }
      }
    }
  
    // Fallback: /proc/cpuinfo
    if (auto ci = Slurp("/proc/cpuinfo"))
    {
      std::istringstream iss(*ci);
      std::string line;
      const char *keys[] = { "model name", "Hardware", "Processor", "cpu model" };
      while (std::getline(iss, line))
      {
        for (const char *k : keys)
        {
          if (line.rfind(k, 0) == 0)
          {
            auto pos = line.find(":");
            if (pos != std::string::npos)
            {
              auto v = Trim(line.substr(pos + 1));
              if (!v.empty()) return v;
            }
          }
        }
      }
    }
  
    struct utsname uts = {};
    if (uname(&uts) == 0) return std::string(uts.machine);
    return std::string("Unknown CPU");
  }

  bool NicExists(const Evaluator::IDataSource& dataSource, const std::string& nic)
  {
    return dataSource.Read(std::string("/sys/class/net/") + nic + "/operstate") ||
           dataSource.Read(std::string("/sys/class/net/") + nic + "/carrier") ||
           dataSource.Read(std::string("/sys/class/net/") + nic + "/address");
  }

  // Pretty printing

  const char* Color(Evaluator::Status status)
  {
    switch (status)
    {
      case Evaluator::Status::Pass: return "\033[32m";   // green
      case Evaluator::Status::Fail: return "\033[31m";   // red
      case Evaluator::Status::Unknown: return "\033[33m"; // yellow
    }
    return "\033[0m";
  }
  
  const char* Emoji(Evaluator::Status status)
  {
    switch (status)
    {
      case Evaluator::Status::Pass: return "✔️";
      case Evaluator::Status::Fail: return "❌";
      case Evaluator::Status::Unknown: return "❔";
    }
    return "";
  }
  
  void PrintSectionHeader(const std::string &title)
  {
    std::cout << "\n" << title << "\n";
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
  }

  void PrintResult(const Evaluator::CheckResult &result)
  {
    constexpr int name_width = 36;
    std::string left_side = result.name;
    if ((int)left_side.size() < name_width) left_side.append(name_width - left_side.size(), ' ');
  
    std::cout << left_side
              << Color(result.status) << Emoji(result.status) << "\033[0m"
              << (result.status == Evaluator::Status::Pass ? "    " : "   ")  // Extra space for alignment
              << result.reason
              << "\n";
  }

  [[nodiscard]] long CpuCount() noexcept
  {
    long count = sysconf(_SC_NPROCESSORS_CONF);
    return (count > 0) ? count : 1;
  }
} // end anonymous namespace for utility and helper functions

namespace Evaluator
{
  // SystemFileSystemDataSource implementation
  class SystemFileSystemDataSource final : public IDataSource
  {
  public:
    [[nodiscard]] std::optional<std::string> Read(const std::string &path) const override { return Slurp(path); }
    [[nodiscard]] std::optional<std::string> CmdLineParam(std::string_view key) const override { return GetCmdLineParam(key); }
  };

  // Check implementations
  // All check classes are kept private to this compilation unit

  class NohzFullCheck final : public ICheck
  {
  public:
    CheckKind Kind() const noexcept override { return CheckKind::NohzFull; }
    const std::string& Name() const noexcept override { static const std::string k = "nohz_full on RT core"; return k; }
    Domain GetDomain() const noexcept override { return Domain::Cpu; }
  
    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      const int cpu = *checkContext.cpu;
      if (auto sysfs_value = dataSource.Read("/sys/devices/system/cpu/nohz_full"))
      {
        std::string raw = Trim(*sysfs_value);
        auto set = ParseCpuList(raw);
        if (set.count(cpu))
        {
          return { Kind(), Status::Pass, Name(), std::string("nohz_full list: ") + (raw.empty() ? std::string("(empty)") : raw) };
        }
        return { Kind(), Status::Fail, Name(), std::string("CPU") + std::to_string(cpu) + " not in nohz_full: " + (raw.empty() ? std::string("(empty)") : raw) };
      }
      if (auto cmdline_value = dataSource.CmdLineParam("nohz_full"))
      {
        auto set = ParseCpuList(*cmdline_value);
        if (set.count(cpu)) return { Kind(), Status::Pass, Name(), std::string("cmdline nohz_full=") + *cmdline_value };
        return { Kind(), Status::Fail, Name(), std::string("RT core not in cmdline nohz_full=") + *cmdline_value };
      }
      return { Kind(), Status::Unknown, Name(), "no sysfs entry and no cmdline param" };
    }
};
  
  class NicPresenceCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::NicPresent; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "NIC interface present"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Nic; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.nic) return { Kind(), Status::Unknown, Name(), "no NIC in context" };
      const std::string nic = *checkContext.nic;
      if (NicExists(dataSource, nic))
      {
        return { Kind(), Status::Pass, Name(), "exists" };
      }
      return { Kind(), Status::Unknown, Name(), "interface not found" };
    }
  };
  
  class NicLinkUpCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::NicLinkUp; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "NIC link is UP"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Nic; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.nic) return { Kind(), Status::Unknown, Name(), "no NIC in context" };
      const std::string nic = *checkContext.nic;
      if (!NicExists(dataSource, nic))
      {
        return { Kind(), Status::Unknown, Name(), "NIC not found" };
      }
      if (auto oper = dataSource.Read(std::string("/sys/class/net/") + nic + "/operstate"))
      {
        auto v = Trim(*oper);
        if (v == "up") return { Kind(), Status::Pass, Name(), "operstate=up" };
        if (!v.empty()) return { Kind(), Status::Fail, Name(), std::string("operstate=") + v };
      }
      if (auto car = dataSource.Read(std::string("/sys/class/net/") + nic + "/carrier"))
      {
        auto v = Trim(*car);
        if (v == "1") return { Kind(), Status::Pass, Name(), "carrier=1" };
        if (v == "0") return { Kind(), Status::Fail, Name(), "carrier=0" };
      }
      return { Kind(), Status::Unknown, Name(), "no operstate/carrier" };
    }
  };
  
  class PreemptRTActiveCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::PreemptRTActive; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "PREEMPT_RT active"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::System; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      if (auto sysfs_value = dataSource.Read("/sys/kernel/realtime"))
      {
        auto value = Trim(*sysfs_value);
        if (value == "1") return { Kind(), Status::Pass, Name(), "/sys/kernel/realtime=1" };
        if (value == "0") return { Kind(), Status::Fail, Name(), "/sys/kernel/realtime=0" };
      }
  
      struct utsname uname_info = {};
      if (uname(&uname_info) == 0)
      {
        std::string version = uname_info.version;
        if (version.find("PREEMPT RT") != std::string::npos || version.find("PREEMPT_RT") != std::string::npos)
        {
          return { Kind(), Status::Pass, Name(), std::string("uname -v: ") + version };
        }
        std::string release = uname_info.release;
        if (auto config = dataSource.Read(std::string("/boot/config-") + release))
        {
          if (config->find("CONFIG_PREEMPT_RT=y") != std::string::npos || config->find("CONFIG_PREEMPT_RT_FULL=y") != std::string::npos)
            return { Kind(), Status::Pass, Name(), std::string("/boot/config-") + release + " has CONFIG_PREEMPT_RT=y" };
          if (config->find("CONFIG_PREEMPT=y") != std::string::npos)
            return { Kind(), Status::Fail, Name(), "Only low-latency PREEMPT, not RT" };
        }
      }
      return { Kind(), Status::Fail, Name(), "No evidence of RT kernel" };
    }
  };
  
  class CoreIsolatedCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::CoreIsolated; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "RT core isolated"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      const int cpu = *checkContext.cpu;
      auto s = dataSource.Read("/sys/devices/system/cpu/isolated");
      if (!s) return { Kind(), Status::Unknown, Name(), "no /sys/.../isolated" };
      std::string raw = Trim(*s);
      auto set = ParseCpuList(raw);
      if (cpu == 0)
      {
        if (set.count(cpu)) return { Kind(), Status::Fail, Name(), "CPU0 is isolated but should not be your RT core" };
        return { Kind(), Status::Fail, Name(), "CPU0 selected; choose non-CPU0" };
      }
      if (set.count(cpu)) return { Kind(), Status::Pass, Name(), std::string("isolated list: ") + (raw.empty() ? std::string("(empty)") : raw) };
      return { Kind(), Status::Fail, Name(), std::string("CPU") + std::to_string(cpu) + " not in isolated: " + (raw.empty() ? std::string("(empty)") : raw) };
    }
  };
  
  class CpuGovernorCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::CpuGovernor; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "CPU governor = performance"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      int cpu = *checkContext.cpu;
      auto path = std::string("/sys/devices/system/cpu/cpu") + std::to_string(cpu) + "/cpufreq/scaling_governor";
      auto s = dataSource.Read(path);
      if (!s) return { Kind(), Status::Unknown, Name(), std::string("no cpufreq governor for cpu") + std::to_string(cpu) };
      auto gov = Trim(*s);
      if (gov == "performance") return { Kind(), Status::Pass, Name(), std::string("governor=") + gov };
      return { Kind(), Status::Fail, Name(), std::string("governor=") + gov };
    }
  };
  
  class CpuFrequencyCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::CpuFrequency; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "CPU current frequency"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    static std::optional<int64_t> read_int64(const IDataSource& dataSource, const std::string& path)
    {
      auto value = dataSource.Read(path);
      if (!value) return std::nullopt;
      try { return std::stoll(Trim(*value)); } catch (...) { return std::nullopt; }
    }
  
    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      int cpu = *checkContext.cpu;
      std::string base = std::string("/sys/devices/system/cpu/cpu") + std::to_string(cpu) + "/cpufreq/";
      auto current_freq = read_int64(dataSource, base + "scaling_cur_freq");
      auto min_freq = read_int64(dataSource, base + "scaling_min_freq");
      auto max_freq = read_int64(dataSource, base + "scaling_max_freq");
      if (current_freq && min_freq && max_freq)
      {
        // Check if frequency is locked (min == max == current, with small tolerance for current)
        if (*min_freq == *max_freq)
        {
          int64_t tolerance = (*max_freq * 5) / 100;  // 5% tolerance for current frequency
          bool current_matches = std::abs(*current_freq - *max_freq) <= tolerance;
          if (current_matches)
          {
            return { Kind(), Status::Pass, Name(), std::to_string(*max_freq) + " kHz (locked)" };
          }
          else
          {
            std::string detail = "cur=" + std::to_string(*current_freq) + " kHz, locked=" + std::to_string(*max_freq) + " kHz";
            return { Kind(), Status::Fail, Name(), detail };
          }
        }
        else
        {
          std::string detail = "cur=" + std::to_string(*current_freq) + " kHz" +
                               ", min=" + std::to_string(*min_freq) + " kHz" +
                               ", max=" + std::to_string(*max_freq) + " kHz";
          return { Kind(), Status::Fail, Name(), detail };
        }
      }
      else if (current_freq || min_freq || max_freq)
      {
        std::string detail = "cur=" + (current_freq ? std::to_string(*current_freq) + " kHz" : std::string("?")) +
                             ", min=" + (min_freq ? std::to_string(*min_freq) + " kHz" : std::string("?")) +
                             ", max=" + (max_freq ? std::to_string(*max_freq) + " kHz" : std::string("?"));
        return { Kind(), Status::Unknown, Name(), detail };
      }
      if (auto cpuinfo = dataSource.Read("/proc/cpuinfo"))
      {
        std::istringstream cpuinfo_stream(*cpuinfo);
        std::string line;
        int current_processor = -1;
        std::optional<double> mhz_value;
        while (std::getline(cpuinfo_stream, line))
        {
          if (line.rfind("processor", 0) == 0)
          {
            auto pos = line.find(":");
            if (pos != std::string::npos) { try { current_processor = std::stoi(Trim(line.substr(pos + 1))); } catch (...) { current_processor = -1; } }
          }
          else if (line.rfind("cpu MHz", 0) == 0 && current_processor == cpu)
          {
            auto pos = line.find(":");
            if (pos != std::string::npos) { try { mhz_value = std::stod(Trim(line.substr(pos + 1))); } catch (...) { mhz_value.reset(); } }
          }
        }
        if (mhz_value) return { Kind(), Status::Unknown, Name(), std::to_string(*mhz_value) + " MHz (/proc/cpuinfo)" };
      }
      return { Kind(), Status::Unknown, Name(), "unavailable" };
    }
  };
  
  class RcuNoCbsCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::RcuNoCbs; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "rcu_nocbs includes RT core"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }
  
    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      int cpu = *checkContext.cpu;
      std::string raw;
      if (auto sysfs_value = dataSource.Read("/sys/devices/system/cpu/rcu_nocbs")) raw = Trim(*sysfs_value);
      else if (auto cmdline_value = dataSource.CmdLineParam("rcu_nocbs")) raw = Trim(*cmdline_value);
      else return { Kind(), Status::Unknown, Name(), "no sysfs and no cmdline param" };
      auto set = ParseCpuList(raw);
      if (set.count(cpu)) return { Kind(), Status::Pass, Name(), raw };
      return { Kind(), Status::Fail, Name(), std::string("CPU") + std::to_string(cpu) + " not in rcu_nocbs: " + (raw.empty() ? std::string("(empty)") : raw) };
    }
  };
  
  class IrqAffinityDefaultAvoidsRtCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::IrqAffinityDefaultAvoidsRt; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "irqaffinity excludes RT core"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      int cpu = *checkContext.cpu;
      auto value = dataSource.CmdLineParam("irqaffinity");
      if (!value) return { Kind(), Status::Unknown, Name(), "no irqaffinity kernel param" };
      auto set = ParseCpuList(*value);
      if (set.empty()) return { Kind(), Status::Unknown, Name(), "empty list" };
      if (set.count(cpu)) return { Kind(), Status::Fail, Name(), std::string("RT core present in irqaffinity: ") + *value };
      return { Kind(), Status::Pass, Name(), *value };
    }
  };
  
  class NoUnrelatedIrqsOnRtCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::NoUnrelatedIrqsOnRt; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "No unrelated IRQs on RT core"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::System; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      int cpu = *checkContext.cpu;
      auto content = dataSource.Read("/proc/interrupts");
      if (!content) return { Kind(), Status::Unknown, Name(), "cannot read /proc/interrupts" };
      std::istringstream interrupt_stream(*content);
      std::string line;
      int cpu_column = -1; bool header_done = false;
      std::vector<std::string> offenders;
      const std::string nic_filter = checkContext.nic.value_or("");
      while (std::getline(interrupt_stream, line))
      {
        if (line.empty()) continue;
        if (!header_done)
        {
    if (line.find(std::string(CpuPrefix) + "0") != std::string::npos)
          {
            std::istringstream header_stream(line);
            std::string column; int index = -1;
            while (header_stream >> column)
            {
              if (column.rfind(CpuPrefix, 0) == 0)
              {
                ++index;
                if (std::to_string(cpu) == column.substr(strlen(CpuPrefix))) cpu_column = index;
              }
            }
            header_done = true;
          }
          continue;
        }
        size_t pos = 0; while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
        if (pos >= line.size() || !std::isdigit(static_cast<unsigned char>(line[pos]))) continue;
        size_t colon = line.find(':', pos); if (colon == std::string::npos) continue;
        std::string irq_number = line.substr(pos, colon - pos);
        std::istringstream column_stream(line.substr(colon + 1));
        std::string token;
        int index = -1;
        int64_t value_at_rt_core = 0;
        bool saw_label = false;
        while (column_stream >> token)
        {
          bool is_num = std::all_of(token.begin(), token.end(), [](unsigned char c){ return std::isdigit(c); });
          if (!is_num)
          {
            std::string label = token;
            std::string rest;
            std::getline(column_stream, rest);
            label += rest;
            saw_label = true;
            if (value_at_rt_core > 0 && label.find(nic_filter) == std::string::npos) offenders.push_back(irq_number + " " + Trim(label));
            break;
          }
          ++index; if (index == cpu_column) { try { value_at_rt_core = std::stoll(token); } catch (...) { value_at_rt_core = 0; } }
        }
        if (!saw_label && value_at_rt_core > 0) offenders.push_back(irq_number + " (unlabeled)");
      }
      if (cpu_column < 0) return { Kind(), Status::Unknown, Name(), "could not map CPU column" };
      if (offenders.empty()) return { Kind(), Status::Pass, Name(), "clean" };
    std::ostringstream output_stream; for (size_t i = 0; i < offenders.size() && i < MaxIrqsToShow; ++i) { if (i) output_stream << ", "; output_stream << offenders[i]; }
    if (offenders.size() > MaxIrqsToShow) output_stream << ", +" << (offenders.size() - MaxIrqsToShow) << " more";
      return { Kind(), Status::Fail, Name(), output_stream.str() };
    }
  };
  
  class NicIrqsPinnedCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::NicIrqsPinned; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "NIC IRQs pinned to RT core"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Nic; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      const int cpu = *checkContext.cpu;
      if (!checkContext.nic) return { Kind(), Status::Unknown, Name(), "no NIC in context" };
      const std::string nic = *checkContext.nic;
      if (!NicExists(dataSource, nic))
      {
        return { Kind(), Status::Unknown, Name(), "NIC not found" };
      }
      auto content = dataSource.Read("/proc/interrupts");
      if (!content) return { Kind(), Status::Unknown, Name(), "cannot read /proc/interrupts" };
      std::istringstream interrupt_stream(*content);
      std::string line; std::vector<int> nic_irqs;
      while (std::getline(interrupt_stream, line))
      {
        if (line.find(nic) == std::string::npos) continue;
        size_t pos = 0; while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos]))) ++pos;
        if (pos >= line.size() || !std::isdigit(static_cast<unsigned char>(line[pos]))) continue;
        size_t colon = line.find(':', pos); if (colon == std::string::npos) continue;
        try { nic_irqs.push_back(std::stoi(line.substr(pos, colon - pos))); } catch (...) {}
      }
      if (nic_irqs.empty()) return { Kind(), Status::Unknown, Name(), "no NIC IRQs seen" };
      std::vector<int> bad_irqs;
      for (int irq : nic_irqs)
      {
        auto value = dataSource.Read(std::string("/proc/irq/") + std::to_string(irq) + "/smp_affinity_list");
        if (!value) return { Kind(), Status::Unknown, Name(), std::string("cannot read smp_affinity_list for IRQ ") + std::to_string(irq) };
        auto set = ParseCpuList(*value);
        if (!(set.size() == 1 && set.count(cpu) == 1)) bad_irqs.push_back(irq);
      }
      if (bad_irqs.empty()) return { Kind(), Status::Pass, Name(), std::string("all pinned to CPU") + std::to_string(cpu) };
      std::ostringstream output_stream; for (size_t i = 0; i < bad_irqs.size(); ++i) { if (i) output_stream << ","; output_stream << bad_irqs[i]; }
      return { Kind(), Status::Fail, Name(), std::string("not pinned: ") + output_stream.str() };
    }
  };
  
  class RpsDisabledCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::RpsDisabled; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "RPS disabled on NIC"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Nic; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.nic) return { Kind(), Status::Unknown, Name(), "no NIC in context" };
      const std::string nic = *checkContext.nic;
      if (!NicExists(dataSource, nic))
      {
        return { Kind(), Status::Unknown, Name(), "NIC not found" };
      }
      fs::path queue_directory = fs::path("/sys/class/net") / nic / "queues";
      if (!PathExists(queue_directory)) return { Kind(), Status::Unknown, Name(), "no queues dir" };
      auto all_zero_mask = [&dataSource](const fs::path &path) -> std::optional<bool>
      {
        auto content = dataSource.Read(path.string());
        if (!content) return std::nullopt;
        std::string value = Trim(*content);
        if (value.empty()) return std::optional<bool>(true);
        bool zero = true; for (char c : value) { if (c == ',' || c == '\n' || c == ' ' || c == '\t') continue; if (c != '0') { zero = false; break; } }
        return std::optional<bool>(zero);
      };
      bool any_bad = false; int checked = 0; std::error_code error_code;
      for (auto &entry : fs::directory_iterator(queue_directory, error_code))
      {
        if (!entry.is_directory()) continue;
        auto queue_name = entry.path().filename().string();
        if (queue_name.rfind("rx-", 0) == 0)
        {
          auto result = all_zero_mask(entry.path() / "rps_cpus");
          if (!result) return { Kind(), Status::Unknown, Name(), std::string("cannot read ") + (entry.path() / "rps_cpus").string() };
          if (!*result) { any_bad = true; }
          ++checked;
        }
      }
      if (checked == 0) return { Kind(), Status::Unknown, Name(), "no rx/tx queues found" };
      if (!any_bad) return { Kind(), Status::Pass, Name(), "all zero masks" };
      return { Kind(), Status::Fail, Name(), "non-zero masks present" };
    }
  };
  
  class NicQuietCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::NicQuiet; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "NIC is quiet"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Nic; }

    static bool default_route_v4_via_nic(const IDataSource& dataSource, const std::string& nic)
    {
      auto content = dataSource.Read("/proc/net/route");
      if (!content) return false;
      std::istringstream route_stream(*content);
      std::string line;
      bool header = true;
      while (std::getline(route_stream, line))
      {
        if (header) { header = false; continue; }
        if (line.empty()) continue;
        std::istringstream line_stream(line);
        std::string iface, dest;
        if (!(line_stream >> iface >> dest)) continue;
        if (iface == nic && dest == "00000000") return true;
      }
      return false;
    }
  
    static bool default_route_v6_via_nic(const IDataSource& dataSource, const std::string& nic)
    {
      auto content = dataSource.Read("/proc/net/ipv6_route");
      if (!content) return false;
      std::istringstream route_stream(*content);
      std::string line;
      const std::string zeros(32, '0');
      while (std::getline(route_stream, line))
      {
        if (line.empty()) continue;
        std::istringstream line_stream(line);
        std::vector<std::string> tokens;
        std::string token;
        while (line_stream >> token) tokens.push_back(token);
        if (tokens.size() < 10) continue;
        const std::string &dest = tokens[0];
        const std::string &prefix_len = tokens[1];
        const std::string &device = tokens.back();
        if (dest == zeros && (prefix_len == "0" || prefix_len == "00000000") && device == nic) return true;
      }
      return false;
    }
  
    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.nic) return { Kind(), Status::Unknown, Name(), "no NIC in context" };
      const std::string nic = *checkContext.nic;
      if (!NicExists(dataSource, nic))
      {
        return { Kind(), Status::Unknown, Name(), "NIC not found" };
      }
      int ipv4_count = 0;
      int ipv6_count = 0;
      bool addr_known = true;
      ifaddrs *interface_addrs = nullptr;
      if (getifaddrs(&interface_addrs) == 0)
      {
        for (auto *iface = interface_addrs; iface; iface = iface->ifa_next)
        {
          if (!iface || !iface->ifa_name || !iface->ifa_addr) continue;
          if (std::string(iface->ifa_name) != nic) continue;
          sa_family_t family = iface->ifa_addr->sa_family;
          if (family == AF_INET) ++ipv4_count; else if (family == AF_INET6) ++ipv6_count;
        }
        freeifaddrs(interface_addrs);
      }
      else { addr_known = false; }
  
      bool has_default_v4 = default_route_v4_via_nic(dataSource, nic);
      bool has_default_v6 = default_route_v6_via_nic(dataSource, nic);
      if (addr_known && ipv4_count == 0 && ipv6_count == 0 && !has_default_v4 && !has_default_v6) return { Kind(), Status::Pass, Name(), "no IPs, no default route" };
      std::ostringstream output_stream;
      if (!addr_known) output_stream << "addr=?";
      else output_stream << "v4=" << ipv4_count << ", v6=" << ipv6_count;
      output_stream << ", def4=" << (has_default_v4 ? "yes" : "no") << ", def6=" << (has_default_v6 ? "yes" : "no");
      if (!addr_known) return { Kind(), Status::Unknown, Name(), output_stream.str() };
      return { Kind(), Status::Fail, Name(), output_stream.str() };
    }
  };
  
  class RtThrottlingCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::RtThrottlingDisabled; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "RT throttling disabled"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      auto value = dataSource.Read("/proc/sys/kernel/sched_rt_runtime_us");
      if (!value) return { Kind(), Status::Unknown, Name(), "cannot read sched_rt_runtime_us" };
      std::string trimmed_value = Trim(*value);
      if (trimmed_value == "-1") return { Kind(), Status::Pass, Name(), "sched_rt_runtime_us=-1" };
      return { Kind(), Status::Fail, Name(), std::string("sched_rt_runtime_us=") + trimmed_value };
    }
  };

  class TimerMigrationCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::TimerMigration; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "Timer Migration disabled"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::System; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      auto value = dataSource.Read("/proc/sys/kernel/timer_migration");
      if (!value) return { Kind(), Status::Unknown, Name(), "cannot read timer_migration" };
      std::string trimmed_value = Trim(*value);
      if (trimmed_value == "0") return { Kind(), Status::Pass, Name(), "timer_migration=0" };
      return { Kind(), Status::Fail, Name(), std::string("timer_migration=") + trimmed_value };
    }
  };

  class SwapDisabledCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::SwapDisabled; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "Swap disabled"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::System; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      auto swaps = dataSource.Read("/proc/swaps");
      if (!swaps) return { Kind(), Status::Unknown, Name(), "cannot read /proc/swaps" };

      std::istringstream stream(*swaps);
      std::string line;
      if (!std::getline(stream, line)) return { Kind(), Status::Unknown, Name(), "unexpected /proc/swaps format" };

      std::vector<std::string> active_entries;
      while (std::getline(stream, line))
      {
        line = Trim(line);
        if (line.empty()) continue;

        std::istringstream line_stream(line);
        std::vector<std::string> tokens;
        std::string token;
        while (line_stream >> token) tokens.push_back(token);
        if (tokens.empty()) continue;

        std::ostringstream entry;
        entry << tokens[0];
        if (tokens.size() >= 4)
        {
          entry << " size=" << tokens[2];
          entry << " used=" << tokens[3];
        }
        active_entries.push_back(entry.str());
      }

      if (active_entries.empty())
      {
        return { Kind(), Status::Pass, Name(), "/proc/swaps empty" };
      }

      std::ostringstream reason;
      reason << "active: " << active_entries[0];
      for (size_t i = 1; i < active_entries.size(); ++i)
      {
        reason << ", " << active_entries[i];
      }
      return { Kind(), Status::Fail, Name(), reason.str() };
    }
  };

  class CStatesCappedCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::DeepCStatesCapped; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "Deep C-states capped"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      if (auto cmdline = dataSource.Read("/proc/cmdline"))
      {
        if (cmdline->find("cpuidle.off=1") != std::string::npos) return { Kind(), Status::Pass, Name(), "cpuidle.off=1" };
        if (cmdline->find("intel_idle.max_cstate=1") != std::string::npos || cmdline->find("processor.max_cstate=1") != std::string::npos)
          return { Kind(), Status::Pass, Name(), "cmdline caps to C1" };
      }
      if (auto intel_cstate = dataSource.Read("/sys/module/intel_idle/parameters/max_cstate"))
      {
        auto value = Trim(*intel_cstate);
        if (value == "1" || value == "0") return { Kind(), Status::Pass, Name(), std::string("intel_idle.max_cstate=") + value };
        return { Kind(), Status::Fail, Name(), std::string("intel_idle.max_cstate=") + value };
      }
      if (auto processor_cstate = dataSource.Read("/sys/module/processor/parameters/max_cstate"))
      {
        auto value = Trim(*processor_cstate);
        if (value == "1" || value == "0") return { Kind(), Status::Pass, Name(), std::string("processor.max_cstate=") + value };
        return { Kind(), Status::Fail, Name(), std::string("processor.max_cstate=") + value };
      }
      return { Kind(), Status::Unknown, Name(), "no indicators" };
    }
  };
  
  class TurboPolicyCheck final : public ICheck
  {
  public:
    [[nodiscard]] CheckKind Kind() const noexcept override { return CheckKind::TurboBoostPolicy; }
    [[nodiscard]] const std::string& Name() const noexcept override { static const std::string k = "Turbo/boost disabled"; return k; }
    [[nodiscard]] Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      if (auto boost_value = dataSource.Read("/sys/devices/system/cpu/cpufreq/boost"))
      {
        auto value = Trim(*boost_value);
        if (value == "0") return { Kind(), Status::Pass, Name(), "cpufreq/boost=0" };
        if (value == "1") return { Kind(), Status::Fail, Name(), "cpufreq/boost=1" };
      }
      if (auto intel_turbo = dataSource.Read("/sys/devices/system/cpu/intel_pstate/no_turbo"))
      {
        auto value = Trim(*intel_turbo);
        if (value == "1") return { Kind(), Status::Pass, Name(), "intel_pstate/no_turbo=1" };
        if (value == "0") return { Kind(), Status::Fail, Name(), "intel_pstate/no_turbo=0" };
      }
      return { Kind(), Status::Unknown, Name(), "no boost knobs" };
    }
  };
  
  class ClocksourceCheck final : public ICheck
  {
  public:
    CheckKind Kind() const noexcept override { return CheckKind::ClocksourceStable; }
    const std::string& Name() const noexcept override { static const std::string k = "Clocksource stable"; return k; }
    Domain GetDomain() const noexcept override { return Domain::System; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext&, const IDataSource& dataSource) const override
    {
      std::string base = "/sys/devices/system/clocksource/clocksource0/";
      auto current_source = dataSource.Read(base + "current_clocksource");
      if (!current_source) return { Kind(), Status::Unknown, Name(), "cannot read current_clocksource" };
      auto value = Trim(*current_source);

      // Check for known good clocksources
      if (value == "tsc") return { Kind(), Status::Pass, Name(), "tsc" };

      // On ARM systems, arch_sys_counter is the standard and acceptable clocksource
      if (value == "arch_sys_counter") {
        // Check if there are any alternatives available
        auto available = dataSource.Read(base + "available_clocksource");
        if (available) {
          std::string available_sources = Trim(*available);
          // If arch_sys_counter is the only option, it's acceptable
          if (available_sources == "arch_sys_counter" || available_sources.find(' ') == std::string::npos) {
            return { Kind(), Status::Pass, Name(), "arch_sys_counter (ARM standard)" };
          }
        }
        // Even without checking available, arch_sys_counter is generally fine on ARM
        return { Kind(), Status::Pass, Name(), "arch_sys_counter" };
      }
      
      // Other known acceptable clocksources
      if (value == "hpet") return { Kind(), Status::Pass, Name(), "hpet" };

      // For unknown or problematic clocksources
      auto available = dataSource.Read(base + "available_clocksource");
      std::string detail = value;
      if (available) detail += "; available=" + Trim(*available);

      // jiffies is generally not good for RT
      if (value == "jiffies") return { Kind(), Status::Fail, Name(), detail };
      
      // Unknown clocksource - mark as warning rather than fail
      return { Kind(), Status::Unknown, Name(), detail };
    }
  };

  class SmtSiblingIsolatedCheck final : public ICheck
  {
  public:
    CheckKind Kind() const noexcept override { return CheckKind::SmtSiblingIsolated; }
    const std::string& Name() const noexcept override { static const std::string k = "SMT sibling isolated/disabled"; return k; }
    Domain GetDomain() const noexcept override { return Domain::Cpu; }

    [[nodiscard]] CheckResult Evaluate(const CheckContext& checkContext, const IDataSource& dataSource) const override
    {
      if (!checkContext.cpu) return { Kind(), Status::Unknown, Name(), "no CPU subject" };
      int cpu = *checkContext.cpu;
      auto siblings_value = dataSource.Read(std::string("/sys/devices/system/cpu/cpu") + std::to_string(cpu) + "/topology/thread_siblings_list");
      if (!siblings_value) return { Kind(), Status::Unknown, Name(), "no thread_siblings_list" };
      auto siblings = ParseCpuList(*siblings_value);
      siblings.erase(cpu);
      if (siblings.empty()) return { Kind(), Status::Pass, Name(), "no sibling" };
      auto isolated_value = dataSource.Read("/sys/devices/system/cpu/isolated");
      if (!isolated_value) return { Kind(), Status::Unknown, Name(), "cannot read isolated" };
      auto isolated_set = ParseCpuList(*isolated_value);
      for (int sibling : siblings)
      {
        if (!isolated_set.count(sibling))
          return { Kind(), Status::Fail, Name(), std::string("sibling CPU") + std::to_string(sibling) + " not isolated" };
      }
      return { Kind(), Status::Pass, Name(), "siblings all isolated" };
    }
  };

  // Helper functions for system info

  std::string GetCpuInfo()
  {
    std::ostringstream output;
    output << "CPU: " << CpuModelString();
    
    long online = sysconf(_SC_NPROCESSORS_ONLN);
    if (online > 0)
    {
      std::set<std::pair<int,int>> cores;
      std::error_code error_code;
      for (auto &entry : fs::directory_iterator("/sys/devices/system/cpu", error_code))
      {
        if (!entry.is_directory()) continue;
        auto name = entry.path().filename().string();
        if (name.rfind("cpu", 0) != 0) continue;
        auto core_id = Slurp(entry.path() / "topology/core_id");
        auto pkg_id  = Slurp(entry.path() / "topology/physical_package_id");
        if (!core_id || !pkg_id) continue;
        try { cores.insert({ std::stoi(Trim(*pkg_id)), std::stoi(Trim(*core_id)) }); } catch (...) {}
      }
      output << " (" << online << " logical";
      if (!cores.empty()) output << ", " << cores.size() << " physical";
      
      int performance_cores = 0;
      int efficiency_cores = 0;
      bool any = false;
      for (auto &entry : fs::directory_iterator("/sys/devices/system/cpu", error_code))
      {
        if (!entry.is_directory()) continue;
        auto name = entry.path().filename().string();
        if (name.rfind("cpu", 0) != 0) continue;
        auto type_value = Slurp(entry.path() / "topology/core_type");
        if (!type_value) continue;
        any = true;
        std::string type_string = Trim(*type_value);
        for (auto &ch : type_string) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (type_string.find("perf") != std::string::npos || type_string == "core") ++performance_cores;
        else if (type_string.find("eff") != std::string::npos || type_string == "atom") ++efficiency_cores;
      }
      if (any && (performance_cores + efficiency_cores) > 0) 
        output << "; P=" << performance_cores << ", E=" << efficiency_cores;
      output << ")";
    }
    
    return output.str();
  }
  
  std::string GetKernelInfo()
  {
    std::ostringstream output;
    output << "Kernel: ";
    
    struct utsname buffer;
    if (uname(&buffer) == 0)
    {
      output << buffer.sysname << " " << buffer.release << " " << buffer.version << " " << buffer.machine;
    }
    
    return output.str();
  }

  void ReportSystemConfiguration(int cpu, std::string_view nicName)
  {
    auto cpuCount = CpuCount();
    if (cpu < 0 || cpu >= cpuCount)
    {
      std::cerr << "Invalid CPU core " << cpu << "; must be between 0 and " << (cpuCount - 1) << "\n";
      return;
    }

    std::cout << GetCpuInfo() << "\n";
    std::cout << GetKernelInfo() << "\n";


    PrintSectionHeader("System Checks");

    Evaluator::CheckContext checkContext;
    checkContext.cpu = cpu;
    if (!nicName.empty())
    {
      checkContext.nic = std::string(nicName);
    }
    Evaluator::SystemFileSystemDataSource data;

    // System-wide checks
    std::vector<std::unique_ptr<Evaluator::ICheck>> system_checks;
    system_checks.emplace_back(std::make_unique<Evaluator::PreemptRTActiveCheck>());
    system_checks.emplace_back(std::make_unique<Evaluator::SwapDisabledCheck>());
    system_checks.emplace_back(std::make_unique<Evaluator::TimerMigrationCheck>());
    system_checks.emplace_back(std::make_unique<Evaluator::RtThrottlingCheck>());
    system_checks.emplace_back(std::make_unique<Evaluator::ClocksourceCheck>());

    for (const auto &check : system_checks)
    {
      auto result = check->Evaluate(checkContext, data);
      PrintResult(result);
    }

    // CPU Core checks
    PrintSectionHeader("Core " + std::to_string(cpu) + " Checks");
    std::vector<std::unique_ptr<Evaluator::ICheck>> core_checks;
    core_checks.emplace_back(std::make_unique<Evaluator::CoreIsolatedCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::NohzFullCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::RcuNoCbsCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::CpuGovernorCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::CpuFrequencyCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::IrqAffinityDefaultAvoidsRtCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::NoUnrelatedIrqsOnRtCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::SmtSiblingIsolatedCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::CStatesCappedCheck>());
    core_checks.emplace_back(std::make_unique<Evaluator::TurboPolicyCheck>());

    for (const auto &check : core_checks)
    {
      auto result = check->Evaluate(checkContext, data);
      PrintResult(result);
    }

    if (checkContext.nic)
    {
      PrintSectionHeader("NIC " + *checkContext.nic + " Checks");

      Evaluator::NicPresenceCheck presence_check;
      auto presence = presence_check.Evaluate(checkContext, data);
      PrintResult(presence);
      const bool nic_ok = (presence.status == Evaluator::Status::Pass);

      if (nic_ok)
      {
        std::vector<std::unique_ptr<Evaluator::ICheck>> nic_checks;
        nic_checks.emplace_back(std::make_unique<Evaluator::NicLinkUpCheck>());
        nic_checks.emplace_back(std::make_unique<Evaluator::NicQuietCheck>());
        nic_checks.emplace_back(std::make_unique<Evaluator::NicIrqsPinnedCheck>());
        nic_checks.emplace_back(std::make_unique<Evaluator::RpsDisabledCheck>());
        for (const auto &check : nic_checks)
        {
          auto result = check->Evaluate(checkContext, data);
          PrintResult(result);
        }
      }
    }

    // Add an extra newline to separate from any following console output
    std::cout << "\n";
  }
} // end namespace Evaluator

// Unit Test Hook: Define CONFIG_AS_EXECUTABLE to compile as a library for testing
// This allows linking the checks into a test harness without main()
#ifdef CONFIG_AS_EXECUTABLE
int main()
{
  ReportSystemConfiguration();
  return 0;
}

#endif // defined(CONFIG_AS_EXECUTABLE)
