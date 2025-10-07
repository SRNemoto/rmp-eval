// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#include <algorithm>
#include <atomic>
#include <arpa/inet.h>
#include <array>
#include <barrier>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/net_tstamp.h>
#include <linux/time_types.h>
#include <linux/sockios.h>
#include <memory>
#include <sys/mman.h>
#include <mutex>
#include <netpacket/packet.h>
#include <net/if.h>  // Gets the ifreq
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <unordered_set>
#include <variant>
#include <vector>

#include "quantileestimator.h"
#include "reporter.h"
#include "nictest.h"
#include "commandlineparser.h"
#include "config.h"
#include "version.h"

static std::mutex reportMutex;
static std::atomic_bool testRunning = true;

namespace Evaluator
{

std::string GetEstimatedRunTime(const uint64_t iterations, const uint64_t sleepNanoseconds)
{
  uint64_t totalNanoseconds = iterations * sleepNanoseconds;

  uint64_t hours = totalNanoseconds / (60ULL * 60ULL * Evaluator::NanoPerSec);
  uint64_t minutes = (totalNanoseconds / (60ULL * Evaluator::NanoPerSec)) % 60ULL;
  uint64_t seconds = (totalNanoseconds / Evaluator::NanoPerSec) % 60ULL;
  uint64_t milliseconds = (totalNanoseconds / 1'000'000ULL) % 1000ULL;

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(2) << hours << ":"
      << std::setfill('0') << std::setw(2) << minutes << ":"
      << std::setfill('0') << std::setw(2) << seconds << "."
      << std::setfill('0') << std::setw(3) << milliseconds;

  return oss.str();
}


void ConfigureThisThread(int priority, int cpuCore)
{
  sched_param sched_params;
  sched_params.sched_priority = priority;

  // Get the current thread
  pthread_t thisThread = pthread_self();

  if (pthread_setschedparam(thisThread, SCHED_FIFO, &sched_params)) {
    std::string errorString = "Failed to set thread priority to " + std::to_string(priority);
    throw std::runtime_error(AppendErrorCode(errorString));
  }

  cpu_set_t affinityMask;
  CPU_ZERO(&affinityMask);
  CPU_SET(cpuCore, &affinityMask);

  if (pthread_setaffinity_np(thisThread, sizeof(cpu_set_t), &affinityMask))
  {
    std::string errorString = "Failed to set the cpu affinity to CPU_CORE: " + std::to_string(cpuCore);
    throw std::runtime_error(AppendErrorCode(errorString));
  }
}

static constexpr uint64_t RunIndefinitely = std::numeric_limits<uint64_t>::max();
static constexpr uint64_t NanoPerMicro = 1000;

void AddNanoToTimespec(struct timespec* time, const uint64_t nanos)
{
  // Combine both second and nanosecond components into a single time value in nanoseconds
  // This is to account for rollover when the nanosecond component grows larger than a second
  // and we must instead increase the second component and reset the nanosecond component.
  const uint64_t nanoEpoch = Evaluator::ToEpoch(*time) + nanos;

  // Calculate the seconds component by doing integer division (dropping the fractional component)
  time->tv_sec = nanoEpoch / Evaluator::NanoPerSec;

  // Put the remainder within the nanosecond component.
  time->tv_nsec = nanoEpoch % Evaluator::NanoPerSec;
}

void SenderThread(TestParameters params, std::shared_ptr<INicTest> tester)
{
  try
  {
    ConfigureThisThread(params.SendPriority, params.SendCpu);

    TimerReport report(params.SendSleep, params.BucketWidth, params.SendData);
    bool recordTime = true;
    uint64_t index = 0;
    struct timespec next = {};
    clock_gettime(CLOCK_MONOTONIC, &next);
    uint64_t previous = 0;
    while (testRunning.load(std::memory_order_acquire) && (params.Iterations == RunIndefinitely || index < params.Iterations))
    {
      // decide whether to record this iteration's time
      recordTime = (index != 0 && index != (params.Iterations -1));
  
      // call the desired method
      if (tester != nullptr)
      {
        tester->Send();
      }

      // ScopedTimer timer(report, recordTime, index);
      uint64_t current = Evaluator::GetCurrentTime();
      if (recordTime)
      {
        report.AddObservation(current - previous, index);
      }
  
      // Set up the next time to wake up
      AddNanoToTimespec(&next, params.SendSleep);
      // If we are falling behind, skip ahead.
      while (current > Evaluator::ToEpoch(next))
      {
        AddNanoToTimespec(&next, params.SendSleep);
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

      previous = current;
      ++index;
    }
  }
  catch (const std::exception& error)
  {
    testRunning.store(false, std::memory_order_release);
    std::cout << "Error occurred in Sender Thread: " << error.what() << std::endl;
  }
}


void ReceiverThread(TestParameters params, std::shared_ptr<INicTest> tester)
{
  try
  {
    ConfigureThisThread(params.ReceivePriority, params.ReceiveCpu);

    TimerReport report(params.SendSleep, params.BucketWidth, params.ReceiveData);
    bool recordTime = true;

    uint64_t index = 0;
    uint64_t previous = 0;
    while (testRunning.load(std::memory_order_acquire) && (params.Iterations == RunIndefinitely || index < params.Iterations))
    {
      // decide whether to record this iteration's time
      recordTime = (index != 0 && index != (params.Iterations -1));

      // call the desired method
      if (tester->Receive() != true)
      {
        testRunning.store(false, std::memory_order_release);
        std::cout << "Failed to receive message on index " << index << std::endl;
        break;
      }

      uint64_t current = Evaluator::GetCurrentTime();
      if (recordTime)
      {
        report.AddObservation(current - previous, index);
      }

      previous = current;
      ++index;
    }
  }
  catch (const std::exception& error)
  {
    testRunning.store(false, std::memory_order_release);
    std::cout << "Error occurred in Receiver Thread: " << error.what() << std::endl;
  }
}

// Write a trace marker to be read via trace-cmd
void WriteTraceMarker(const std::string& message)
{
  // Newer path; older kernels use /sys/kernel/debug/tracing/trace_marker
  const char* paths[] =
  {
    "/sys/kernel/tracing/trace_marker",
    "/sys/kernel/debug/tracing/trace_marker"
  };
  int fileDescriptor = -1;
  for (const char* path : paths)
  {
    fileDescriptor = open(path, O_WRONLY | O_CLOEXEC);
    if (fileDescriptor >= 0) { break; }
  }
  if (fileDescriptor < 0)
  {
    perror("open trace_marker");
    return;
  }
  if (write(fileDescriptor, message.c_str(), message.size()) < 0)
  {
    perror("write trace_marker");
  }
  close(fileDescriptor);
}

class FileDescriptor
{
public:
  FileDescriptor() noexcept = default;
  explicit FileDescriptor(int fd) noexcept : fd(fd) {}

  // Disable copying
  FileDescriptor(const FileDescriptor&) = delete;
  // Disable copying
  FileDescriptor& operator=(const FileDescriptor&) = delete;

  int Get() const { return fd; }
  int Release() noexcept
  {
    int old = fd;
    fd = -1;
    return old;
  }
  int Reset(int newFd = -1) noexcept
  {
    if (newFd != fd && fd >= 0) close(fd);
    fd = newFd;
    return fd;
  }
  FileDescriptor(FileDescriptor&& other) noexcept : fd(other.Release()) {}

  FileDescriptor& operator=(FileDescriptor&& other) noexcept
  {
    if (this != &other)
    {
      this->Reset(other.Release());
    }
    return *this;
  }
  ~FileDescriptor() { Reset(); }
private:
  int fd = -1;
};

// This method, `SetLatencyTarget()` was adapted from the cyclictest source code.
/* Latency trick
 * if the file /dev/cpu_dma_latency exists,
 * open it and write a zero into it. This will tell
 * the power management system not to transition to
 * a high cstate (in fact, the system acts like idle=poll)
 * When the fd to /dev/cpu_dma_latency is closed, the behavior
 * goes back to the system default.
 */
FileDescriptor SetLatencyTarget()
{
  struct stat s;

  if (stat("/dev/cpu_dma_latency", &s) == -1) {
    std::cout << "WARN: stat /dev/cpu_dma_latency failed: " << std::strerror(errno) << "\n";
    return FileDescriptor{};
  }

  int rawFd = open("/dev/cpu_dma_latency", O_RDWR);
  if (rawFd == -1) {
    std::cout << "WARN: open /dev/cpu_dma_latency failed: " << std::strerror(errno) << "\n";
    return FileDescriptor{};
  }

  FileDescriptor latencyTargetFd(rawFd);

  errno = 0;
  int32_t latencyTargetValue = 0;
  if (write(latencyTargetFd.Get(), &latencyTargetValue, sizeof(latencyTargetValue)) < 1) {
    std::cout << "WARN: error setting cpu_dma_latency to " << latencyTargetValue << "\n";
  }
  return latencyTargetFd;
}

using ReportPair = std::pair<std::string_view, ReportData*>;
using ReportVector = std::vector<ReportPair>;

void PrintReport(ReportVector& reports, int& lineCount, Evaluator::TableMaker& tableMaker,
  std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime,
  std::ostream& stream)
{
  // Recalculate column widths based on actual data
  tableMaker.OptimizeColumnWidthsFromData(reports);

  // Move cursor up and clear from cursor to end of screen
  if (lineCount > 0)
  {
    stream << "\033[" << lineCount << "A"; // Move cursor up
    stream << "\033[J"; // Clear from cursor to end of screen
  }
  lineCount = 0;

  // Reprint header with updated widths
  lineCount += tableMaker.PrintLabels(stream);

  std::stringstream summary;
  for (auto [label, dataPtr] : reports)
  {
    if (dataPtr != nullptr)
    {
      lineCount += tableMaker.PrintRow(label, *dataPtr, stream);
      tableMaker.PrintMaxLatencySummary(summary, label, *dataPtr);
      lineCount += 1;
    }
  }
  lineCount += Evaluator::FormatDuration(startTime, endTime);
  stream << summary.str();
  stream << "\n\n";
  lineCount += 2;
}

// live reporter interval at 20Hz
static constexpr auto ReportInterval = std::chrono::milliseconds(50);
void ReportThread(ReportVector& reports, int& lineCount, Evaluator::TableMaker& tableMaker,
  std::chrono::steady_clock::time_point startTime, std::atomic_bool& liveReport, std::ostream& stream)
{
  while(liveReport.load(std::memory_order_acquire))
  {
    std::unique_lock lock(reportMutex);
    auto currentTime = std::chrono::steady_clock::now();
    PrintReport(reports, lineCount, tableMaker, startTime, currentTime, stream);
    std::this_thread::sleep_for(ReportInterval);
  }
}
} // end namespace Evaluator



int main(int argc, char* argv[])
{
  int exitCode = 0;
  try
  {
    static constexpr char NoNicSelected[] = "NoNicSelected";
    static constexpr int DefaultSendSleepMicroseconds = 1000;
    static constexpr int DefaultSendPriority = 42;
    static constexpr int DefaultReceivePriority = 45;
    static constexpr uint64_t AutomaticBucketWidth = 0;
    const auto DefaultCpuCore = std::max(std::thread::hardware_concurrency() - 1, 0U);

    Evaluator::TestParameters params;
    params.NicName = NoNicSelected;
    params.Iterations = Evaluator::RunIndefinitely;
    params.SendSleep = DefaultSendSleepMicroseconds;
    params.SendPriority = DefaultSendPriority;
    params.ReceivePriority = DefaultReceivePriority;
    params.SendCpu = DefaultCpuCore;
    params.ReceiveCpu = DefaultCpuCore;
    params.IsVerbose = false;
    params.BucketWidth = AutomaticBucketWidth;
    Evaluator::ReportData sendData, receiveData, hardwareData, softwareData;
    params.SendData = &sendData;
    params.ReceiveData = &receiveData;

    bool noConfig = false;

    std::atomic<bool> liveReport = true;

    std::vector<Evaluator::Argument> arguments;
    Evaluator::AddArgument(arguments, {"--nic", "-n"}, &params.NicName, "Network interface card name");
    Evaluator::AddArgument(arguments, {"--iterations", "-i"}, &params.Iterations, "Number of iterations");
    Evaluator::AddArgument(arguments, {"--send-sleep", "-s"}, &params.SendSleep, "Send sleep duration in microseconds");
    Evaluator::AddArgument(arguments, {"--send-priority", "-sp"}, &params.SendPriority, "Send thread priority");
    Evaluator::AddArgument(arguments, {"--receive-priority", "-rp"}, &params.ReceivePriority, "Receive thread priority");
    Evaluator::AddArgument(arguments, {"--send-cpu", "-sc"}, &params.SendCpu, "CPU core to use for the sender thread");
    Evaluator::AddArgument(arguments, {"--receive-cpu", "-rc"}, &params.ReceiveCpu, "CPU core to use for the receiver thread");
    Evaluator::AddArgument(arguments, {"--verbose", "-v"}, &params.IsVerbose, "Enable verbose output");
    Evaluator::AddArgument(arguments, {"--no-config", "-nc"}, &noConfig, "Skip system configuration checks");
    Evaluator::AddArgument(arguments, {"--bucket-width", "-b"}, &params.BucketWidth, "Bucket width in microseconds for counting occurrences.");

    bool showHelp = false;
    Evaluator::AddArgument(arguments, {"--help", "-h"}, &showHelp, "Show this help message");

    bool showVersion = false;
    Evaluator::AddArgument(arguments, {"--version"}, &showVersion, "Show version information");

    if (!Evaluator::ParseArguments(arguments, argc, argv))
    {
      Evaluator::PrintHelp(std::cout, arguments, "A program to test NIC performance using raw sockets.");
      return 1;
    }

    if (showHelp)
    {
      Evaluator::PrintHelp(std::cout, arguments, "A program to test NIC performance using raw sockets.");
      return 0;
    }

    if (showVersion)
    {
      std::cout << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_MICRO << std::endl;
      return 0;  // Exit after showing version
    }

    if (!noConfig)
    {
      Evaluator::ReportSystemConfiguration(params.SendCpu, params.NicName);
    }

    if (geteuid() != 0)
    {
      std::cerr << "Error: Not running as root. This may cause failures when opening raw sockets or setting thread priorities.\n";
      return 1;
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
      std::cerr << "Error: Failed to lock memory. Memory swapping might occur. Error: " << strerror(errno) << std::endl;
      return 1;
    }

    params.SendSleep *= Evaluator::NanoPerMicro; // convert to nanoseconds for internal use
    if (params.BucketWidth == 0)
    {
      params.BucketWidth = params.SendSleep * 0.125;
    }
    else
    {
      params.BucketWidth *= Evaluator::NanoPerMicro; // convert to nanoseconds for internal use
    }

    auto latencyFd = Evaluator::SetLatencyTarget();

    Evaluator::TableMaker tableMaker = Evaluator::TableMaker::CreateTableMaker(params.BucketWidth, params.IsVerbose);

    if (params.Iterations != Evaluator::RunIndefinitely)
    {
      std::cout << "Estimated run time: " << Evaluator::GetEstimatedRunTime(params.Iterations, params.SendSleep) << "\n";
    }
    std::cout << "Target period: " << static_cast<int>(params.SendSleep / Evaluator::NanoPerMicro) << " us\n\n" << std::flush;

    // Evaluator::DurationReporter durationReporter("Total test duration");

    int lineCount = 0;
    Evaluator::ReportVector reports;

    auto startTime = std::chrono::steady_clock::now();

    if (params.NicName == NoNicSelected)
    {
      reports.push_back({"Cyclic", &sendData});

      tableMaker.OptimizeRowLabelWidth(reports);

      std::thread cyclicThread(Evaluator::SenderThread, params, nullptr);

      std::thread reportThread(Evaluator::ReportThread, std::ref(reports), std::ref(lineCount), std::ref(tableMaker),
        startTime, std::ref(liveReport), std::ref(std::cout));

      cyclicThread.join();
      testRunning.store(false, std::memory_order_release);
      liveReport.store(false, std::memory_order_release);
      reportThread.join();
    }
    else
    {
      reports.push_back({"Sender", &sendData});
      reports.push_back({"Receiver", &receiveData});
      if (params.IsVerbose)
      {
        reports.push_back({"HW delta", &hardwareData});
        reports.push_back({"SW delta", &softwareData});
      }

      tableMaker.OptimizeRowLabelWidth(reports);

      std::shared_ptr<Evaluator::INicTest> tester = std::make_shared<Evaluator::EthercatNicTest>(params, 
        Evaluator::TimerReport(params.SendSleep, params.BucketWidth, &hardwareData),
        Evaluator::TimerReport(params.SendSleep, params.BucketWidth, &softwareData));

      std::thread receiverThread(Evaluator::ReceiverThread, params, tester);
      std::thread senderThread(Evaluator::SenderThread, params, tester);

      std::thread reportThread(Evaluator::ReportThread, std::ref(reports), std::ref(lineCount), std::ref(tableMaker),
        startTime, std::ref(liveReport), std::ref(std::cout));

      receiverThread.join();
      testRunning.store(false, std::memory_order_release);
      senderThread.join();

      liveReport.store(false, std::memory_order_release);
      reportThread.join();
    }

    std::cout << std::flush;
    Evaluator::PrintReport(reports, lineCount, tableMaker, startTime, std::chrono::steady_clock::now(), std::cout);
    std::cout << std::flush;

  }
  catch(const std::exception& error)
  {
    std::cerr << "Evaluator exiting due to error: " << error.what() << std::endl;
    exitCode = 1;
  }
  
  return exitCode;
}
