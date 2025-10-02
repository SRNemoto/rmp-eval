// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/net_tstamp.h>
#include <linux/time_types.h>
#include <linux/sockios.h>
#include <memory>
#include <netpacket/packet.h>
#include <net/if.h>  // Gets the ifreq
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "nictest.h"

static constexpr uint16_t EthernetFrameTypeBKHF = 0x88A4;

namespace Evaluator
{
  void DoNothing() {}

  std::string AppendErrorCode(const std::string_view message)
  {
    static constexpr size_t bufferSize = 256;
    char buffer[bufferSize] = {};
    std::snprintf(buffer, bufferSize, "%s | [%d] %s", message.data(), errno, std::strerror(errno));
    return std::string(buffer);
  }

  EthercatNicTest::EthercatNicTest(const TestParameters& argParams, TimerReport&& hardwareReport, TimerReport&& softwareReport)
    : params(argParams)
    , hardwareReport(std::move(hardwareReport))
    , softwareReport(std::move(softwareReport))
  {
    // Create the socket
    socketDescriptor = socket(PF_PACKET, SOCK_RAW, htons(EthernetFrameTypeBKHF));
    if (socketDescriptor == -1)
    {
      throw std::runtime_error(AppendErrorCode("Failed to create socket."));
    }

    // Set socket timeout to 1 second
    struct timeval socketTimeout { .tv_sec = 1, .tv_usec = 0 };
    if (setsockopt(socketDescriptor, SOL_SOCKET, SO_RCVTIMEO, &socketTimeout, sizeof(socketTimeout)) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to set socket receive timeout.")); }
    if (setsockopt(socketDescriptor, SOL_SOCKET, SO_SNDTIMEO, &socketTimeout, sizeof(socketTimeout)) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to set socket send timeout.")); }

    // Don't send packets via a gateway, just to directly connected hosts.
    constexpr int dontRoute = 1; 
    if (setsockopt(socketDescriptor, SOL_SOCKET, SO_DONTROUTE, &dontRoute, sizeof(dontRoute)) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to set socket routing to 'dont route'.")); }

    // constexpr int pollMicroseconds = 50;
    // if (setsockopt(socketDescriptor, SOL_SOCKET, SO_BUSY_POLL, &pollMicroseconds, sizeof(pollMicroseconds)) == -1)
    // {
    //   throw std::runtime_error("Failed to set SO_BUSY_POLL option on socket interface.");
    // }

    // constexpr int pollPacketBudget = 32;
    // if (setsockopt(socketDescriptor, SOL_SOCKET, SO_BUSY_POLL_BUDGET, &pollPacketBudget, sizeof(pollPacketBudget)) == -1)
    // {
    //   throw std::runtime_error("Failed to set SO_BUSY_POLL_BUDGET option on socket interface.");
    // }

    // enable timestamping
    struct ifreq ifrts = {};
    std::snprintf(ifrts.ifr_name, sizeof(ifrts.ifr_name), "%s", params.NicName.c_str());
    struct hwtstamp_config cfg = {};
    cfg.tx_type = HWTSTAMP_TX_ON;
    cfg.rx_filter = HWTSTAMP_FILTER_ALL; // fallback to *_PTP_* if ALL unsupported
    ifrts.ifr_data = (char *)&cfg;
    ioctl(socketDescriptor, SIOCSHWTSTAMP, &ifrts); // check cfg echoed back

    int tflags = SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_RX_HARDWARE |
              SOF_TIMESTAMPING_RAW_HARDWARE | SOF_TIMESTAMPING_SOFTWARE;
    if (setsockopt(socketDescriptor, SOL_SOCKET, SO_TIMESTAMPING_NEW, &tflags, sizeof(tflags)) == -1)
    {
      throw std::runtime_error(AppendErrorCode("Failed to set SO_TIMESTAMPING_NEW option on socket interface."));
    }

    // Get the index of the interface
    struct ifreq ifr;

    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", params.NicName.c_str());
    if (ioctl(socketDescriptor, SIOCGIFINDEX, &ifr) == -1)
    {
      std::string errorMessage = "Failed to get interface index for NIC: ";
      errorMessage += params.NicName;
      throw std::runtime_error(AppendErrorCode(errorMessage));
    }

    int interfaceIndex = ifr.ifr_ifindex;

    // reset the flags of the NIC
    ifr.ifr_flags = 0;
    if (ioctl(socketDescriptor, SIOCGIFFLAGS, &ifr) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to reset interface flags.")); }

    // Set NIC flags to be promiscuous and broadcast
    ifr.ifr_flags = ifr.ifr_flags | IFF_PROMISC | IFF_BROADCAST;
    if (ioctl(socketDescriptor, SIOCSIFFLAGS, &ifr) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to set promiscuous and broadcast flags on NIC.")); }

    // bind the socket
    struct sockaddr_ll address;
    address.sll_family = AF_PACKET;
    address.sll_ifindex = interfaceIndex;
    address.sll_protocol = htons(EthernetFrameTypeBKHF);
    if (bind(socketDescriptor, (struct sockaddr *)&address, sizeof(address)) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to bind socket on interface.")); }

    // std::cout << "Successfully set up EthercatNicTest" << std::endl;
  }

  void EthercatNicTest::Send()
  {
    static constexpr size_t packetSize = 29;
    unsigned char pData[packetSize] = {0};

    // Set the broadcast address as the destination in the frame data.
    unsigned char* macDestination = &pData[0];
    memset(macDestination, 0xff, 6);

    // Set the mac address source to all zeroes
    unsigned char* macSource = &pData[6];
    memset(macSource, 0x00, 6);

    // Set the frame type to EtherCAT
    pData[12] = 0x88; pData[13] = 0xa4;

    // Set the Ethercat frame length
    pData[14] = 0x0d;
    // Set the EtherCAT frame type (command)
    pData[15] = 0x10;
    // Set the EtherCAT command
    pData[16] = 0x08;
    // Set the index
    pData[17] = 0xff;
    // Set the subordinate address
    memset(&pData[18], 0x00, 2);
    // Set the offset address
    pData[20] = 0x00; pData[21] = 0x05;
    // Set No roundtrip - Last Sub Command, also length?
    pData[22] = 0x01;

    {
      std::unique_lock lock(mutex);
      if (!condition.wait_for(lock, SocketTimeout,
        [this]
        {
          return receiveIteration > sendIteration;
        }))
      {
        char buffer[128] = {};
        std::snprintf(buffer, sizeof(buffer), "Timed out waiting for receiver to be ready. sendIteration=%lu, receiveIteration=%lu",
          sendIteration, receiveIteration);
        throw std::runtime_error(buffer);
      }
    }

    if (send(socketDescriptor, pData, packetSize, 0) == -1)
    { throw std::runtime_error(AppendErrorCode("Failed to send data on socket.")); }

    ++sendIteration;
  }

  // Convert (timespec-like) to ns
  template <class TS>
  static inline int64_t to_ns(const TS& ts) {
      // guard against overflow (tv_sec typically 64-bit on modern kernels)
      long double ns = static_cast<long double>(ts.tv_sec) * 1000000000.0L
                    + static_cast<long double>(ts.tv_nsec);
      if (ns > static_cast<long double>(std::numeric_limits<int64_t>::max()))
          return std::numeric_limits<int64_t>::max();
      if (ns < static_cast<long double>(std::numeric_limits<int64_t>::min()))
          return std::numeric_limits<int64_t>::min();
      return static_cast<int64_t>(ns);
  }

  bool EthercatNicTest::Receive()
  {
    // Set up polling
    constexpr int numFds = 1; // number of file descriptors
    static pollfd pollFd = { .fd=socketDescriptor, .events=POLLIN, .revents=0 };
    pollfd pollFds[numFds] = {};	// array of pollfd structs.
    pollFds[0] = pollFd;
    constexpr int timeoutMs = 1000;

    {
      std::unique_lock lock(mutex);
      ++receiveIteration;
    }
    condition.notify_all();

    int ready = poll(pollFds, numFds, timeoutMs);
    if (ready < 0)
    {
      throw std::runtime_error(AppendErrorCode("There was an error during frame polling on socket."));
    }
    else if (ready == 0)
    {
      return false;
    }

    /**** Some code to use base recv() ****/
    // int bytesReceived = recv(socketDescriptor, recvBuffer, bufferSize, 0);
    // if (bytesReceived < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    // { throw std::runtime_error("recv() had an error when reading bytes from socket."); }

    /**** Some code to use recvmsg() to obtain timestamps from the socket ****/
    // Payload + control buffers
    char data[2048];
    char control[512];

    struct iovec iov = {};
    iov.iov_base = data;
    iov.iov_len  = sizeof(data);

    struct msghdr msg = {};
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = control;
    msg.msg_controllen = sizeof(control);

    ssize_t n = recvmsg(socketDescriptor, &msg, 0);
    if (n < 0) {
      perror("recvmsg");
      return false;
    }

    bool haveHardware = false, haveSoftware = false;
    int64_t hardwareNanoseconds = 0, softwareNanoseconds = 0;

    for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
        cmsg;
        cmsg = CMSG_NXTHDR(&msg, cmsg))
    {
      if (cmsg->cmsg_level == SOL_SOCKET &&
          (cmsg->cmsg_type == SO_TIMESTAMPING_NEW || cmsg->cmsg_type == SCM_TIMESTAMPING))
      {
        {
          // Legacy API: timespec[3] -> [SW, legacy, HW]
          struct timespec* ts = reinterpret_cast<struct timespec*>(CMSG_DATA(cmsg));
          haveSoftware = (ts[0].tv_sec || ts[0].tv_nsec);
          haveHardware = (ts[2].tv_sec || ts[2].tv_nsec);
          if (haveSoftware) softwareNanoseconds = to_ns(ts[0]);
          if (haveHardware) hardwareNanoseconds = to_ns(ts[2]);
        }
      }
    }

    // --- Inter-arrival delta for HW clock ---
    if (haveHardware)
    {
      if (prev.HaveHardware)
      {
        int64_t delta = hardwareNanoseconds - prev.HardwareNanoseconds;
        // Inter-arrival should be non-negative; if negative, skip (clock step/rollover)
        if (delta >= 0)
        {
          hardwareReport.AddObservation(static_cast<uint64_t>(delta), static_cast<int>(receiveIteration));
          stats.HardwareDeltaNanoseconds.update(delta, receiveIteration);
        }
      }
      prev.HardwareNanoseconds = hardwareNanoseconds;
      prev.HaveHardware = true;
    }

    // --- Inter-arrival delta for SW clock ---
    if (haveSoftware)
    {
      if (prev.HaveSoftware)
      {
        int64_t delta = softwareNanoseconds - prev.SoftwareNanoseconds;
        if (delta >= 0)
        {
          softwareReport.AddObservation(static_cast<uint64_t>(delta), static_cast<int>(receiveIteration));
          stats.SoftwareDeltaNanoseconds.update(delta, receiveIteration);
        }
      }
      prev.SoftwareNanoseconds = softwareNanoseconds;
      prev.HaveSoftware = true;
    }

    return true;
  }

  EthercatNicTest::~EthercatNicTest()
  {
    // auto print_one = [](const char* name, const RunningStats& current) {
    //   if (current.count == 0) {
    //     std::printf("%s: no samples\n", name);
    //     return;
    //   }
    //   std::printf(
    //     "%s (us):\n"
    //     "  count = %zu\n"
    //     "  min   = %.3f  (iter %zu)\n"
    //     "  mean  = %.3f\n"
    //     "  max   = %.3f  (iter %zu)\n",
    //     name, current.count, (current.MinValue / 1000.0), current.MinIndex, (current.Mean / 1000.0), (current.MaxValue / 1000.0), current.MaxIndex
    //   );
    // };
    // print_one("HW inter-arrival Delta", stats.HardwareDeltaNanoseconds);
    // print_one("SW inter-arrival Delta", stats.SoftwareDeltaNanoseconds);


    close(socketDescriptor);
    socketDescriptor = -1;
  }
} // end namespace Evaluator
