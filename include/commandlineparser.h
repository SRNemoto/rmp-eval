// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#pragma once

#ifndef RMP_EVAL_COMMANDLINEPARSER_H
#define RMP_EVAL_COMMANDLINEPARSER_H

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace Evaluator
{
  // Command line parsing based off of Simon Schneegans' command line parsing class
  //    http://schneegans.github.io/tutorials/2019/08/06/commandline
  
  using ParsedValue = std::variant<uint16_t*, int32_t*, uint32_t*, double*, float*, bool*, std::string*, uint64_t*>;

  struct Argument
  {
    std::vector<std::string> Flags;
    ParsedValue              Value;
    std::string              Help;
  };

  bool ParseArguments(std::vector<Argument>& arguments, int argc, const char* const argv[]);
  void AddArgument(std::vector<Argument>& arguments, const std::vector<std::string>& flags, const ParsedValue& value, const std::string& help);
  void PrintHelp(std::ostream& stream, const std::vector<Argument>& arguments, const std::string& description);
} // end namespace Evaluator

#endif // !defined(RMP_EVAL_COMMANDLINEPARSER_H)
