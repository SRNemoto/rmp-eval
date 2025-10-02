// Copyright (c) 2025 Robotic Systems Integration, Inc.
// Licensed under the MIT License. See LICENSE file in the project root for details.

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unordered_set>

#include "commandlineparser.h"

namespace Evaluator
{
  bool ParseArguments(std::vector<Argument>& arguments, int argc, const char* const argv[]) 
  {

    // This is for detecting duplicate arguments
    // The element being stored in the set is the help string for an argument
    std::unordered_set<std::string> parsedSet;

    // Skip the first argument (name of the program).
    bool success = true;
    int tokenIndex = 1;
    while (tokenIndex < argc)
    {

      // First we have to identify whether the value is separated by a space
      // or a '='.
      std::string flag(argv[tokenIndex]);
      std::string value;
      bool        valueIsSeparate = false;

      // If there is an '=' in the flag, the part after the '=' is actually
      // the value.
      size_t equalPos = flag.find('=');
      if (equalPos != std::string::npos)
      {
        value = flag.substr(equalPos + 1);
        flag = flag.substr(0, equalPos);
      }
      // Else the following argument is the value.
      else if (tokenIndex + 1 < argc)
      {
        value = argv[tokenIndex + 1];
        valueIsSeparate = true;
      }

      // Search for an argument with the provided flag.
      bool foundArgument = false;

      for (auto const& argument : arguments)
      {
        if (std::find(argument.Flags.begin(), argument.Flags.end(), flag) != std::end(argument.Flags))
        {
          // Check if we've already parsed this flag
          if (parsedSet.find(argument.Help) != parsedSet.end())
          {
            std::cerr << "Duplicate command line argument \"" + flag + "\"!\n";
            return false;
          }
          parsedSet.insert(argument.Help);

          foundArgument = true;

          // In the case of booleans, there must not be a value present.
          // So if the value is neither 'true' nor 'false' it is considered
          // to be the next argument.
          if (std::holds_alternative<bool*>(argument.Value))
          {
            if (!value.empty() && value != "true" && value != "false")
            {
              valueIsSeparate = false;
            }
            *std::get<bool*>(argument.Value) = (value != "false");
          }
          // In all other cases there must be a value.
          else if (value.empty())
          {
            std::cerr <<
              "Failed to parse command line arguments: "
              "Missing value for argument \"" + flag + "\"!\n";
            return false; // immediately cease execution
          }
          // For a std::string, we take the entire value.
          else if (std::holds_alternative<std::string*>(argument.Value)) {
            *std::get<std::string*>(argument.Value) = value;
          }
          // In all other cases we use a std::stringstream to
          // convert the value.
          else {
            std::visit(
              [&value](auto&& arg) {
                std::stringstream sstr(value);
                sstr >> *arg;
              },
              argument.Value);
          }

          break;
        }
      }

      // Print a warning if there was an unknown argument.
      if (!foundArgument) {
        std::cerr << "Unknown command line argument \"" << flag
          << "\"." << std::endl;
        success = false;
      }

      // Advance to the next flag.
      ++tokenIndex;

      // If the value was separated, we have to advance our index once more.
      if (foundArgument && valueIsSeparate) {
        ++tokenIndex;
      }
    }
    return success;
  }

  void AddArgument(std::vector<Argument>& arguments, const std::vector<std::string>& flags, const ParsedValue& value, const std::string& help)
  {
    arguments.emplace_back(Argument{ flags, value, help });
  }

  void PrintHelp(std::ostream& stream, const std::vector<Argument>& arguments, const std::string& description)
  {

    // Print the general description.
    stream << "\n" << description << std::endl;

    if (arguments.size() == 0)
    {
      return; // we don't have to continue if no arguments
    }

    // Find the argument with the longest combined flag length (in order
    // to align the help messages).

    uint32_t maxFlagLength = 0;

    for (auto const& argument : arguments)
    {
      uint32_t flagLength = 0;
      for (auto const& flag : argument.Flags)
      {
        // Plus comma and space.
        flagLength += static_cast<uint32_t>(flag.size()) + 2;
      }

      maxFlagLength = std::max(maxFlagLength, flagLength);
    }

    stream << "\nOptions:\n";

    // Now print each argument.
    for (auto const& argument : arguments)
    {

      std::string flags;
      for (auto const& flag : argument.Flags)
      {
        flags += flag + ", ";
      }

      // Remove last comma and space and add padding according to the
      // longest flags in order to align the help messages.
      std::stringstream sstr;
      sstr << std::left << std::setw(maxFlagLength) << flags.substr(0, flags.size() - 2);

      // Print the help for each argument. This is a bit more involved
      // since we do line wrapping for long descriptions.
      size_t spacePos = 0;
      size_t lineWidth = 0;
      while (spacePos != std::string::npos) 
      {
        size_t nextspacePos = argument.Help.find_first_of(' ', spacePos + 1);
        sstr << argument.Help.substr(spacePos, nextspacePos - spacePos);
        lineWidth += nextspacePos - spacePos;
        spacePos = nextspacePos;

        if (lineWidth > 60)
        {
          stream << sstr.str() << std::endl;
          sstr = std::stringstream();
          sstr << std::left << std::setw(maxFlagLength - 1) << " ";
          lineWidth = 0;
        }
      }
    }
  }
} // end namespace Evaluator
