#include "command_line_utility.hpp"

#include <iostream>

void CommandLineUtility::registerHandler(const std::string& parameter, ParameterHandler handler, bool isMandatory)
{
  handlers[parameter] = handler;
  if (isMandatory)
  {
    mandatoryParams.insert(parameter);
  }
}

bool CommandLineUtility::parse(int argc, char* argv[])
{
  for (int i = 1; i < argc; i++)
  {
    auto it = handlers.find(argv[i]);
    if (it != handlers.end())
    {
      receivedParams.insert(it->first);

      if (std::holds_alternative<LambdaWithValue>(it->second))
      {
        auto& handler = std::get<LambdaWithValue>(it->second);
        i++;
        if (i < argc)
        {
          handler(argv[i]);
        }
        else
        {
          std::cerr << it->first << " requires a value.\n";
        }
      }
      else if (std::holds_alternative<LambdaWithoutValue>(it->second))
      {
        auto& handler = std::get<LambdaWithoutValue>(it->second);
        handler();
      }
    }
    else
    {
      std::cerr << "Unknown argument: " << argv[i] << "\n";
      return false;
    }
  }

  for (const auto& param : mandatoryParams)
  {
    if (receivedParams.find(param) == receivedParams.end())
    {
      std::cerr << "Missing mandatory argument: " << param << "\n";
      return false;
    }
  }

  return true;
}
