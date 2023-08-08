#pragma once

#include <algorithm>
#include <cstdlib>
#include <string>

namespace epick_driver::test
{

// Trim from start.
static inline std::string ltrim(const std::string& s)
{
  std::string value{ s };
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  return value;
}

// Trim from end.
static inline std::string rtrim(const std::string& s)
{
  std::string value{ s };
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
              value.end());
  return value;
}

// Trim from both ends.
static inline std::string trim(const std::string& s)
{
  std::string value{ s };
  return ltrim(rtrim(value));
}

// To lower case.
static inline std::string to_lower(const std::string& s)
{
  std::string value{ s };
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return std::tolower(ch); });
  return value;
}

/**
 * @brief If there is a global variable RUN_HARDWARE_TESTS set to true,
 * return false (do not skip the test) else true (skip the test).
 * @return True to skip a test, false to execute it.
 */
bool skip_test()
{
  const char* env = std::getenv("RUN_HARDWARE_TESTS");
  if (env)
  {
    return to_lower(trim(std::string{ env })) != "true";
  }
  return true;
}
}  // namespace epick_driver::test
