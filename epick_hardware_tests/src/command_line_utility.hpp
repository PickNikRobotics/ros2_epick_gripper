// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <map>
#include <set>
#include <functional>
#include <variant>

/**
 *  Class to parse the command line.
 */
class CommandLineUtility
{
  using LambdaWithValue = std::function<void(const char*)>;
  using LambdaWithoutValue = std::function<void()>;

public:
  void registerHandler(const std::string& parameter, std::variant<LambdaWithValue, LambdaWithoutValue> handler,
                       bool isMandatory = false);

  bool parse(int argc, char* argv[]);

private:
  // Associate a parameter to a lambda function.
  std::map<std::string, std::variant<LambdaWithValue, LambdaWithoutValue>> handlers;

  // Store all mandatory parameters.
  std::set<std::string> mandatoryParams;

  // Store the parameters which are parsed in the command line. If a parameter is mandatory,
  // but cannot be found in the received  parameters, then print an error.
  std::set<std::string> receivedParams;
};
