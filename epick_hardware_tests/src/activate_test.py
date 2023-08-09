#!/usr/bin/env python3

# Copyright (c) 2023 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This is a very basic test in Python to check if the gripper is correctly wired.
We send an activation request and await for an expected response.
"""

import serial
import sys

com_port = "/dev/ttyUSB0"
baud_rate = 115200

activate_request = bytes.fromhex('09 10 03 E8 00 03 06 01 00 00 00 00 00 72 E1')
expected_response = bytes.fromhex('09 10 03 e8 00 03 01 30')

print("Checking if the gripper is connected to /dev/ttyUSB0...")

with serial.Serial(com_port, baud_rate, timeout=1) as serial_interface:
    
    print("Gripper connected.")
    print("Sending request...")

    serial_interface.write(activate_request)
    serial_interface.flush()

    print("Reading response...")

    response = serial_interface.read(len(expected_response))

    if len(response) != len(expected_response):
        print(f"Requested {len(expected_response)} bytes, but only got {len(response)}")
        sys.exit()

    print("Gripper successfully activated.")
