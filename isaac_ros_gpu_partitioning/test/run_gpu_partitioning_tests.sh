#!/bin/bash

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

set -u

if [[ $# -lt 2 ]]; then
  echo "Expected at least one test and a cleanup executable." >&2
  exit 2
fi

runfile_path() {
  echo "${TEST_SRCDIR}/${TEST_WORKSPACE}/$1"
}

arguments=("$@")
cleanup_index=$((${#arguments[@]} - 1))
cleanup_executable=$(runfile_path "${arguments[$cleanup_index]}")
unset 'arguments[$cleanup_index]'

trap '"$cleanup_executable" || true' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

status=0
for test_runfile in "${arguments[@]}"; do
  test_executable=$(runfile_path "$test_runfile")
  if ! "$test_executable"; then
    status=1
  fi
done

exit "$status"
