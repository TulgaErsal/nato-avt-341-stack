#!/usr/bin/env python3

# Copyright 2022 PickNik Inc.
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
#    * Neither the name of the PickNik Inc. nor the names of its
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

"""Generate every C++ parameter header for a package in a single process.

The CMake macros used to spawn one interpreter per template yaml. Interpreter
start-up plus the jinja/yaml imports dominated that cost (~1.4 s each), so a
package with 20 templates paid ~25 s of pure start-up on every clean build.
This driver takes a manifest describing all the jobs and runs them one after
another with the imports paid exactly once.

Manifest format (written by avt_341_generate_cpp_parameters):

    {
      "jobs": [
        {"kind": "mixin",
         "dto": "<out>/foo_params_dto.hpp",
         "yaml": "<src>/mixins/foo.yaml"},
        {"kind": "template",
         "dto": "<out>/bar_params_dto.hpp",
         "service": "<out>/bar_params_service.hpp",
         "yaml": "<src>/bar.yaml",
         "validate_header": "",
         "mixin_include_prefix": "my_package"}
      ]
    }
"""

import argparse
import json
import sys
import traceback

from avt_341_param_lib.codegen import generate_cpp_header
from avt_341_param_lib.codegen import generate_cpp_mixin_header


def _run_job(job):
    """Dispatch a single manifest entry to the matching single-file generator."""
    kind = job.get('kind')
    if kind == 'mixin':
        generate_cpp_mixin_header.run(job['dto'], job['yaml'])
    elif kind == 'template':
        generate_cpp_header.run(
            job['dto'],
            job['service'],
            job['yaml'],
            job.get('validate_header', ''),
            job.get('mixin_include_prefix', ''),
        )
    else:
        raise ValueError("unknown job kind '{kind}'".format(kind=kind))


def run(manifest_file):
    """Run every job in <manifest_file>. Returns the number of failures."""
    with open(manifest_file, encoding='utf-8') as handle:
        jobs = json.load(handle)['jobs']

    # Mixins first: a template yaml may pull struct definitions out of a mixin,
    # so their DTO headers should exist before the templates that include them.
    ordered = ([job for job in jobs if job.get('kind') == 'mixin']
               + [job for job in jobs if job.get('kind') != 'mixin'])

    failures = 0
    for job in ordered:
        try:
            _run_job(job)
        except Exception:  # noqa: B902 - report and keep going
            failures += 1
            # One process now generates many headers, so the traceback alone no
            # longer identifies the offending template. Name it explicitly.
            sys.stderr.write(
                "\navt_341_param_lib: failed to generate parameters from "
                "'{yaml}':\n".format(yaml=job.get('yaml', '<unknown>')))
            traceback.print_exc()
    return failures


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'manifest_file',
        help='JSON manifest describing the headers to generate')
    return parser.parse_args()


def main():
    args = parse_args()
    return 1 if run(args.manifest_file) else 0


if __name__ == '__main__':
    sys.exit(main())
