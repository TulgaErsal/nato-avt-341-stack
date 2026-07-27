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

import argparse
import sys
import os

from avt_341_param_lib.codegen.parse_yaml import GenerateCode
from avt_341_param_lib.common.output_file import write_if_changed


def run(dto_output_file, service_output_file, yaml_file, validate_header='',
        mixin_include_prefix=''):
    gen_param_struct = GenerateCode('cpp')
    gen_param_struct.mixin_include_prefix = mixin_include_prefix
    for output_file in (dto_output_file, service_output_file):
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)

    gen_param_struct.parse(yaml_file, validate_header)

    dto_header_include = os.path.basename(dto_output_file)
    # write_if_changed, not a plain write: these headers are compile inputs and
    # touching one with identical content recompiles every dependent TU.
    write_if_changed(dto_output_file, gen_param_struct.render_cpp_dto())
    write_if_changed(service_output_file,
                     gen_param_struct.render_cpp_service(dto_header_include))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_cpp_dto_header_file')
    parser.add_argument('output_cpp_service_header_file')
    parser.add_argument('input_yaml_file')
    parser.add_argument('validate_header', nargs='?', default='')
    parser.add_argument(
        '--mixin-include-prefix', default='',
        help='Include sub-path used to reference generated mixin DTO headers, '
             'e.g. "avt_341" for #include <avt_341/<stem>_params_dto.hpp>')
    return parser.parse_args()


def main():
    args = parse_args()
    dto_output_file = args.output_cpp_dto_header_file
    service_output_file = args.output_cpp_service_header_file
    yaml_file = args.input_yaml_file
    validate_header = args.validate_header

    run(dto_output_file, service_output_file, yaml_file, validate_header,
        args.mixin_include_prefix)


if __name__ == '__main__':
    sys.exit(main())
