#!/usr/bin/env python3
"""Generate the shared C++ DTO header for one mixin template.

Each root-level parameter group of the mixin becomes a standalone struct
definition in the mixin's code_namespace, so several node templates that
``__include_mixins`` the file share one C++ type instead of re-defining
structurally identical structs in their own namespaces.
"""

import argparse
import os
import sys

from avt_341_param_lib.parse_yaml import GenerateCode


def run(dto_output_file, yaml_file):
    gen_param_struct = GenerateCode('cpp')
    output_dir = os.path.dirname(dto_output_file)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    gen_param_struct.parse_mixin(yaml_file)

    with open(dto_output_file, 'w') as f:
        f.write(gen_param_struct.render_cpp_mixin_dto())


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('output_cpp_dto_header_file')
    parser.add_argument('input_yaml_file')
    return parser.parse_args()


def main():
    args = parse_args()
    run(args.output_cpp_dto_header_file, args.input_yaml_file)


if __name__ == '__main__':
    sys.exit(main())
