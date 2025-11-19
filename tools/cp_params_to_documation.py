from argparse import ArgumentParser
from ament_index_python.packages import get_package_share_directory
import os

PYTHON_EVAL_STR = '$Python:'

if __name__ == '__main__':

    arg_parser = ArgumentParser(description="Copy parameters file definitions to documentation.")
    arg_parser.add_argument('--root_dir', type=str, default='../avt_341/config/parameters')
    arg_parser.add_argument('--target_dir', type=str, default='../docs/source/pages/getting-started/usage/param-data')
    args, _ = arg_parser.parse_known_args()

    for d in os.listdir(args.root_dir):
        print("> Processing file %s" % d)
        results = []
        param_prefix = ''

        # Gather parameters
        with open(os.path.join(args.root_dir, d), 'r') as f:
            for l in f.readlines():
                param_def = l.split(':')
                if len(param_def) == 1:
                    continue

                is_section_param = param_def[0].startswith(' ')        # Leading whitespace
                param_name = param_def[0].strip()

                # Section splitter
                if param_def[1] == '\n':
                    param_prefix = param_name
                    continue
                elif not is_section_param:
                    param_prefix = ''

                param_name = (param_prefix + "/" if param_prefix else "") + param_name
                param_val_description = l[len(param_def[0])+1:].split("#")
                param_default_val = param_val_description[0].strip().strip('\"')

                if type(param_default_val) is str and param_default_val.startswith(PYTHON_EVAL_STR):
                    param_default_val_orig = param_default_val
                    param_default_val = eval(param_default_val[len(PYTHON_EVAL_STR):])
                    if "os.path.join" in param_default_val_orig or "get_package_share_directory" in param_default_val_orig and param_default_val.find("avt_341", 0) > 0:
                        param_default_val = param_default_val[param_default_val.find("avt_341", 0):].replace("\\", "/")

                if type(param_default_val) is str and ',' in param_default_val:
                    param_default_val = "\"" + param_default_val + "\""

                param_description = param_val_description[1].strip() if len(param_val_description) > 1 else ""
                results.append((param_name, param_default_val, "\"" + param_description + "\""))

        # Write csv file
        target_f = os.path.join(args.target_dir, d).replace('.yaml', '.csv')
        print("     > Found %d parameters. Writing to file %s" % (len(results), target_f))
        with open(target_f, 'w') as f:
            f.write(",".join(["Parameter", "Default", "Description"]) + "\n")
            for r in results:
                f.write(",".join(r) + "\n")


