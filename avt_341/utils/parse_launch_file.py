"""
Creates ros1 base.launch from base_template.launch and yaml configuration files.
"""

import yaml
import os
from rospkg import RosPack
import re


class ParseConstants:
    PYTHON_EVAL_STR = '$Python:'
    VAL_PREFIX = '$val{'
    REF_PREFIX = '$ref{'
    ARG_LIST = '$arg_list{%s}$'
    PARAM_LIST = '$param_list{%s}$'
    ARG_ASSIGN = '$arg_assign{%s}$'


def parse_yaml_config():
    ros_pack = RosPack()
    param_dir = os.path.join(ros_pack.get_path('avt_341'), 'config', 'parameters')
    param_files = {f[:-len('.yaml')]: os.path.join(param_dir, f) for f in os.listdir(param_dir) if f.endswith('.yaml')}
    params = {}
    param_refs = {}
    for k, v in param_files.items():
        with open(v) as f:
            params[k] = yaml.load(f, Loader=yaml.FullLoader)
            param_refs[k] = {}
        keys_list = list(params[k].keys())
        # Flatten sub-dictionaries
        for ki in keys_list:
            vi = params[k][ki]
            if type(vi) is dict:
                for kii, vii in vi.items():
                    params[k]['_'.join([ki, kii])] = vii
                del params[k][ki]
        for ki, vi in params[k].items():
            if type(vi) is str and vi.startswith(ParseConstants.PYTHON_EVAL_STR):
                python_str = vi[len(ParseConstants.PYTHON_EVAL_STR):]
                if 'get_package_share_directory' in python_str:
                    # $(find avt_341)
                    pkg = re.findall(r'get_package_share_directory\(\'(.*?)\'\)', python_str)[0]
                    python_str = python_str.replace("get_package_share_directory('%s')" % pkg, "'$(find %s)'" % pkg)
                params[k][ki] = eval(python_str)
            if type(vi) is str and vi.startswith(ParseConstants.VAL_PREFIX):
                key_sub = vi[len(ParseConstants.VAL_PREFIX):-1].split(':')
                params[k][ki] = params[key_sub[0]][key_sub[1]]
            if type(vi) is str and vi.startswith(ParseConstants.REF_PREFIX):
                param_refs[k][ki] = vi[len(ParseConstants.REF_PREFIX):-1]
        for ki in param_refs[k].keys():
            del params[k][ki]
    for k in ['robot_description', 'robot_description_veh2', 'robot_description_veh3', 'robot_description_veh4']:
        params['common'][k + '_file'] = params['common'][k]
        del params['common'][k]

    return params, param_refs


def generate_launch_file():
    ros_pack = RosPack()
    params, param_refs = parse_yaml_config()

    with open(os.path.join(ros_pack.get_path('avt_341'), 'launch', 'base_template.launch'), 'r') as f:
        launch_str = f.read()

    launch_str_split = launch_str.split('\n')

    for k, v in params.items():

        line = [l for l in launch_str_split if ParseConstants.ARG_LIST % k in l]
        if len(line) > 0:
            spaces = line[0][:-len(line[0].lstrip(' '))]
            section_value = os.linesep.join([('' if i == 0 else spaces) + '<arg name="%s" default="%s" />'
                                             % (ki, vi.__str__().lower() if isinstance(vi, bool) else vi.__str__())
                                             for i, (ki, vi) in enumerate(v.items())])
            launch_str = launch_str.replace(ParseConstants.ARG_LIST % k, section_value)

        line = [l for l in launch_str_split if ParseConstants.PARAM_LIST % k in l]
        if len(line) > 0:
            spaces = line[0][:-len(line[0].lstrip(' '))]
            section_value_param = [('' if i == 0 else spaces) + '<param name="%s" value="$(arg %s)" />'
                                                   % (ki, ki)
                                                   for i, (ki, vi) in enumerate(v.items())]
            for ki, vi in param_refs[k].items():
                section_value_param.append(spaces + '<param name="%s" value="$(arg %s)" />' % (ki, vi))
            launch_str = launch_str.replace(ParseConstants.PARAM_LIST % k, os.linesep.join(section_value_param))

        line = [l for l in launch_str_split if ParseConstants.ARG_ASSIGN % k in l]
        if len(line) > 0:
            spaces = line[0][:-len(line[0].lstrip(' '))]
            section_arg_assign = os.linesep.join([('' if i == 0 else spaces) + '<arg name="%s" value="$(arg %s)" />" />'
                                                   % (ki, ki)
                                                   for i, (ki, vi) in enumerate(v.items())])
            launch_str = launch_str.replace(ParseConstants.ARG_ASSIGN % k, section_arg_assign)

    with open(os.path.join(ros_pack.get_path('avt_341'), 'launch', 'base.launch'), 'w') as f:
        f.write(launch_str)


if __name__ == '__main__':
    generate_launch_file()
