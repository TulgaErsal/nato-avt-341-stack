import subprocess

output = subprocess.check_output('pipenv --py', shell=True)
# Convert the output to a string
output_str = output.decode('utf-8')
# Specify the file path of the Python file
paths = [
    'avt_341_depth_estimation_node'
]
for path in paths:
    # Read the contents of the file
    with open(path, 'r') as file:
        lines = file.readlines()
    # Update the first line with the new shebang
    lines[0] = f'#!{output_str}\n'
    # Write the updated contents back to the file
    with open(path, 'w') as file:
        file.writelines(lines)