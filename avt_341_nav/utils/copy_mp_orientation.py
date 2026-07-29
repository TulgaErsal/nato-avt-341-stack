import argparse
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description='Copy orientation fields from input CSV to output CSV.')
    parser.add_argument('--input_csv',
                        default='../../../traxrospackages/avt_341_interface/config/mission_points.csv',
                        help='Path to the input CSV file'
                        )
    parser.add_argument('--output_csv',
                        default='../config/krc_mission_points.csv',
                        help='Path to the input CSV file')
    args = parser.parse_args()

    input_df = pd.read_csv(args.input_csv)
    output_df = pd.read_csv(args.output_csv)

    input_df = input_df.set_index('name')
    output_df = output_df.set_index('name')

    orientation_fields = ['rot_x', 'rot_y', 'rot_z', 'rot_w']
    common_names = output_df.index.intersection(input_df.index)
    for field in orientation_fields:
        output_df.loc[common_names, field] = input_df.loc[common_names, field].values

    output_df = output_df.reset_index()

    # Write manually to preserve per-column formatting
    with open(args.output_csv, 'w', newline='') as f:
        f.write(','.join(output_df.columns) + '\n')
        for _, row in output_df.iterrows():
            values = []
            for col in output_df.columns:
                if col in orientation_fields:
                    values.append('%.3f' % row[col])
                else:
                    values.append(str(row[col]))
            f.write(','.join(values) + '\n')


if __name__ == '__main__':
    main()
