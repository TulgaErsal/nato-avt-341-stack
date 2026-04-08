import argparse
import geopandas
import json

"""
Converts GIS shape file representing no-go zones to JSON format polygon zones used by navstack.

NOTE: pip install geopandas to use script

Output example format:

{
	"zones": [
		{
			"label": "Zone 4",
			"occupancy": 100,
			"vertices": [
				[ 34.626497192382814, 19.300000000000001 ],
				[ 34.626497192382814, -0.70000000000000007 ],
				[ 51.947004394531248, 9.3000000000000007 ]
			],
		},
		{
			"label": "Zone 3",
			"occupancy": 100,
			"vertices": [
				[ 10.626497192382812, 52.800000000000004 ],
				[ 10.626497192382812, 32.799999999999997 ],
				[ 27.947004394531252, 42.800000000000004 ]
			],
		}
	]
}

"""

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--in_file',
                        type=str,
                        default='./no_go_zones.shp',
                        help='Path to input GIS .shp file.')
    parser.add_argument('--out_file',
                        type=str,
                        default='./no_go_zones.json',
                        help='Path to output .json file.')
    parser.add_argument('--target_crs',
                        type=str,
                        default='EPSG:6495',
                        help='CRS to convert to. Default of michigan north at 1 meter resolution')
    parser.add_argument('--local_x_origin',
                        type=float,
                        default=7886033.981,
                        help='Origin x component offset in CRS. Default corresponds to Traxara simulated KRC environment origin.')
    parser.add_argument('--local_y_origin',
                        type=float,
                        default=265598.6841,
                        help='Origin y component offset in CRS. Default corresponds to Traxara simulated KRC environment origin.')
    parser.add_argument('--label_field',
                        type=str,
                        default="",
                        help='Optional field in shape file to use as zone label. If not specified, index will be used as label.')
    parser.add_argument('--occ_value',
                        type=float,
                        default=100.0,
                        help='Occupancy height value to assign to no-go zones in costmap data structure. This value will be checked against the occupancy threshold setting.')
    args = parser.parse_args()
    return args


def main():

    args = parse_arguments()

    # Read the input shapefile
    gdf = geopandas.read_file(args.in_file)

    # CRS transform
    gdf = gdf.to_crs(args.target_crs)

    # Build zones list
    zones = []
    for idx, row in gdf.iterrows():
        polygon = row.geometry
        coords = list(polygon.exterior.coords)
        # Remove the closing vertex (duplicate of first)
        coords = coords[:-1]
        vertices = [[x - args.local_x_origin, y - args.local_y_origin] for x, y in coords]

        # Determine label
        if args.label_field and args.label_field in gdf.columns:
            label = str(row[args.label_field])
        else:
            label = str(idx)

        zone = {
            "label": label,
            "occupancy": args.occ_value,
            "vertices": vertices
        }
        zones.append(zone)

    output = {"zones": zones}
    with open(args.out_file, 'w') as f:
        json.dump(output, f, indent=4)


if __name__ == '__main__':
    main()