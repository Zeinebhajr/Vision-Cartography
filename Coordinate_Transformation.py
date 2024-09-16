from pyproj import Transformer
import pandas as pd
csv_path = "location1.csv"
gps_data = pd.read_csv(csv_path)
transformer = Transformer.from_crs("EPSG:4326","EPSG:3857")
def calculate_xy(row):
    x, y = transformer.transform(row[' Latitude'], row[' Longitude'])
    return pd.Series({'x': x, 'y': y})

gps_data[['x', 'y']] = gps_data.apply(calculate_xy, axis=1)
gps_data.to_csv('location_with_xy1.csv', index=False)

