import numpy as np
def calculate_distance(x1, y1, x2, y2):
    distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


import math
#def distance_entre_points(lat1, lon1, lat2, lon2):
    # Convertir les coordonnées degrés en radians
   # lat1 = math.radians(lat1)
    #lon1 = math.radians(lon1)
    #lat2 = math.radians(lat2)
    #lon2 = math.radians(lon2)

    # Rayon moyen de la Terre en kilomètres
    #rayon_terre = 6371000.0

    # Calculer la distance orthodromique
    #d = rayon_terre * math.acos(math.sin(lat1) * math.sin(lat2) + math.cos(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    #return d


# Coordonnées GPS des deux points (par exemple, Paris et New York)
#lat1 = 36.88411000  # Latitude de Paris
#lon1 = 10.18361500 # Longitude de Paris
#lat2 = 36.88400167  # Latitude de New York
#lon2 = 10.18357333  # Longitude de New York

# Calculer la distance entre les deux points
#distance = distance_entre_points(lat1, lon1, lat2, lon2)
#print("La distance est d'environ", round(distance,2) "mètres.")