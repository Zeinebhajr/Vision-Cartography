# Importation des bibliothèques nécessaires
from pyproj import Transformer
from scipy.spatial.transform import Rotation
import pickle
from Rotation_Functions import quaternion_to_rotation_matrix
from Rotation_Functions import transform_object_coordinates
from ultralytics import YOLO
import cv2
import pandas as pd
from DISTANCE import calculate_distance
from Triangulation import find_depth
import numpy  as np
# Définition des classes d'objets
clas = ['Magasin-générale','Carrefour','Monoprix','ATB','BT','Biat','Pharmacie','Ooredoo','Orange','Tunisie-Télécom']
# Chargement des matrices de calibration de la caméra
with open('../Camera_Calibration/cameraMatrix.pkl', 'rb') as f:
    cameraMatrix = pickle.load(f)
with open('../Camera_Calibration/dist.pkl', 'rb') as f:
    dist = pickle.load(f)
# Chemin du fichier vidéo et des fichiers CSV contenant les données GPS et des capteurs
video_path = "camera_1.mp4"
cap = cv2.VideoCapture(video_path)
csv_path = "location_with_xy1.csv"
gps_data = pd.read_csv(csv_path)
csv_path2="sensor_filtered1.csv"
gps_data2 = pd.read_csv(csv_path2)
# Chargement du modèle YOLO pour la détection d'objets
model=YOLO('../YOLO-Weights/POI4.pt')
# Paramètre de l'angle alpha
alpha=75
# Fichier de sortie pour les coordonnées des objets
output_file = open('object_coordinates.txt', 'w')
# Initialisation des historiques
history=[]
history1=[]
history2=[]
history3=[]
history4=[]
# Boucle pour traiter chaque frame de la vidéo
while True:
   ret, frame = cap.read()
   # Correction de la distorsion de la caméra
   h, w = frame.shape[:2]
   newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))
   mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w, h), 5)
   a = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
   history3.append(frame)
   # Temps actuel de la vidéo en secondes
   current_time = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000  # Convert milliseconds to seconds
   # Recherche des coordonnées GPS et des données capteurs les plus proches en temps
   closest_gps = gps_data.iloc[(gps_data[' Time'] - current_time).abs().argsort()[:1]]
   closest_sensor = gps_data2.iloc[(gps_data2[' Time'] - current_time).abs().argsort()[:1]]
   # Extraction des coordonnées GPS et des valeurs du quaternion
   XG = closest_gps['x'].values[0]
   YG = closest_gps['y'].values[0]
   q0 = closest_sensor[' Value0'].values
   q1 = closest_sensor[' Value1'].values
   q2= closest_sensor[' Value2'].values
   q3 = closest_sensor[' Value3'].values
   quaternion = np.array([q0, q1, q2, q3]).reshape(4)
   # Conversion du quaternion en matrice de rotation
   rotation_matrix = quaternion_to_rotation_matrix(quaternion)
   # Détection et suivi des objets avec YOLO
   results = model.track(a, persist=True, conf=0.7)
   if results:
      for r in results:
         boxes = r.boxes
         for box in boxes:
           if box.id is not None:
              x1, y1, x2, y2 = box.xyxy[0]
              x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
              w, h = x2 - x1, y2 - y1
              xc,yc=x1+(w/2),y1+(h/2)
              cls = int(box.cls[0])
              id=box.id[0]
              id=int(id)
              # Ajout des coordonnées et de l'ID à l'historique
              history1.append([xc,yc,id])
              if[XG,YG,id] not in history:
                     if id in history2:
                          for p in range(len(history)):
                              if history[p][2]==id:
                                  index=p
                          distance = calculate_distance(history[p][0],history[p][1],XG,YG)
                          print(distance)
                          for p in range(len(history1)-1):
                              if history1[p][2]==id:
                                  index=p
                          X,Y,Z=find_depth(history1[-1][:2],history1[index][:2],history3[1],history3[0],distance,alpha)
                          print(X, Y, Z)
                          object_coordinates_camera=np.array([X,Y,Z])
                          camera_position=np.array([XG,YG,0])
                          # Transformation des coordonnées de la caméra aux coordonnées globales
                          object_coordinates_global = transform_object_coordinates(rotation_matrix,object_coordinates_camera,camera_position)
                          output_file.write(f"{object_coordinates_global[0]} {object_coordinates_global[1]} { clas[cls] }\n" )
                          print(object_coordinates_global[:2])
                          # Affichage des distances et coordonnées sur l'image
                          cv2.putText(a, "Distance: " + str(round(Z, 1)), (50, 50),  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                          cv2.putText(a, "X: " + str(round(object_coordinates_global[0], 1)), (80, 50),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                          cv2.putText(a, "Y: " + str(round(object_coordinates_global[1], 1)), (120, 50),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                          # Mise à jour des historiques
                          history1 = [item for item in history1 if item[2] != id]
                          history2.remove(id)
                          history4.append(id)
                     history.append([XG,YG,id])
              if id not in history2 and id not in history4 :
                history2.append(id)
   # Limite de la taille de l'historique des frames
   if len(history3) > 1:
          history3.pop(0)
   # Affichage de l'image
   cv2.imshow("Image",a)
   if not ret:
        break
# Release the video capture object
cap.release()
cv2.destroyAllWindows()
output_file.close()