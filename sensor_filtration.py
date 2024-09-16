import pandas as pd
# Charger le fichier CSV
df = pd.read_csv('sensors1.csv')

# Filtrer les lignes où la valeur de l'attribut "sensor" est égale à 10
df_filtered = df[df['Sensor'] == 10]

# Enregistrer le DataFrame filtré dans un nouveau fichier CSV
df_filtered.to_csv('sensor_filtered1.csv', index=False)