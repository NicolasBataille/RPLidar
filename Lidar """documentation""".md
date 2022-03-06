# <u>Documentation pour le RPLidar de Slamtech</u>

## 

## SDK :

- #### Normalement, seul "rplidar.h" suffit pour avoir toute les méthodes.

- #### Les données retournées sont sous la forme d'une matrice de forme suivante :

| Theta (angle de la prise de vue, en degré) | Dist (Distance entre le lidar et l'élément correspondant, en mm ==> diviser par 4.0) | Q (Quality  of the analysis, from 0 to 255) |     |
|:------------------------------------------:| ------------------------------------------------------------------------------------ | ------------------------------------------- | --- |

#### Tableau des valeurs retournées par le scan (source documentation officielle)![](/home/batum/.var/app/com.github.marktext.marktext/config/marktext/images/2022-02-24-12-56-37-image.png)

- #### Fonctionnement des 4 états possible du Lidar :(idle, scanning, Request processing et protection stop rate)![](/home/batum/.var/app/com.github.marktext.marktext/config/marktext/images/2022-02-24-12-47-06-image.png)

- #### 

https://bucket-download.slamtec.com/351a5409ddfba077ad11ec5071e97ba5bf2c5d0a/LR002_SLAMTEC_rplidar_sdk_v1.0_en.pdf  => Doc complète Lidar
