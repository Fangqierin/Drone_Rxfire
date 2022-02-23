
import numpy as np
import matplotlib.pyplot as plt
import sys
import geopandas as gpd
sys.path.append('../../../')
from cpp_algorithms.common_helpers import imshow, imshow_scatter
from shapely.geometry import Point, Polygon
from pathlib import Path
from zipfile import ZipFile
from geopy import distance
from skimage import measure
from skimage.draw import polygon
import shapely
import shapely.geometry
shape=[]
from skimage.draw import polygon
import utm 

# Create an empty geopandas GeoDataFrame
newdata = gpd.GeoDataFrame()
# Let's see what we have at the moment
# Create a new column called 'geometry' to the GeoDataFrame
newdata['geometry'] = None
# Let's again see what's inside
#print(newdata)
# Coordinates of the Helsinki Senate square in Decimal Degrees
coordinates = [(24.950899, 60.169158), (24.953492, 60.169158), (24.953510, 60.170104), (24.950958, 60.169990)]

import math

def utmToLatLong(utmNorthing, utmEasting, utmZone):
    eastingOffset = 500000.0
    northingOffset = 10000000.0
    k0 = 0.9996
    equatorialRadius = 6378137.0
    eccSquared = 0.006694380023
    eccPrimeSquared = eccSquared / (1 - eccSquared)
    e1 = (1 - math.sqrt(1 - eccSquared)) / (1 + math.sqrt(1 - eccSquared));
    rad2deg = 180.0/math.pi

    # Casts input from string to floats or ints
    # Removes 500,000 metre offset for longitude
    xUTM = float(utmEasting) - eastingOffset
    yUTM = float(utmNorthing)
    zoneNumber = int(utmZone)

    # This line below is for debug purposes only, remove for batch processes.
    print ('The input is: ' + str(utmEasting) + 'm E, ' + str(utmNorthing) + 'm N in NAD83 UTM Zone ' + str(utmZone) + '\n')

    # Finds the origin longitude for the zone
    lonOrigin = (zoneNumber - 1) * 6 - 180 + 3 # +3 puts in zone centre

    M = yUTM / k0 #This finds the meridional arc
    mu = M / (equatorialRadius * (1- eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -5 * eccSquared * eccSquared * eccSquared /256))

    # Calculates the footprint latitude
    phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 /32) * math.sin(2*mu) + ( 21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * math.sin( 4 * mu) + (151 * e1 * e1 * e1 / 96) * math.sin(6 * mu)
    phi1 = phi1Rad * rad2deg

    # Variables for conversion equations
    N1 = equatorialRadius / math.sqrt( 1 - eccSquared * math.sin(phi1Rad) *  math.sin(phi1Rad))
    T1 = math.tan(phi1Rad) * math.tan(phi1Rad)
    C1 = eccPrimeSquared * math.cos(phi1Rad) * math.cos(phi1Rad)
    R1 = equatorialRadius * (1 - eccSquared) / math.pow(1 - eccSquared * math.sin(phi1Rad) * math.sin(phi1Rad), 1.5)
    D = xUTM / (N1 * k0)

    # Calculate latitude, in decimal degrees
    lat = phi1Rad - ( N1 * math.tan(phi1Rad) / R1) * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D / 720)
    lat = lat * rad2deg
    
    # Calculate longitude, in decimal degrees
    lon = (D - (1 + 2 * T1 + C1) * D * D * D / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1) * D * D * D * D * D / 120) / math.cos(phi1Rad)
    lon = lonOrigin + lon * rad2deg

    # Print function below is for debug purposes
#NOTE: THIS IS THE LOCATION WHERE THE NUMBERS ARE ROUNDED TO 5 DECIMAL PLACES
    print ("Lat: " + str(round(lat, 5)) + ", Long: " + str(round(lon,5)))
    
    return lat
    return lon

#For manual input
northing =  4308525
easting =  701235
zone = 10

utmToLatLong(northing, easting, zone)
coordinates=[(38.91922,-120.65556),(38.91706,-120.65802),(38.91546,-120.66233)]

BunsiteBound=(-2096325.0,2037105.0,-2092695.0,2040735.0)

# Create a Shapely polygon from the coordinate-tuple list
poly = Polygon(coordinates)
# Let's see what we have
#print(poly)
# Insert the polygon into 'geometry' -column at index 0
# newdata.loc[0, 'geometry'] = poly
# sh=newdata.set_crs(crs='EPSG:32633')
# print(sh)
u = utm.from_latlon(38.91922,120.000000)
print(f"FQ {u}")
# newdata.plot()
# plt.show()
print(newdata)
#imshow_scatter(matrix, alpha=0.4, color="black",s=5)