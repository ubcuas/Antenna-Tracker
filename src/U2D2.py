#  U2D2 / Dynamixel interface
#  read motor states, init serial, write pos, get status updates from motors
#  emergency stop function, motor max min pos, read accelerometer, read gps

#  given target lat lng alt point at it
#  proj4 convert from EPSG4326 to EPSG3857
#  use delta x,y,z for vector angles
#  feed vector angles to motors
#
#

from pyproj import CRS, Transformer

at_coord = {
    "lat":49.260890,
    "lon":-123.252026,
    "alt": 212
}

target_coord = {
    "lat":49.259269,
    "lon":-123.250599,
    "alt": 240
}

# metric projected coords ref
# https://epsg.io/4326
EPSG3857_CRS = CRS.from_proj4("+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext  +no_defs")

# GPS coords ref
# https://epsg.io/3857
EPSG4326_CRS = CRS.from_proj4("+proj=longlat +datum=WGS84 +no_defs")

a = Transformer.from_crs(crs_from=EPSG4326_CRS, crs_to=EPSG3857_CRS)

at_coord_m = a.transform(at_coord["lon"], at_coord["lat"])
target_coord_m = a.transform(target_coord["lon"], target_coord["lat"])

lon_delta = target_coord_m[0] - at_coord_m[0]
lat_delta = target_coord_m[1] - at_coord_m[1]
alt_delta = target_coord["alt"] - at_coord["alt"]

delta_list = [lat_delta,lon_delta]

def n_vector(v):
    x, y = v
    d = ((x**2)+(y**2))**0.5
    return [x/d,y/d]

def dot_prod(v1,v2):
    x1, y1 = v1
    x2, y2 = v2
    return x1*x2+y1*y2

dir_pointing = n_vector([20,30])
# will be the vector given by onboard magnetometer

a = n_vector(delta_list)
b = dir_pointing

ACTIONS = {
    "STAY":0,
    "RIGHT":1,
    "LEFT":2
}

def action(c, t):
    p1 = [c[1], -c[0]] # left perpendicular
    p2 = [-c[1], c[0]] # right perpendicular
    pd1 = dot_prod(p1, t)
    pd2 = dot_prod(p2, t)
    if pd1 < 0:
        return ACTIONS["RIGHT"]
    elif pd2 < 0:
        return ACTIONS["LEFT"]
    elif pd1 == pd2:
        return ACTIONS["STAY"]

print(action(b, b))
