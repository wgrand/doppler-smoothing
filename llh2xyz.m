% Reference: http://wiki.gis.com/wiki/index.php/Geodetic_system#Local_north.2C_east.2C_down_.28NED.29_coordinates
% Convert WGS-84 to ECEF
function [X,Y,Z] = llh2xyz(lat,lon,h)
    % Convert lat, lon, height in WGS84 to ECEF X,Y,Z
    % lat and lon given in decimal degrees.
    % altitude should be given in meters
    lat = lat/180*pi; % converting to radians
    lon = lon/180*pi; % converting to radians
    a = 6378137.0; % earth semimajor axis in meters
    f = 1/298.257223563; % reciprocal flattening
    e2 = 2*f -f^2; % eccentricity squared
   
    chi = sqrt(1-e2*(sin(lat)).^2);
    X = (a./chi +h).*cos(lat).*cos(lon);
    Y = (a./chi +h).*cos(lat).*sin(lon);
    Z = (a*(1-e2)./chi + h).*sin(lat); %((1 - WGS84_E * WGS84_E) * N + llh[2]) * sin(llh[0]);