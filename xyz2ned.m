function ned = xyz2ned(refLat, refLon, refH, X, Y, Z)
  % convert ECEF coordinates to local north, east, down
 
  % find reference location in ECEF coordinates
  [Xr,Yr,Zr] = llh2xyz(refLat, refLon, refH);
 
  % convert from degrees to radians
  refLat = refLat/180*pi;
  refLon = refLon/180*pi;
  
  % compute rotation matrix
  sin_lat=sin(refLat);
  sin_lon=sin(refLon);
  cos_lat=cos(refLat);
  cos_lon=cos(refLon);
  R = [-sin_lat*cos_lon -sin_lat*sin_lon cos_lat;
       -sin_lon cos_lon 0;
       -cos_lat*cos_lon -cos_lat*sin_lon -sin_lat];

  ned = R*[X-Xr;Y-Yr;Z-Zr]; 
  
  % the following is the result of a rotation matrix
%   N = -sin(refLat).*cos(refLon).*(X-Xr) - sin(refLat).*sin(refLon).*(Y-Yr) + cos(refLat).*(Z-Zr);
%   E = -sin(refLon).*(X-Xr) + cos(refLon).*(Y-Yr);
%   D = -cos(refLat).*cos(refLon).*(X-Xr) - cos(refLat).*sin(refLon).*(Y-Yr) - sin(refLat).*(Z-Zr);
%   ned = [N,E,D];