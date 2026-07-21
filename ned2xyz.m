function [X, Y, Z] = ned2xyz(refLat, refLon, refH, n, e, d)
  % Convert north, east, down coordinates (labeled n, e, d) to ECEF
  % coordinates. The reference point (phi, lambda, h) must be given. All distances are in metres
 
  [Xr,Yr,Zr] = llh2xyz(refLat,refLon,refH); % location of reference point
    
  refLat=refLat/180*pi;
  refLon=refLon/180*pi;
  sin_lat=sin(refLat);
  sin_lon=sin(refLon);
  cos_lat=cos(refLat);
  cos_lon=cos(refLon);
  
  R = [-sin_lon -cos_lon*sin_lat cos_lon*cos_lat;
        cos_lon -sin_lon*sin_lat cos_lat*sin_lon;
        0        cos_lat  sin_lat];
    
  
xyz = R*[e;n;d] + [Xr;Yr;Zr];
X=xyz(1);
Y=xyz(2);
Z=xyz(3);
     
%   X = -sin(refLon)*e - cos(refLon)*sin(refLat)*n + cos(refLon)*cos(refLat)*d + Xr;
%   Y = cos(refLon)*e - sin(refLon)*sin(refLat)*n + cos(refLat)*sin(refLon)*d + Yr;
%   Z = cos(refLat)*n + sin(refLat)*d + Zr;