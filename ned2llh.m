function [phi,lambda,h] = ned2llh(ref, ned)
    % Returns new Geodetic coordinate with displacement in Cartesian
    % coordinates (meters).

    % Step 1: NED to ECEF
    [X,Y,Z] = ned2xyz(ref(1), ref(2), ref(3), ned(1), ned(2), ned(3));

    % Step 2: ECEF to WGS-84
    [phi,lambda,h] = xyz2llh(X, Y, Z);