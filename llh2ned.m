function ned = llh2ned(ref, llh)
    % Convert Geodetic coordinates to Cartesian coordinates (in meters)
    % with respect to an reference coordinate.
    
    % Step 1: llh to ECEF
    [X,Y,Z] = llh2xyz(llh(1),llh(2),llh(3));
    % Step 2: ECEF to NED
    ned = xyz2ned(ref(1), ref(2), ref(3), X, Y, Z);