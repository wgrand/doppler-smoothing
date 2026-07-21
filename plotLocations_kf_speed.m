clearvars;
%% initialization
x=[0;
     0;
     0;
     0];
L=readmatrix('2026-07-11_18_50_37_my_iOS_device.csv');

% Column indices for this raw iOS CSV export (1-indexed):
%   3 =locationTimestamp_since1970(s)
%   4 =locationLatitude(WGS84)
%   5 =locationLongitude(WGS84)
%   7 =locationSpeed(m/s)
%   8 =locationSpeedAccuracy(m/s)
%   9 =locationCourse(deg)          -- GPS-derived direction of travel
%   10=locationCourseAccuracy(deg)
%   12=locationHorizontalAccuracy(m)
maxCourseAccuracy=45; % deg; discard course samples less confident than this
maneuverThresholdDeg=20; % deg; a course change bigger than this between
% consecutive fixes is treated as a turn/corner
maneuverBoost=8; % how much to inflate process noise during a detected turn
lastValidCourseDeg=NaN;

P=diag([L(2,12)^2, L(2,12)^2, L(2,8)^2, L(2,8)^2]); % hAccuracy, hAccuracy, sAccuracy, sAccuracy of first sample
P_LAST=P;
Q_hacc=1; % Typical GPS drift is about 5-meters, but the typical drift from point to point is about 1-meter
Q_sacc=0.6; % Drift from speed to speed
Q=[Q_hacc^2 0 Q_sacc^2 0;
   0 Q_hacc^2 0 Q_sacc^2;
   0 0 Q_sacc^2 0;
   0 0 0 Q_sacc^2]; % environment noise (we have to compute this from our results because we don't know it to start with)
N=0;
C=0;
psi=0;
for k=2:length(L)-1
    %% initialize variables
    phi1=L(k,4); % latitude
    lambda1=L(k,5); % longitude
    s=L(k,7); % speed (m/s)
    % A plain constant-velocity KF predicts position by extrapolating the
    % previous velocity in a straight line, and only trusts the new
    % measurement as much as Q says it should -- around a sharp corner this
    % makes it overshoot before the new heading pulls it back. Detect large
    % course changes (real turns) and temporarily inflate the process noise
    % for that step so the filter lets go of the straight-line assumption
    % right where it needs to.
    isManeuvering=false;
    if L(k,9)~=-1 && L(k,10)~=-1 && L(k,10)<=maxCourseAccuracy
        if ~isnan(lastValidCourseDeg)
            courseDelta=mod(L(k,9)-lastValidCourseDeg+180, 360)-180;
            if abs(courseDelta) > maneuverThresholdDeg
                isManeuvering=true;
            end
        end
        lastValidCourseDeg=L(k,9);
        psi=L(k+1,9)/180*pi; % use course from next step as heading
    end
    hAccuracy=L(k,12); % horizontal accuracy
    sAccuracy=L(k,8);
    dt=max(L(k,3)-L(k-1,3), eps);
    q=Q_sacc^2; % noise power density
    if isManeuvering
        q=q * maneuverBoost; % let the filter respond faster right at this turn
    end
    Q=[dt^3/3*q     0        dt^2/2*q     0;
       0            dt^3/3*q 0            dt^2/2*q;
       dt^2/2*q     0        dt*q         0;
       0            dt^2/2*q 0            dt*q];
    % acquire first location
    if k == 2
        lla0=[phi1 lambda1 0];
    end
    ned=llh2ned(lla0,[phi1 lambda1 0]);
    %% predict (process) based on basic kinematics (predict new location based on speed and heading)
    F=[1 0 dt 0;
       0 1 0 dt;
       0 0 1 0;
       0 0 0 1]; % 1
    x=F*x; % 2
    P=F*P*F'+Q; % 3
    %% update (measurement)
    if s<0.5
        H=[1 0 0 0;
           0 1 0 0]; % 4
        R=[hAccuracy^2 0;
           0 hAccuracy^2]; % 4: Assume that accuracy values represent 1 standard deviation (1-sigma (68% of measurements))
    else
        H=eye(4); % 4
        R=[hAccuracy^2 0 0 0;
           0 hAccuracy^2 0 0;
           0 0 sAccuracy^2 0;
           0 0 0 sAccuracy^2]; % 4: Assume that accuracy values represent 1 standard deviation (1-sigma (68% of measurements))
    end
    % Kalman gain
    K=P*H'/(H*P*H'+R); % 5
    if s<0.5
        z=[ned(1);ned(2)]; % 6
    else
        z=[ned(1);ned(2);s*cos(psi);s*sin(psi)]; % 6
    end
    x_update=x + K*(z - H*x); %7
    x=x_update;
    P=P-K*H*P; % 8 This comes from bzarg and other standard literature on KF
    P_LAST=P;
    [phi2,lambda2,h2]=ned2llh(lla0,[x(1) x(2) 0]);
    L_(k-1,1:4)=[phi2,lambda2,phi1,lambda1];
end
geoplot(L_(20000:26000,1),L_(20000:26000,2),'LineWidth', 2); % new curve
legend('KF');