L=readmatrix('locationSamples_stationary+wiggle.csv');

lats=L(:,2);
lons=L(:,3);
speeds=L(:,5);

llh_ref=[lats(1),lons(1),0];

x=double.empty(0); 
y=double.empty(0);
s_filtered=double.empty(0);

for k=2:length(L)
    ned=llh2ned([lats(k-1),lons(k-1),0],[lats(k),lons(k),0]);
    x=[x;ned(1)];
    y=[y;ned(2)];
    if speeds(k)>0
%         s_filtered=[s_filtered;speeds(k)-speeds(k-1)];
        s_filtered=[s_filtered;speeds(k)];
    end
end

"Variance of differences of speed"
std(s_filtered)