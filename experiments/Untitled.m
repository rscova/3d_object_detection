clear;clc
load('csv_wks1.mat');

name = 'Figs/fpfhevaluatepiggy.png';
name_fig = 'Figs/fpfhevaluatepiggy.fig';

d1 = fpfhevaluatepiggy{:,{'Features'}};
d2 = fpfhevaluatepiggy{:,{'Descriptor'}};
d3 = fpfhevaluatepiggy{:,{'FPFH'}};
middle_dist_index = find(d1 == 'Middle');

middle_dist = zeros(1,length(middle_dist_index));
radius_kpts = zeros(1,length(middle_dist_index));
radius_desc = zeros(1,length(middle_dist_index));
radius_inlier = zeros(1,length(middle_dist_index));


for i = 1:length(middle_dist_index)
    radius_kpts(1,i) = d3(middle_dist_index(i)-3,1);
    radius_desc(1,i) = d3(middle_dist_index(i)-2,1);
    radius_inlier(1,i) = d3(middle_dist_index(i)-1,1);
    middle_dist(1,i) = d3(middle_dist_index(i),1);
end


outlier_reject_index = find(middle_dist == 10000);
outlier_reject = zeros(1,length(outlier_reject_index));

for i = 1:length(outlier_reject_index)
    middle_dist(1,outlier_reject_index(1,i)) = nan;
end

h = figure(1);
subplot(231);
plot(middle_dist(1:125)*1000);
title('FPFH PIGGY - ISS'); ylabel('error (mm)'); xlabel('samples');axis([0 125 0 2]);
subplot(232);
plot(middle_dist(126:250)*1000);
title('US'); ylabel('error (mm)'); xlabel('samples');axis([0 125 0 2]);
subplot(233)
plot(middle_dist(251:375)*1000);
title('SUSAN'); ylabel('error (mm)'); xlabel('samples');axis([0 125 0 2]);
subplot(234)
plot(middle_dist(376:500)*1000);
title('HARRIS3D'); ylabel('error (mm)'); xlabel('samples');axis([0 125 0 2]);
subplot(235)
plot(middle_dist(501:625)*1000);
title('HARRIS6D'); ylabel('error (mm)'); xlabel('samples');axis([0 125 0 2]);
subplot(236)
plot(middle_dist(626:length(middle_dist_index))*1000);
title('SIFT'); ylabel('error (mm)'); xlabel('samples');axis([0 125 0 2]);


print(h, '-dpng','-r500',name);
savefig(h,name_fig);
