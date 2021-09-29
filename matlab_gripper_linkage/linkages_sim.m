clc; clear ; close all;

% linkages parameter (mm) measured in SolidWorks
para = struct;
para.f1_L = 96.17692031; para.f1_R = para.f1_L; %fingertip2joint4
para.f2_L = 110.51918617; para.f2_R = para.f2_L; %fingertip2joint5
para.L1 = 31.75;
para.L2 = 86.4889215;
para.L3 = 19.05;
para.L4 = 100;
para.L5 = para.L3; %ground link
para.q5_offset = 20; %angle offset of ground link 
para.Ori_offsetX = 12.7; %offset from palm origin to five bar origin
para.Ori_offsetY = 9.38;
para.phi_L = acos((para.f2_L^2-para.f1_L^2-para.L3^2)/(-2*para.f1_L*para.L3));
para.phi_R = acos((para.f2_R^2-para.f1_R^2-para.L3^2)/(-2*para.f1_R*para.L3));

J1 = FiveBarJaw(0,para,gca);
J2 = FiveBarJaw(1,para,gca);

Q1Min = 50;
Q1Max = 95.52334673;
Q21Min = 106;
Q21Max = 130;

%mapping matrix
ft2joint = [];%zeros((Q1Max-Q1Min)*(Q21Max-Q21Min),4); %[dx dy Q1_ Q21]
idx = 1;

% hold on;
set(gcf,'position',[0,0,1000,1000]);
xlabel(gca,'x');
ylabel(gca,'y');
grid on;
axis equal;
itv = 20;
xMax = 140; xMin = -xMax;
yMax = 190; yMin = -30;
xlim([xMin,xMax]); ylim([yMin,yMax]);
set(gca,'xTick',(xMin:itv:xMax)); set(gca,'yTick',(yMin:itv:yMax));

for Q1_ = Q1Min:Q1Max
    disp("Q1:"+num2str(Q1_));
    Q1_L = deg2rad(90+Q1_);
    Q1_R = deg2rad(90-Q1_);

    J2.compute_joint_pos(para,Q1_R,Q21Min);
    J2.redraw();

    for Q21 = Q21Min:Q21Max
        J1.compute_joint_pos(para,Q1_L,Q21);

        if ~isreal(J1.q3)
            % disp("Q21 too large!");
            break;
        end

        J1.redraw()

        % assign mapped value to matrix
        dx = J2.ft(1) - J1.ft(1);
        dy = J2.ft(2) - J1.ft(2);
        ft2joint(idx,:) = [dx dy Q1_ Q21];
        idx = idx + 1;

        %pause(0.0000); %comment this to disable the animation
    end 

end

%plot mapping data in dx dy space
figure;
set(gcf,'position',[1100,0,800,1000]);
workspace = subplot(2,1,1);
title(workspace,'Fingertip workspace')
xlabel(workspace,'delta x')
ylabel(workspace,'delta y')
hold on;
axis equal;
axis([-90 110 0 100])
grid on;
plot(ft2joint(:,1), ft2joint(:,2), '.b'); 
%plot input dx dy as arc
tilt_angle = 90;
step = 1;
obj_thick = 15;
joint_output = [];
idx = 1;
for i = 0:step:tilt_angle
    x = obj_thick * cos(deg2rad(i));
    y = obj_thick * sin(deg2rad(i));
    plot(x,y,'.r');
    
    % 2d interpolation
    joint_output(idx,1) = griddata(ft2joint(:,1),ft2joint(:,2),ft2joint(:,3),x,y);
    joint_output(idx,2) = griddata(ft2joint(:,1),ft2joint(:,2),ft2joint(:,4),x,y); %angle q21
    joint_output(idx,3) = sqrt(para.L1^2+para.L2^2-2*para.L1*para.L2*cos(deg2rad(joint_output(idx,2)))); % link21 
    idx = idx +1;
end
%disp(joint_output);

% plot action from input
action = subplot(2,1,2);
title(action,'Gripper action')
grid on;
axis equal;
xMax = 140; xMin = -xMax;
yMax = 190; yMin = -30;
xlim([xMin,xMax]); ylim([yMin,yMax]);
J1_test = FiveBarJaw(0,para,action);
J2_test = FiveBarJaw(1,para,action);
for i = 1:length(joint_output)
    J1_test.compute_joint_pos(para,deg2rad(90+joint_output(i,1)),joint_output(i,2));
    J2_test.compute_joint_pos(para,deg2rad(90-joint_output(i,1)),Q21Min);
    J1_test.redraw();
    J2_test.redraw();
    ft_dist = sqrt((J1_test.ft(1) - J2_test.ft(1))^2 + (J1_test.ft(2) - J2_test.ft(2))^2);
    %disp(ft_dist);
    pause(0.05);
end