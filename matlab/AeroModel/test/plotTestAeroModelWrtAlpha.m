plotVector3(alphaVec, DCL, "C_d, C_y, C_L vs α (deg)");

figure; plot(alphaVec,CmInB);
plotForceMomentDashboard(alphaVec, CfTotInW, CmTotInW);
figure; labels ={"l", "m", "n"}; colors = {"r", "b", "g"};
for i = 1:3
    plot(alphaVec, CmDampingInB(:,i), "DisplayName", sprintf("C_%s", labels{i}), ...
        "LineWidth",1.5,...
    "Color" , colors{i}); hold on;
end
legend; grid on; xlabel("α (deg)");
hold off;