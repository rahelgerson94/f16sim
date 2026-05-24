figure; plot(betaVec,CmInB, "LineWidth",1.5); title("C_m"); grid on;
figure; labels ={"l", "m", "n"}; colors = {"r", "b", "g"};
for i = 1:3
    plot(betaVec, CmDampingInB(:,i), "DisplayName", sprintf("C_%s", labels{i}), ...
        "LineWidth",1.5,...
    "Color" , colors{i}); hold on;
end

legend; grid on; xlabel("β (deg)"); title("Damping Derivatives");
hold off;