function plot_depth_observations(depthMm,observation,destination)
%PLOT_DEPTH_OBSERVATIONS Diagnostic preview, separate from costly estimation.
fig=figure('Visible','off','Position',[100 100 1000 400]);
cleanup=onCleanup(@()close(fig));
ax=subplot(1,2,1); imagesc(ax,depthMm,'AlphaData',isfinite(depthMm));
set(ax,'Color',[0.8 0.8 0.8]); axis(ax,'image'); colorbar(ax);
title(ax,'Synthetic Z-depth (mm), missing = gray');
ax=subplot(1,2,2); imagesc(ax,observation.inlierMask); colormap(ax,gray(2));
axis(ax,'image'); title(ax,'Sampled plane inliers (white)');
exportgraphics(fig,destination,'Resolution',150);
end
