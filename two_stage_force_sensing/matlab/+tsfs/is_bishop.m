function yes=is_bishop(fr)
% Explicit observation type; legacy frames retain material-curvature behavior.
yes=isfield(fr,'observationType')&&strcmp(fr.observationType,'bishop-curvature');
end
