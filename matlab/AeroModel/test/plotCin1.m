function fig = plotCin1(f16struct, inpToVary, ...
    inp1name, inp1value, ...
    inp2name, inp2value, ...
    dimensionalize)
%PLOTCIN1 Plot the six baseline aero channels while sweeping one input.
%
%   fig = plotCin1(F16AeroData, inpToVary, inp1name, inp1value, ...
%       inp2name, inp2value, dimensionalize)
%
% One held-fixed input may be a vector. In that case each subplot will
% contain one curve per fixed-input value, with a shared legend.

if nargin < 7
    dimensionalize = false;
end

if nargin < 1 || isempty(f16struct)
     h5File = fullfile( '../data/F16AeroData.h5');
    f16struct = load_aerodynamics(h5File);
end

if nargin < 6
    error(['Usage: plotCin1(f16struct, inpToVary, inp1name, inp1value, ' ...
        'inp2name, inp2value, dimensionalize)']);
end

param = load_F16_params();

variedVarName = normalizeInputName(inpToVary);
fixedVar1Name = normalizeInputName(inp1name);
fixedVar2Name = normalizeInputName(inp2name);

if strcmp(variedVarName, fixedVar1Name) || strcmp(variedVarName, fixedVar2Name) || strcmp(fixedVar1Name, fixedVar2Name)
    error('inpToVary, inp1name, and inp2name must refer to three distinct inputs.');
end

[alphaSweep, betaSweep, dhSweep] = getIndependentVariableRanges(f16struct);
[alphaMin, alphaMax, betaMin, betaMax, dh1Min, dh1Max, dh2Min, dh2Max] = getInputRanges(f16struct);

validateFixedInput(fixedVar1Name, inp1value, alphaMin, alphaMax, betaMin, betaMax, dh1Min, dh1Max);
validateFixedInput(fixedVar2Name, inp2value, alphaMin, alphaMax, betaMin, betaMax, dh1Min, dh1Max);

[curveFamilyName, curveFamilyValues, otherFixedName, otherFixedValue] = ...
    getCurveFamily(fixedVar1Name, inp1value, fixedVar2Name, inp2value);

switch variedVarName
    case 'alpha'
        xBase = alphaSweep;
    case 'beta'
        xBase = betaSweep;
    case 'dh'
        xBase = dhSweep;
end

x = makeDenseSweep(xBase);

vals = nan(numel(x), 6, numel(curveFamilyValues));
for j = 1:numel(curveFamilyValues)
    for k = 1:numel(x)
        inputs = struct('alpha', 0, 'beta', 0, 'dh', 0);
        inputs.(variedVarName) = x(k);
        inputs.(curveFamilyName) = curveFamilyValues(j);
        inputs.(otherFixedName) = otherFixedValue;

        vals(k, 1, j) = f16struct.Cx(inputs.alpha, inputs.beta, inputs.dh);
        vals(k, 2, j) = f16struct.Cy(inputs.alpha, inputs.beta);
        vals(k, 3, j) = f16struct.Cz(inputs.alpha, inputs.beta, inputs.dh);

        if inputs.dh < dh2Min || inputs.dh > dh2Max
            vals(k, 4, j) = nan;
            vals(k, 6, j) = nan;
        else
            vals(k, 4, j) = f16struct.Cl(inputs.alpha, inputs.beta, inputs.dh);
            vals(k, 6, j) = f16struct.Cn(inputs.alpha, inputs.beta, inputs.dh);
        end

        vals(k, 5, j) = f16struct.Cm(inputs.alpha, inputs.beta, inputs.dh);
    end
end

if dimensionalize
    vals(:, 1:3, :) = vals(:, 1:3, :) * param.S;
    vals(:, 4, :) = vals(:, 4, :) * param.S * param.b;
    vals(:, 5, :) = vals(:, 5, :) * param.S * param.cbar;
    vals(:, 6, :) = vals(:, 6, :) * param.S * param.b;

    yLabels = {'F_x / qbar', 'F_y / qbar', 'F_z / qbar', ...
        'M_x / qbar', 'M_y / qbar', 'M_z / qbar'};
else
    yLabels = {'C_x', 'C_y', 'C_z', 'C_l', 'C_m', 'C_n'};
end

titles = {'X-axis', 'Y-axis', 'Z-axis', 'Roll', 'Pitch', 'Yaw'};
xLabel = inputAxisLabel(variedVarName);

fig = figure('Name', 'F16 aero sweep', 'Color', 'w');
t = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
lineHandles = gobjects(numel(curveFamilyValues), 1);

for i = 1:6
    nexttile;
    hold on;
    for j = 1:numel(curveFamilyValues)
        h = plot(x, vals(:, i, j), 'LineWidth', 1.5);
        if i == 1
            lineHandles(j) = h;
        end
    end
    hold off;
    grid on;
    xlabel(xLabel);
    ylabel(yLabels{i}, 'Interpreter', 'tex');
    title(titles{i});
end

if numel(curveFamilyValues) > 1
    lgd = legend(lineHandles, composeLegendEntries(curveFamilyName, curveFamilyValues));
    lgd.Layout.Tile = 'north';
end

sgtitle(makeFigureTitle(xLabel, fixedVar1Name, inp1value, fixedVar2Name, inp2value));

end

function name = normalizeInputName(name)
name = lower(strtrim(string(name)));
switch char(name)
    case {'alpha', 'a'}
        name = 'alpha';
    case {'beta', 'b'}
        name = 'beta';
    case {'dh', 'de', 'delta_h', 'deltae', 'delta_e', 'elevator', 'ele'}
        name = 'dh';
    otherwise
        error('Unsupported input name: %s', name);
end
name = char(name);
end

function label = inputAxisLabel(name)
switch name
    case 'alpha'
        label = '\alpha (deg)';
    case 'beta'
        label = '\beta (deg)';
    case 'dh'
        label = '\delta_e (deg)';
    otherwise
        label = name;
end
end

function [alphaVec, betaVec, dhVec] = getIndependentVariableRanges(f16struct)
alphaVec = f16struct.Cx.GridVectors{1};
betaVec = f16struct.Cx.GridVectors{2};
dhVec = f16struct.Cx.GridVectors{3};
end

function [alphaMin, alphaMax, betaMin, betaMax, dh1Min, dh1Max, dh2Min, dh2Max] = getInputRanges(f16struct)
alphaGrid = f16struct.Cx.GridVectors{1};
betaGrid = f16struct.Cx.GridVectors{2};
dh1Grid = f16struct.Cx.GridVectors{3};
dh2Grid = f16struct.Cl.GridVectors{3};

alphaMin = min(alphaGrid);
alphaMax = max(alphaGrid);
betaMin = min(betaGrid);
betaMax = max(betaGrid);
dh1Min = min(dh1Grid);
dh1Max = max(dh1Grid);
dh2Min = min(dh2Grid);
dh2Max = max(dh2Grid);
end

function validateFixedInput(name, value, alphaMin, alphaMax, betaMin, betaMax, dhMin, dhMax)
for k = 1:numel(value)
    switch name
        case 'alpha'
            validateRange(name, value(k), alphaMin, alphaMax);
        case 'beta'
            validateRange(name, value(k), betaMin, betaMax);
        case 'dh'
            validateRange(name, value(k), dhMin, dhMax);
        otherwise
            error('Unsupported input name: %s', name);
    end
end
end

function validateRange(name, value, lowerBound, upperBound)
if value < lowerBound || value > upperBound
    error('Input %s = %.3g is outside the valid range [%.3g, %.3g].', ...
        name, value, lowerBound, upperBound);
end
end

function [curveFamilyName, curveFamilyValues, otherFixedName, otherFixedValue] = ...
    getCurveFamily(fixedVar1Name, inp1value, fixedVar2Name, inp2value)
if numel(inp1value) > 1 && numel(inp2value) > 1
    error('Only one held-fixed input may be a vector at a time.');
elseif numel(inp1value) > 1
    curveFamilyName = fixedVar1Name;
    curveFamilyValues = reshape(inp1value, 1, []);
    otherFixedName = fixedVar2Name;
    otherFixedValue = inp2value;
elseif numel(inp2value) > 1
    curveFamilyName = fixedVar2Name;
    curveFamilyValues = reshape(inp2value, 1, []);
    otherFixedName = fixedVar1Name;
    otherFixedValue = inp1value;
else
    curveFamilyName = fixedVar1Name;
    curveFamilyValues = inp1value;
    otherFixedName = fixedVar2Name;
    otherFixedValue = inp2value;
end
end

function entries = composeLegendEntries(name, values)
entries = cell(1, numel(values));
for k = 1:numel(values)
    entries{k} = sprintf('%s = %.3g deg', inputAxisLabel(name), values(k));
end
end

function titleText = makeFigureTitle(xLabel, fixedVar1Name, inp1value, fixedVar2Name, inp2value)
titleText = sprintf('%s sweep, %s, %s', xLabel, ...
    formatFixedInput(fixedVar1Name, inp1value), ...
    formatFixedInput(fixedVar2Name, inp2value));
end

function txt = formatFixedInput(name, value)
if numel(value) == 1
    txt = sprintf('%s = %.3g deg', inputAxisLabel(name), value);
else
    txt = sprintf('%s = [%s] deg', inputAxisLabel(name), num2str(value));
end
end

function xDense = makeDenseSweep(xBase)
if numel(xBase) < 2
    xDense = xBase;
    return;
end

nDense = max(200, 8 * numel(xBase));
xDense = linspace(min(xBase), max(xBase), nDense);
end
