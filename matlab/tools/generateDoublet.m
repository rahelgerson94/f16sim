function u = generateDoublet(numSamples, amp, trimVal, stepDurationSamples)
% MAKEDOUBLET Create a doublet input vector.
%   numSamples = SIM_DURATION/dt
%   amp: input amplitude
%   trim: input trim value
%   stepDurationSamples: the duration (in samples) of the step input
%   First step:  trimVal + amp
%   Second step: trimVal - amp
%   Rest:        trimVal

    u = trimVal * ones(numSamples, 1);

    % Positive pulse
    u(1:stepDurationSamples) = trimVal + amp;

    % Negative pulse
    u(stepDurationSamples+1 : 2*stepDurationSamples) = trimVal - amp;

end