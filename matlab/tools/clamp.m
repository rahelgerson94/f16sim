function y = clamp(x, lowerBound, upperBound)
    y = min(max(x, lowerBound), upperBound);
end
