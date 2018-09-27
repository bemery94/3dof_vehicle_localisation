function MAD = medianAbsoluteDeviation(x)
    MAD = median(abs(x - median(x)));
end