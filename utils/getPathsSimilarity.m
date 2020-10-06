function similarity = getPathsSimilarity(path1, path2, distThreshold)
    switch nargin
        case 3
            n = size(path1,1);
            nDiff = 0;
            for i = 1:n
                distance = norm(path1(i,1:2) - path2(i,1:2));
                if distance > distThreshold
                    nDiff = nDiff+1;
                end
            end
            similarity = 1 - nDiff/n;
        otherwise
           error('Not enough input arguments.'); 
    end
end