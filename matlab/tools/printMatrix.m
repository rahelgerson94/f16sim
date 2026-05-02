function printMatrix(A, precision)
    if nargin < 2
        precision = 2;
    end

    [rows, cols] = size(A);

    % Convert to strings first
    strMat = strings(rows, cols);
    maxWidth = 0;

    for i = 1:rows
        for j = 1:cols
            str = sprintf(['%.' num2str(precision) 'f'], A(i,j));
            strMat(i,j) = str;
            maxWidth = max(maxWidth, strlength(str));
        end
    end

    % Print with padding
    for i = 1:rows
        for j = 1:cols
            fprintf('%*s ', maxWidth, strMat(i,j));
        end
        fprintf('\n');
    end
    fprintf('\n');
end