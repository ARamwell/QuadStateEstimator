function symArrToCharTable(A_sym)
%SYMARRTOCHARTABLE Summary of this function goes here
% Convert each element to a character array and store in a cell array
[rows, cols] = size(A_sym);
B_char = cell(rows, cols);
for ii = 1:rows
    for jj = 1:cols
        B_char{ii, jj} = char(A_sym(ii, jj));
    end
end

% Export the cell array
% To a delimited text file (e.g., CSV)
writecell(B_char, 'symbolic_data.csv');
end

