function [Ffr, ind_removed, ind_keeped, ind_similar] = getFullRankMatrix(F)
%GETFULLRANKMATRIX Extract a full rank matrix removing linear dependencies
%   The input matrix F is decomposed in Q, R using the MatLab built-in
%   function qr. Then, the linear dependencies are removed and a new Ffr
%   full rank matrix is generated. The removed columns and rows indexes are
%   saved in variable ind_removed, and the maintained ones are saved in
%   variable ind_keeped. Lasly, the similar linear dependant column and
%   rows indexes are saved in variable ind_similar
    n = size(F,1);
    m = size(F,2);

    if (n ~= m)
        error('Input matrix must be square')
    end
%     if (~det(F))
%         error('The input matrix has full zero rows/columns')
%     end

    Ffr = F;
    ind_removed = [];
    ind_similar = [];
    ind_keeped = 1:size(F,1);
    
    if (rank(F) >= n)
        cprintf('comments','The input matrix is already full rank\n')
        return
    else        
        while rank(Ffr) < size(Ffr,1)
            Faux = F;
            [~, ~, E] = qr(Ffr,0);            
            ind_removed = [ind_removed ind_keeped(E(end))];
            ind_keeped(E(end)) = [];
            Faux(:,ind_removed) = [];
            Faux(ind_removed,:) = [];            
            Ffr = Faux;
        end
        for i = ind_removed
            for j = ind_keeped
                if(F(i,:)/F(j,:) >= 0.9999999999 && F(i,:)/F(j,:) <= 1.0000000001 ||...
                   isequal(F(i,:),F(j,:)))
                    ind_similar = [ind_similar j];
                    break;
                end
            end
        end
    end
end

