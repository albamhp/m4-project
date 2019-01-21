function [edgePot,edgeStruct]=CreateGridUGMModel(nRows, nCols, nStates, lambda)
%
%
% NumFils, NumCols: image dimension
% K: number of states
% lambda: smoothing factor


tic

nNodes = nRows*nCols;
 
adj = sparse(nNodes,nNodes);
 
% Add Down Edges
ind = 1:nNodes;
exclude = sub2ind([nRows nCols],repmat(nRows,[1 nCols]),1:nCols); % No Down edge for last row
ind = setdiff(ind,exclude);
adj(sub2ind([nNodes nNodes],ind,ind+1)) = 1;
 
% Add Right Edges
ind = 1:nNodes;
exclude = sub2ind([nRows nCols],1:nRows,repmat(nCols,[1 nRows])); % No right edge for last column
ind = setdiff(ind,exclude);
adj(sub2ind([nNodes nNodes],ind,ind+nRows)) = 1;
 
% Add Up/Left Edges
adj = adj+adj';
edgeStruct = UGM_makeEdgeStruct(adj,nStates);

edgePot = zeros(nStates, nStates,edgeStruct.nEdges);
for e = 1:edgeStruct.nEdges
%    n1 = edgeStruct.edgeEnds(e,1);
%    n2 = edgeStruct.edgeEnds(e,2);
%    pot_same = exp(lambda(1) + lambda(2)*1/(1+abs(Xstd(n1)-Xstd(n2))));
   pot_same = exp(lambda(1) + lambda(2)*1);
   m = ones([nStates, nStates]);
   m(1:1+size(m,1):end) = pot_same;
   edgePot(:,:,e) = m;
end

toc;