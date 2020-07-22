function D = omega(U)

D=U*inv(U'*U)*U'-eye(max(size(U)));

end