function mu = manipulability(J,method)

[U,S,V] = svd(J);
lamda = diag(S);

if (method == "sigmamin")
    sigma_min = min(lamda); 
    mu = sigma_min; 
elseif (method == "invcond")
    sigma_min = min(lamda);
    sigma_max = max(S(:));
    mu = sigma_min / sigma_max; 
elseif (method == "detjac" )
    m = diag(S); 
    e_mul = 1; 
    for i = 1:size(m)
        e_mul = e_mul * m(i); 
    end
    mu = e_mul; 
else 
    fprintf('No matching method found \n')
end

end