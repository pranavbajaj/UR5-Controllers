function [e] = getXi(g)

    e = Skew2Vec(logm(g))';

end

