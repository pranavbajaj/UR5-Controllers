function [Vec] = Skew2Vec(M_skew)
    Vec(6) = -M_skew(1,2);
    Vec(5) = M_skew(1,3);
    Vec(4) = -M_skew(2,3);
    Vec(1:3) = M_skew(1:3, 4);
end

