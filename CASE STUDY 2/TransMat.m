function Trans = TransMat(e1,e2,el)

% Triad basis definition 
b1 = (e2-e1)/norm(e2-e1);
%~~~~~~~~~~~~~~~~~~~~~~~
d = (el-e1)/norm(el-e1);
h = d - (b1'*d)*b1;
%~~~~~~~~~~~~~~~~~~~~~~~
b2= h/norm(h);
b3 = cross(b1,b2);

% Transformation matrix from I to B
T_B_to_I = [b1 b2 b3 e1; 0  0  0  1];

T_I_to_B = T_B_to_I^(-1);

%============= output
Trans = T_I_to_B;
end