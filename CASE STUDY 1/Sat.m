
function sat = Sat(Innovation)

D = diag(Innovation);

for i = 1:1
    
    if D(i,i) > 1
        D(i,i) = 1;
        
    else
         if D(i,i) < -1
            D(i,i) = -1;
         else
            D(i,i) = D(i,i);
         end
    end
end
    sat = D; 
end
