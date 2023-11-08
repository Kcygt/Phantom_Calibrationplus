

for c = 1:100000
    
    X(1,c)= positionnumbers0{c,1};
    X(2,c)= positionnumbers1{c,1};
    
    Y(1,c)= positionnumbers0{c,2};
    Y(2,c)= positionnumbers1{c,2};
    
    Z(1,c)= positionnumbers0{c,3};
    Z(2,c)= positionnumbers1{c,3};
    
end

plot3(X(1,:),Y(1,:),Z(1,:))
figure
plot3(X(2,:),Y(2,:),Z(2,:))