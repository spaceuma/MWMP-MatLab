figure(2)
plot(pathi(1,:),pathi(2,:))
hold on
plot(x0(10,1:end),x0(11,1:end))
plot(x(10,i:interIndex2),x(11,i:interIndex2))
scatter(pathi(1,interIndex1),pathi(2,interIndex1))
scatter(x0(10,interIndex2),x0(11,interIndex2))
scatter(x0(10,switchIndex),x0(11,switchIndex))
hold off