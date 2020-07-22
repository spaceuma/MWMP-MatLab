% Plot logarithmic cost
topLimit = 180;
botLimit = -180;
res = 0.01;
x = botLimit+1:res:topLimit-1;

t = 10;


Jux = getSimpleLogBarrierCost(x,topLimit,t,0);
Jdx = getSimpleLogBarrierCost(x,botLimit,t,1);

figure
plot(x,Jux+Jdx)
hold on
plot([topLimit topLimit],[min(Jux+Jdx) max(Jux+Jdx)+0.1*abs(max(Jux+Jdx))], '--', 'Color', 'r')
plot([botLimit botLimit],[min(Jux+Jdx) max(Jux+Jdx)+0.1*abs(max(Jux+Jdx))], '--', 'Color', 'r')
title('Logarithmic barrier cost function to ensure no violation of joint limits', 'interpreter', ...
    'latex','fontsize',18)
legend('Cost function','Joint limits')
xlabel('$\theta(^{\circ})$', 'interpreter', 'latex','fontsize',18)
ylabel('$Cost$', 'interpreter', 'latex','fontsize',18)
