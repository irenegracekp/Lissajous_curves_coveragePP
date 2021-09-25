t = 0:2*pi/100:2*pi;

lissajous = @(m,n,d) sin(m*t) + 1i*cos(n*t+pi*d);

f = lissajous(5,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl = real(f);
ylk = imag(f);

length = 2;
r = 20*length/(2*pi);

theta1 = xl*2*pi/length;

k = 1;
for i=1:101
    
  if(abs(theta1(i)- 3.1416) > 0)
      theta(k) = theta1(i);
      yl(k) = ylk(i);
      k = k + 1;
  end 
end

x = 0; y = 0; z = 0;

x = r*sin(theta);
y = r*cos(theta);
z = yl;

path(1,:) = [0 6.3662 2.5];
path(2:102,:) = [x' y' z'+1.5];


a_fit = 4.5;
b_fit = 4.5;

samples = 10;
samp_theta = linspace(0,2*pi,samples);
samp_r = sqrt(1./((cos(samp_theta')).^2./a_fit.^2+(sin(samp_theta')).^2./b_fit.^2));
[samp_X,samp_Y] = pol2cart(samp_theta',samp_r);
samp_Z_1=repmat(0,samples,1);
samp_Z_2=repmat(2.5,samples,1);

figure(1);
hold on
patch('XData',samp_X,'YData',samp_Y,'ZData',samp_Z_1,'FaceColor',[0.92,0.71,0.71]);
alpha(0.3)
patch('XData',samp_X,'YData',samp_Y,'ZData',samp_Z_2,'FaceColor',[0.92,0.71,0.71]);
alpha(0.3)
% This is new. I define the vertices using the existing variables
% The data is transposed so that they are 4xSamples
lX = [samp_X samp_X samp_X samp_X]'; 
lY = [samp_Y samp_Y samp_Y samp_Y]'; 
lZ = [samp_Z_1 samp_Z_2 samp_Z_2 samp_Z_1]'; 
% circshift the X and Y values to allow for my vertex numbering scheme
lX(3:4,:)=circshift(lX(3:4,:),-1,2);
lY(3:4,:)=circshift(lY(3:4,:),-1,2);
patch('XData',lX,'YData',lY,'ZData',lZ,'FaceColor',[0.75,0.26,0.26],'EdgeColor',[0.90,0.90,0.90]);
alpha(0.3)
hold off
xlabel('X');
ylabel('Y');
zlabel('Z'); 
view(3)
hold on 
plot3(path(:,1), path(:,2), path(:,3),"--*",'Color',[0.00,0.45,0.74],'LineWidth', 1.5)
hold off