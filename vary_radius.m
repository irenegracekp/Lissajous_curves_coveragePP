t = 0:2*pi/550:2*pi;

lissajous = @(m,n,d) sin(m*t) + 1i*cos(n*t+pi*d);

f = lissajous(23,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl = real(f);
yl = imag(f);

length = 2;
r_main = 20*length/(2*pi);

theta = xl*2*pi/length;

x = 0; y = 0; z = 0;

z_real = yl +2 ;


for i=1:size(z_real,2)
    r = r_main/(exp(0.1*z_real(i)));
    x(i) = r*sin(theta(i));
    y(i) = r*cos(theta(i));
    z(i) = 10*yl(i);  
end

t1 = 0:2*pi/300:2*pi;

lissajous = @(m,n,d) sin(m*t1) + 1i*cos(n*t1+pi*d);

f1 = lissajous(23,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl1 = real(f1);
yl1 = imag(f1);

length1 = 2;
r_main1 = 20*length1/(2*pi);

theta1 = xl1*2*pi/length1;

x1 = 0; y1 = 0; z1 = 0;

z_real1 = yl1 + 4 ;

k1 = 2/10;

for i=1:size(z_real1,2)
  
    r1 = r_main1/(exp(0.1*z_real1(i)));
    x1(i) = r1*sin(theta1(i));
    y1(i) = r1*cos(theta1(i));
    z1(i) = 10*yl1(i);
    
end

t3 = 0:2*pi/1000:2*pi;

lissajous3 = @(m,n,d) sin(m*t3) + 2i*cos(n*t3+pi*d);

f3 = lissajous3(23,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl3 = real(f3);
yl3 = imag(f3);

length3 = 2;
r_main3 = 20*length3/(2*pi);

theta3 = xl3*2*pi/length3;

z_real3 = yl3 +2 ;

k = 2/10;

for i=1:size(z_real3,2)
  
    r3 = r_main3/(3*exp(0.1*z_real3(i)));
    x3(i) = r3*sin(theta3(i));
    y3(i) = r3*cos(theta3(i));
    z3(i) = 10*yl3(i);
    
end



path3 = [x3' y3' z3'+20];
path1 = [x' y' z'+10];
path2 = [x1' y1' z1'+30];

thetad = wrapToPi([0 theta]' + pi);

figure
subplot(1,2,1)
hold on
plot3(path1(:,1), path1(:,2), path1(:,3),"-*",'MarkerSize',4)
plot3(path2(:,1), path2(:,2), path2(:,3),"-*",'color',[0.93,0.39,0.39],'MarkerSize',4)
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off

subplot(1,2,2)
hold on
plot3(path3(:,1), path3(:,2), path3(:,3),"-*",'MarkerSize',4)
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off