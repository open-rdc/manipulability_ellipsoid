clear;
clf;

function [t1, t2] = inverse_kinematics(x, y, l1, l2)
  l = sqrt(x^2+y^2);
  t1 = t2 = 0;
  if (l <= (l1+l2))
    t1 = atan2(y,x) + acos((l1^2+l^2-l2^2)/(2*l2*l));
    t2 = pi + acos((l1^2+l2^2-l^2)/(2*l1*l2));
  endif
endfunction

l1 = 0.11;
l2 = 0.11;
ratio = 0.1;
motor_velocity = 4.81; # 46rpm

P = [0, 0; 0, 0.06; 0.1+0.07, 0.06; 0.1, 0.0];

i = 0;
for t = 0.0: 0.2: 1.0
  [xt, yt] = bezier(P, t);
  x = xt - 0.05;
  y = yt - 0.16;
  [t1, t2] = inverse_kinematics(x, y, l1, l2);
  if (t1 != 0 || t2 != 0)
    i = i + 1;
    x1 = l1 * cos(t1);
    y1 = l1 * sin(t1);
    arm_x(i,:) = [0 x1 x];
    arm_y(i,:) = [0 y1 y];
    jaco = [-l1*sin(t1)-l2*sin(t1+t2) -l2*sin(t1+t2); l1*cos(t1)+l2*cos(t1+t2) l2*cos(t1+t2)];
    [U,S,V] = svd(jaco);
    mani(i,:) = [x y atan2(U(2),U(1)) S(1,1) S(2,2)];
  endif
endfor

hold on;
axis([-0.2,0.2,-0.2,0.2],"square");
for n = 1:i
  plot(arm_x(n,:), arm_y(n,:));
  ellipse(mani(n,1), mani(n,2), mani(n,4)*ratio, mani(n,5)*ratio, mani(n,3));
  if (mani(n,3) < 0)
    mani(n,3) += 2 * pi;
  endif
  printf("(velocity, angle): (%f, %f, %f)\n", mani(n,4) * motor_velocity, mani(n,5) * motor_velocity, mani(n,3)-pi);
end
hold off;

