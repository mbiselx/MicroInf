
%in : list {dom_p} of form  p= {x, y, phi, l}
%phi = p(3,:);
%l = p(4,:);

O = [0; 0; 0; 0];
p = zeros(4, size(xd,2));
conversion_factor = 1000/13;%  1000 steps / cm


% recreate data as it is available in robot
for n = 1:size(xd,2)
  if (n < size(xd,2))
    p(3, n) = atan2((xd(n+1)-xd(n)), (yd(n+1)-yd(n)));
  endif
  if (n > 1)
    p(4, n) = p(4, n-1) + sqrt((xd(n)-xd(n-1))^2 + (yd(n)-yd(n-1))^2);
    while(p(3,n-1)-p(3,n) >= pi)
      p(3,n) += 2*pi;
    endwhile
    while(p(3,n)-p(3,n-1) >= pi)
      p(3,n) -= 2*pi;
    endwhile
  endif
endfor
p(1:2, :) = [xd; yd];


%Parameters
lmin = 10;      % minimum loop size
h=50;           %sampling length (steps)
progress = 0;   % loading bar (aesthetic)

%create phi(l), sampled at h
lcnt = 2;
lmax = floor(p(4, end)/h);
phi = zeros(1, 2*lmax+1);
for n = 1:lmax + lmax+1
  while ( (size(p,2) >= lcnt) && (p(4, lcnt) < (n - lmax)*h))
    lcnt = lcnt+1;
  endwhile
  phi(n) = p(3, lcnt-1);
endfor
disp("phi done")

figure(2)
subplot(2,1,1)
plot([-lmax:lmax]*h/conversion_factor, phi);
title("\phi (l)")
axis([0,lmax]*h/conversion_factor);
xlabel("l [cm]");
ylabel("\phi [rad]");
pause(.1);


%autocorrelation
fprintf("[          ]%c%c%c%c%c%c%c%c%c%c%c", 8,8,8,8,8,8,8,8,8,8,8)  % loading bar (aesthetic)
c = zeros(1, lmax);
for k = lmin : lmax    %correlation loop
  for n = 1:300                       %integral loop (limited nh)
    c(k) = c(k) + ((phi(end-n) - phi(end)) - (phi(end -k - n) - phi(end - k)))^2;
  endfor
  progress++;
  if (progress > 0.1*(lmax-lmin))
    fprintf("-")
    figure(2)
    subplot(2,1,2)
    plot(c)
    title("c")
    axis([lmin,lmax]*h/conversion_factor);
    pause(.1);
    progress = 0;
  endif
endfor

[A, i] = min(c(lmin+1:lmax));

fprintf("-]\nc done\n")
figure(2)
subplot(2,1,1)
plot([-lmax:lmax]*h/conversion_factor, phi ,';\phi(l);', [lmax-(lmin+i), lmax-1]*h/conversion_factor, [phi(2*lmax-(lmin+i)), phi(2*lmax)], 'rd;loop start and end;', 'Linewidth', 2);
title("\\phi")
axis([0,lmax-1]*h/conversion_factor);
xlabel("l [cm]");
ylabel("\phi [rad]");
subplot(2,1,2)
plot([0:lmax-1]*h/conversion_factor, c,';autocorrelation error(\lambda);', [i+lmin]*h/conversion_factor, [A], 'rd;\lambda_{loop};', 'Linewidth', 2)
title("autocorrelation error")
axis([0,lmax-1]*h/conversion_factor);
xlabel("l [cm]");
ylabel("corr error [arb. units]");
pause(.1);


k=0;
for n = 1:size(p,2)
  if (p(4,n) > (i+lmin)*h)
    k = n;
    break;
  endif
endfor

figure(3)
plot(p(1,1:k+20), p(2,1:k+20), 'rd-', p(1,1), p(2,1),'bx', p(1, k), p(2,k), 'bx')
axis("equal")
