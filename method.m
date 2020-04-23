epsil_phi = -0.1  % angular error
epsil_l   = .2    % dist error
d_phi     = 2*pi/5 + epsil_phi;
d_l       = 1 + epsil_l;

O  = [0; 0; 0];
p0 = [1; 0; 0];
p1 = p0 + d_l/d_phi*[sin(p0(3) + d_phi) - sin(p0(3)); cos(p0(3)) - cos(p0(3) + d_phi); 0] + [0; 0; d_phi];
d_l -= epsil_l;
p2 = p1 + d_l/d_phi*[sin(p1(3) + d_phi) - sin(p1(3)); cos(p1(3)) - cos(p1(3) + d_phi); 0] + [0; 0; d_phi];
d_l += epsil_l;
p3 = p2 + d_l/d_phi*[sin(p2(3) + d_phi) - sin(p2(3)); cos(p2(3)) - cos(p2(3) + d_phi); 0] + [0; 0; d_phi];
d_l -= epsil_l;
p4 = p3 + d_l/d_phi*[sin(p3(3) + d_phi) - sin(p3(3)); cos(p3(3)) - cos(p3(3) + d_phi); 0] + [0; 0; d_phi];
d_l += epsil_l;
p5 = p4 + d_l/d_phi*[sin(p4(3) + d_phi) - sin(p4(3)); cos(p4(3)) - cos(p4(3) + d_phi); 0] + [0; 0; d_phi];
d_l -= epsil_l;
p6 = p5 + d_l/d_phi*[sin(p5(3) + d_phi) - sin(p5(3)); cos(p5(3)) - cos(p5(3) + d_phi); 0] + [0; 0; d_phi];

p = [p0,p1,p2,p3,p4,p5,p6];
% we "know" [p0,p5] and [p1,p6] are loop closing pairs

plot(p(1, :), p(2, :), 'rd-');
axis("equal");
