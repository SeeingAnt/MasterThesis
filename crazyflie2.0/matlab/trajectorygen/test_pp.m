[pp, vec] = takeoff_traj();
mass = 0.03;

fig8_traj(0.5, 0.5)

[pos, vel, rtmx, ~, ~, rpy, omega, t] = trajectory_eval_piecewise(pp, mass, 1000);
yaw = rpy(3,:);

coefs = flipud(reshape(vec, 8, 4));
[pos_c, vel_c, omega_c, yaw_c] = pptraj_mex(coefs, mass, t);

assert_close = @(a,b) assert(~any(abs(a(:) - b(:)) > 0.0001));

assert_close(pos, pos_c);
assert_close(vel, vel_c);
assert_close(yaw, yaw_c);
assert_close(omega, omega_c);


render_traj(pos_c, rtmx, 1, 0.046, 1)