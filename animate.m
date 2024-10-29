r = FrankaER;
axis([-2 2 -2 2 0 2])
traj_steps = 100;

function animateFranka()
    desired_pos = transl(0.1, 0.2, 0.9);
    q = r.model.ikcon(desired_pos);

    qMatrix = jtraj(r.model.getpos, q, traj_steps);

    for j = 1:traj_steps
        % Animate the UR3e robot at each step
        r.model.animate(qMatrix(j, :));
        drawnow;
    end
end


