function qplot(q, goal, origin, path, traj)
    % clear existing plot
    clf
    
    % setup plot
    xlabel('x'); ylabel('y'); grid on; title('Simulated robot path')
    axis([-.5 2.5 -1.5 2.5]);
    axis square;
    hold on
    
    % plot bot
    plot(q(1), q(2), 'pg')
    plot(q(1), q(2), 'og')
    
    % plot goal
    plot(goal(1), goal(2), 'pk')
    
    % plot origin
    plot(origin(1), origin(2), 'ok')
    
    % plot path
    plot(path(:,1), path(:,2), 'r')
    plot(path(:,1), path(:,2), 'r*')
    
    % plot traj
    plot(traj(:,1), traj(:,2), 'b--');
    
%     legend('path', 'goal', 'start')
    hold off
end