function result = calculateMetrics(MPC, APC)
    % Initialize result struct
    result = struct('winner', 'inconclusive','APCbonus',0,'MPCbonus',0);
    
    % Bonus metric: Computation time
    if max(APC.exec_time) < max(MPC.exec_time)
        result.APCbonus=result.APCbonus+1;
    else
        result.MPCbonus=result.MPCbonus+1;
    end

    % Bonus metric: control effort
    if max(APC.control_effort) < max(MPC.control_effort)
        result.APCbonus=result.APCbonus+1;
    else
        result.MPCbonus=result.MPCbonus+1;
    end
    
    % First metric: Failure to solve
    if APC.failure && ~MPC.failure
        result.winner = 'MPC';
        return;
    elseif ~APC.failure && MPC.failure
        result.winner = 'APC';
        return;
    elseif APC.failure && MPC.failure
        return;  % Inconclusive if both failed
    end
    
    % Second metric: Obstacle avoidance
    APC_minDist = min(APC.obs_mapping(:));
    MPC_minDist = min(MPC.obs_mapping(:));
    
    if APC_minDist >= APC.violation_line && MPC_minDist < APC.violation_line
        result.winner = 'APC';
        return;
    elseif MPC_minDist > APC.violation_line && APC_minDist < APC.violation_line
        result.winner = 'MPC';
        return;
    elseif APC_minDist < APC.violation_line && MPC_minDist < APC.violation_line
        % Both violated, check who violated lesser
        if APC_minDist > MPC_minDist
            result.APCbonus=result.APCbonus+1;
        else
            result.MPCbonus=result.MPCbonus+1;
        end
    end
    
    % Third metric: Convergence to goal
    if min(APC.error_ego) <= 0.01 && min(MPC.error_ego) > 0.01
        result.winner = 'APC';
        return;
    elseif min(MPC.error_ego) <= 0.01 && min(APC.error_ego) > 0.01
        result.winner = 'MPC';
        return;
    elseif min(APC.error_ego) <= 0.01 && min(MPC.error_ego) <= 0.01
        % Both converged, check who converged faster
        if max(APC.t) < max(MPC.t)
            result.winner = 'APC';
        else
            result.winner = 'MPC';
        end
        return;
    end
end


