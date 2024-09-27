function result = calculateMetrics(APC, MPC)
    % Initialize result struct
    result = struct('winner', 'inconclusive','APCbonus',0,'MPCbonus',0);
    
    % First metric: Failure to solve
    if APC.failed && ~MPC.failed
        result.winner = 'MPC';
        result.MPC_wins = 1;
        return;
    elseif ~APC.failed && MPC.failed
        result.winner = 'APC';
        result.APC_wins = 1;
        return;
    elseif APC.failed && MPC.failed
        return;  % Inconclusive if both failed
    end
    
    % Second metric: Obstacle avoidance
    APC_minDist = min(APC.obstacleDist(:));
    MPC_minDist = min(MPC.obstacleDist(:));
    
    if APC_minDist > 0 && MPC_minDist == 0
        result.winner = 'APC';
        result.APC_wins = 1;
        return;
    elseif MPC_minDist > 0 && APC_minDist == 0
        result.winner = 'MPC';
        result.MPC_wins = 1;
        return;
    elseif APC_minDist == 0 && MPC_minDist == 0
        % Both violated, check who violated lesser
        if APC_minDist > MPC_minDist
            result.winner = 'APC';
            result.APC_wins = 1;
        else
            result.winner = 'MPC';
            result.MPC_wins = 1;
        end
        return;
    end
    
    % Third metric: Convergence to goal
    if APC.goalDist == 0 && MPC.goalDist > 0
        result.winner = 'APC';
        result.APC_wins = 1;
        return;
    elseif MPC.goalDist == 0 && APC.goalDist > 0
        result.winner = 'MPC';
        result.MPC_wins = 1;
        return;
    elseif APC.goalDist == 0 && MPC.goalDist == 0
        % Both converged, check who converged faster
        if APC.convergenceTime < MPC.convergenceTime
            result.winner = 'APC';
            result.APC_wins = 1;
        else
            result.winner = 'MPC';
            result.MPC_wins = 1;
        end
        return;
    end
    
    % Bonus metric: Computation time or control effort
    if APC.compTime < MPC.compTime || APC.controlEffort < MPC.controlEffort
        result.bonus = 1;
        result.APC_wins = result.APC_wins + 1;
    elseif MPC.compTime < APC.compTime || MPC.controlEffort < APC.controlEffort
        result.bonus = -1;
        result.MPC_wins = result.MPC_wins + 1;
    end
end


