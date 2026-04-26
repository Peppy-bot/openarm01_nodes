use std::sync::Arc;

use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use sim_bridge_core::types::primitives::ArmId;

/// Drive the move_arm action server loop until the cancellation token fires.
///
/// For each incoming goal from openarm01_joint_commander:
///   1. TODO: call inverse_kinematics node → joint positions (contract not yet finalized)
///   2. TODO: call collisions_detection node → safety check (contract not yet finalized)
///   3. Forward move_arm goal to openarm01_arm
///   4. Stream arm feedback back to joint_commander
///   5. Return arm result to joint_commander
pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken, arm_id: ArmId) {
    use peppygen::exposed_actions::move_arm;

    tracing::info!(arm_id = arm_id.as_u8(), "move_arm action server started");

    loop {
        tokio::select! {
            _ = token.cancelled() => {
                tracing::info!("move_arm action server shutting down");
                break;
            }
            result = move_arm::handle_next_goal(&runner) => {
                match result {
                    Ok(goal) => handle_goal(runner.clone(), token.clone(), goal, arm_id).await,
                    Err(e) => tracing::warn!("move_arm goal receive error: {e}"),
                }
            }
        }
    }
}

async fn handle_goal(
    runner: Arc<NodeRunner>,
    token: CancellationToken,
    goal: peppygen::exposed_actions::move_arm::Goal,
    arm_id: ArmId,
) {
    use peppygen::consumed_actions::move_arm as arm_action;
    use peppygen::exposed_actions::move_arm;

    tracing::info!(
        arm_id = arm_id.as_u8(),
        feedback_frequency = goal.data.feedback_frequency,
        desired_position = ?goal.data.desired_position,
        "move_arm goal received"
    );

    // Accept the goal immediately — validation happens during execution.
    if let Err(e) = move_arm::accept_goal(&runner, move_arm::Response { accepted: true }).await {
        tracing::warn!("failed to accept move_arm goal: {e}");
        return;
    }

    // TODO: call inverse_kinematics node once its contract is finalized.
    // let joint_positions = match ik::solve(&runner, goal.data.desired_position, goal.data.desired_orientation).await {
    //     Ok(jp) => jp,
    //     Err(e) => {
    //         tracing::warn!("IK failed: {e}");
    //         send_failure(&runner, "IK failed").await;
    //         return;
    //     }
    // };

    // TODO: call collisions_detection node once its contract is finalized.
    // if !collisions::check(&runner, &joint_positions).await { ... }

    // Forward to openarm01_arm — pass through position/orientation directly
    // until IK provides joint-space goals.
    let arm_result = tokio::select! {
        _ = token.cancelled() => {
            tracing::info!("move_arm cancelled mid-execution");
            let _ = move_arm::send_result(&runner, move_arm::ActionResult {
                success: false,
                message: "cancelled".into(),
                final_joint_positions: vec![],
                final_ee_position: [0.0; 3],
                action_time: 0.0,
            }).await;
            return;
        }
        result = arm_action::send_goal(
            &runner,
            arm_action::Goal {
                feedback_frequency: goal.data.feedback_frequency,
                desired_position: goal.data.desired_position,
                desired_orientation: goal.data.desired_orientation,
            },
            |fb: arm_action::Feedback| {
                // Clone runner for feedback forwarding — fire-and-forget.
                let runner_fb = runner.clone();
                tokio::spawn(async move {
                    let _ = move_arm::send_feedback(&runner_fb, move_arm::Feedback {
                        joint_positions: fb.data.joint_positions,
                        current_ee_position: fb.data.current_ee_position,
                        action_time: fb.data.action_time,
                    }).await;
                });
            },
        ) => result,
    };

    match arm_result {
        Ok(result) => {
            tracing::info!(
                success = result.data.success,
                action_time = result.data.action_time,
                "move_arm completed"
            );
            let _ = move_arm::send_result(&runner, move_arm::ActionResult {
                success: result.data.success,
                message: result.data.message,
                final_joint_positions: result.data.final_joint_positions,
                final_ee_position: result.data.final_ee_position,
                action_time: result.data.action_time,
            })
            .await;
        }
        Err(e) => {
            tracing::warn!("arm move_arm failed: {e}");
            let _ = move_arm::send_result(&runner, move_arm::ActionResult {
                success: false,
                message: format!("arm action failed: {e}"),
                final_joint_positions: vec![],
                final_ee_position: [0.0; 3],
                action_time: 0.0,
            })
            .await;
        }
    }
}
