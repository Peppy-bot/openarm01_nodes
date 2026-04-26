use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use sim_bridge_core::{types::primitives::ArmId, DaemonState};

use crate::types::GripperId;

const RETRY_DELAY: Duration = Duration::from_secs(1);

/// Read PeppyOS daemon connection info from the standard state file.
///
/// Tries `$HOME/.peppy/daemon_state.json` first, then a local-path fallback
/// for development environments where the daemon runs in-tree.
pub fn read_daemon_state() -> Result<DaemonState, String> {
    let path = std::env::var("HOME")
        .ok()
        .map(|h| std::path::PathBuf::from(h).join(".peppy/daemon_state.json"))
        .filter(|p| p.exists())
        .unwrap_or_else(|| std::path::PathBuf::from(".peppy/daemon_state.json"));

    let raw = std::fs::read_to_string(&path)
        .map_err(|e| format!("read {}: {e}", path.display()))?;

    let v: serde_json::Value =
        serde_json::from_str(&raw).map_err(|e| format!("parse daemon_state.json: {e}"))?;

    Ok(DaemonState {
        core_node_name: v["core_node_name"]
            .as_str()
            .ok_or("daemon_state.json missing 'core_node_name'")?
            .to_string(),
        messaging_port: v["messaging_port"]
            .as_u64()
            .ok_or("daemon_state.json missing 'messaging_port'")? as u16,
    })
}

/// Call `get_arm_id` and `get_gripper_id` on the paired component nodes.
///
/// Both calls retry indefinitely until they succeed or the token fires.
pub async fn discover_ids(
    runner: Arc<NodeRunner>,
    token: CancellationToken,
) -> Result<(ArmId, GripperId), String> {
    let arm_id = tokio::select! {
        _ = token.cancelled() => return Err("cancelled before arm_id discovered".into()),
        id = discover_arm_id(runner.clone()) => id,
    };

    let gripper_id = tokio::select! {
        _ = token.cancelled() => return Err("cancelled before gripper_id discovered".into()),
        id = discover_gripper_id(runner.clone()) => id,
    };

    Ok((arm_id, gripper_id))
}

async fn discover_arm_id(runner: Arc<NodeRunner>) -> ArmId {
    use peppygen::consumed_services::get_arm_id;

    loop {
        match get_arm_id::call(&runner).await {
            Ok(response) => {
                tracing::info!(arm_id = response.data.arm_id, "arm_id resolved");
                return ArmId::from(response.data.arm_id);
            }
            Err(e) => {
                tracing::warn!("get_arm_id failed: {e} — retrying in {RETRY_DELAY:?}");
                tokio::time::sleep(RETRY_DELAY).await;
            }
        }
    }
}

async fn discover_gripper_id(runner: Arc<NodeRunner>) -> GripperId {
    use peppygen::consumed_services::get_gripper_id;

    loop {
        match get_gripper_id::call(&runner).await {
            Ok(response) => {
                tracing::info!(gripper_id = response.data.gripper_id, "gripper_id resolved");
                return GripperId::from(response.data.gripper_id);
            }
            Err(e) => {
                tracing::warn!("get_gripper_id failed: {e} — retrying in {RETRY_DELAY:?}");
                tokio::time::sleep(RETRY_DELAY).await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_daemon_state_missing_field_returns_err() {
        let v: serde_json::Value =
            serde_json::from_str(r#"{"messaging_port": 7448}"#).unwrap();
        let result = v["core_node_name"]
            .as_str()
            .ok_or("daemon_state.json missing 'core_node_name'");
        assert!(result.is_err());
    }

    #[test]
    fn read_daemon_state_missing_port_returns_err() {
        let v: serde_json::Value =
            serde_json::from_str(r#"{"core_node_name": "peppy"}"#).unwrap();
        let result = v["messaging_port"]
            .as_u64()
            .ok_or("daemon_state.json missing 'messaging_port'");
        assert!(result.is_err());
    }
}
