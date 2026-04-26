use std::sync::Arc;

use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use sim_bridge_core::{
    BridgeConfig, DaemonState, SimBridge,
    types::messages::{
        ClockMsg, ContactForcesMsg, EePoseMsg, GripperStateMsg, ImuMsg, JointStatesMsg,
        OdometryMsg, TfTreeMsg, WrenchMsg,
    },
};

const TOPIC_JOINT_STATES: &str = "joint_states";
const TOPIC_IMU: &str = "imu";
const TOPIC_EE_POSE: &str = "ee_pose";
const TOPIC_TF_TREE: &str = "tf_tree";
const TOPIC_CLOCK: &str = "clock";
const TOPIC_ODOMETRY: &str = "odometry";
const TOPIC_WRENCH: &str = "wrench";
const TOPIC_CONTACT_FORCES: &str = "contact_forces";
const TOPIC_GRIPPER_STATE: &str = "gripper_state";

pub fn build(
    runner: Arc<NodeRunner>,
    daemon: DaemonState,
    token: CancellationToken,
    sim_node: Arc<str>,
    config: &BridgeConfig,
) -> SimBridge<NodeRunner> {
    let mut enabled = EnabledTopics::from_config(config);
    tracing::info!(
        sim_node = %sim_node,
        joint_states = enabled.joint_states,
        imu = enabled.imu,
        ee_pose = enabled.ee_pose,
        tf_tree = enabled.tf_tree,
        clock = enabled.clock,
        odometry = enabled.odometry,
        wrench = enabled.wrench,
        contact_forces = enabled.contact_forces,
        gripper_state = enabled.gripper_state,
        "building sim bridge"
    );

    let mut bridge = SimBridge::new(runner, daemon, token, sim_node);

    if enabled.joint_states {
        bridge = bridge.sim_to_os::<JointStatesMsg, _>(
            Arc::from(TOPIC_JOINT_STATES),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::joint_states;
                    joint_states::emit(
                        &runner,
                        joint_states::Data {
                            robot: msg.robot,
                            step: msg.step,
                            positions: msg.positions,
                            velocities: msg.velocities,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.imu {
        bridge = bridge.sim_to_os::<ImuMsg, _>(
            Arc::from(TOPIC_IMU),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::imu;
                    imu::emit(
                        &runner,
                        imu::Data {
                            robot: msg.robot,
                            step: msg.step,
                            orientation: msg.orientation,
                            angular_velocity: msg.angular_velocity,
                            linear_acceleration: msg.linear_acceleration,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.ee_pose {
        bridge = bridge.sim_to_os::<EePoseMsg, _>(
            Arc::from(TOPIC_EE_POSE),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::ee_pose;
                    ee_pose::emit(
                        &runner,
                        ee_pose::Data {
                            robot: msg.robot,
                            step: msg.step,
                            position: msg.position,
                            orientation: msg.orientation,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.tf_tree {
        bridge = bridge.sim_to_os::<TfTreeMsg, _>(
            Arc::from(TOPIC_TF_TREE),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::tf_tree;
                    tf_tree::emit(
                        &runner,
                        tf_tree::Data {
                            robot: msg.robot,
                            step: msg.step,
                            frames: msg.frames,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.clock {
        bridge = bridge.sim_to_os::<ClockMsg, _>(
            Arc::from(TOPIC_CLOCK),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::clock;
                    clock::emit(
                        &runner,
                        clock::Data {
                            step: msg.step,
                            sim_time: msg.sim_time,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.odometry {
        bridge = bridge.sim_to_os::<OdometryMsg, _>(
            Arc::from(TOPIC_ODOMETRY),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::odometry;
                    odometry::emit(
                        &runner,
                        odometry::Data {
                            robot: msg.robot,
                            step: msg.step,
                            position: msg.position,
                            orientation: msg.orientation,
                            linear_velocity: msg.linear_velocity,
                            angular_velocity: msg.angular_velocity,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.wrench {
        bridge = bridge.sim_to_os::<WrenchMsg, _>(
            Arc::from(TOPIC_WRENCH),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::wrench;
                    wrench::emit(
                        &runner,
                        wrench::Data {
                            robot: msg.robot,
                            step: msg.step,
                            force: msg.force,
                            torque: msg.torque,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.contact_forces {
        bridge = bridge.sim_to_os::<ContactForcesMsg, _>(
            Arc::from(TOPIC_CONTACT_FORCES),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::contact_forces;
                    contact_forces::emit(
                        &runner,
                        contact_forces::Data {
                            robot: msg.robot,
                            step: msg.step,
                            contacts: msg.contacts,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    if enabled.gripper_state {
        bridge = bridge.sim_to_os::<GripperStateMsg, _>(
            Arc::from(TOPIC_GRIPPER_STATE),
            |runner, msg| {
                Box::pin(async move {
                    use peppygen::exposed_topics::gripper_state;
                    gripper_state::emit(
                        &runner,
                        gripper_state::Data {
                            robot: msg.robot,
                            step: msg.step,
                            joint_names: msg.joint_names,
                            positions: msg.positions,
                            applied_forces: msg.applied_forces,
                            stamp: msg.stamp,
                        },
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            },
        );
    }

    bridge
}

// ── EnabledTopics ─────────────────────────────────────────────────────────────

struct EnabledTopics {
    joint_states: bool,
    imu: bool,
    ee_pose: bool,
    tf_tree: bool,
    clock: bool,
    odometry: bool,
    wrench: bool,
    contact_forces: bool,
    gripper_state: bool,
}

impl EnabledTopics {
    fn from_config(config: &BridgeConfig) -> Self {
        let mut s = Self {
            joint_states: false,
            imu: false,
            ee_pose: false,
            tf_tree: false,
            clock: false,
            odometry: false,
            wrench: false,
            contact_forces: false,
            gripper_state: false,
        };
        for pub_cfg in &config.publishers {
            match pub_cfg.type_name.as_str() {
                TOPIC_JOINT_STATES => s.joint_states = true,
                TOPIC_IMU => s.imu = true,
                TOPIC_EE_POSE => s.ee_pose = true,
                TOPIC_TF_TREE => s.tf_tree = true,
                TOPIC_CLOCK => s.clock = true,
                TOPIC_ODOMETRY => s.odometry = true,
                TOPIC_WRENCH => s.wrench = true,
                TOPIC_CONTACT_FORCES => s.contact_forces = true,
                TOPIC_GRIPPER_STATE => s.gripper_state = true,
                unknown => tracing::warn!("unknown publisher type '{unknown}' in config — skipped"),
            }
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_bridge_core::config::PublisherConfig;

    fn make_config(types: &[&str]) -> BridgeConfig {
        BridgeConfig {
            sim_node: "sim".into(),
            robot: None,
            publishers: types
                .iter()
                .map(|t| PublisherConfig {
                    type_name: t.to_string(),
                    topic: t.to_string(),
                    prim: None,
                    robot_name: None,
                    params: None,
                })
                .collect(),
            subscribers: vec![],
        }
    }

    #[test]
    fn enabled_topics_from_config() {
        let config = make_config(&["joint_states", "imu", "clock"]);
        let enabled = EnabledTopics::from_config(&config);
        assert!(enabled.joint_states);
        assert!(enabled.imu);
        assert!(enabled.clock);
        assert!(!enabled.ee_pose);
        assert!(!enabled.wrench);
    }

    #[test]
    fn enabled_topics_unknown_type_is_skipped() {
        let config = make_config(&["joint_states", "totally_unknown"]);
        let enabled = EnabledTopics::from_config(&config);
        assert!(enabled.joint_states);
        assert!(!enabled.imu);
    }
}
