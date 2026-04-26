/// Gripper identifier — 0 = left, 1 = right.
/// Symmetric to ArmId; newtype prevents mixing the two.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GripperId(pub u8);

impl GripperId {
    pub fn as_u8(self) -> u8 {
        self.0
    }
}

impl From<u8> for GripperId {
    fn from(v: u8) -> Self {
        Self(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gripper_id_roundtrip() {
        let id = GripperId::from(1u8);
        assert_eq!(id.as_u8(), 1);
    }
}
