use std::sync::Arc;

use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};

fn main() -> Result<()> {
    NodeBuilder::new().run(|_args: Parameters, runner: Arc<NodeRunner>| async move {
        let token = runner.cancellation_token();
        token.cancelled().await;
        Ok(())
    })
}
