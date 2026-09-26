//! 进程级关闭令牌（Runtime 不依赖 tokio CancellationToken）。

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// 全进程共享的关闭标志。
#[derive(Clone, Debug)]
pub struct ShutdownFlag {
    inner: Arc<AtomicBool>,
}

impl ShutdownFlag {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn request(&self) {
        self.inner.store(true, Ordering::SeqCst);
    }

    pub fn is_requested(&self) -> bool {
        self.inner.load(Ordering::SeqCst)
    }
}

impl Default for ShutdownFlag {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;
