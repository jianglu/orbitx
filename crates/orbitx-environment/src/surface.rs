//! Optional SurfaceDataset binding for primary body (G4).
//! Sampling stays in `orbitx-surface`; this module only holds the handle.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use orbitx_surface::SurfacePackage;

/// Opened surface package attached to a planetary system.
#[derive(Clone)]
pub struct SurfaceBinding {
    pub dataset_id: String,
    pub root: PathBuf,
    pub package: Arc<SurfacePackage>,
}

impl std::fmt::Debug for SurfaceBinding {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SurfaceBinding")
            .field("dataset_id", &self.dataset_id)
            .field("root", &self.root)
            .finish_non_exhaustive()
    }
}

impl SurfaceBinding {
    pub fn open(root: impl AsRef<Path>) -> Result<Self, String> {
        let root = root.as_ref().to_path_buf();
        let package = Arc::new(SurfacePackage::open(&root)?);
        Ok(Self {
            dataset_id: package.dataset_id.clone(),
            root,
            package,
        })
    }
}

/// Max |R_elev − primary.size| accepted by `attach_surface` (metres).
pub const RADIUS_MATCH_EPS_M: f64 = 0.5;
