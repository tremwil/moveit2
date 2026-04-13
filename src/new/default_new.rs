use crate::new;
use crate::new::New;

/// A default constructible type.
///
/// These types can be in-place initialized without any additional arguments.
pub trait DefaultNew: Sized {
    /// Default-construct this type, returning a [`New`] that in-place
    /// initializes the value.
    fn default_new() -> impl New<Self>;
}

impl<T: Default> DefaultNew for T {
    fn default_new() -> impl New<Self> {
        new::by(|| Self::default())
    }
}
