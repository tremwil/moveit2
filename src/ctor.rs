//! Flexible proc-macro in-place construction.

use core::{
    cell::Cell,
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    pin::Pin,
    ptr::NonNull,
};

use crate::{New, TryNew};

#[doc(inline)]
pub use moveit2_proc_macros::ctor;

#[doc(hidden)]
pub mod __private {
    use super::MaybeInit;
    use core::pin::Pin;

    pub use moveit2_proc_macros::Ctor;

    trait Sealed {}

    /// # Safety
    ///
    /// If `INIT` is true, then `DerefTarget<T> = T`. Otherwise,
    /// `DerefTarget<T> = MaybeUninit<T>`.
    #[allow(private_bounds)]
    pub unsafe trait IsInit: Sealed + 'static {
        const INIT: bool;

        type DerefTarget<T>;
    }

    /// Implemented by marker types indicating that a field of an in-place struct constructor
    /// is not initialized.
    pub trait UninitMarker: 'static {}

    impl<T: ?Sized + UninitMarker> Sealed for T {}
    unsafe impl<T: ?Sized + UninitMarker> IsInit for T {
        const INIT: bool = false;

        type DerefTarget<U> = super::MaybeUninit<U>;
    }

    /// Marker type indicating that a field of an in-place struct constructor is initialized.
    pub struct Init;

    impl Sealed for Init {}
    unsafe impl IsInit for Init {
        const INIT: bool = true;

        type DerefTarget<T> = T;
    }

    pub type PinMaybeInit<'a, T, I> = Pin<MaybeInit<'a, T, I>>;
}

#[doc(inline)]
pub use __private::{Init, UninitMarker};

use __private::IsInit;

/// A field of an struct that is being constructed in place that may or may not be initialized.
///
/// `I` acts as a proof of (un)initialization.
pub struct MaybeInit<'a, T, I: IsInit> {
    ptr: NonNull<T>,
    init_counter: &'a Cell<usize>,
    #[allow(clippy::type_complexity)]
    phantom: PhantomData<(&'a mut (), fn() -> I, T)>,
}

impl<T, I: IsInit> Drop for MaybeInit<'_, T, I> {
    fn drop(&mut self) {
        if I::INIT {
            self.init_counter.update(|c| c - 1);
            unsafe { core::ptr::drop_in_place(self.ptr.as_ptr()) }
        }
    }
}

impl<T, I: IsInit> Deref for MaybeInit<'_, T, I> {
    type Target = I::DerefTarget<T>;

    fn deref(&self) -> &Self::Target {
        unsafe { self.ptr.cast::<I::DerefTarget<T>>().as_ref() }
    }
}

impl<T> DerefMut for MaybeInit<'_, T, Init> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.ptr.as_mut() }
    }
}

impl<'a, T, I: UninitMarker> MaybeInit<'a, T, I> {
    /// # Safety
    ///
    /// `ptr` memory must be fresh uninitialized memory.
    #[doc(hidden)]
    pub const unsafe fn new(ptr: *mut T, ctr: &'a Cell<usize>) -> Pin<Self> {
        unsafe {
            Pin::new_unchecked(Self {
                ptr: NonNull::new_unchecked(ptr),
                init_counter: ctr,
                phantom: PhantomData,
            })
        }
    }

    /// Attempt to construct a value in-place inside the slot pointed at by this [`MaybeInit`].
    pub fn try_emplace<C>(this: Pin<Self>, ctor: C) -> Result<Pin<MaybeInit<'a, T, Init>>, C::Error>
    where
        C: TryNew<Output = T>,
    {
        unsafe {
            let this = Pin::into_inner_unchecked(this);
            let slot = Pin::new_unchecked(this.ptr.cast::<MaybeUninit<T>>().as_mut());
            ctor.try_new(slot)?;

            this.init_counter.update(|c| c.strict_add(1));
            Ok(Pin::new_unchecked(MaybeInit {
                ptr: this.ptr,
                init_counter: this.init_counter,
                phantom: PhantomData,
            }))
        }
    }

    /// Construct a value in-place inside the slot pointed at by this [`MaybeInit`].
    pub fn emplace<C>(this: Pin<Self>, ctor: C) -> Pin<MaybeInit<'a, T, Init>>
    where
        C: New<Output = T>,
    {
        MaybeInit::try_emplace(this, ctor).unwrap_or_else(|e| match e {})
    }
}
