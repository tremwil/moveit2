//! Flexible proc-macro based in-place construction.

use core::{
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    pin::Pin,
};

use crate::{New, TryNew, drop_flag::DropFlag};

#[doc(inline)]
pub use moveit2_proc_macros::Ctor;

#[doc(hidden)]
pub mod __private {
    pub use core::convert::Infallible;
    pub use core::mem::forget;
    pub use core::pin::Pin;
}

/// Trait implemented on struct types that can be in-place constructed field by field.
///
/// When applied to a struct using [`derive`], this generates a `ctor` member function
/// that generates a [`TryNew`] implementation initializing the struct in place, field
/// by field.
///
/// Due to [type system restrictions around higher-kinded lifetimes](https://users.rust-lang.org/t/for-a-a-t-seems-to-require-t-static/137175/5?u=tremwil)
/// and the poor type inference of workarounds to said restrictions, the [`Ctor`] trait
/// does not expose the constructor function on types itself.
pub trait Ctor: Sized {
    /// A projection of the uninitialized fields of `Self`.
    ///
    /// This struct exposes the same fields as `Self` through [`Uninit`] references.
    type Fields<'a>
    where
        Self: 'a;

    /// A projection of the fields of `Self` as [`PinInit`] references.
    ///
    /// This is called a "proof" as constructing this type requires initializing every
    /// field exposed through [`Ctor::Fields`], thus proving that `Self` is now initialized.
    type Proof<'a>
    where
        Self: 'a;
}

/// The proof of initialization for an in-place constructible struct.
///
/// This type alias provides a canonical path so that `<T as Ctor>::Proof`
/// can be constructed through a regular struct initialization expression.
pub type InitProof<'a, T> = <T as Ctor>::Proof<'a>;

/// A [`Slot`] referencing an uninitialized field of a struct represented by
/// some marker type `F`.
///
/// This can [`Deref`] into a [`MaybeUninit`], but only immutably. To initialize
/// the field, use the [`put`], [`emplace`] and [`try_emplace`] methods.
///
/// [`put`]: Uninit::put
/// [`emplace`]: Uninit::emplace
/// [`try_emplace`]: Uninit::try_emplace
pub struct Uninit<'a, T, F> {
    slot: &'a mut MaybeUninit<T>,
    drop_flag: DropFlag<'a>,
    phantom: PhantomData<F>,
}

impl<'a, T, F> Deref for Uninit<'a, T, F> {
    type Target = MaybeUninit<T>;

    fn deref(&self) -> &Self::Target {
        self.slot
    }
}

impl<'a, T, F> Uninit<'a, T, F> {
    /// This function is internal to the [`Ctor`] derive macro and should never be called
    /// manually. The function and its safety invariants below are not part of semver and
    /// may change at any time.
    ///
    /// # Safety
    /// - `slot` must be uninitialized memory that is exclusively accessible for lifetime `'a`
    /// - `drop_flag` must only be used with other `Uninit`s in frame 'a
    /// - after the scope of `'a` ends, `drop_flag` must be checked to ensure that no created `Init`s
    ///   have been forgotten. In this situation the program *must* abort to avoid violating [`Pin`]
    ///   obligations.
    #[doc(hidden)]
    pub unsafe fn new_unchecked(slot: *mut T, drop_flag: DropFlag<'a>) -> Self {
        Self {
            slot: unsafe { &mut *slot.cast() },
            drop_flag,
            phantom: PhantomData,
        }
    }

    /// Initialize this slot by moving a value into it.
    pub fn put(self, value: T) -> PinInit<'a, T, F> {
        self.slot.write(value);
        self.drop_flag.inc();
        // SAFETY:
        // - slot is initialized
        // - `Init` can be pinned as drop flag will prevent data from being
        // forgotten after pinning (promise of new_unchecked)
        unsafe { Pin::new_unchecked(Init(self)) }
    }

    /// Initialize this slot by constructing a value inside of it.
    pub fn emplace(self, new: impl New<Output = T>) -> PinInit<'a, T, F> {
        self.try_emplace(new).unwrap_or_else(|e| match e {})
    }

    /// Try to initialize this struct by constructing a value inside of it.
    pub fn try_emplace<N>(self, new: N) -> Result<PinInit<'a, T, F>, N::Error>
    where
        N: TryNew<Output = T>,
    {
        // SAFETY:
        // - pinning &'a mut MaybeUninit is fine, as it has a trivial drop
        // - memory is uninitialized for `TryNew` by new_unchecked invariants
        // - slot is initialized after `TryNew` succeeds
        // - resulting `Init` can be pinned as drop flag will prevent data from being
        //   forgotten after pinning (promise of new_unchecked)
        unsafe {
            let pinned_slot = Pin::new_unchecked(&mut *self.slot);
            new.try_new(pinned_slot)?;
            self.drop_flag.inc();
            Ok(Pin::new_unchecked(Init(self)))
        }
    }
}

/// Like a [`MoveRef`], but references an initialized field of a struct represented
/// by some marker type `F`.
///
/// When [pinned], this type acts as proof that the field was initialized; with the
/// other fields of the struct it can be used to construct a [`Ctor::Proof`] to
/// finalize in-place construction of the struct.
///
/// [`MoveRef`]: crate::MoveRef
/// [pinned]: PinInit
pub struct Init<'a, T, F>(Uninit<'a, T, F>);

/// Type alias for a pinned [`Init`].
pub type PinInit<'a, T, F> = Pin<Init<'a, T, F>>;

impl<'a, T, F> Deref for Init<'a, T, F> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.0.slot.assume_init_ref() }
    }
}

impl<'a, T, F> DerefMut for Init<'a, T, F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.0.slot.assume_init_mut() }
    }
}

impl<'a, T, F> Drop for Init<'a, T, F> {
    fn drop(&mut self) {
        unsafe { self.0.slot.assume_init_drop() };
        self.0.drop_flag.dec_and_check_if_died();
    }
}
