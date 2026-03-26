//! Flexible proc-macro based in-place construction.
//!
//! While the factory functions provided by the [`new`](crate::new) module make
//! it easy to safely define in-place constructors (e.g. via
//! [`new::by`](crate::new::by)), Thanks to [`New::with`](crate::TryNew::with),
//! we can even write constructors for [`!Unpin`](Unpin) types without any
//! unsafe code.
//!
//! However, this approach has a fundamental limitation: It must be possible to
//! construct a valid instance of the struct in an unpinned state, so that it
//! can be moved into the pinned memory before the `with` clause initializes the
//! address-sensitive state. However, when such a state does not exist, we have
//! to rely on the unsafe [`new::by_raw`](crate::new::by_raw):
//!
//! ```ignore
//! use moveit2::{moveit, new, New};
//!
//! #[pin_project::pin_project]
//! pub struct SelfRef<T> {
//!     #[pin] // this field is structurally pinned!
//!     val: T,
//!     val_ptr: *mut T
//! }
//!
//! impl<T> SelfRef<T> {
//!     pub fn new(val_new: impl New<Output = T>) -> impl New<Output = Self> {
//!         unsafe {
//!             new::by_raw(|this| {
//!                 // `this` is a `Pin<&mut MaybeUninit<Self>>`, so to emplace `val_new`
//!                 // we have to...
//!                 // 1. into_inner the pin (unsafe)
//!                 // 2. turn the reference into a raw pointer and project to `this.val`
//!                 //    without creating a reference
//!                 // 3. cast that pointer to a MaybeUninit<T> and then back into a
//!                 //    mutable reference (unsafe)
//!                 // 4. pin that reference (unsafe)
//!                 todo!()
//!             })
//!         }
//!     }
//! }
//! ```
//!
//! The above is messy and error-prone. In comparison, by annotating `SelfRef`
//! with the [`Ctor`] derive macro we get a `ctor` function that allows doing
//! the above in a completely safe manner:
//!
//! ```
//! # use moveit2::{moveit, new, New};
//! #
//! # #[derive(moveit2::Ctor)]
//! # #[pin_project::pin_project]
//! # pub struct SelfRef<T> {
//! #    #[pin]
//! #    val: T,
//! #    val_ptr: *mut T
//! # }
//! #
//! use moveit2::InitProof;
//!
//! impl<T> SelfRef<T> {
//!     pub fn new(val_new: impl New<Output = T>) -> impl New<Output = Self> {
//!         Self::ctor(|fields| InitProof::<Self> {
//!             // `fields` gives us pointers that Deref into `MaybeUninit`s to
//!             // take addresses/etc:
//!             val_ptr: fields.val_ptr.put(fields.val.as_ptr().cast_mut()),
//!             // simultaneously they are what we emplace fields into:
//!             val: fields.val.emplace(val_new)
//!         })
//!     }
//! }
//! ```
//!
//! ...or even more concisely using the higher-level [`ctor`] declarative
//! macro:
//!
//! ```
//! # use moveit2::{moveit, new, New};
//! #
//! # #[derive(moveit2::Ctor)]
//! # #[pin_project::pin_project]
//! # pub struct SelfRef<T> {
//! #    #[pin]
//! #    val: T,
//! #    val_ptr: *mut T
//! # }
//! #
//! impl<T> SelfRef<T> {
//!     pub fn new(val_new: impl New<Output = T>) -> impl New<Output = Self> {
//!         moveit2::ctor!(Self {
//!             val: val_new,
//!             // we effectively have full access to a `Pin<&mut T>` here!
//!             // Note that since `T` may be `!Unpin`, we can't use `&raw mut`.
//!             val_ptr: (&raw const *val).cast_mut(),
//!         })
//!     }
//! }
//! ```
//!
//! # How it Works
//!
//! ### The [`Uninit`] type
//!
//! `Uninit` represents a struct field that is uninitialized. It has an API
//! similar to a [`Slot`](crate::Slot), but exposes the underlying
//! [`MaybeUninit`] immutably to help with unsafe code, and is tagged with
//! a marker type that uniquely identifies the field it points to.
//!
//! `Uninit`s are exposed to the closure passed to the `ctor` function via
//! the input argument.
//!
//! ### The [`PinInit`] type
//!
//! `PinInit` is similar to a `Pin<MoveRef<T>>`, but tagged with a marker
//! type that uniquely identifies the field it points to. It is called
//! a *proof of initialization* because the only way to construct it is
//! by successfully emplacing into its corresponding [`Uninit`].
//!
//! ### The [`Ctor`] derive macro
//!
//! `Ctor` generates two field projections that are part of the [`Ctor`] trait:
//! - [`Self::Fields<'a>`], a projection to [`Uninit`] pointers;
//! - [`Self::Proof<'a>`], a projection to [`PinInit`] pointers.
//!
//! As such the only way to construct a `Proof` value is by initializing each
//! field of a `Fields` type. By forcing this initialization to occur in a
//! `FnOnce` that is generic over the projection lifetime, we can guarantee
//! that the struct was fully initialized without any unsafe code.
//!
//! This is how the `ctor` and `try_ctor` member functions work.
//!
//! # Partial Initialization
//!
//! If initialization fails midway (due to a panic or `try_ctor` returning
//! `Err`), any initialized fields will be dropped normally.
//!
//! Note that although there is no point to doing this, using
//! [`core::mem::forget`] or [`ManuallyDrop`](core::mem::ManuallyDrop) to forget
//! an initialized field will lead to an LLVM abort. This is because letting the
//! program continue to run (or unwind) would violate the [`Pin`] invariant that
//! a field must be dropped before its storage is reused.
//!
//! # Limitations
//!
//! - Currently, the `Ctor` derive (and by extension the decl macros) do not
//!   support tuple structs or enums. This is a limitation that may be lifted in
//!   the future.
//!
//! - When using the `try_ctor` function, the usual limitation of error type
//!   inference in closures applies: The generic error type must be explicitly
//!   specified, either on the function, the closure's return type, or the final
//!   `Ok`.
//!
//! - Due to the above, the `try_ctor` macro requires explicitly specifying the
//!   error type before the struct expression/closure.
//!
//! # Examples
//!
//! ### Infallible constructors
//!
//! ```
//! use core::marker::PhantomPinned;
//! use moveit2::{Ctor, InitProof, moveit, new};
//!
//! #[derive(Ctor)]
//! pub struct Immovable {
//!     val: String,
//!     addr_of_val: *const String,
//! }
//!
//! let string_ctor = new::of("abcdef".to_string());
//!
//! let immovable_ctor = Immovable::ctor(|fields| InitProof::<Immovable> {
//!     // `put` to move something into the field
//!     addr_of_val: fields.addr_of_val.put(fields.val.as_ptr()),
//!     // `emplace` to construct into the field without moving
//!     val: fields.val.emplace(string_ctor),
//! });
//!
//! moveit!(let immovable = immovable_ctor);
//! assert_eq!(&raw const immovable.val, immovable.addr_of_val);
//! ```
//!
//! ### Fallible constructors
//!
//! ```
//! # fn main() -> Result<(), ()> {
//! #
//! use core::marker::PhantomPinned;
//! use moveit2::{Ctor, InitProof, slot, new};
//!
//! #[derive(Ctor)]
//! pub struct Immovable {
//!     val: String,
//!     addr_of_val: *const String,
//! }
//!
//! let string_ctor = new::try_by(|| Ok::<_, ()>("abcdef".to_string()));
//!
//! let immovable_ctor = Immovable::try_ctor(|fields| -> Result<_, ()> {
//!     Ok(InitProof::<Immovable> {
//!         // `put` to move something into the field
//!         addr_of_val: fields.addr_of_val.put(fields.val.as_ptr()),
//!         // `try_emplace` to try to construct into the field without moving
//!         val: fields.val.try_emplace(string_ctor)?,
//!     })
//! });
//!
//! slot!(immovable);
//! let immovable = immovable.try_emplace(immovable_ctor)?;
//! assert_eq!(&raw const immovable.val, immovable.addr_of_val);
//! #
//! # Ok(())
//! # }
//! ```
//!
//! [`Self::Fields<'a>`]: Ctor::Fields
//! [`Self::Proof<'a>`]: Ctor::Proof

use core::{
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    pin::Pin,
};

use crate::{New, TryNew, drop_flag::DropFlag};

#[doc(inline)]
pub use moveit2_proc_macros::Ctor;

/// Trait implemented on struct types that can be in-place constructed field by
/// field.
///
/// When applied to a struct using [`derive`], this generates `ctor` and
/// `try_ctor` member functions that respectively return [`New`] and [`TryNew`]
/// implementations for initializing the struct in place, field by field:
///
/// ```rust
/// use moveit2::{Ctor, InitProof, moveit, new};
///
/// #[derive(Ctor)]
/// pub struct SelfRef<T> {
///     value: T,
///     value_ptr: *const T,
/// }
///
/// let value_new = new::of("abcdef");
/// let ctor = SelfRef::ctor(|fields| InitProof::<SelfRef<_>> {
///     // initialize this field by moving into it.
///     value_ptr: fields.value_ptr.put(fields.value.as_ptr()),
///     // initialize this field via in-place construction.
///     value: fields.value.emplace(value_new),
/// });
///
/// moveit!(let self_ref = ctor);
/// assert_eq!(&raw const self_ref.value, self_ref.value_ptr);
/// ```
///
/// # Limitations
///
/// Due to [type system restrictions around higher-kinded lifetimes](https://users.rust-lang.org/t/for-a-a-t-seems-to-require-t-static/137175/5?u=tremwil)
/// and the poor type inference of workarounds to said restrictions, the
/// [`Ctor`] trait does not expose the constructor function on types itself.
pub trait Ctor: Sized {
    /// A projection of the uninitialized fields of `Self`.
    ///
    /// This struct exposes the same fields as `Self` through [`Uninit`]
    /// references.
    type Fields<'a>
    where
        Self: 'a;

    /// A projection of the fields of `Self` as [`PinInit`] references.
    ///
    /// This is called a "proof" as constructing this type requires initializing
    /// every field exposed through [`Ctor::Fields`], thus proving that
    /// `Self` is now initialized.
    ///
    /// It is easiest to access this associated type through the [`InitProof`]
    /// type alias for constructing instances.
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
/// [`Slot`]: crate::Slot
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
    /// This function is internal to the [`Ctor`] derive macro and should never
    /// be called manually. The function and its safety invariants below are
    /// not part of semver and may change at any time.
    ///
    /// # Safety
    /// - `slot` must be uninitialized memory that is exclusively accessible for
    ///   lifetime `'a`
    /// - `drop_flag` must only be used with other `Uninit`s in frame 'a
    /// - after the scope of `'a` ends, `drop_flag` must be checked to ensure
    ///   that no created `Init`s have been forgotten. In this situation the
    ///   program *must* abort to avoid violating [`Pin`] obligations.
    #[doc(hidden)]
    #[inline]
    pub unsafe fn new_unchecked(slot: *mut T, drop_flag: DropFlag<'a>) -> Self {
        Self {
            slot: unsafe { &mut *slot.cast() },
            drop_flag,
            phantom: PhantomData,
        }
    }

    /// Initialize this slot by moving a value into it.
    #[inline]
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
    #[inline]
    pub fn emplace(self, new: impl New<Output = T>) -> PinInit<'a, T, F> {
        self.try_emplace(new).unwrap_or_else(|e| match e {})
    }

    /// Try to initialize this struct by constructing a value inside of it.
    #[inline]
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

/// Like a [`MoveRef`], but references an initialized field of a struct
/// represented by some marker type `F`.
///
/// When [pinned], this type acts as proof that the field was initialized; with
/// the other fields of the struct it can be used to construct a [`Ctor::Proof`]
/// to finalize in-place construction of the struct.
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

/// Create a [`New`] implementation for initializing a struct field by field.
///
/// The struct must implemented [`Ctor`] through the derive macro for this to
/// work.
#[macro_export]
macro_rules! ctor {
    ($struct:ty { $($tokens:tt)* }) => {
        $crate::ctor!(|__fields| $struct {
            $($tokens)*
        })
    };

    (|$fields:ident| $struct:ty {
            $($(#[$attrs:meta])* $field:ident $(:$expr:expr)?),*
            $(,)?
    }) => {
        <$struct>::ctor(|$fields| {
            use $crate::ctor::__private::{
                ViaEmplace as _,
                ViaPut as _
            };

            $(
                $crate::ctor!(@assign_field[$fields]($(#[$attrs])* $field $(:$expr)?));
            )*

            $crate::ctor::InitProof::<$struct> {
                $($(#[$attrs])* $field),*
            }
        })
    };

    ({ $($tokens:tt)* }) => {
        $crate::ctor!(|__fields| { $($tokens)* })
    };

    (|$fields:ident| { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[$fields]{}{ $($tokens)* })
    };

    (@gather_stmt[$fields:ident]{ $($statements:stmt)* }{
        $struct:ty {
            $($(#[$attrs:meta])* $field:ident $(:$expr:expr)?),*
            $(,)?
        }
    }) => {
        <$struct>::ctor(|$fields| {
            $($statements;)*

            use $crate::ctor::__private::{
                ViaEmplace as _,
                ViaPut as _
            };

            $(
                $crate::ctor!(@assign_field[$fields]($(#[$attrs])* $field $(:$expr)?));
            )*

            $crate::ctor::InitProof::<$struct> {
                $($(#[$attrs])* $field),*
            }
        })
    };

    (@gather_stmt[$fields:ident]{ $($statements:stmt)* }{
        $stmt:stmt;
        $($tokens:tt)*
    }) => {
        $crate::ctor!(@gather_stmt[$fields]{$($statements;)* $stmt; }{ $($tokens)* })
    };

    (@assign_field[$fields:ident]($(#[$attrs:meta])* $field:ident)) => {
        $(#[$attrs])*
        let mut $field = (&&$crate::ctor::__private::CtorSpec::new(&$fields.$field, &$field))
            .init($fields.$field, $field);
    };

    (@assign_field[$fields:ident]($(#[$attrs:meta])* $field:ident: $expr:expr)) => {
        $(#[$attrs])*
        let mut $field = match $expr {
            __expr => (&&$crate::ctor::__private::CtorSpec::new(&$fields.$field, &__expr))
                .init($fields.$field, __expr)
        };
    };
}

#[doc(inline)]
pub use ctor;

/// Create a [`TryNew`] implementation for initializing a struct field by field.
///
/// The struct must implemented [`Ctor`] through the derive macro for this to
/// work.
#[macro_export]
macro_rules! try_ctor {
    ($err:ty, $struct:ty { $($tokens:tt)* }) => {
        $crate::try_ctor!($err, |__fields| $struct {
            $($tokens)*
        })
    };
    ($err:ty, |$fields:ident| $struct:ty {
            $($(#[$attrs:meta])* $field:ident $(:$expr:expr)?),*
            $(,)?
    }) => {
        <$struct>::try_ctor::<$err, _>(|$fields| {
            use $crate::ctor::__private::{
                TryViaTryEmplace as _,
                TryViaEmplace as _,
                TryViaPut as _
            };

            $(
                $crate::try_ctor!(@assign_field[$err, $fields]($(#[$attrs])* $field $(:$expr)?));
            )*

            Ok($crate::ctor::InitProof::<$struct> {
                $($(#[$attrs])* $field),*
            })
        })
    };

    ($err:ty, { $($tokens:tt)* }) => {
        $crate::try_ctor!($err:ty, |__fields| { $($tokens)* })
    };

    ($err:ty, |$fields:ident| { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[$err, $fields]{}{ $($tokens)* })
    };

    (@gather_stmt[$err:ty, $fields:ident]{ $($statements:stmt)* }{
        $struct:ty {
            $($(#[$attrs:meta])* $field:ident $(:$expr:expr)?),*
            $(,)?
        }
    }) => {
        <$struct>::try_ctor::<$err, _>(|$fields| {
            $($statements;)*

            use $crate::ctor::__private::{
                TryViaTryEmplace as _,
                TryViaEmplace as _,
                TryViaPut as _
            };

            $(
                $crate::try_ctor!(@assign_field[$err, $fields]($(#[$attrs])* $field $(:$expr)?));
            )*

            Ok($crate::ctor::InitProof::<$struct> {
                $($(#[$attrs])* $field),*
            })
        })
    };

    (@gather_stmt[$err:ty, $fields:ident]{ $($statements:stmt)* }{
        $stmt:stmt;
        $($tokens:tt)*
    }) => {
        $crate::try_ctor!(@gather_stmt[$err, $fields]{$($statements;)* $stmt; }{ $($tokens)* })
    };

    (@assign_field[$err:ty, $fields:ident]($(#[$attrs:meta])* $field:ident)) => {
        $(#[$attrs])*
        let mut $field = (
            &&&$crate::ctor::__private::TryCtorSpec::<_, _, _, $err>::new(&$fields.$field, &$field)
        ).try_init($fields.$field, $field)?;
    };

    (@assign_field[$err:ty, $fields:ident]($(#[$attrs:meta])* $field:ident: $expr:expr)) => {
        $(#[$attrs])*
        let mut $field = match $expr {
            __expr => (
                &&&$crate::ctor::__private::TryCtorSpec::<_, _, _, $err>::new(&$fields.$field, &__expr)
            ).try_init($fields.$field, __expr)?
        };
    };
}

#[doc(inline)]
pub use try_ctor;

#[doc(hidden)]
pub mod __private {
    pub use core::convert::Infallible;
    pub use core::marker::PhantomData;
    pub use core::mem::forget;
    pub use core::pin::Pin;

    use crate::ctor::{PinInit, Uninit};
    use crate::{New, TryNew};

    // The types below are used for autoref specialization in `ctor` and `try_ctor`
    // macros. This allows picking out the right impl for the user-provided type
    //
    // Reference:
    //
    // https://lukaskalbertodt.github.io/2019/12/05/generalized-autoref-based-specialization.html

    #[allow(clippy::type_complexity)]
    pub struct CtorSpec<T, F, U>(PhantomData<fn() -> (T, F, U)>);

    impl<T, F, U> CtorSpec<T, F, U> {
        #[inline]
        pub fn new<'a>(_uninit: &Uninit<'a, T, F>, _src: &U) -> Self {
            Self(PhantomData)
        }
    }

    pub trait ViaEmplace<T, F, U> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> PinInit<'a, T, F>;
    }

    impl<T, F, U> ViaEmplace<T, F, U> for &CtorSpec<T, F, U>
    where
        U: New<Output = T>,
    {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> PinInit<'a, T, F> {
            uninit.emplace(src)
        }
    }

    pub trait ViaPut<T, F> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> PinInit<'a, T, F>;
    }

    impl<T, F> ViaPut<T, F> for CtorSpec<T, F, T> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> PinInit<'a, T, F> {
            uninit.put(src)
        }
    }

    #[allow(clippy::type_complexity)]
    pub struct TryCtorSpec<T, F, U, E>(PhantomData<fn() -> (T, F, U, E)>);

    impl<T, F, U, E> TryCtorSpec<T, F, U, E> {
        #[inline]
        pub fn new<'a>(_uninit: &Uninit<'a, T, F>, _src: &U) -> Self {
            Self(PhantomData)
        }
    }

    pub trait TryViaEmplace<T, F, U, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<PinInit<'a, T, F>, E>;
    }
    impl<T, F, U, E> TryViaEmplace<T, F, U, E> for &&TryCtorSpec<T, F, U, E>
    where
        U: New<Output = T>,
    {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<PinInit<'a, T, F>, E> {
            Ok(uninit.emplace(src))
        }
    }

    pub trait TryViaTryEmplace<T, F, U, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<PinInit<'a, T, F>, E>;
    }
    impl<T, F, U, E> TryViaTryEmplace<T, F, U, E> for &TryCtorSpec<T, F, U, E>
    where
        U: TryNew<Output = T, Error: Into<E>>,
    {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<PinInit<'a, T, F>, E> {
            uninit.try_emplace(src).map_err(Into::into)
        }
    }

    pub trait TryViaPut<T, F, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Result<PinInit<'a, T, F>, E>;
    }
    impl<T, F, E> TryViaPut<T, F, E> for TryCtorSpec<T, F, T, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Result<PinInit<'a, T, F>, E> {
            Ok(uninit.put(src))
        }
    }
}
