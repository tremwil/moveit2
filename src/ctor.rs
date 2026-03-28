//! Flexible proc-macro based in-place construction of structs.
//!
//! ## Quick Start
//!
//! Simply apply the [`Ctor`] derive macro on your struct:
//!
//! ```
//! # struct PinnedData;
//! use moveit2::Ctor;
//!
//! // this derive is compatible (and pairs effectively) with the
//! // pin-project crate, but it is not required in any way.
//! #[pin_project::pin_project]
//! #[derive(moveit2::Ctor)]
//! pub struct Immovable {
//!     #[pin]
//!     foo: PinnedData,
//!     bar: String
//! }
//! ```
//!
//! This generates the member functions called `ctor` and `try_ctor` for
//! in-place initializing it field by field in a safe manner. These
//! functions take a [`FnOnce`] closure whose input is the set of fields
//! to initialize, with the output being a *proof of initialization*: A type
//! that can only be constructed by initializing every input field.
//!
//! ```
//! # struct PinnedData;
//! # use moveit2::Ctor;
//! # #[pin_project::pin_project]
//! # #[derive(moveit2::Ctor)]
//! # pub struct Immovable {
//! #     #[pin]
//! #     foo: PinnedData,
//! #     bar: String
//! # }
//! use moveit2::{New, InitProof};
//!
//! impl Immovable {
//!     pub fn new(foo: impl New<Output = PinnedData>, bar: String) -> impl New<Output = Self> {
//!         Self::ctor(|fields| InitProof::<Self> {
//!             foo: fields.foo.emplace(foo),
//!             bar: fields.bar.put(bar)
//!         })
//!     }
//! }
//! ```
//!
//! Alternatively, the [`ctor`] and [`try_ctor`] macros can make this
//! initialization even more terse:
//!
//! ```
//! # struct PinnedData;
//! # use moveit2::{New, Ctor, InitProof, ctor};
//! # #[pin_project::pin_project]
//! # #[derive(moveit2::Ctor)]
//! # pub struct Immovable {
//! #     #[pin]
//! #     foo: PinnedData,
//! #     bar: String
//! # }
//! impl Immovable {
//!     pub fn new(foo: impl New<Output = PinnedData>, bar: String) -> impl New<Output = Self> {
//!         moveit2::ctor!(Self { foo, bar })
//!     }
//! }
//! ```
//!
//! These use a powerful trick known as [autoderef specialization] to
//! automatically tell whether the input is a [`New`] or [`TryNew`] that needs
//! to be emplaced, or a plain value that can be moved into the field.
//!
//! The `ctor` and `try_ctor` functions and macros have the minimum visibility
//! of the struct and its fields, so only code that has full access to the
//! internals of the struct can call them.
//!
//! ## How it Works
//!
//! ### The [`Ctor`] derive macro
//!
//! the `Ctor` derive macro generates two field projections that are part of the
//! [`Ctor`] trait:
//! - [`Self::Fields<'a>`], a projection to [`Uninit`] pointers;
//! - [`Self::Proof<'a>`], a projection to [`PinInit`] pointers.
//!
//! These are the input and output types of the `ctor` function, respectively.
//! The only way to construct a `Proof` value is by calling initializer methods
//! on each field of the input `Fields` value. By forcing this initialization to
//! occur in a `FnOnce` that is generic over the projection lifetime, we can
//! guarantee that the struct was fully initialized without any unsafe code.
//!
//! ### The [`Uninit`] type
//!
//! `Uninit` represents a struct field that is uninitialized. It has an API
//! similar to a [`Slot`](crate::Slot), but
//! - Is tagged with a marker type that uniquely represents the field;
//! - Provides a [raw pointer to the field](Uninit::as_ptr) to help with
//!   constructing self-referential structs.
//!
//! `Uninit`s are exposed to the closure passed to the `ctor` function via
//! the input argument.
//!
//! ### The [`PinInit`] type
//!
//! `PinInit` is similar to a [`Pin<MoveRef<T>>`](crate::MoveRef), but tagged
//! with a marker type that uniquely identifies the field it points to. It is
//! called a *proof of initialization* because the only way to construct it is
//! by successfully emplacing into its corresponding [`Uninit`].
//!
//! ## Motivation
//!
//! The factory functions provided by the [`new`](crate::new) module make
//! it easy to safely define in-place constructors (e.g. via
//! [`new::by`](crate::new::by)). Thanks to [`New::with`](crate::TryNew::with),
//! we can even write constructors for [`!Unpin`](Unpin) types without any
//! unsafe code.
//!
//! However, this approach has a fundamental limitation: It must be possible to
//! construct a valid instance of the struct in an unpinned state, so that it
//! can be moved into the pinned memory before the `with` clause initializes the
//! address-sensitive state. For the above `Immovable` example, for instance,
//! we would have to use [`new::by_raw`](crate::new::by_raw) with a copious
//! amount of `unsafe` code:
//!
//! ```
//! # struct PinnedData;
//! use core::{pin::Pin, mem::MaybeUninit};
//! use moveit2::{new, New};
//!
//! #[pin_project::pin_project]
//! #[derive(moveit2::Ctor)]
//! pub struct Immovable {
//!     #[pin]
//!     foo: PinnedData,
//!     bar: String
//! }
//!
//! impl Immovable {
//!     pub fn new(foo: impl New<Output = PinnedData>, bar: String) -> impl New<Output = Self> {
//!         unsafe {
//!             // note: `this` is a Pin<&mut MaybeUninit<Self>>
//!             new::by_raw::<Self, _>(|this| {
//!                 // 1. get a raw pointer
//!                 let this_ptr = Pin::into_inner_unchecked(this).as_mut_ptr();
//!                 // 2. get a pointer to the `foo` field and cast to MaybeUninit
//!                 let foo_ptr = (&raw mut (*this_ptr).foo).cast::<MaybeUninit<PinnedData>>();
//!                 // 3. convert foo_ptr to a pinned reference so it can be initialized
//!                 foo.new(Pin::new_unchecked(&mut *foo_ptr));
//!                 // 4. write the `bar` value to its field
//!                 (&raw mut (*this_ptr).bar).write(bar);
//!             })
//!         }
//!     }
//! }
//! ```
//!
//! The above is messy and error-prone. For example, it would still compile if
//! we changed the type of `foo`, or we added a new field!
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
//! [`Self::Fields<'a>`]: Ctor::Fields
//! [`Self::Proof<'a>`]: Ctor::Proof
//! [autoderef specialization]: https://lukaskalbertodt.github.io/2019/12/05/generalized-autoref-based-specialization.html

use core::{
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    pin::Pin,
    ptr::NonNull,
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
/// See the [module-level documentation](mod@crate::ctor) for more information.
///
/// # Limitations
///
/// Due to [type system restrictions around higher-kinded lifetimes](https://users.rust-lang.org/t/for-a-a-t-seems-to-require-t-static/137175/5?u=tremwil)
/// and the poor type inference of workarounds to said restrictions, the
/// [`Ctor`] trait does not expose the constructor functions itself.
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
/// To initialize the field, use the [`put`], [`emplace`] and [`try_emplace`]
/// methods. It is also possible to get a raw pointer to the field via
/// [`as_ptr`] and [`as_non_null`].
///
/// [`Slot`]: crate::Slot
/// [`put`]: Uninit::put
/// [`emplace`]: Uninit::emplace
/// [`try_emplace`]: Uninit::try_emplace
/// [`as_ptr`]: Uninit::as_ptr
/// [`as_non_null`]: Uninit::as_non_null
pub struct Uninit<'a, T, F> {
    slot: &'a mut MaybeUninit<T>,
    drop_flag: DropFlag<'a>,
    phantom: PhantomData<F>,
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

    /// Get a mutable pointer to this field's uninitialized memory.
    ///
    /// After writing an initialized value to this pointer, [`assume_init`] must
    /// be called to get a proof of initialization.
    ///
    /// [`assume_init`]: Self::assume_init
    pub fn as_ptr(&self) -> *mut T {
        (&raw const *self.slot) as *mut T
    }

    /// Get a [`NonNull`] pointer to this field's uninitialized memory.
    ///
    /// After writing an initialized value to this pointer, [`assume_init`] must
    /// be called to get a proof of initialization.
    ///
    /// [`assume_init`]: Self::assume_init
    pub fn as_non_null(&self) -> NonNull<T> {
        NonNull::from_ref(self.slot).cast()
    }

    /// Unsafely assume that this field has been initialized.
    ///
    /// # Safety
    ///
    /// A valid, initialized `T` must have been written to the pointer
    /// returned by [`as_ptr`] or [`as_non_null`].
    ///
    /// [`as_ptr`]: Self::as_ptr
    /// [`as_non_null`]: Self::as_non_null
    pub unsafe fn assume_init(self) -> PinInit<'a, T, F> {
        self.drop_flag.inc();
        // SAFETY:
        // - slot is initialized
        // - `Init` can be pinned as drop flag will prevent data from being
        // forgotten after pinning (promise of new_unchecked)
        unsafe { Pin::new_unchecked(Init(self)) }
    }

    /// Initialize this slot by moving a value into it.
    #[inline]
    pub fn put(self, value: T) -> PinInit<'a, T, F> {
        self.slot.write(value);
        unsafe { self.assume_init() }
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
            Ok(self.assume_init())
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
/// The struct must implement [`Ctor`] through the derive macro for this to
/// work.
#[macro_export]
macro_rules! ctor {
    (move || $struct:ty { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[move][__fields]{}{ $struct { $($tokens)* } })
    };

    (move |$fields:ident| $struct:ty { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[move][$fields]{}{ $struct { $($tokens)* } })
    };

    (move || { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[move][__fields]{}{ $($tokens)* })
    };

    (move |$fields:ident| { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[move][$fields]{}{ $($tokens)* })
    };

    ($(||)? $struct:ty { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[][__fields]{}{ $struct { $($tokens)* } })
    };

    (|$fields:ident| $struct:ty { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[][$fields]{}{ $struct { $($tokens)* } })
    };

    ($(||)? { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[][__fields]{}{ $($tokens)* })
    };

    (|$fields:ident| { $($tokens:tt)* }) => {
        $crate::ctor!(@gather_stmt[][$fields]{}{ $($tokens)* })
    };

    (@gather_stmt[$($mov:tt)?][$fields:ident]{ $($statements:stmt)* }{
        $struct:ty {
            $($(#[$attrs:meta])* $field:ident $(:$expr:expr)?),*
            $(,)?
        }
    }) => {
        <$struct>::ctor($($mov)? |$fields| {
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

    (@gather_stmt[$($mov:tt)?][$fields:ident]{ $($statements:stmt)* }{
        $stmt:stmt;
        $($tokens:tt)*
    }) => {
        $crate::ctor!(@gather_stmt[$($mov)?][$fields]{$($statements;)* $stmt; }{ $($tokens)* })
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
    ($err:ty, move || $struct:ty { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[move][$err, __fields]{}{ $struct { $($tokens)* } })
    };

    ($err:ty, move |$fields:ident| $struct:ty { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[move][$err, $fields]{}{ $struct { $($tokens)* } })
    };

    ($err:ty, move || { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[move][$err, __fields]{}{ $($tokens)* })
    };

    ($err:ty, move |$fields:ident| { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[move][$err, $fields]{}{ $($tokens)* })
    };

    ($err:ty, $(||)? $struct:ty { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[][$err, __fields]{}{ $struct { $($tokens)* } })
    };

    ($err:ty, |$fields:ident| $struct:ty { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[][$err, $fields]{}{ $struct { $($tokens)* } })
    };

    ($err:ty, $(||)? { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[][$err, __fields]{}{ $($tokens)* })
    };

    ($err:ty, |$fields:ident| { $($tokens:tt)* }) => {
        $crate::try_ctor!(@gather_stmt[][$err, $fields]{}{ $($tokens)* })
    };

    (@gather_stmt[$($mov:tt)?][$err:ty, $fields:ident]{ $($statements:stmt)* }{
        $struct:ty {
            $($(#[$attrs:meta])* $field:ident $(:$expr:expr)?),*
            $(,)?
        }
    }) => {
        <$struct>::try_ctor::<$err, _>($($mov)? |$fields| {
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

    (@gather_stmt[$($mov:tt)?][$err:ty, $fields:ident]{ $($statements:stmt)* }{
        $stmt:stmt;
        $($tokens:tt)*
    }) => {
        $crate::try_ctor!(@gather_stmt[$($mov)?][$err, $fields]{$($statements;)* $stmt; }{ $($tokens)* })
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
