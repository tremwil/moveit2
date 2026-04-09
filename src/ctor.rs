//! Flexible proc-macro based in-place construction of structs.
//!
//! ## Quick Start
//!
//! Simply apply the [`Ctor`] derive macro on your struct:
//!
//! ```
//! # struct PinnedData;
//! // this derive is compatible (and pairs effectively) with the
//! // pin-project crate, but it is not required in any way.
//! #[pin_project::pin_project]
//! #[derive(moveit2::Ctor)]
//! pub struct Immovable {
//!     #[pin] // implies that this field is structurally pinned.
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
//! - [`Self::Proof<'a>`], a projection to [`Init`] and/or [`PinInit`] pointers
//!   (depending on whether the fields are [structurally pinned], which is
//!   controlled by the `#[pin]` attribute).
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
//! ### The [`Init`] and [`PinInit`] types
//!
//! `Init` is similar to a [`MoveRef<T>`](crate::MoveRef), but tagged
//! with a marker type that uniquely identifies the field it points to. It is
//! called a *proof of initialization* because the only way to construct it is
//! by successfully moving/emplacing into its corresponding [`Uninit`].
//!
//! `PinInit` is simply an alias for a [pinned](Pin) `Init`. An `Init` can be
//! trivially promoted to it via [`Init::into_pin`], but the reverse is not
//! true.
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
//! [structurally pinned]: core::pin#projections-and-structural-pinning

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
///     // signifies that the field is structurally pinned and can thus be emplaced.
///     #[pin]
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

    /// A projection of the fields of `Self` as [`Init`] or [`PinInit`] (when
    /// structurally pinned) references.
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
    pub unsafe fn assume_init(self) -> Init<'a, T, F> {
        self.drop_flag.inc();
        // SAFETY:
        // - slot is initialized
        Init(self)
    }

    /// Unsafely assume that this field has been initialized, and return a
    /// pinned pointer to it.
    ///
    /// # Safety
    ///
    /// A valid, initialized `T` must have been written to the pointer
    /// returned by [`as_ptr`] or [`as_non_null`].
    ///
    /// [`as_ptr`]: Self::as_ptr
    /// [`as_non_null`]: Self::as_non_null
    pub unsafe fn assume_init_pin(self) -> PinInit<'a, T, F> {
        // SAFETY:
        // - slot is initialized
        Init::into_pin(unsafe { self.assume_init() })
    }

    /// Initialize this slot by moving a value into it.
    #[inline]
    pub fn put(self, value: T) -> Init<'a, T, F> {
        self.slot.write(value);
        unsafe { self.assume_init() }
    }

    /// Initialize this slot by moving a value into it, then pinning it.
    ///
    /// This is a shorthand for `Init::into_pin(self.put(value))`.
    #[inline]
    pub fn pin(self, value: T) -> PinInit<'a, T, F> {
        Init::into_pin(self.put(value))
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
            Ok(self.assume_init_pin())
        }
    }
}

/// Like a [`MoveRef`], but references an initialized field of a struct
/// represented by some marker type `F`.
///
/// This type acts as proof that the field was initialized; with the other
/// fields of the struct it can be used to construct a [`Ctor::Proof`]
/// to finalize in-place construction of the struct.
///
/// [`MoveRef`]: crate::MoveRef
pub struct Init<'a, T, F>(Uninit<'a, T, F>);

impl<T, F> Init<'_, T, F> {
    /// Pin the target of this pointer.
    ///
    /// This is safe as soundly creating an [`Uninit`] reference requires that
    /// the memory may be able to be treated as pinned.
    pub fn into_pin(this: Self) -> Pin<Self> {
        unsafe { Pin::new_unchecked(this) }
    }
}

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

macro_rules! hidden_macro_internals {
    (
        $(#[$attrs:meta])*
        macro_rules! $name:ident {
            $($toks:tt)*
        }
    ) => {
        $(#[$attrs])*
        #[cfg(doc)]
        macro_rules! $name {
            (...) => { ... }
        }

        $(#[$attrs])*
        #[cfg(not(doc))]
        macro_rules! $name {
            $($toks)*
        }
    };
}

hidden_macro_internals!(
    /// Create a [`New`] implementation for initializing a struct field by
    /// field.
    ///
    /// This macro is merely syntactic sugar for the closure passed to the
    /// `ctor` constructor factory function generated by the [`Ctor`] derive
    /// and as such the struct must `#[derive(Ctor)]` for it to work. For more
    /// information about in-place struct initializers, see the
    /// [module level documentation](mod@crate::ctor).
    ///
    /// For a fallible version of this macro, see [`try_ctor`].
    ///
    /// ### Basic use
    ///
    /// At its simplest use, it wraps a struct literal expression:
    ///
    /// ```
    /// use moveit2::{new, Ctor, ctor};
    ///
    /// #[derive(Ctor)]
    /// struct Data<T> {
    ///     #[pin]
    ///     foo: T,
    ///     bar: usize,
    ///     baz: String,
    /// };
    ///
    /// let foo_new = new::of("abcdef");
    /// let baz: String = "123456".to_owned();
    ///
    /// ctor!(Data::<_> {
    ///     foo: foo_new, // emplaces into the field using `New`
    ///     bar: 5,       // moves 5 into the field
    ///     baz,          // short syntax for matching names
    /// });
    /// ```
    ///
    /// Note the explicit turbofish operator (`::<_>`) to specify generic
    /// parameters of `Data`. This is a limitation of this macro; while it is
    /// not required when writing a regular struct literal, it is here.
    ///
    /// Thanks to [autoderef specialization], this macro can automatically
    /// detect whether a field should be initialized in-place using [`New`] or
    /// by moving into the field, as is done above with `foo_new`.
    ///
    /// ### Using initialized fields post-initialization
    ///
    /// After initializing a field, it is exposed as a mutable [`PinInit`]
    /// pointer for subsequent field assignments. This makes it easy to e.g.
    /// take a pointer to a field in a self-referential struct:
    ///
    /// ```
    /// use moveit2::{New, Ctor, ctor};
    ///
    /// #[derive(Ctor)]
    /// pub struct SelfRef<T> {
    ///     #[pin]
    ///     val: T,
    ///     val_ptr: *mut T,
    /// };
    ///
    /// impl<T: Unpin> SelfRef<T> {
    ///     pub fn new(val: impl New<Output = T>) -> impl New<Output = Self> {
    ///         ctor!(Self {
    ///             val,
    ///             val_ptr: &raw mut *val,
    ///         })
    ///     }
    /// }
    /// ```
    ///
    /// Note that since for initialized structurally pinned fields, the regular
    /// `Pin` dereference rules apply. We have access to a mutable borrow
    /// above because of the `T: Unpin` bound, but without it would only be
    /// able to borrow immutably and would have to write `(&raw const
    /// *val).cast_mut()` instead.
    ///
    /// ### Including pre-initialization logic
    ///
    /// The macro also accepts a block expression `{ ... }`, in which case
    /// additional statements can be included before the final struct
    /// literal:
    ///
    /// ```
    /// # use moveit2::{new, Ctor, ctor};
    /// #
    /// # #[derive(Ctor)]
    /// # struct Data<T> {
    /// #     foo: T,
    /// #     bar: usize,
    /// #     #[pin]
    /// #     baz: String,
    /// # };
    /// #
    /// ctor!({
    ///     let baz = new::of("1234".to_string());
    ///     Data::<_> {
    ///         foo: "abcdef",
    ///         bar: 5,
    ///         baz       
    ///     }
    /// });
    /// ```
    ///
    /// ### Explicitly specifying `move` capturing
    ///
    /// Since the provided expression desugars into a closure, in some
    /// situations it may be necessary to capture by value. Like for normal
    /// closures, this may be done by prefixing the struct literal / block
    /// expression with `move ||`:
    ///
    /// ```
    /// use moveit2::{Ctor, ctor};
    ///
    /// #[derive(Ctor)]
    /// struct Data<'a> {
    ///     foo: &'a str,
    ///     bar: usize,
    /// };
    ///
    /// let string = "123".to_owned();
    ///
    /// // move `string` into the constructor, even if it could be taken
    /// // by reference
    /// ctor!(move || Data {
    ///     foo: "abcdef",
    ///     bar: string.len(),
    /// });
    /// ```
    ///
    /// ### Accessing uninitialized fields
    ///
    /// The projection of uninitialized fields as [`Uninit`] values can be
    /// accessed by specifying a closure argument:
    ///
    /// ```
    /// use moveit2::{New, Ctor, ctor};
    ///
    /// #[derive(Ctor)]
    /// pub struct SelfRef<T> {
    ///     #[pin]
    ///     val: T,
    ///     val_ptr: *mut T,
    /// };
    ///
    /// impl<T> SelfRef<T> {
    ///     pub fn new(val: impl New<Output = T>) -> impl New<Output = Self> {
    ///         ctor!(|fields| Self {
    ///             val_ptr: fields.val.as_ptr(),
    ///             val,
    ///         })
    ///     }
    /// }
    /// ```
    ///
    /// Note that while this is supported, if you need it it is likely
    /// preferable to use the `ctor` member function generated by the
    /// [`Ctor`] derive macro directly.
    ///
    /// ### Initialization failures
    ///
    /// If initialization panics before all fields are initialized,
    /// they will be dropped in reverse order of declaration. This mimics the
    /// behavior of regular struct literal expressions.
    ///
    /// [autoderef specialization]: https://lukaskalbertodt.github.io/2019/12/05/generalized-autoref-based-specialization.html
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
                    ViaPin as _,
                    ViaPut as _,
                    ViaNever as _,
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
            let mut $field = (&&&&$crate::ctor::__private::CtorSpec::new(&$fields.$field, &$field))
                .init($fields.$field, $field);
        };

        (@assign_field[$fields:ident]($(#[$attrs:meta])* $field:ident: $expr:expr)) => {
            $(#[$attrs])*
            let mut $field = match $expr {
                __expr => (&&&&$crate::ctor::__private::CtorSpec::new(&$fields.$field, &__expr))
                    .init($fields.$field, __expr)
            };
        };
    }
);

#[doc(inline)]
pub use ctor;

hidden_macro_internals!(
    /// Create a [`TryNew`] implementation for initializing a struct
    /// fallibly, field by field.
    ///
    /// This macro is merely syntactic sugar for the closure passed to the
    /// `try_ctor` constructor factory function generated by the [`Ctor`] derive
    /// and as such the struct must `#[derive(Ctor)]` for it to work. For more
    /// information about in-place struct initializers, see the
    /// [module level documentation](mod@crate::ctor).
    ///
    /// ### Syntax
    ///
    /// The macro accepts the same input syntax as [`ctor`], with the exception
    /// that the error type must be explicitly given before the struct literal
    /// expression. As such, for a comprehensive overview of this macro's
    /// features, see the [`ctor`] documentation first.
    ///
    /// ```
    /// use moveit2::{new, Ctor, try_ctor};
    ///
    /// struct GenericError;
    /// struct SpecificError;
    ///
    /// impl From<SpecificError> for GenericError {
    ///     fn from(value: SpecificError) -> Self {
    ///         GenericError
    ///     }
    /// }
    ///
    /// #[derive(Ctor)]
    /// struct Data<T> {
    ///     #[pin]
    ///     foo: T,
    ///     #[pin]
    ///     bar: usize,
    ///     baz: String,
    /// };
    ///
    /// let try_foo = new::try_by(|| Ok::<_, SpecificError>("abcdef"));
    /// let bar = new::of(5);
    /// let baz: Result<String, GenericError> = Ok("123".to_string());
    ///
    /// try_ctor!(GenericError, Data::<_> {
    ///     foo: try_foo, // emplaces into the field using `TryNew`
    ///     bar,          // emplaces into the field using `New`
    ///     baz: baz?,    // use `?` operator and move the value regularly
    /// });
    /// ```
    ///
    /// Note the explicit turbofish operator (`::<_>`) to specify generic
    /// parameters of `Data`. This is a limitation of this macro; while it is
    /// not required when writing a regular struct literal, it is here.
    ///
    /// Thanks to [autoderef specialization], this macro can automatically
    /// detect whether a field should (in order of priority):
    /// - be initialized in-place using [`New`];
    /// - be fallibly initialized in-place using [`TryNew`] with a an error type
    ///   that is convertible to the declared error type, like the `?` operator
    ///   allows;
    /// - by moving a value into the field.
    ///
    /// ### Initialization failures
    ///
    /// If initialization errors or panics before all fields are initialized,
    /// they will be dropped in reverse order of declaration. This mimics the
    /// behavior of regular struct literal expressions.
    ///
    /// [autoderef specialization]: https://lukaskalbertodt.github.io/2019/12/05/generalized-autoref-based-specialization.html
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
                    TryViaPin as _,
                    TryViaPut as _,
                    TryViaNever as _,
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
                &&&&&$crate::ctor::__private::TryCtorSpec::<_, _, _, $err, _>::new(
                    &$fields.$field, &$field, $crate::ctor::__private::returns_never
                )
            ).try_init($fields.$field, $field)?;
        };

        (@assign_field[$err:ty, $fields:ident]($(#[$attrs:meta])* $field:ident: $expr:expr)) => {
            $(#[$attrs])*
            let mut $field = match $expr {
                __expr => (
                    &&&&&$crate::ctor::__private::TryCtorSpec::<_, _, _, $err, _>::new(
                        &$fields.$field, &__expr, $crate::ctor::__private::returns_never
                    )
                ).try_init($fields.$field, __expr)?
            };
        };
    }
);

#[doc(inline)]
pub use try_ctor;

#[doc(hidden)]
pub mod __private {
    pub use core::convert::Infallible;
    pub use core::marker::PhantomData;
    pub use core::mem::forget;
    pub use core::pin::Pin;

    use crate::ctor::{Init, PinInit, Uninit};
    use crate::{New, TryNew};

    pub trait UnpinField {}

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

    pub trait ViaNever<T, F, U> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> !;
    }

    impl<T, F, U> ViaNever<T, F, U> for &&&CtorSpec<T, F, U>
    where
        U: Into<Infallible>,
    {
        fn init<'a>(&self, _uninit: Uninit<'a, T, F>, src: U) -> ! {
            #[allow(unreachable_code)]
            match src.into() {}
        }
    }

    pub trait ViaPut<T, F> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Init<'a, T, F>;
    }

    impl<T, F: UnpinField> ViaPut<T, F> for &&CtorSpec<T, F, T> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Init<'a, T, F> {
            uninit.put(src)
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

    pub trait ViaPin<T, F> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> PinInit<'a, T, F>;
    }

    impl<T, F> ViaPin<T, F> for CtorSpec<T, F, T> {
        fn init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> PinInit<'a, T, F> {
            uninit.pin(src)
        }
    }

    // For TryCtorSpec never type coercion to work, we need TryViaNever to return a
    // `Result<!, E>`.
    //
    // To achieve this on stable, we make TryCtorSpec generic over an extra argument
    // N that we set to this function in the macro. We can then use a trait to
    // refer to its return type by type parameter and construct a result with
    // it.
    pub fn returns_never() -> ! {
        panic!()
    }

    #[allow(clippy::type_complexity)]
    pub struct TryCtorSpec<T, F, U, E, N>(PhantomData<fn() -> (T, F, U, E, N)>);

    impl<T, F, U, E, N> TryCtorSpec<T, F, U, E, N> {
        #[inline]
        pub fn new<'a>(_uninit: &Uninit<'a, T, F>, _src: &U, _never: N) -> Self {
            Self(PhantomData)
        }
    }

    pub trait HasReturn {
        type Ret;
    }

    impl<F: Fn() -> R, R> HasReturn for F {
        type Ret = R;
    }

    pub trait TryViaNever<T, F, U, E, N: HasReturn> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<N::Ret, E>;
    }

    impl<T, F, U, E, N: HasReturn> TryViaNever<T, F, U, E, N> for &&&&TryCtorSpec<T, F, U, E, N>
    where
        U: Into<Infallible>,
    {
        fn try_init<'a>(&self, _uninit: Uninit<'a, T, F>, src: U) -> Result<N::Ret, E> {
            #[allow(unreachable_code)]
            match src.into() {}
        }
    }

    pub trait TryViaPut<T, F, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Result<Init<'a, T, F>, E>;
    }
    impl<T, F: UnpinField, E, N> TryViaPut<T, F, E> for &&&TryCtorSpec<T, F, T, E, N> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Result<Init<'a, T, F>, E> {
            Ok(uninit.put(src))
        }
    }

    pub trait TryViaEmplace<T, F, U, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<PinInit<'a, T, F>, E>;
    }
    impl<T, F, U, E, N> TryViaEmplace<T, F, U, E> for &&TryCtorSpec<T, F, U, E, N>
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
    impl<T, F, U, E, N> TryViaTryEmplace<T, F, U, E> for &TryCtorSpec<T, F, U, E, N>
    where
        U: TryNew<Output = T, Error: Into<E>>,
    {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: U) -> Result<PinInit<'a, T, F>, E> {
            uninit.try_emplace(src).map_err(Into::into)
        }
    }

    pub trait TryViaPin<T, F, E> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Result<PinInit<'a, T, F>, E>;
    }
    impl<T, F, E, N> TryViaPin<T, F, E> for TryCtorSpec<T, F, T, E, N> {
        fn try_init<'a>(&self, uninit: Uninit<'a, T, F>, src: T) -> Result<PinInit<'a, T, F>, E> {
            Ok(uninit.pin(src))
        }
    }
}
