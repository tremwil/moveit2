// Copyright 2021 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! In-place constructors.
//!
//! This module provides a range of helpers such as [`new::by()`] and
//! [`new::from()`] for creating constructors. It is preferred style to
//! `use moveit2::new;` and refer to these helpers with a `new::` prefix.

use core::convert::Infallible;
use core::mem::MaybeUninit;
use core::ops::Deref;
use core::pin::Pin;

#[cfg(doc)]
use {crate::new, core::ops::DerefMut};

#[cfg(feature = "alloc")]
use alloc::{boxed::Box, rc::Rc, sync::Arc};

mod copy_new;
mod default_new;
mod factories;
mod move_new;

mod impls;

pub use copy_new::*;
pub use default_new::*;
pub use factories::*;
pub use move_new::*;

/// An in-place constructor for a particular type, which can potentially fail.
///
/// Emplacing a `TryNew<T>` may allocate even when construction fails; prefer to
/// use `Result<impl New<T>>` when possible, instead.
///
/// # Safety
///
/// - [`TryNew::try_new()`] must leave its destination argument in a valid,
///   initialized state when it returns `Ok`.
///
/// - If [`TryNew::try_new()`] successfully initializes its argument and
///   constructs a pinning reference to it, it is considered to own the value
///   until returning `Ok`. This has soundness implications for `!Unpin` types:
///   a `this` that has been exposed as a `Pin<Ptr<Self::Output>>` *must* be
///   dropped if the function panics or returns `Err`.
#[must_use = "`New`s do nothing until emplaced into storage"]
pub unsafe trait TryNew<T>: Sized {
    /// The error the construction operation may return.
    type Error;
    /// Try to construct a new value using the arguments stored in `self`.
    ///
    /// # Safety
    ///
    /// `this` must be freshly-created memory; this function must not
    /// be used to mutate a previously-pinned pointer that has had `self: Pin`
    /// functions called on it.
    unsafe fn try_new(self, this: Pin<&mut MaybeUninit<T>>) -> Result<(), Self::Error>;

    /// Adds a (potentially fallible) post-construction operation.
    ///
    /// The operation can either return nothing or a [`Result<(),
    /// Self::Error>`](TryNew::Error).
    ///
    /// This function wraps `self` in an another [`TryNew`] type which will call
    /// `post` once the main emplacement operation is complete. This is most
    /// useful for the case where creation of the value itself does not
    /// depend on the final address, but where some address-sensitive setup
    /// may want to occur; this can help minimize the scope (or even need
    /// for) `unsafe`.
    ///
    /// This function is best combined with other helpers:
    ///
    /// ```
    /// # use moveit2::{new, moveit, New, TryNew};
    /// # use std::pin::Pin;
    /// pub struct MyType { /* ... */ }
    ///
    /// impl MyType {
    ///     pub fn new() -> impl New<Self> {
    ///         new::of(MyType { /* ... */ }).with(|this| {
    ///         // Address-sensitive setup can occur here.
    ///     })
    ///   }
    /// }
    /// ```
    fn with<F, R>(self, post: F) -> With<Self, F>
    where
        F: FnOnce(Pin<&mut T>) -> R,
        R: IntoResult<Self::Error>,
    {
        With(self, post)
    }
}

/// An in-place constructor for a particular type.
///
/// This is an extension trait for [`TryNew<T, Error = Infallible>`].
///
/// # Safety
///
/// [`New::new()`] must leave its destination argument in a valid, initialized
/// state.
#[must_use = "`New`s do nothing until emplaced into storage"]
pub unsafe trait New<T>: TryNew<T, Error = Infallible> {
    /// Construct a new value using the arguments stored in `self`.
    ///
    /// # Safety
    ///
    /// `this` must be freshly-created memory; this function must not
    /// be used to mutate a previously-pinned pointer that has had `self: Pin`
    /// functions called on it.
    unsafe fn new(self, this: Pin<&mut MaybeUninit<T>>) {
        unsafe { self.try_new(this) }.unwrap_or_else(|e| match e {})
    }
}

unsafe impl<T, N: TryNew<T, Error = Infallible>> New<T> for N {}

/// A pointer type that may be "emplaced" as a stable address which a [`New`]
/// may be used to construct a value with.
///
/// The `Emplace<T>::Output` type is usually either `Self` or `Pin<Self>`
/// depending on the API of `Self` with respect to [`DerefMut`].
///
/// For example, `Arc<T>`, `Box<T>`, and `Rc<T>` are all `Emplace<T, Output =
/// Pin<Self>>`.
///
/// However, `cxx::UniquePtr<T>: Emplace<T, Output = Self>`, since
/// `cxx::UniquePtr<T>` already only allows obtaining pinned mutable references
/// to `T` due to its more restrictive API, and hence `cxx::UniquePtr<T>` does
/// not need to be pinned itself.
pub trait Emplace<T>: Sized + Deref {
    /// The stable address type within which a value of type `T` is emplaced.
    type Output: Deref<Target = Self::Target>;

    /// Constructs a new smart pointer and emplaces `n` into its storage.
    fn emplace<N: New<T>>(n: N) -> Self::Output {
        Self::try_emplace(n).unwrap_or_else(|e| match e {})
    }

    /// Constructs a new smart pointer and tries to emplace `n` into its
    /// storage.
    fn try_emplace<N: TryNew<T>>(n: N) -> Result<Self::Output, N::Error>;
}

#[cfg(feature = "alloc")]
mod alloc_support {
    use super::*;

    impl<T> Emplace<T> for Box<T> {
        type Output = Pin<Self>;

        fn try_emplace<N: TryNew<T>>(n: N) -> Result<Self::Output, N::Error> {
            let mut uninit = Box::new(MaybeUninit::<T>::uninit());
            unsafe {
                let pinned = Pin::new_unchecked(&mut *uninit);
                n.try_new(pinned)?;
                Ok(Pin::new_unchecked(Box::from_raw(
                    Box::into_raw(uninit).cast::<T>(),
                )))
            }
        }
    }

    impl<T> Emplace<T> for Rc<T> {
        type Output = Pin<Self>;

        fn try_emplace<N: TryNew<T>>(n: N) -> Result<Self::Output, N::Error> {
            let uninit = Rc::new(MaybeUninit::<T>::uninit());
            unsafe {
                let pinned = Pin::new_unchecked(&mut *(Rc::as_ptr(&uninit) as *mut _));
                n.try_new(pinned)?;
                Ok(Pin::new_unchecked(Rc::from_raw(
                    Rc::into_raw(uninit).cast::<T>(),
                )))
            }
        }
    }

    impl<T> Emplace<T> for Arc<T> {
        type Output = Pin<Self>;

        fn try_emplace<N: TryNew<T>>(n: N) -> Result<Self::Output, N::Error> {
            let uninit = Arc::new(MaybeUninit::<T>::uninit());
            unsafe {
                let pinned = Pin::new_unchecked(&mut *(Arc::as_ptr(&uninit) as *mut _));
                n.try_new(pinned)?;
                Ok(Pin::new_unchecked(Arc::from_raw(
                    Arc::into_raw(uninit).cast::<T>(),
                )))
            }
        }
    }
}

#[doc(hidden)]
pub trait IntoResult<E>: Sized {
    fn into_result(this: Self) -> Result<(), E>;
}

impl<E> IntoResult<E> for () {
    fn into_result(this: Self) -> Result<(), E> {
        Ok(this)
    }
}

impl<E> IntoResult<E> for Result<(), E> {
    fn into_result(this: Self) -> Result<(), E> {
        this
    }
}

#[doc(hidden)]
pub struct With<N, F>(N, F);

// SAFETY:
// - `this` is left in a valid state when `try_new` returns `Ok`.
unsafe impl<T, N: TryNew<T>, F, R> TryNew<T> for With<N, F>
where
    F: FnOnce(Pin<&mut T>) -> R,
    R: IntoResult<N::Error>,
{
    type Error = N::Error;

    #[inline]
    unsafe fn try_new(self, mut this: Pin<&mut MaybeUninit<T>>) -> Result<(), Self::Error> {
        // SAFETY: `this` is freshly created memory by `try_new` invariants.
        unsafe { self.0.try_new(this.as_mut())? };

        // Now that `new()` has returned, we can assume `this` is initialized.
        // However, since we own the initialized pinning reference, we *must* drop the
        // value on errors or panics!

        struct DropGuard<T>(*mut T);
        impl<T> Drop for DropGuard<T> {
            fn drop(&mut self) {
                unsafe { core::ptr::drop_in_place(self.0) };
            }
        }

        let drop_guard = DropGuard(this.as_ptr().cast_mut());

        // SAFETY:
        // - `this` is initialized
        // - if the operation fails due to `F` panicking or returning Err, `drop_guard`
        //   will be dropped, thus dropping the pointee of `this`.
        // - if the operation succeeds, `drop_guard` is forgotten so the initialized
        //   value is not mistakenly dropped.
        let this = unsafe { this.map_unchecked_mut(|x| x.assume_init_mut()) };

        IntoResult::into_result((self.1)(this)).inspect(|_| core::mem::forget(drop_guard))
    }
}

/// A swappable type, which is able to efficiently swap the contents of two of
/// its values.
///
/// Unlike [`New`], `Swap` is safe, because it does not impose any requirements
/// on the swapped pointers.
///
/// It is possible to implement swapping with a source type that isn't `Self`.
pub trait Swap<Rhs = Self> {
    /// Swaps the contents of `self` and `src` without running any destructors.
    fn swap_with(self: Pin<&mut Self>, src: Pin<&mut Rhs>);
}
