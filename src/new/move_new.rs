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

use core::mem::MaybeUninit;
use core::pin::Pin;

use crate::move_ref::AsMove;
use crate::move_ref::MoveRef;
use crate::new;
use crate::new::New;
use crate::slot;

/// A move constructible type: a destination-aware `Clone` that destroys the
/// moved-from value.
///
/// # Safety
///
/// After [`MoveNew::move_new()`] is called:
/// - `src` should be treated as having been destroyed.
/// - `this` must have been initialized.
pub unsafe trait MoveNew: Sized {
    /// Move-construct `src` into `this`, effectively re-pinning it at a new
    /// location.
    ///
    /// # Safety
    ///
    /// The same safety requirements of [`New::new()`] apply, but, in addition,
    /// `*src` must not be used after this function is called, because it has
    /// effectively been destroyed.
    unsafe fn move_new(src: Pin<MoveRef<Self>>, this: Pin<&mut MaybeUninit<Self>>);
}

/// An helper trait that generates a [`MoveNew`] implementation from a
/// return-position [`New`].
pub trait SafeMoveNew: Sized {
    /// copy-construct this value, returning a [`New`] to emplace the copy.
    fn move_new(src: Pin<MoveRef<Self>>) -> impl New<Self>;
}

unsafe impl<T: SafeMoveNew> MoveNew for T {
    unsafe fn move_new(src: Pin<MoveRef<Self>>, this: Pin<&mut MaybeUninit<Self>>) {
        unsafe { SafeMoveNew::move_new(src).new(this) }
    }
}

/// Returns a [`New`] that move-constructs an [`Unpin`] value by trivially
/// moving it.
pub fn trivial_mov<P>(ptr: P) -> impl New<P::Target>
where
    P: AsMove<Target: Sized + Unpin>,
{
    new::by(move || MoveRef::into_inner(Pin::into_inner(ptr.as_move(slot!(#[dropping])))))
}

/// Returns a [`New`] that forwards to [`MoveNew`].
///
/// ```
/// # use moveit2::{MoveRef, moveit, new};
/// let foo = Box::new(42);
/// moveit! {
///     let bar = &move foo;
///     let baz = new::mov(bar);
/// }
/// ```
#[inline]
pub fn mov<P>(ptr: P) -> impl New<P::Target>
where
    P: AsMove<Target: MoveNew>,
{
    unsafe {
        new::by_raw(move |this| {
            MoveNew::move_new(ptr.as_move(slot!(#[dropping])), this);
        })
    }
}
