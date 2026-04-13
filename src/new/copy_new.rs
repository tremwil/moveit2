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

use core::ops::Deref;

use crate::new;
use crate::new::New;

/// A copy constructible type: a destination-aware `Clone`.
pub trait CopyNew: Sized {
    /// Copy-construct `src`, returning a [`New`] that creates the same value
    /// in a different memory location.
    fn copy_new(&self) -> impl New<Self>;
}

/// Returns a [`New`] that copy-constructs an [`Unpin`] value by cloning it.
pub fn trivial_copy<P>(ptr: P) -> impl New<P::Target>
where
    P: Deref<Target: Clone + Unpin>,
{
    new::by(move || ptr.clone())
}

/// Returns a new `New` that uses a copy constructor.
#[inline]
pub fn copy<P>(ptr: P) -> impl New<P::Target>
where
    P: Deref<Target: CopyNew>,
{
    unsafe { new::by_raw(move |this| ptr.copy_new().new(this)) }
}
