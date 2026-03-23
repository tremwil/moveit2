// use moveit2::{New, TryNew, ctor, new};
// use pin_project::pin_project;

// //#[ctor]
// #[pin_project]
// pub struct SelfReferential<T> {
//     #[pin]
//     val: T,
//     addr_of_val: *mut T,
// }

// // impl<T> SelfReferential<T> {
// //     pub fn construct(val_ctor: impl New<Output = T>) -> impl New<Output = Self> {
// //         Self::ctor(|b| {
// //             let addr = b.val.as_ptr().cast_mut();
// //             b.val(val_ctor).addr_of_val(new::of(addr))
// //         })
// //     }
// // }

// // impl<T> SelfReferential<T> {
// //     pub fn construct(val_ctor: impl New<Output = T>) -> impl New<Output = Self> {
// //         Self::ctor(|mut fields| Builder::<Self> {
// //             addr_of_val: ctor::emplace(val_ctor),
// //             val: ctor::write(fields.val.as_ptr()),
// //         })
// //     }
// // }
// 2
// fn test(x: impl TryNew<Output = usize, Error = usize>) -> impl TryNew<Output = usize> {
//     let n = x.with(|val| match val.is_multiple_of(2) {
//         true => Ok(()),
//         false => Err(*val),
//     });

//     n
// }
