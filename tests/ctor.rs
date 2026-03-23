use moveit2::{Ctor, InitProof, New};
use pin_project::pin_project;

#[derive(Ctor)]
#[pin_project]
pub struct SelfReferential<T> {
    #[pin]
    val: T,
    addr_of_val: *mut T,
}

// impl<T> SelfReferential<T> {
//     pub fn construct(val_ctor: impl New<Output = T>) -> impl New<Output = Self> {
//         Self::ctor(|b| {
//             let addr = b.val.as_ptr().cast_mut();
//             b.val(val_ctor).addr_of_val(new::of(addr))
//         })
//     }
// }

impl<T> SelfReferential<T> {
    pub fn construct(val_ctor: impl New<Output = T>) -> impl New<Output = Self> {
        Self::ctor(|fields| InitProof::<Self> {
            addr_of_val: fields.addr_of_val.put(fields.val.as_ptr().cast_mut()),
            val: fields.val.emplace(val_ctor),
        })
    }
}

// macro_rules! init {
//     ($s:ty {
//         $($field:ident: $e:expr),*
//         $(,)?
//     }) => {
//         <$s>::ctor(|__fields| InitProof::<$s> {
//             $($field: __fields.$field.emplace($e)),*
//         })
//     };
// }

// macro_rules! _init_block {
//     () => {

//     };
// }

// fn test(val_ctor: impl New<Output = usize>) {
//     let s = Box::emplace(init!(SelfReferential<_> {
//         val: val_ctor,
//         addr_of_val: new::of(null_mut())
//     }));
// }

// fn test(x: impl TryNew<Output = usize, Error = usize>) -> impl TryNew<Output = usize> {
//     let n = x.with(|val| match val.is_multiple_of(2) {
//         true => Ok(()),
//         false => Err(*val),
//     });

//     n
// }
