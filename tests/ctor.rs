use moveit2::{Ctor, Emplace, InitProof, New, ctor};
use pin_project::pin_project;

#[derive(Ctor)]
#[pin_project]
pub struct SelfReferential<T> {
    #[pin]
    val: T,
    addr_of_val: *mut T,
}

impl<T> SelfReferential<T> {
    pub fn construct(val_ctor: impl New<Output = T>) -> impl New<Output = Self> {
        Self::ctor(|fields| InitProof::<Self> {
            addr_of_val: fields.addr_of_val.put(fields.val.as_ptr().cast_mut()),
            val: fields.val.emplace(val_ctor),
        })
    }
}

#[test]
fn basic_construction() {
    let self_ref = Box::emplace(ctor!(|fields| {
        SelfReferential::<_> {
            val: "abcdef",
            addr_of_val: &raw mut *val,
        }
    }));

    assert_eq!(&raw const self_ref.val, self_ref.addr_of_val);
}
