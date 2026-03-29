use moveit2::{Ctor, Emplace, ctor, new, try_ctor};
use pin_project::pin_project;

#[derive(Ctor)]
#[pin_project]
pub struct SelfRef<T> {
    #[pin]
    val: T,
    addr_of_val: *mut T,
}

#[test]
fn basic_construction() {
    let t = new::by(|| "abcdef");

    let self_ref = Box::emplace(ctor!(SelfRef::<_> {
        val: t,
        addr_of_val: &raw mut *val,
    }));

    assert_eq!(&raw const self_ref.val, self_ref.addr_of_val);
}

#[cfg(moveit2_no_drop_flag_abort)]
#[test]
#[should_panic]
fn pin_leak_aborts() {
    let _ = Box::try_emplace(try_ctor!(
        (),
        SelfRef::<_> {
            val: 42,
            addr_of_val: {
                std::mem::forget(val);
                return Err(());
            },
        }
    ));
}
