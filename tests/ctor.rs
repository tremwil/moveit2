use std::{cell::Cell, ptr::null_mut};

use moveit2::{Ctor, Emplace, ctor, new, try_ctor};
use pin_project::pin_project;

#[derive(Debug, Ctor)]
#[pin_project]
pub struct SelfRef<T> {
    #[pin]
    val: T,
    val_ptr: *mut T,
}

/// Check that the macro compiles without error for structs where idents
/// have been specifically chosen to conflict with implementation details
#[allow(unused)]
fn adversarial_idents() {
    // conflicts with default generic idents for generated types/fns
    #[derive(Ctor)]
    struct ConflictingTyParams<'lt, C, E> {
        r#value: &'lt (E, C),
    }
    // conflict between generated marker type and struct name
    #[derive(Ctor)]
    struct __ConflictingMarker {
        conflicting_marker: usize,
    }
    // conflict between marker type and generic ty name
    #[derive(Ctor)]
    struct ConflictingMarkerParam<__Marker> {
        marker: __Marker,
    };

    // Conflict between generic tys and projection types
    #[derive(Ctor)]
    struct ConflictProjTy<ConflictProjTyFields, ConflictProjTyProof> {
        x: ConflictProjTyFields,
        y: ConflictProjTyProof,
    }

    struct __M;
    struct IndirectConflictFields;

    // Indirectly conflict between referred types and generated marker/projection
    // types
    #[derive(Ctor)]
    struct IndirectConflict<'a, __T: From<IndirectConflictFields>> {
        m: Box<__M>,
        t: &'a (__M, __T),
    }
}

#[test]
fn basic_construction() {
    let t = new::by(|| "abcdef");

    let self_ref = Box::emplace(ctor!(SelfRef::<_> {
        val: t,
        val_ptr: &raw mut *val,
    }));

    assert_eq!(&raw const self_ref.val, self_ref.val_ptr);
}

#[test]
#[allow(unreachable_code, clippy::diverging_sub_expression)]
fn never_type_inference() {
    let _ = ctor!(SelfRef::<usize> {
        val: panic!(),
        val_ptr: null_mut()
    });
    let always_fails = try_ctor!(
        _,
        SelfRef::<usize> {
            val: return Err("nope"),
            val_ptr: null_mut()
        }
    );
    Box::try_emplace(always_fails).unwrap_err();
}

#[test]
fn try_ctor_type_specialization() {
    struct GenericError;
    struct SpecialError;

    impl From<SpecialError> for GenericError {
        fn from(_value: SpecialError) -> Self {
            Self
        }
    }

    let _ = try_ctor!(
        GenericError,
        SelfRef::<&str> {
            val: "abc",
            val_ptr: &raw mut *val
        }
    );

    let _ = try_ctor!(
        GenericError,
        SelfRef::<&str> {
            val: new::try_by(|| Ok::<_, SpecialError>("abc")),
            val_ptr: &raw mut *val
        }
    );

    let _ = try_ctor!(
        GenericError,
        SelfRef::<&str> {
            val: new::of("abc"),
            val_ptr: &raw mut *val
        }
    );

    // Infer E = infallible though the new specialization
    let _ = try_ctor!(
        GenericError,
        SelfRef::<&str> {
            val: new::try_by(|| Ok("abc")),
            val_ptr: &raw mut *val
        }
    );
}

#[test]
fn closure_syntax_variations() {
    let _ = ctor!(SelfRef::<_> {
        val: 42,
        val_ptr: null_mut()
    });

    let _ = ctor!(|| SelfRef::<_> {
        val: 42,
        val_ptr: null_mut()
    });

    let _ = ctor!(move || SelfRef::<_> {
        val: 42,
        val_ptr: null_mut()
    });

    let _ = ctor!(|fields| SelfRef::<_> {
        val_ptr: fields.val.as_ptr(),
        val: 42,
    });

    let _ = ctor!(move |fields| SelfRef::<_> {
        val_ptr: fields.val.as_ptr(),
        val: 42,
    });

    let _ = ctor!({
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = ctor!({
        let _stmt = 123;
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = ctor!(|| {
        let _stmt = 123;
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = ctor!(move || {
        let _stmt = 123;
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = ctor!(|fields| {
        let val_ptr = fields.val.as_ptr();
        SelfRef::<_> { val_ptr, val: 42 }
    });

    let _ = ctor!(move |fields| {
        let val_ptr = fields.val.as_ptr();
        SelfRef::<_> { val_ptr, val: 42 }
    });
}

#[test]
fn closure_syntax_variations_try() {
    let _ = try_ctor!(
        (),
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut()
        }
    );

    let _ = try_ctor!((), || SelfRef::<_> {
        val: 42,
        val_ptr: null_mut()
    });

    let _ = try_ctor!((), move || SelfRef::<_> {
        val: 42,
        val_ptr: null_mut()
    });

    let _ = try_ctor!((), |fields| SelfRef::<_> {
        val_ptr: fields.val.as_ptr(),
        val: 42,
    });

    let _ = try_ctor!((), move |fields| SelfRef::<_> {
        val_ptr: fields.val.as_ptr(),
        val: 42,
    });

    let _ = try_ctor!((), {
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = try_ctor!((), {
        let _stmt = 123;
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = try_ctor!((), || {
        let _stmt = 123;
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = try_ctor!((), move || {
        let _stmt = 123;
        SelfRef::<_> {
            val: 42,
            val_ptr: null_mut(),
        }
    });

    let _ = try_ctor!((), |fields| {
        let val_ptr = fields.val.as_ptr();
        SelfRef::<_> { val_ptr, val: 42 }
    });

    let _ = try_ctor!((), move |fields| {
        let val_ptr = fields.val.as_ptr();
        SelfRef::<_> { val_ptr, val: 42 }
    });
}

#[test]
#[should_panic]
fn init_fields_drop_on_panic() {
    let dropped = Cell::new(false);
    struct SetDropped<'a>(&'a Cell<bool>);
    impl Drop for SetDropped<'_> {
        fn drop(&mut self) {
            self.0.set(true);
        }
    }

    struct AssertDropped<'a>(&'a Cell<bool>);
    impl Drop for AssertDropped<'_> {
        fn drop(&mut self) {
            if !self.0.get() {
                // if this runs during unwind, a double panic
                // will occur and abort the entire test
                panic!("value not dropped")
            }
        }
    }

    let _assert = AssertDropped(&dropped);

    #[allow(unreachable_code, clippy::diverging_sub_expression)]
    let panics = ctor!(SelfRef::<_> {
        val: SetDropped(&dropped),
        val_ptr: panic!("intentional unwind")
    });

    let _ = Box::emplace(panics);
}

#[cfg(moveit2_no_drop_flag_abort)]
#[test]
#[should_panic]
fn pin_leak_aborts() {
    use moveit2::try_ctor;

    #[allow(unreachable_code)]
    let _ = Box::try_emplace(try_ctor!(
        (),
        SelfRef::<_> {
            val: 42,
            val_ptr: {
                std::mem::forget(val);
                return Err(());
            },
        }
    ));
}
