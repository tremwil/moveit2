pub mod outer {
    use moveit2_proc_macros::Ctor;

    #[derive(Ctor)]
    pub struct Test<T>
    where
        T: Clone,
    {
        /// Foo doc.
        #[cfg(abc)]
        pub foo: usize,
        /// Bar doc.
        pub(super) bar: T,
        /// Baz doc 1
        /// Baz doc 2
        pub(crate) baz: String,
    }
}
