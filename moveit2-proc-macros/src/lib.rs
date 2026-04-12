use std::collections::HashSet;
use std::fmt::Display;

use inflections::Inflect;
use proc_macro::TokenStream;
use proc_macro2::Span as Span2;
use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, quote};

mod attr_helpers;
mod vis_helpers;

fn spanned_error<T, U, Ok>(tokens: T, message: U) -> syn::Result<Ok>
where
    T: quote::ToTokens,
    U: Display,
{
    Err(syn::Error::new_spanned(tokens, message))
}

struct ProjField {
    field: syn::Field,
    marker_ident: syn::Ident,
    pinned: bool,
}

impl ProjField {
    pub fn new(mut field: syn::Field, unsafe_idents: &HashSet<String>) -> Self {
        let ident = field.ident.as_ref().unwrap();
        let ident_pascal = ident.to_string().to_pascal_case();
        let marker_ident = syn::Ident::new(
            &Context::make_safe_name(&format!("__{ident_pascal}"), |s| unsafe_idents.contains(s)),
            ident.span(),
        );

        let pinned = field.attrs.iter().any(|attr| attr.path().is_ident("pin"));

        // strip all non-built-in attributes
        field
            .attrs
            .retain(|attr| attr_helpers::is_builtin_field_attr(&attr.meta));

        ProjField {
            field,
            marker_ident,
            pinned,
        }
    }

    pub fn marker_def(&self) -> TokenStream2 {
        let vis = &self.field.vis;
        let marker = &self.marker_ident;

        let unpin_impl = (!self.pinned)
            .then(|| quote!(impl ::moveit2::ctor::__private::UnpinField for #marker {}));

        quote! {
            #vis struct #marker;
            #unpin_impl
        }
    }
}

struct Context {
    vis: syn::Visibility,
    ctor_vis: syn::Visibility,
    ident: syn::Ident,
    fields_ident: syn::Ident,
    proof_ident: syn::Ident,
    proj_lt: syn::Lifetime,
    generics: syn::Generics,
    proj_generics: syn::Generics,
    fields: Vec<ProjField>,
    unsafe_idents: HashSet<String>,
}

impl Context {
    pub fn new(input: syn::DeriveInput) -> syn::Result<Self> {
        let data = match input.data {
            syn::Data::Struct(data) => data,
            syn::Data::Enum(e) => {
                return spanned_error(e.enum_token, "Ctor cannot be derived on enums");
            }
            syn::Data::Union(u) => {
                return spanned_error(u.union_token, "Ctor cannot be derived on unions");
            }
        };

        let mut fields = match data.fields {
            syn::Fields::Unnamed(t) => {
                return spanned_error(
                    t,
                    "Ctor cannot be derived on tuple structs. \
                    This is a limitation that may be lifted in the future.",
                );
            }
            syn::Fields::Unit => {
                return spanned_error(data.semi_token, "Ctor cannot be derived on unit structs");
            }
            syn::Fields::Named(n) if n.named.is_empty() => {
                return spanned_error(n, "Ctor cannot be derived on empty structs");
            }
            syn::Fields::Named(n) => n.named,
        };

        let mut unsafe_idents = Self::collect_unsafe_idents(&fields, &input.generics);
        unsafe_idents.insert(input.ident.to_string());

        let ident = input.ident;
        let fields_ident = syn::Ident::new(
            &Self::make_safe_name(&format!("{}Fields", ident), |s| unsafe_idents.contains(s)),
            ident.span(),
        );
        let proof_ident = syn::Ident::new(
            &Self::make_safe_name(&format!("{}Proof", ident), |s| unsafe_idents.contains(s)),
            ident.span(),
        );
        let proj_lt: syn::Lifetime = Self::make_safe_lt("lt", |s| {
            input.generics.lifetimes().any(|lt| lt.lifetime.ident == s)
        });
        let ctor_vis = fields.iter().fold(input.vis.clone(), |acc, f| {
            vis_helpers::vis_min(&acc, &f.vis)
        });
        for f in &mut fields {
            f.vis = ctor_vis.clone();
        }

        let fields: Vec<_> = fields
            .into_iter()
            .map(|f| ProjField::new(f, &unsafe_idents))
            .collect();

        let generics = input.generics;
        let mut proj_generics = generics.clone();
        proj_generics.params.insert(
            0,
            syn::GenericParam::Lifetime(syn::LifetimeParam::new(proj_lt.clone())),
        );

        Ok(Self {
            vis: input.vis,
            ctor_vis,
            ident,
            fields_ident,
            proof_ident,
            proj_lt,
            generics,
            proj_generics,
            fields,
            unsafe_idents,
        })
    }

    fn collect_unsafe_idents<'a>(
        fields: impl IntoIterator<Item = &'a syn::Field>,
        generics: &syn::Generics,
    ) -> HashSet<String> {
        struct Visitor(HashSet<String>);
        impl<'ast> syn::visit::Visit<'ast> for Visitor {
            fn visit_type_path(&mut self, i: &'ast syn::TypePath) {
                if i.path.leading_colon.is_none() && i.path.segments.len() == 1 {
                    self.0
                        .insert(i.path.segments.first().unwrap().ident.to_string());
                }
                syn::visit::visit_type_path(self, i);
            }

            fn visit_type_param(&mut self, i: &'ast syn::TypeParam) {
                self.0.insert(i.ident.to_string());
                syn::visit::visit_type_param(self, i);
            }
        }
        let mut v = Visitor(HashSet::new());
        fields
            .into_iter()
            .for_each(|f| syn::visit::visit_field(&mut v, f));
        syn::visit::visit_generics(&mut v, generics);
        v.0
    }

    fn make_safe_name(name: &str, exists: impl Fn(&str) -> bool) -> String {
        let mut name = name.to_owned();
        while exists(&name) {
            name = format!("_{name}");
        }
        name
    }

    fn make_safe_lt(name: &str, exists: impl Fn(&str) -> bool) -> syn::Lifetime {
        syn::Lifetime::new(
            &format!("'{}", Self::make_safe_name(name, exists)),
            Span2::call_site(),
        )
    }

    fn make_safe_ident(name: &str, exists: impl Fn(&str) -> bool) -> syn::Ident {
        syn::Ident::new(&Self::make_safe_name(name, exists), Span2::call_site())
    }

    pub fn proj_field_def(&self, field: &ProjField, init: bool) -> TokenStream2 {
        let vis = &field.field.vis;
        let ident = &field.field.ident;
        let marker = &field.marker_ident;

        let attrs = &field.field.attrs;
        let ty = &field.field.ty;
        let lt = &self.proj_lt;

        let path = if init && field.pinned {
            quote!(::moveit2::ctor::PinInit)
        } else if init {
            quote!(::moveit2::ctor::Init)
        } else {
            quote!(::moveit2::ctor::Uninit)
        };

        quote! {
            #(#attrs)*
            #vis #ident: #path<#lt, #ty, #marker>,
        }
    }

    fn proj_def(&self, init: bool) -> TokenStream2 {
        let vis = &self.vis;
        let ident = if init {
            &self.proof_ident
        } else {
            &self.fields_ident
        };

        let field_defs = self.fields.iter().map(|f| self.proj_field_def(f, init));
        let generics = &self.proj_generics;
        let where_clause = &generics.where_clause;

        quote! {
            #vis struct #ident #generics #where_clause {
                #(#field_defs)*
            }
        }
    }

    fn ctor_impl(&self) -> TokenStream2 {
        let ident = &self.ident;
        let fields_ident = &self.fields_ident;
        let proof_ident = &self.proof_ident;
        let lt = &self.proj_lt;
        let (impl_gen, ty_gen, where_clause) = self.generics.split_for_impl();
        let (_, ty_proj_gen, _) = self.proj_generics.split_for_impl();

        quote! {
            #[automatically_derived]
            impl #impl_gen ::moveit2::ctor::Ctor for #ident #ty_gen #where_clause {
                type Fields<#lt> = #fields_ident #ty_proj_gen where Self: #lt;
                type Proof<#lt> = #proof_ident #ty_proj_gen where Self: #lt;
            }
        }
    }

    fn ctor_fn(&self) -> TokenStream2 {
        let vis = &self.ctor_vis;
        let ident = &self.ident;
        let fields_ident = &self.fields_ident;
        let proof_ident = &self.proof_ident;
        let lt = &self.proj_lt;

        let (s_impl, s_ty, s_where) = self.generics.split_for_impl();
        let s_ty_unpacked: Vec<_> = self
            .generics
            .params
            .iter()
            .map(|t| match t {
                syn::GenericParam::Const(c) => c.ident.to_token_stream(),
                syn::GenericParam::Lifetime(lt) => lt.lifetime.to_token_stream(),
                syn::GenericParam::Type(ty) => ty.ident.to_token_stream(),
            })
            .collect();

        let field_assignments = self.fields.iter().map(|f| {
            let ident = &f.field.ident;
            let cfgs = f
                .field
                .attrs
                .iter()
                .filter(|attr| attr.path().is_ident("cfg"));

            quote!(
                #(#cfgs)* #ident: ::moveit2::ctor::Uninit::new_unchecked(&raw mut (*ptr).#ident, trap.flag()),
            )
        });

        let ctor_ty = Self::make_safe_ident("F", |s| self.unsafe_idents.contains(s));
        let err_ty = Self::make_safe_ident("E", |s| self.unsafe_idents.contains(s));

        quote! {
            impl #s_impl #ident #s_ty #s_where {
                /// Create a [`New`](::moveit2::New) implementation that in-place initializes each field
                /// of this struct, using purely safe code.
                ///
                /// This is done by providing a *proof of initialization*: A closure that takes
                /// [`Self::Fields<'_>`] (the fields of the struct as [`Uninit`] slots), and
                /// returns a [`Self::Proof<'_>`] (the initialized fields as [`Init`] values).
                ///
                /// This function was generated by the [`Ctor`] derive macro. For more information see
                /// the [moveit2 `ctor` documentation](::moveit2::ctor).
                ///
                /// [`Self::Fields<'a>`]: ::moveit2::ctor::Ctor::Fields
                /// [`Self::Proof<'a>`]: ::moveit2::ctor::Ctor::Proof
                /// [`Uninit`]: ::moveit2::ctor::Uninit
                /// [`Init`]: ::moveit2::ctor::Init
                #vis fn ctor<#ctor_ty>(ctor: #ctor_ty) -> impl ::moveit2::New<Self>
                where
                    #ctor_ty: for<#lt> FnOnce(#fields_ident<#lt, #(#s_ty_unpacked,)*>) ->
                        #proof_ident<#lt, #(#s_ty_unpacked,)*>
                {
                    Self::try_ctor::<::moveit2::ctor::__private::Infallible, _>(|fields| Ok(ctor(fields)))
                }

                /// Create a [`TryNew`](::moveit2::TryNew) implementation that in-place initializes
                /// each field of this struct, using purely safe code.
                ///
                /// This is done by providing a *proof of initialization*: A closure that takes
                /// [`Self::Fields<'_>`] (the fields of the struct as [`Uninit`] slots), and
                /// fallibly returns a [`Self::Proof<'_>`] (the initialized fields as [`Init`] values).
                ///
                /// Note that the usual limitations around type inference for errors in closures apply,
                /// so you will probably have to specify the error type explicitly. This can be done
                /// by annotating the `try_ctor` function itself, the closure's return type, or the
                /// final `Ok`.
                ///
                /// See [the module-level documentation](::moveit2::ctor) for more information.
                ///
                /// [`Self::Fields<'a>`]: ::moveit2::ctor::Ctor::Fields
                /// [`Self::Proof<'a>`]: ::moveit2::ctor::Ctor::Proof
                /// [`Uninit`]: ::moveit2::ctor::Uninit
                /// [`Init`]: ::moveit2::ctor::Init
                #vis fn try_ctor<#err_ty, #ctor_ty>(ctor: #ctor_ty) ->
                    impl ::moveit2::TryNew<Self, Error = #err_ty>
                where
                    #ctor_ty: for<#lt> FnOnce(#fields_ident<#lt, #(#s_ty_unpacked,)*>) ->
                        Result<#proof_ident<#lt, #(#s_ty_unpacked,)*>, #err_ty>
                {
                    unsafe {
                        ::moveit2::new::try_by_raw::<Self, #err_ty, _>(|slot| {
                            let trap = ::moveit2::drop_flag::TrappedFlag::new();
                            let ptr = ::moveit2::ctor::__private::Pin::into_inner_unchecked(slot)
                                .as_mut_ptr();

                            let fields = #fields_ident {
                                #(#field_assignments)*
                            };
                            let proof = ctor(fields)?;

                            ::moveit2::ctor::__private::forget(proof);
                            ::moveit2::ctor::__private::forget(trap);
                            Ok(())
                        })
                    }
                }
            }
        }
    }

    fn generate(self) -> TokenStream2 {
        let marker_defs = self.fields.iter().map(|f| f.marker_def());
        let fields_struct = self.proj_def(false);
        let proof_struct = self.proj_def(true);
        let ctor_impl = self.ctor_impl();
        let ctor_fn = self.ctor_fn();

        quote! {
            const _: () = {
                #(#marker_defs)*
                #fields_struct
                #proof_struct
                #ctor_impl
                #ctor_fn
            };
        }
    }
}

#[proc_macro_derive(Ctor, attributes(pin))]
pub fn ctor_derive(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    match Context::new(input) {
        Ok(ctx) => ctx.generate(),
        Err(e) => e.into_compile_error(),
    }
    .into()
}
