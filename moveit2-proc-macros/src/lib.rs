use std::fmt::Display;

use inflections::Inflect;
use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::parse_quote;

mod attr_helpers;
mod vis_helpers;

fn spanned_error<T, U, Ok>(tokens: T, message: U) -> syn::Result<Ok>
where
    T: quote::ToTokens,
    U: Display,
{
    Err(syn::Error::new_spanned(tokens, message))
}

struct BuilderField {
    field: syn::Field,
    marker_ident: syn::Ident,
}

impl BuilderField {
    pub fn new(mut field: syn::Field) -> Self {
        let ident = field.ident.as_ref().unwrap();
        let ident_pascal = ident.to_string().to_pascal_case();
        let marker_ident = syn::Ident::new(&format!("_{ident_pascal}_"), ident.span());

        // strip all non-built-in attributes
        field
            .attrs
            .retain(|attr| attr_helpers::is_builtin_field_attr(&attr.meta));

        BuilderField {
            field,
            marker_ident,
        }
    }

    pub fn marker_def(&self) -> TokenStream2 {
        let vis = &self.field.vis;
        let marker = &self.marker_ident;
        quote!(#vis struct #marker;)
    }

    pub fn proj_field_def(&self, init: bool) -> TokenStream2 {
        let vis = &self.field.vis;
        let ident = &self.field.ident;
        let marker = &self.marker_ident;

        let attrs = &self.field.attrs;
        let ty = &self.field.ty;

        let path = if init {
            quote!(::moveit2::ctor::PinInit)
        } else {
            quote!(::moveit2::ctor::Uninit)
        };

        quote! {
            #(#attrs)*
            #vis #ident: #path<'__ctor, #ty, #marker>,
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
    fields: Vec<BuilderField>,
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

        let ident = input.ident;
        let fields_ident = format_ident!("{}Fields", ident);
        let proof_ident = format_ident!("{}Proof", ident);
        let proj_lt: syn::Lifetime = parse_quote!('__ctor);
        let ctor_vis = fields.iter().fold(input.vis.clone(), |acc, f| {
            vis_helpers::vis_min(&acc, &f.vis)
        });
        for f in &mut fields {
            f.vis = ctor_vis.clone();
        }

        let fields: Vec<_> = fields.into_iter().map(BuilderField::new).collect();

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
        })
    }

    fn proj_def(&self, init: bool) -> TokenStream2 {
        let vis = &self.vis;
        let ident = if init {
            &self.proof_ident
        } else {
            &self.fields_ident
        };

        let field_defs = self.fields.iter().map(|f| f.proj_field_def(init));
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

        quote! {
            impl #s_impl #ident #s_ty #s_where {
                #vis fn ctor<__C>(ctor: __C) -> impl ::moveit2::New<Output = Self>
                where
                    __C: FnOnce(#fields_ident<'_, #(#s_ty_unpacked,)*>) ->
                        #proof_ident<'_, #(#s_ty_unpacked,)*>
                {
                    Self::try_ctor::<_, ::moveit2::ctor::__private::Infallible>(|fields| Ok(ctor(fields)))
                }

                #vis fn try_ctor<__C, __E>(ctor: __C) -> impl ::moveit2::TryNew<Output = Self, Error = __E>
                where
                    __C: FnOnce(#fields_ident<'_, #(#s_ty_unpacked,)*>) ->
                        Result<#proof_ident<'_, #(#s_ty_unpacked,)*>, __E>
                {
                    unsafe {
                        ::moveit2::new::try_by_raw::<Self, __E, _>(|slot| {
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

#[proc_macro_derive(Ctor)]
pub fn ctor_derive(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    match Context::new(input) {
        Ok(ctx) => ctx.generate(),
        Err(e) => e.into_compile_error(),
    }
    .into()
}
