use std::fmt::Display;

use inflections::Inflect;
use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::parse_quote;

mod cfg_helpers;
mod vis_helpers;

const IMPL_TY_MACRO: &str = "__moveit2_ctor_impl_ty";

fn spanned_error<T, U, Ok>(tokens: T, message: U) -> syn::Result<Ok>
where
    T: quote::ToTokens,
    U: Display,
{
    Err(syn::Error::new_spanned(tokens, message))
}

struct BuilderField {
    index: usize,
    field: syn::Field,
    ty_ident: syn::Ident,
    marker_ident: syn::Ident,
    doc_attrs: Vec<syn::Attribute>,
    cfg: Option<syn::Meta>,
    ty_param: syn::TypeParam,
}

impl BuilderField {
    pub fn new(index: usize, field: syn::Field) -> Self {
        let ident = field.ident.as_ref().unwrap();
        let ident_pascal = ident.to_string().to_pascal_case();
        let marker_ident = syn::Ident::new(&format!("__{ident_pascal}IsUninit"), ident.span());
        let ty_ident = syn::Ident::new(&format!("__{ident_pascal}"), ident.span());

        let cfg = cfg_helpers::combine_cfg(&field.attrs);
        let meta = cfg.iter();
        let ty_param = syn::parse_quote!(
            #(#[cfg(#meta)])* #ty_ident: ::moveit2::ctor::__private::IsInit = #marker_ident
        );
        BuilderField {
            index,
            marker_ident,
            ty_ident,
            doc_attrs: cfg_helpers::doc_attrs(&field.attrs),
            cfg,
            ty_param,
            field,
        }
    }

    pub fn builder_def(&self) -> TokenStream2 {
        let vis = &self.field.vis;
        let ident = &self.field.ident;
        let ty_ident = &self.ty_ident;
        let ty = &self.field.ty;
        let doc = &self.doc_attrs;
        let meta = self.cfg.iter();

        quote! {
            #(#doc)*
            #(#[cfg(#meta)])*
            #vis #ident: ::moveit2::ctor::__private::PinMaybeInit<'__ctor, #ty, #ty_ident>,
        }
    }

    pub fn impl_ty_macro_def(&self) -> TokenStream2 {
        let ident = format_ident!("{}{}", IMPL_TY_MACRO, self.index);
        let next_ident = format_ident!("{}{}", IMPL_TY_MACRO, self.index + 1);
        let meta = self.cfg.iter();
        let meta2 = self.cfg.iter();

        quote! {
            #(#[cfg(#meta)])*
            macro_rules! #ident {
                ([$($tts:tt)*]  $f:ty, $($tail:ty,)*) => {
                    #next_ident!([$($tts)* $f,] $($tail,)*)
                };
            }
            #(
                #[cfg(not(#meta2))]
                macro_rules! #ident {
                ([$($tts:tt)*]  $f:ty, $($tail:ty,)*) => {
                    #next_ident!([$($tts)*] $($tail,)*)
                };
                }
            )*
        }
    }
}

struct Context {
    vis: syn::Visibility,
    ident: syn::Ident,
    builder_ident: syn::Ident,
    ctor_lt: syn::Lifetime,
    base_generics: syn::Generics,
    builder_generics: syn::Generics,
    fields: Vec<BuilderField>,
}

impl Context {
    pub fn new(input: syn::DeriveInput) -> syn::Result<Self> {
        let data = match input.data {
            syn::Data::Struct(data) => data,
            syn::Data::Enum(e) => {
                return spanned_error(e.enum_token, "#[ctor] does not support enums");
            }
            syn::Data::Union(u) => {
                return spanned_error(u.union_token, "#[ctor] does not support unions");
            }
        };

        let mut fields = match data.fields {
            syn::Fields::Unnamed(t) => {
                return spanned_error(t, "#[ctor] does not support tuple structs");
            }
            syn::Fields::Unit => {
                return spanned_error(data.semi_token, "#[ctor] does not support unit structs");
            }
            syn::Fields::Named(n) if n.named.is_empty() => {
                return spanned_error(n, "#[ctor] does not support unit structs");
            }
            syn::Fields::Named(n) => n.named,
        };

        let ident = input.ident;
        let builder_ident = format_ident!("{}CtorBuilder", ident);
        let ctor_lt: syn::Lifetime = parse_quote!('__ctor);
        let vis = fields.iter().fold(input.vis.clone(), |acc, f| {
            vis_helpers::vis_min(&acc, &f.vis)
        });
        for f in &mut fields {
            f.vis = vis.clone();
        }

        let fields: Vec<_> = fields
            .into_iter()
            .enumerate()
            .map(|(i, f)| BuilderField::new(i, f))
            .collect();

        let mut base_generics = input.generics;
        base_generics.params.insert(
            0,
            syn::GenericParam::Lifetime(syn::LifetimeParam::new(ctor_lt.clone())),
        );

        let mut builder_generics = base_generics.clone();
        builder_generics.params.extend(
            fields
                .iter()
                .map(|f| syn::GenericParam::Type(f.ty_param.clone())),
        );

        Ok(Self {
            vis,
            ident,
            builder_ident,
            ctor_lt,
            base_generics,
            builder_generics,
            fields,
        })
    }

    fn marker_idents(&self) -> impl Iterator<Item = &syn::Ident> {
        self.fields.iter().map(|f| &f.marker_ident)
    }

    fn marker_defs(&self) -> TokenStream2 {
        let marker_idents = self.marker_idents();
        let vis = &self.vis;
        quote! {
            #(
                #vis struct #marker_idents;
                impl ::moveit2::ctor::UninitMarker for #marker_idents {}
            )*
        }
    }

    fn builder_method(&self, index: usize) -> TokenStream2 {
        let mut generics = self.base_generics.clone();
        generics.params.extend(
            self.fields
                .iter()
                .filter(|f| f.index != index)
                .map(|f| syn::GenericParam::Type(f.ty_param.clone())),
        );

        let base_tys: Vec<_> = self
            .base_generics
            .params
            .iter()
            .map(|t| match t {
                syn::GenericParam::Const(c) => c.ident.to_token_stream(),
                syn::GenericParam::Lifetime(lt) => lt.lifetime.to_token_stream(),
                syn::GenericParam::Type(ty) => ty.ident.to_token_stream(),
            })
            .collect();

        let in_builder_tys = self.fields.iter().map(|f| {
            if f.index == index {
                &f.marker_ident
            } else {
                &f.ty_ident
            }
        });

        let out_builder_tys = self.fields.iter().map(|f| {
            if f.index == index {
                parse_quote!(::moveit2::ctor::Init)
            } else {
                syn::Path::from(f.ty_ident.clone())
            }
        });

        let field_assignments = self.fields.iter().map(|f| {
            let ident = &f.field.ident;
            if f.index == index {
                quote!(#ident: ::moveit2::ctor::MaybeInit::emplace(self.#ident, ctor))
            } else {
                let meta = f.cfg.iter();
                quote!(#(#[cfg(#meta)])* #ident: self.#ident)
            }
        });

        let (impl_generics, _, where_clause) = generics.split_for_impl();

        let macro_ident = format_ident!("{IMPL_TY_MACRO}0");
        let vis = &self.vis;

        let field = &self.fields[index];
        let fn_ident = &field.field.ident;
        let field_ty = &field.field.ty;
        let cfg_meta = field.cfg.iter();
        let doc = &field.doc_attrs;
        let builder_ident = &self.builder_ident;

        quote! {
            #(#[cfg(#cfg_meta)])*
            impl #impl_generics #macro_ident!([#(#base_tys,)*] #(#in_builder_tys,)*) #where_clause {
                #(#doc)*
                #vis fn #fn_ident(self, ctor: impl ::moveit2::New<Output = #field_ty>) ->
                    #macro_ident!([#(#base_tys,)*] #(#out_builder_tys,)*)
                {
                    #builder_ident {
                        #(#field_assignments,)*
                    }
                }
            }
        }
    }

    fn generate(self) -> TokenStream2 {
        let builder_ident = &self.builder_ident;
        let marker_defs = self.marker_defs();
        let field_macros = self.fields.iter().map(|f| f.impl_ty_macro_def());
        let builder_field_defs = self.fields.iter().map(|f| f.builder_def());
        let final_ty_macro = format_ident!("{}{}", IMPL_TY_MACRO, self.fields.len());
        let vis = &self.vis;
        let generics = &self.builder_generics;
        let where_clause = &generics.where_clause;

        let builder_methods = (0..self.fields.len()).map(|i| self.builder_method(i));

        quote! {
            const _: () = {
                #marker_defs
                #(#field_macros)*

                #vis struct #builder_ident #generics #where_clause {
                    #(#builder_field_defs)*
                }

                macro_rules! #final_ty_macro {
                    ([$($tts:tt)*]) => { #builder_ident<$($tts)*> }
                }

                #(#builder_methods)*
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

#[proc_macro_attribute]
pub fn ctor(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let mut tokens: TokenStream = quote! {
        #[derive(::moveit2::ctor::__private::Ctor)]
    }
    .into();
    tokens.extend([item]);
    tokens
}
