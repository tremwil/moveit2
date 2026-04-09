use syn::{parse::Parse, punctuated::Punctuated};

pub fn is_builtin_field_attr(attr_meta: &syn::Meta) -> bool {
    let Some(ident) = attr_meta.path().get_ident() else {
        return false;
    };

    match ident.to_string().as_str() {
        "cfg" | "doc" | "allow" | "expect" | "warn" | "deny" | "forbid" | "deprecated" => true,
        "cfg_attr" => {
            let syn::Meta::List(mlist) = attr_meta else {
                return false;
            };

            struct Metas(Punctuated<syn::Meta, syn::Token![,]>);
            impl Parse for Metas {
                fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
                    Ok(Self(
                        input.parse_terminated(syn::Meta::parse, syn::Token![,])?,
                    ))
                }
            }

            let Ok(Metas(metas)) = syn::parse2(mlist.tokens.clone()) else {
                return false;
            };

            metas.len() == 2 && is_builtin_field_attr(&metas[1])
        }
        _ => false,
    }
}
