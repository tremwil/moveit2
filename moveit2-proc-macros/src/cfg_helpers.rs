pub fn doc_attrs(attrs: &[syn::Attribute]) -> Vec<syn::Attribute> {
    attrs
        .iter()
        .filter(|attr| attr.path().is_ident("doc"))
        .cloned()
        .collect()
}

pub fn combine_cfg(attrs: &[syn::Attribute]) -> Option<syn::Meta> {
    let metas: Vec<_> = attrs
        .iter()
        .filter_map(|attr| {
            let contents = attr.meta.require_list().ok()?;
            contents.path.is_ident("cfg").then_some(contents)
        })
        .collect();

    (!metas.is_empty()).then(|| syn::parse_quote!(all(#(#metas),*)))
}
