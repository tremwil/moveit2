use syn::Visibility;

/// Returns the strictest of two visibilities, or private/inherited if the
/// comparison is impossible.
pub fn vis_min(a: &Visibility, b: &Visibility) -> Visibility {
    use Visibility::*;

    match (a, b) {
        (Inherited, _) => Inherited,
        (_, Inherited) => Inherited,
        (Public(_), b) => b.clone(),
        (a, Public(_)) => a.clone(),
        (Restricted(ra), Restricted(rb)) => {
            let pa = &ra.path;
            let pb = &rb.path;

            // self is the same as private
            if pa.is_ident("self") || pb.is_ident("self") {
                return Inherited;
            }
            // if either path is crate, that dominates the other
            if pb.is_ident("crate") {
                return a.clone();
            }
            if pa.is_ident("crate") {
                return b.clone();
            }
            // otherwise we'll have to inspect paths. For an empty path just assume private
            let Some(root_a) = pa.segments.first() else {
                return Inherited;
            };
            let Some(root_b) = pb.segments.first() else {
                return Inherited;
            };
            // if both paths start with super, take the shortest one
            if root_a.ident == "super" && root_b.ident == "super" {
                if pa.segments.len() <= pb.segments.len() {
                    a.clone()
                } else {
                    b.clone()
                }
            }
            // if both paths don't start with super, take the longest one
            else if root_a.ident != "super" && root_b.ident != "super" {
                if pa.segments.len() >= pb.segments.len() {
                    a.clone()
                } else {
                    b.clone()
                }
            // otherwise, it's impossible to compare them, be pessimistic
            } else {
                Inherited
            }
        }
    }
}
