#!/usr/bin/env bash
set -eEuo pipefail

PUBLISH=()
METADATA="$(cargo metadata --format-version 1)"

for PACKAGE in moveit2 moveit2-proc-macros; do
    LOCAL=$(echo "$METADATA" | jq ".packages[] | select(.name == \"$PACKAGE\") .version" -r)

    if [[ -z $LOCAL ]]; then
        echo "'$PACKAGE': Package not found in workspace."
        exit 1
    fi

    echo "$PACKAGE=$LOCAL" >> $GITHUB_OUTPUT

    UPSTREAM=$(cargo info $PACKAGE --registry crates-io --color never 2>/dev/null | grep -o '^version: [0-9\.]*' || true)
    UPSTREAM="${UPSTREAM##*: }"

    # Publish if upstream version was bumped
    if [[ "$UPSTREAM" != "$LOCAL" ]]; then
        echo "$PACKAGE has been bumped from ${UPSTREAM:-(unreleased)} to $LOCAL, publishing..."
        PUBLISH+=("-p" "$PACKAGE")
    fi
done

if [[ ${#PUBLISH[@]} -gt 0 ]]; then
    cargo publish --locked --all-features $@ ${PUBLISH[@]}
    echo "need-publish=true" >> $GITHUB_OUTPUT
else
    echo "Packages are all up-to-date!"
fi