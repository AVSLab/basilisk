#!/bin/bash

regex='([0-9]+\.[0-9]+\.)([0-9]+)'

# Read file line by line and process each line
while IFS= read -r version || [[ -n "$version" ]]; do
    if [[ $version =~ $regex ]]; then
        # Extract the last number and increment it by one
        last_number=${BASH_REMATCH[2]}
        incremented_number=$((last_number + 1))
        # Replace the last number in the line
        updated_version=${BASH_REMATCH[1]}$incremented_number
        echo "Bumping from $version to $updated_version"
    else
        echo "Version $version is not in the format X.Y.Z; not updating version number"
        exit 0
    fi
done < $1

echo "$updated_version" > $1