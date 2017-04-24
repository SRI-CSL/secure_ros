#!/bin/bash

if ! git diff-index --quiet HEAD; then
  echo "Git is dirty not touching it"
  return 1
fi

git_commit=`git rev-parse --short HEAD`
doc_tmp=`mktemp`

# echo commands before running them to keep track of what is happening
set -x

make html || (echo failed to make doc; return 1)

tar -cf "${doc_tmp}" -C _build/html .
make clean && rmdir _build

git checkout master

rm -r ../docs
mkdir ../docs
tar -xf "${doc_tmp}" -C ../docs
touch ../docs/.nojekyll

git add -A ../docs &&\
git commit -m "Update doc from ${git_commit}" &&\
git push origin master

set +x
echo 
echo 
echo You are now on updated master branch

return 0
