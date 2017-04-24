Release script
==============

Initialize correctly. ::

  git checkout kinetic-devel 
  git submodule update

* Make the release for `kinetic`. ::

    python scripts/make_release.py

* Make the release for `indigo`. ::

    python scripts/make_release.py -R indigo

  Make a new branch for indigo and commit. ::

    git checkout -b releases/indigo-0.9.2-1
    git commit -a -m 'Add release 0.9.2-1'

  Make an annotated tag and push that to origin. ::

    git tag indigo/0.9.2-1 -f -am "Add release 0.9.2-1"
    git push origin indigo/0.9.2-1

Miscellaneous 
=============

To remove a tag. :: 

  git push --delete origin <TAG_NAME>

