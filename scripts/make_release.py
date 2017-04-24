#!/usr/bin/env python 

import argparse 
import yaml
import os
from collections import OrderedDict
import git

def setup_yaml():
  """ http://stackoverflow.com/a/8661021 """
  represent_dict_order = lambda self, data:  self.represent_mapping('tag:yaml.org,2002:map', data.items())
  yaml.add_representer(OrderedDict, represent_dict_order)    
setup_yaml()


def git_checkout( root, tag ):
  print( "%s: git checkout %s" % ( root, tag ) )
  repo = git.Repo( root )
  repo.git.checkout( tag )


def git_checkout_modules( root, release, modules ):
  for folder, version in modules.items():
    submodule = "%s/%s" % ( root, folder )
    tag = "secure-ros/%s/%s" % ( release, version )
    git_checkout( submodule, tag )


def make_secure_ros( filename, release, version, architectures ):
  data = OrderedDict()
  data["cpus"] = 1
  data["memory"] = 1024
  data["release"] = release
  data["version"] = version
  data["architectures"] = architectures[release]
  with open( filename, "w" ) as handle:
    yaml.dump( data, handle )


def make_release( root, release, version, patch, conf_file ):
  with open( conf_file ) as handle:
    conf = yaml.load( handle )
  git_checkout_modules( root, release, conf["modules"] )
  full_version = "%s-%s" % ( version, patch )
  secure_ros_file = "%s/examples/package/secure_ros.yaml" % root
  make_secure_ros( secure_ros_file, release, full_version, conf["architectures"] )

  print( "\n== Now run the following ==\n" )
  branch_name = "releases/%s-%s" % ( release, full_version )
  tag_name = "%s/%s" % ( release, full_version )
  msg = '"Add release %s"' % tag_name
  if release != "kinetic":
    print( 'git checkout -b %s' % branch_name )
    print( 'git commit -a -m %s' % msg )
  print( 'git tag %s -f -a -m %s' % ( tag_name, msg ) )
  print( 'git push origin %s"' % tag_name )


if __name__ == "__main__":
  parser = argparse.ArgumentParser( description="Make Secure ROS release" )
  parser.add_argument( "--verbosity", "-v", action="count", default = 0, help="Verbosity level" )
  parser.add_argument( "--version", "-V", type = str, default = "0.9.2", help = "Version" )
  parser.add_argument( "--patch", "-P", type = str, default = "1", help = "Patch" )
  parser.add_argument( "--release", "-R", type = str, default = "kinetic", help = "ROS release" )
  parser.add_argument( "--conf_file", "-C", type = str, default = "scripts/conf.yaml", help = "Configuration" )
  parser.add_argument( "--root", "-r", type = str, default = ".", help = "Git root" )
  args = parser.parse_args()
  setup_yaml()

  root = os.path.realpath( args.root )
  print( "Root: %s" % root )

  make_release( root, args.release, args.version, args.patch, args.conf_file )

    




