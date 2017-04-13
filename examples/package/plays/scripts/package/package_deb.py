#!/usr/bin/env python 

import argparse 
import yaml 
import subprocess
import os
from collections import OrderedDict

control_dict = {
    "secure-ros": OrderedDict( { 
      "Maintainer": "Aravind Sundaresan <asundaresan@gmail.com>",
      "Description": "",
      "Package": None,
      "Section": "misc",
      "Priority": "extra",
      "Depends": "",
      "Version": None,
      "Architecture": None,
      "Description": """Secure ROS core package 
  This is the equivalent of the ros-indigo-ros-comm meta-package (but is packaged as a single binary package)"""
      } )
    }

def get_dependencies_from_src( src, verbosity = 0 ):
  cmd = "/usr/local/bin/rosdep install --from-paths %s --ignore-src --rosdistro indigo -s --reinstall --skip-keys python-catkin-pkg --skip-keys python-rosdep --skip-keys python-wstool  --skip-keys python-rospkg" % src
  output = subprocess.check_output( cmd.split() )
  packages = list()
  for line in output.splitlines():
    fields = line.split()
    if fields[0] == "sudo" and fields[2] == "apt-get":
      if verbosity > 0:
        print( "%s: %s" % ( fields[3], fields[4:] ) )
      packages.extend( fields[4:] )
  return ", ".join( dep for dep in packages )

def get_dependencies( filename, verbosity = 0 ):
  packages = list()
  with open( filename, "r" ) as handle:
    for line in handle.readlines():
      fields = line.split()
      if fields[0] == "sudo" and fields[2] == "apt-get":
        if verbosity > 0:
          print( "%s: %s" % ( fields[3], fields[4:] ) )
        packages.extend( fields[4:] )
  return ", ".join( dep for dep in packages )


def get_arch( verbosity = 0 ):
  cmd = "dpkg-architecture -qDEB_HOST_ARCH"
  output = subprocess.check_output( cmd.split() ).strip()
  if verbosity > 0:
    print( "%s ->\n%s" % ( cmd, output ) )
  return output


def write_control( filename, control, verbosity = 0 ):
  if not os.path.exists( os.path.dirname( filename ) ):
    print( "Creating folder: %s" % os.path.dirname( filename ) )
    os.makedirs( os.path.dirname( filename ) )
  print( "Writing to %s" % filename )
  with open( filename, "w" ) as handle:
    for key, val in control.items():
      if key == "Description":
        handle.write( "%s: %s\n" % ( key, "\n ".join( v for v in val.splitlines() ) ) )
      else:
        handle.write( "%s: %s\n" % ( key, val ) )


def make_control( name, version, depends, verbosity = 0 ):
  control = OrderedDict( (k,v) for k,v in control_dict["secure-ros"].items() )
  control["Package"] = name 
  control["Version"] = version
  control["Depends"] = ", ".join( d for d in depends )
  control["Architecture"] = get_arch( verbosity )
  return control 


if __name__ == "__main__":
  parser = argparse.ArgumentParser( description = "Create DEBIAN control file" )
  parser.add_argument( "--name", "-P", type = str, help="Package name" )
  parser.add_argument( "--version", "-V", type = str, help="Package version" )
  parser.add_argument( "--depends", "-D", type = str, help="Package dependencies as CSV" )
  parser.add_argument( "--output", "-O", type = str, help="Output control file" )
  parser.add_argument( "--verbosity", "-v", action="count", default = 0, help="Verbosity level" )
  args = parser.parse_args()
  depends = list( s.lstrip("u").strip( "'\"" ) for s in yaml.load( args.depends ) )
  control = make_control( args.name, args.version, depends, verbosity = args.verbosity )
  write_control( args.output, control, verbosity = args.verbosity )


