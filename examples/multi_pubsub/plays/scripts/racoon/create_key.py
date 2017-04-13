#!/usr/bin/env python 

import os
import subprocess
import argparse

def make_plainrsa_private( filename, bits=4096 ):
  cmd = "plainrsa-gen -b %d -f %s" % ( bits, filename )
  print( "Executing: %s" % cmd )
  subprocess.call( cmd.split() )


def make_plainrsa_public( priv_filename, public_filename ):
  pub_found = False
  with open( priv_filename, 'r' ) as f:
    for l in f.readlines():
      words = l.split()
      if len( words ) == 4:
        if words[2] == "PUB":
          pub_found = True
          with open( public_filename, 'w' ) as f2:
            f2.write( ' '.join( '%s' % x for x in words[1:] ) )
          break
  return pub_found


def make_keys( folder, host_name ):
  private_file = "%s/%s" % (folder, host_name)
  public_file = "%s/%s.pub" % (folder, host_name)
  if os.path.exists( private_file ):
    print( "Using existing private key %s" % private_file )
  else:
    print( "Writing private key to %s" % private_file )
    make_plainrsa_private( private_file )
  print( "  Writing public key to %s" % public_file )
  make_plainrsa_public( private_file, public_file )
  return public_file

if __name__ == "__main__":
  parser = argparse.ArgumentParser( description = "Tool to create ipsec.yml file" )
  parser.add_argument( "hostname", help = "Hostname of node" )
  parser.add_argument( "--folder", default="/etc/racoon/certs", help = "Output folder" )
  args = parser.parse_args()
  make_keys( args.folder, args.hostname )
