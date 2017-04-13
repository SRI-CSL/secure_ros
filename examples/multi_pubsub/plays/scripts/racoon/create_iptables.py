#!/usr/bin/env python 

import os
import sys
import shutil 
import subprocess
import argparse
import yaml


def make_iptables_rules( file_obj, allowed_ip_addresses ):
  file_obj.write( '*filter\n' )
  file_obj.write( ':INPUT DROP [0:0]\n' )
  file_obj.write( ':FORWARD DROP [0:0]\n' )
  file_obj.write( ':OUTPUT ACCEPT [0:0]\n' )
  file_obj.write( '-A INPUT -i lo -j ACCEPT\n' )
  for ip in allowed_ip_addresses:
    file_obj.write( '-A INPUT -s %s -j ACCEPT\n' % ip )
  file_obj.write( 'COMMIT\n' )



def make_iptablesload_script( filename ):
  if not os.path.exists( os.path.dirname( filename ) ): 
    os.makedirs( os.path.dirname( filename ) )
  with open( filename, "w" ) as file_obj:
    file_obj.write( '#!/bin/sh\n' )
    file_obj.write( 'iptables-restore < /etc/iptables.rules\n' )
    file_obj.write( 'exit 0\n' )



def make_iptables_conf( filename, hostname, doc, allowed_ip_addresses ):
  if not os.path.exists( os.path.dirname( filename ) ): 
    os.makedirs( os.path.dirname( filename ) )
  print( doc )
  with open( filename, "w" ) as f:
    peers = set( doc.keys() ) - set( [hostname] )
    peer_ip_addresses = list( doc[p] for p in peers )
    make_iptables_rules( f, allowed_ip_addresses + peer_ip_addresses )
    
 

def make_config( doc, hostname, prefix, allowed_ip_addresses = [] ):
  if hostname in doc.keys():
    iptables_conf = '%s/etc/iptables.rules' % prefix
    print( 'Writing to %s' % iptables_conf )
    make_iptables_conf( iptables_conf, hostname, doc, allowed_ip_addresses )

    iptablesload_script = '%s/etc/network/if-pre-up.d/iptablesload' % prefix
    print( 'Writing to %s' % iptablesload_script )
    make_iptablesload_script( iptablesload_script )



if __name__ == "__main__":
  parser = argparse.ArgumentParser( description = "Tool to create iptables configuration files" )
  parser.add_argument( "-A", default = ["10.0.2.2"], nargs = "*",
      metavar = "IP_ADDRESSES", dest = "allowed_ip_addresses",
      help = "Allowed IP addresses" )
  parser.add_argument( "hostname", metavar = "HOSTNAME", help = "hostname" )
  parser.add_argument( "security", metavar = "SECURITY_FILE", help = "YAML Security configuration" )
  parser.add_argument( "--prefix", metavar = "OUTPUT_PREFIX", default="", help = "Output folder prefix" )
  args = parser.parse_args()

  with open( args.security ) as istream:
    doc = yaml.load( istream )

  make_config( doc, args.hostname, args.prefix, allowed_ip_addresses = args.allowed_ip_addresses )





